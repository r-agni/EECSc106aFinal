import time
import sys
import math
import cv2
import cv2.aruco as aruco
import msvcrt

import pyardrone
from pyardrone import at


# --- CONTROL-ONLY DRONE: NO VIDEO CLIENT ---
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """High-level helpers + navdata, but no internal video connection."""
    pass


# ---------------- CONFIGURATION (ALL METERS) ----------------
TAG_SIZE_M = 0.175   # 175 mm ArUco marker side

TARGET_ALT_M = 1.22  # target altitude (~4 ft) in meters
TARGET_ALT_MM = int(TARGET_ALT_M * 1000.0)

SEARCH_SPEED = 0.1        # main lawnmower/search speed in m/s
LOCAL_SEARCH_SPEED = 0.06  # slower, for 0.3 m local search box
CENTER_SPEED = 0.1        # gentle speed while centering over tags
POS_TOL = 0.10             # waypoint position tolerance (meters, ~10 cm)

# 3.05 m box (roughly 10 ft)
BOX_SIDE_M = 3.05
HALF_WIDTH_M = BOX_SIDE_M / 2.0   # 1.525 m

# Step along +x between search legs (approx 0.3 m per pass)
STEP_X_M = 0.30

# Land 0.6 m in front of Tag 1 along +x
FORWARD_LAND_M = 0.60

# Local search box side 0.6 m (0.6 x 0.6), half-size 0.3 m
LOCAL_BOX_HALF_M = 0.15

# Camera intrinsics for ArUco pose (TODO: fill with real calibration)
CAMERA_MATRIX = None
DIST_COEFFS = None

# Optional: set to True if you want detailed pose debug prints
DEBUG_POSE = False


def update_pose_from_nav(drone, dt, x, y, theta):
    """
    Integrate velocities from navdata demo to maintain pose estimate.

    Navdata assumptions:
      - demo.vx, demo.vy are in mm/s in body frame.
      - vx: forward
      - vy: left

    World frame we define:
      +x = forward (from takeoff)
      +y = RIGHT (when facing +x)

    Steps:
      1) convert mm/s -> m/s
      2) define vy_body = -vy_left (so +y_body is right)
      3) rotate body velocities to world using yaw (psi)
      4) integrate to update (x,y)
    """
    demo = getattr(drone.navdata, "demo", None)
    if not demo or dt <= 0:
        return x, y, theta

    # 1) Body-frame velocities in m/s
    vx_body = demo.vx / 1000.0
    vy_left = demo.vy / 1000.0
    vy_body = -vy_left  # +y_body = right

    # 2) Yaw in radians (psi is in milli-degrees)
    yaw_rad = (demo.psi / 1000.0) * (math.pi / 180.0)
    theta = yaw_rad

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # 3) Rotate into world frame
    vx_world = vx_body * cos_t - vy_body * sin_t
    vy_world = vx_body * sin_t + vy_body * cos_t

    # 4) Integrate
    x += vx_world * dt
    y += vy_world * dt

    if DEBUG_POSE:
        print(f"[DEBUG_POSE] dt={dt:.3f}, "
              f"vx_body={vx_body:.3f}, vy_body={vy_body:.3f}, "
              f"theta={theta:.3f}, x={x:.3f}, y={y:.3f}")

    return x, y, theta


def move_towards_xy(drone, x_rel, y_rel, x_target, y_target, speed):
    """
    Position-based controller in meters.

    Inputs:
      x_rel, y_rel: current position in some 2D frame (Tag 0 frame or local box frame)
      x_target, y_target: target waypoint in same frame
      speed: movement speed (m/s) for forward/back/right/left

    Logic:
      - Compute dx, dy.
      - If distance < POS_TOL: hover, return True.
      - Otherwise:
          * dx controls forward/back
          * dy controls right/left
      - Note: no deceleration logic; overshoot can still happen if pose is noisy.
    """
    dx = x_target - x_rel
    dy = y_target - y_rel

    dist = math.sqrt(dx * dx + dy * dy)
    if dist < POS_TOL:
        drone.hover()
        return True

    forward_cmd = 0.0
    backward_cmd = 0.0
    right_cmd = 0.0
    left_cmd = 0.0

    # +x is forward
    if dx > POS_TOL:
        forward_cmd = speed
    elif dx < -POS_TOL:
        backward_cmd = speed

    # +y is right
    if dy > POS_TOL:
        right_cmd = speed
    elif dy < -POS_TOL:
        left_cmd = speed

    drone.move(forward=forward_cmd,
               backward=backward_cmd,
               right=right_cmd,
               left=left_cmd)

    if DEBUG_POSE:
        print(f"[DEBUG_MOVE] target=({x_target:.3f},{y_target:.3f}), "
              f"current=({x_rel:.3f},{y_rel:.3f}), dx={dx:.3f}, dy={dy:.3f}")

    return False


def center_on_tag(drone, frame, target_id, ids, corners,
                  center_tolerance_px=30, speed=CENTER_SPEED, state=None):
    """
    Visual centering over an ArUco tag using only image geometry.

    Steps:
      - Find tag with id == target_id.
      - Compute its pixel center (cx, cy).
      - Compare to image center (cx0, cy0).
      - If |dx| > tol: move right/left.
      - Else if |dy| > tol: move forward/back.
      - If within tolerance for N consecutive frames, return centered=True.

    All motion here is small and at low speed for stable detection.
    """
    if ids is None:
        drone.hover()
        return False, state

    ids_list = ids.flatten().tolist()
    if target_id not in ids_list:
        drone.hover()
        return False, state

    idx = ids_list.index(target_id)
    tag_corners = corners[idx][0]  # (4,2) array

    cx = tag_corners[:, 0].mean()
    cy = tag_corners[:, 1].mean()

    h, w = frame.shape[:2]
    cx0 = w / 2.0
    cy0 = h / 2.0

    dx = cx - cx0  # +dx: tag right in image
    dy = cy - cy0  # +dy: tag below center in image

    if state is None:
        state = {"good_frames": 0}

    good_frames = state.get("good_frames", 0)
    moved = False

    # Lateral correction (right/left)
    if abs(dx) > center_tolerance_px:
        if dx > 0:
            drone.move(right=speed)
        else:
            drone.move(left=speed)
        moved = True

    # Longitudinal correction (forward/back)
    elif abs(dy) > center_tolerance_px:
        if dy > 0:
            drone.move(forward=speed)
        else:
            drone.move(backward=speed)
        moved = True

    if not moved:
        drone.hover()
        good_frames += 1
    else:
        good_frames = 0

    state["good_frames"] = good_frames
    centered = (good_frames >= 5)
    return centered, state


def main():
    print("[INFO] Connecting to AR.Drone (no video)...")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    # ------------- NAVDATA INIT -------------
    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=5.0)

    if drone.navdata_ready.is_set():
        drone.send(at.CONFIG('general:navdata_demo', True))
        time.sleep(0.1)
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            print(f"[NAVDATA] Battery: {demo.vbat_flying_percentage}%")
        else:
            print("[WARN] Demo navdata not populated yet.")
    else:
        print("[WARN] No navdata after 5s, continuing anyway.")

    # ------------- CAMERA SWITCH TO BOTTOM -------------
    print("[INFO] Switching to BOTTOM camera...")
    drone.send(at.CONFIG('video:video_channel', 1))
    time.sleep(0.5)

    # ------------- OPEN VIDEO STREAM -------------
    stream_url = "tcp://192.168.1.1:5555"
    print(f"[INFO] Opening video stream: {stream_url}")
    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] Cannot open video stream with OpenCV.")
        print("       Make sure ffplay is CLOSED and no other process is")
        print("       connected to tcp://192.168.1.1:5555.")
        return

    # ------------- ARUCO SETUP -------------
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    # ------------- TAKEOFF + CLIMB TO 1.22 m -------------
    print("[INFO] Takeoff + climb to ~1.22 m (~4 ft)...")
    drone.takeoff()
    time.sleep(2.0)

    # Pose state in meters (world frame: +x forward, +y right)
    x = 0.0
    y = 0.0
    theta = 0.0
    last_pose_time = time.time()

    climb_start = time.time()
    climb_timeout = 8.0

    while True:
        now = time.time()
        dt = now - last_pose_time
        last_pose_time = now

        x, y, theta = update_pose_from_nav(drone, dt, x, y, theta)

        demo = getattr(drone.navdata, "demo", None)
        if demo:
            alt = demo.altitude  # mm
            print(f"[ALT] {alt} mm", end="\r")
            if alt >= TARGET_ALT_MM:
                break

        if now - climb_start > climb_timeout:
            print("\n[WARN] Altitude not reaching 1.22 m in time, stopping climb.")
            break

        drone.move(up=0.3)
        time.sleep(0.05)

    drone.hover()
    print("\n[INFO] Reached approx. 1.22 m. Letting video stabilize for 2 seconds...")

    # --------- 2s STABILIZATION + TRY TO FIND TAG 0 IN PLACE ----------
    stabilize_start = time.time()
    tag0_seen_in_place = False

    while time.time() - stabilize_start < 2.0:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if ids is not None:
            ids_list = ids.flatten().tolist()
            aruco.drawDetectedMarkers(frame, corners, ids)
            if 0 in ids_list:
                tag0_seen_in_place = True
                cv2.imshow("Stabilize (Tag0 check)", frame)
                cv2.waitKey(1)
                break

        cv2.imshow("Stabilize (Tag0 check)", frame)
        cv2.waitKey(1)
        time.sleep(0.02)

    cv2.destroyWindow("Stabilize (Tag0 check)")
    drone.hover()

    # --------- RESET POSE HERE FOR LOCAL 0.3 m BOX ----------
    # This makes (0,0) the center of the 0.3 m box, independent of climb drift.
    x = 0.0
    y = 0.0
    # keep theta as yaw from navdata
    last_pose_time = time.time()

    print("[INFO] Pose reset: x=0, y=0 at start of local 0.3 m search / Tag0 centering.")

    # ---------------- STATE VARIABLES ----------------
    # If we already saw Tag0 in place, skip local search and go straight to centering
    phase = "CENTER_TAG0" if tag0_seen_in_place else "FIND_TAG0_LOCAL"
    center_tag0_state = None
    center_tag1_state = None

    pose_tag0 = None  # (x0, y0, theta0) in global/world frame
    pose_tag1 = None  # (x1, y1, theta1) in global/world frame

    # Local 0.6 m box: half-size 0.3 m, centered at (0,0)
    local_waypoints = [
        (0.0, 0.0),                           # center
        (0.0, +LOCAL_BOX_HALF_M),             # right 0.3 m
        (0.0, -LOCAL_BOX_HALF_M),             # left  0.3 m
        (+LOCAL_BOX_HALF_M, -LOCAL_BOX_HALF_M),  # forward-left
        (+LOCAL_BOX_HALF_M, +LOCAL_BOX_HALF_M),  # forward-right
        (+LOCAL_BOX_HALF_M, 0.0),                # forward center
    ]
    local_index = 0

    # Lawn mower search variables (in meters)
    leg_index = 0
    leg_dir = -1  # first leg moves toward y = -HALF_WIDTH_M
    max_legs = 10
    leg_target_y = None
    step_target_x = None
    move_front1_target_x = None

    search_paused = False
    emergency_land = False

    search_start_time = time.time()
    max_total_time = 180.0  # 3 min safety

    while True:
        now = time.time()
        dt = now - last_pose_time
        last_pose_time = now

        # Pose integration in meters
        x, y, theta = update_pose_from_nav(drone, dt, x, y, theta)

        # Coordinates relative to Tag 0 frame if we know Tag 0 pose.
        if pose_tag0 is not None:
            x_rel = x - pose_tag0[0]
            y_rel = y - pose_tag0[1]
        else:
            # Before Tag 0 is fixed, x,y are in "local" frame (center of 0.3m box)
            x_rel = x
            y_rel = y

        # Keyboard controls: 'l', 'h', 'c'
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key == 'l':
                print("[INPUT] 'l' pressed → emergency land.")
                emergency_land = True
                break
            elif key == 'h':
                search_paused = True
                drone.hover()
                print("[INPUT] 'h' pressed → search PAUSED (hover).")
            elif key == 'c':
                if search_paused:
                    search_paused = False
                    print("[INPUT] 'c' pressed → search RESUMED.")

        if now - search_start_time > max_total_time:
            print("[TIMER] Global timeout reached → landing.")
            break

        ret, frame = cap.read()
        if not ret:
            print("[WARN] No video frame (retrying)...")
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if ids is not None:
            ids_list = ids.flatten().tolist()
            aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            ids_list = []

        if search_paused:
            drone.hover()
        else:
            # ------ PHASE: FIND_TAG0_LOCAL (0.6 m box) ------
            if phase == "FIND_TAG0_LOCAL":
                if ids is not None and 0 in ids_list:
                    print("[PHASE] Tag 0 detected in local 0.6 m search. Switching to CENTER_TAG0.")
                    phase = "CENTER_TAG0"
                    center_tag0_state = None
                    drone.hover()
                else:
                    # Move around 0.6m box using pose-based waypoints
                    tx, ty = local_waypoints[local_index]
                    reached = move_towards_xy(
                        drone, x_rel, y_rel,
                        x_target=tx, y_target=ty,
                        speed=LOCAL_SEARCH_SPEED
                    )
                    if reached:
                        local_index = (local_index + 1) % len(local_waypoints)

            # ------ PHASE: CENTER_TAG0 ------
            elif phase == "CENTER_TAG0":
                centered, center_tag0_state = center_on_tag(
                    drone, frame, target_id=0,
                    ids=ids, corners=corners,
                    center_tolerance_px=30,
                    speed=CENTER_SPEED,
                    state=center_tag0_state
                )
                if centered:
                    print("[PHASE] Centered above Tag 0.")
                    if pose_tag0 is None:
                        pose_tag0 = (x, y, theta)
                        print(f"[POSE] Tag 0 pose recorded: x={x:.3f} m, "
                              f"y={y:.3f} m, θ={theta:.3f} rad")
                    # Next: move to bottom-right corner (x_rel ≈ 0, y_rel ≈ +HALF_WIDTH_M)
                    phase = "MOVE_TO_RIGHT_EDGE"
                    drone.hover()

            # ------ PHASE: MOVE_TO_RIGHT_EDGE ------
            elif phase == "MOVE_TO_RIGHT_EDGE":
                tx = 0.0
                ty = +HALF_WIDTH_M
                reached = move_towards_xy(
                    drone, x_rel, y_rel,
                    x_target=tx, y_target=ty,
                    speed=SEARCH_SPEED
                )
                if reached:
                    print("[PHASE] Reached bottom-right corner of 3.05m box. Start search legs.")
                    phase = "SEARCH_LEG"
                    leg_index = 0
                    leg_dir = -1             # first leg: y to -HALF_WIDTH_M
                    leg_target_y = -HALF_WIDTH_M
                    drone.hover()

            # ------ PHASE: SEARCH_LEG ------
            elif phase == "SEARCH_LEG":
                if ids is not None and 1 in ids_list:
                    print("[PHASE] Tag 1 detected in search area. Switching to CENTER_TAG1.")
                    phase = "CENTER_TAG1"
                    center_tag1_state = None
                    drone.hover()
                else:
                    # Move along y to leg_target_y while holding x_rel fixed
                    reached = move_towards_xy(
                        drone, x_rel, y_rel,
                        x_target=x_rel,
                        y_target=leg_target_y,
                        speed=SEARCH_SPEED
                    )
                    if reached:
                        if leg_index >= max_legs - 1:
                            print("[PHASE] Completed all search legs, Tag 1 not found.")
                            break
                        print(f"[PHASE] Completed leg {leg_index+1}/{max_legs}, "
                              "stepping +x by ~0.3 m.")
                        phase = "STEP_FORWARD"
                        step_target_x = x_rel + STEP_X_M
                        drone.hover()

            # ------ PHASE: STEP_FORWARD ------
            elif phase == "STEP_FORWARD":
                reached = move_towards_xy(
                    drone, x_rel, y_rel,
                    x_target=step_target_x,
                    y_target=y_rel,
                    speed=SEARCH_SPEED
                )
                if reached:
                    leg_index += 1
                    leg_dir *= -1
                    leg_target_y = HALF_WIDTH_M if leg_dir == 1 else -HALF_WIDTH_M
                    print(f"[PHASE] Starting leg {leg_index+1}/{max_legs}, "
                          f"target y_rel = {leg_target_y:.2f} m.")
                    phase = "SEARCH_LEG"
                    drone.hover()

            # ------ PHASE: CENTER_TAG1 ------
            elif phase == "CENTER_TAG1":
                centered, center_tag1_state = center_on_tag(
                    drone, frame, target_id=1,
                    ids=ids, corners=corners,
                    center_tolerance_px=30,
                    speed=CENTER_SPEED,
                    state=center_tag1_state
                )
                if centered:
                    print("[PHASE] Centered above Tag 1.")
                    if pose_tag1 is None:
                        pose_tag1 = (x, y, theta)
                        print(f"[POSE] Tag 1 pose recorded: x={x:.3f} m, "
                              f"y={y:.3f} m, θ={theta:.3f} rad")
                    # Next: move 0.6 m forward along +x_rel and land
                    phase = "MOVE_FRONT1"
                    move_front1_target_x = x_rel + FORWARD_LAND_M
                    drone.hover()

            # ------ PHASE: MOVE_FRONT1 ------
            elif phase == "MOVE_FRONT1":
                reached = move_towards_xy(
                    drone, x_rel, y_rel,
                    x_target=move_front1_target_x,
                    y_target=y_rel,
                    speed=SEARCH_SPEED
                )
                if reached:
                    print("[PHASE] Reached landing offset point in front of Tag 1.")
                    break

            else:
                drone.hover()

        # Overlay pose + phase on video
        pose_text = f"x_rel={x_rel:.2f} m, y_rel={y_rel:.2f} m"
        theta_text = f"θ={theta:.2f} rad"
        cv2.putText(frame, pose_text, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, theta_text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"phase={phase}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("AR.Drone Bottom Camera (0.3m Tag0 + 3.05m Search)", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC
            print("[INPUT] ESC pressed → landing.")
            break

        time.sleep(0.01)

    # ------------- LANDING -------------
    print("[INFO] Landing...")
    drone.land()
    time.sleep(3.0)

    if drone.navdata_ready.is_set():
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            print(f"[NAVDATA] End Battery: {demo.vbat_flying_percentage}%")

    cap.release()
    cv2.destroyAllWindows()

    # ------------- TRANSFORM OUTPUT T_0^1 (Δθ in degrees) -------------
    if pose_tag0 is not None and pose_tag1 is not None:
        x0, y0, th0 = pose_tag0
        x1, y1, th1 = pose_tag1

        dx = x1 - x0
        dy = y1 - y0
        dtheta = th1 - th0
        dtheta_deg = dtheta * 180.0 / math.pi

        cos_d = math.cos(dtheta)
        sin_d = math.sin(dtheta)

        print("\n[RESULT] Relative transform from Tag 0 frame to Tag 1 frame:")
        print(f"Δx = {dx:.3f} m, Δy = {dy:.3f} m, Δθ = {dtheta_deg:.2f} deg")
        print("Homogeneous 2D transform T_0^1:")
        print(f"[[ {cos_d:.3f}, {-sin_d:.3f}, {dx:.3f} ],")
        print(f" [ {sin_d:.3f},  {cos_d:.3f}, {dy:.3f} ],")
        print( " [  0.000,   0.000,  1.000 ]]")
        print("\nInterpretation (with +x forward, +y right):")
        print("  - Tag 1 is Δx meters forward and Δy meters to the right of Tag 0.")
        print("  - Δθ is the yaw difference (Tag 1 frame relative to Tag 0 frame) in degrees.")
        print("Use this as the straight-line mission vector for the Tello.")
    else:
        print("\n[RESULT] Did not get both Tag 0 and Tag 1 poses.")
        print("         Make sure Tag 0 and Tag 1 were seen and centered at least once.")

    print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C → emergency exit & land.")
        cv2.destroyAllWindows()
        sys.exit(0)
