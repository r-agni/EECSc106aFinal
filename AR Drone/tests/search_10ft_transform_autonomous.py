import time
import sys
import math
import msvcrt

import cv2
import cv2.aruco as aruco

import pyardrone
from pyardrone import at


# --------------------------------------------------------------------
#  DRONE CLASS: Navdata + helpers, NO internal video thread
# --------------------------------------------------------------------
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """High-level helpers + navdata, but no internal video client."""
    pass


# --------------------------------------------------------------------
#  CONSTANTS
# --------------------------------------------------------------------
# Altitude
TARGET_ALT_M = 0.8                     # ≈ 2.6 ft altitude
TARGET_ALT_MM = int(TARGET_ALT_M * 1000)

# Search box (10 ft ~= 3.05 m)
BOX_LEN_M = 3.05                       # total length along x
ROW_STEP_M = 0.305                      # 1 ft between rows (~0.305 m)

# Speeds
SEARCH_SPEED = 0.035                    # m/s-ish for rows and for tag positional centering
STEP_SPEED = 0.1                      # for 1 ft steps between rows
TURN_SPEED = 0.4                      # yaw rate for 90° turns
CENTER_SPEED = 0.025                    #optical centering

# Tolerances
POS_TOL = 0.15                         # ~10 cm for position
PX_TOL = 100                            # pixel tolerance for centering
YAW_TOL_DEG = 90.0                      # yaw tolerance when turning

# Tags
TAG0_ID = 0
TAG1_ID = 5

# If we reach origin with no Tag0 for this many frames, we give up.
ORIGIN_NO_TAG_FRAMES_THRESH = 60 * 6   # ~6s at 60fps-like loop


# --------------------------------------------------------------------
#  POSE INTEGRATION (x forward, y right)
# --------------------------------------------------------------------
def update_pose_from_nav(drone, dt, x, y, theta):
    """
    Integrate navdata velocities to maintain pose (x,y,theta) in meters.

    World frame anchored at takeoff / origin:
      +x_world = forward from Tag 0
      +y_world = right   from Tag 0

    Drone Demo Data:
      demo.vx > 0 when moving FORWARD  (mm/s)
      demo.vy > 0 when moving RIGHT    (mm/s)
      demo.psi is yaw (mdeg)
    """
    demo = getattr(drone.navdata, "demo", None)
    if not demo or dt <= 0:
        return x, y, theta

    # Body-frame velocities (m/s)
    vx_body = demo.vx / 1000.0   # +forward
    vy_body = demo.vy / 1000.0   # +right

    # Yaw (psi) in radians
    yaw_rad = (demo.psi / 1000.0) * (math.pi / 180.0)
    theta = yaw_rad

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # Rotate body → world
    vx_world = vx_body * cos_t - vy_body * sin_t
    vy_world = vx_body * sin_t + vy_body * cos_t

    x += vx_world * dt
    y += vy_world * dt

    return x, y, theta


# --------------------------------------------------------------------
#  MOVE TOWARD (x_target, y_target) USING NAVDATA
# --------------------------------------------------------------------
def move_towards_xy(drone, x_cur, y_cur, x_target, y_target, speed):
    """
    Move drone toward (x_target, y_target) in world frame (x forward, y right).

    We keep this simple: proportional-only steering toward the target
    using forward/backward and left/right commands.
    """
    dx = x_target - x_cur
    dy = y_target - y_cur
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < POS_TOL:
        drone.hover()
        return True  # reached

    forward_cmd = backward_cmd = 0.0
    left_cmd = right_cmd = 0.0

    # Forward/back along x
    if dx > POS_TOL:
        forward_cmd = speed
    elif dx < -POS_TOL:
        backward_cmd = speed

    # Right/left along y (y>0 = right)
    if dy > POS_TOL:
        right_cmd = speed
    elif dy < -POS_TOL:
        left_cmd = speed

    drone.move(
        forward=forward_cmd,
        backward=backward_cmd,
        left=left_cmd,
        right=right_cmd
    )
    return False

# --------------------------------------------------------------------
#  SIMPLE YAW CONTROL
# --------------------------------------------------------------------
def get_yaw_deg(drone):
    demo = getattr(drone.navdata, "demo", None)
    if not demo:
        return 0.0
    return demo.psi / 1000.0  # mdeg → deg


def turn_to_yaw(drone, target_deg):
    """
    Issue a yaw command toward target_deg. We do NOT block here; we just
    issue one command each loop - the state machine will call us repeatedly.

    Returns True when we're within YAW_TOL_DEG of target_deg.
    """
    cur_deg = get_yaw_deg(drone)
    # Wrap into [-180, 180]
    err = (target_deg - cur_deg + 180.0) % 360.0 - 180.0

    if abs(err) < YAW_TOL_DEG:
        drone.hover()
        return True

    # err > 0 => need CCW => use ccw param
    if err > 0:
        drone.move(ccw=TURN_SPEED)
    else:
        drone.move(cw=TURN_SPEED)

    return False

# --------------------------------------------------------------------
#  TAG CENTERING (image geometry only, reused for Tag0 & Tag1)
# --------------------------------------------------------------------
def center_on_tag(drone, frame, target_id, ids, corners,
                  tolerance_px=PX_TOL, speed=CENTER_SPEED, state=None):
    """
    Visually center the drone over a given ArUco tag ID using only pixel
    geometry, independent of navdata.
    """
    if ids is None:
        drone.hover()
        return False, state

    ids_list = ids.flatten().tolist()
    if target_id not in ids_list:
        drone.hover()
        return False, state

    idx = ids_list.index(target_id)
    tag_corners = corners[idx][0]  # (4,2)

    cx = tag_corners[:, 0].mean()
    cy = tag_corners[:, 1].mean()

    h, w = frame.shape[:2]
    cx0 = w / 2.0
    cy0 = h / 2.0

    dx = cx - cx0   # +dx: tag appears right in image
    dy = cy - cy0   # +dy: tag appears below center in image

    if state is None:
        state = {"good_frames": 0}

    moved = False
    good = state["good_frames"]

    # First correct left/right (x-image)
    if abs(dx) > tolerance_px*2:
        if dx > 0:
            drone.move(right=speed)
        else:
            drone.move(left=speed)
        moved = True

    # Then correct forward/back (y-image)
    elif abs(dy) > tolerance_px:
        if dy > 0:
            drone.move(forward=speed)
        else:
            drone.move(backward=speed)
        moved = True

    # Update "good frames" counter
    if moved:
        good = 0
    else:
        drone.hover()
        good += 1

    state["good_frames"] = good
    centered = (good >= 3)  # 3 consecutive “good” frames → stable center
    return centered, state

# --------------------------------------------------------------------
#  MAIN
# --------------------------------------------------------------------
def main():
    print("[INFO] Connecting to AR.Drone…")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    # ----------------------------- NAVDATA ----------------------------------
    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=5.0)

    if drone.navdata_ready.is_set():
        drone.send(at.CONFIG('general:navdata_demo', True))
        time.sleep(0.1)
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            battery_start = demo.vbat_flying_percentage
            print(f"[BATTERY] Start battery: {battery_start}%")
        else:
            print("[WARN] Demo navdata not populated yet.")
    else:
        print("[WARN] No navdata after 5s, continuing anyway.")

    # -------------------------- CAMERA SWITCH -------------------------------
    print("[INFO] Switching to BOTTOM camera...")
    drone.send(at.CONFIG('video:video_channel', 1))   # 1 = bottom
    time.sleep(0.5)

    # --------------------------- OPENCV / VIDEO ----------------------------
    stream_url = "tcp://192.168.1.1:5555"
    print(f"[INFO] Opening video stream: {stream_url}")
    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] Cannot open video stream with OpenCV.")
        return

    # --------------------------- ARUCO SETUP --------------------------------
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    # World pose (x forward, y right) at takeoff
    x = 0.0
    y = 0.0
    theta = 0.0

    # Flags & states
    center0_state = None   # for Tag0 centering
    center1_state = None   # for Tag1 centering
    origin_no_tag_frames = 0
    tag0_centered = False
    tag1_centered = False

    # Search pattern variables
    row_index = 0          # 0,1,2,...
    row_dir = +1           # +1 → from x=0 to x=BOX_LEN, -1 opposite
    search_state = "CLIMB" # CLIMB → CENTER_TAG0 → SEARCH_ROW → TURN1 → STEP_ROW → TURN2 → ...

    # For transform
    tag0_pose = (0.0, 0.0, 0.0)
    tag1_pose = None

    # --------------------------- TAKEOFF ------------------------------------
    drone.send(at.FTRIM())
    time.sleep(1)
    print("[INFO] Takeoff…")
    drone.takeoff()
    t_takeoff = time.time()
    time.sleep(1)

    last_time = time.time()

    print("[INFO] Controls: press 'l' in terminal to land at any time.")

    # --------------------- MAIN CONTROL LOOP -------------------------------
    while True:
        # -------- Emergency/manual land key --------
        if msvcrt.kbhit():
            ch = msvcrt.getwch().lower()
            if ch == "l":
                print("[INPUT] 'l' → manual landing.")
                search_state = "DONE"
                break

        # -------- Time / pose update --------
        now = time.time()
        dt = now - last_time
        last_time = now

        x, y, theta = update_pose_from_nav(drone, dt, x, y, theta)

        # Always try to drift back to origin when Tag 0 is not in view
        dist0 = math.sqrt(x * x + y * y)
        print(f"[ORIGIN] x={x:.3f}, y={y:.3f} (y>0=right), dist={dist0:.3f} m")

        # -------- Grab frame & detect markers --------
        ret, frame = cap.read()
        ids = corners = None
        ids_list = []
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(
                gray, aruco_dict, parameters=aruco_params
            )
            if ids is not None:
                ids_list = ids.flatten().tolist()
                aruco.drawDetectedMarkers(frame, corners, ids)

        # --------------------------------------------------------------------
        #  STATE MACHINE
        # --------------------------------------------------------------------
        # 1) CLIMB to target altitude (aruco detection already running)
        if search_state == "CLIMB":
            # -------- Read navdata altitude --------
            demo = getattr(drone.navdata, "demo", None)
            alt_mm = demo.altitude if demo else 0
            if alt_mm < TARGET_ALT_MM:
                drone.move(up=0.7)
            else:
                print("[INFO] Reached target altitude, switching to Tag0 search.")
                drone.hover()
                search_state = "CENTER_TAG0"
                center0_state = {"good_frames": 0}

        # 2) CENTER_TAG0: continuous Tag0 search & centering (like aruco_continuous)
        elif search_state == "CENTER_TAG0":
            if TAG0_ID in ids_list:
                origin_no_tag_frames = 0

                centered0, center0_state = center_on_tag(
                    drone,
                    frame,
                    target_id=TAG0_ID,
                    ids=ids,
                    corners=corners,
                    tolerance_px=PX_TOL,
                    speed=CENTER_SPEED,
                    state=center0_state,
                )

                if centered0:
                    print("[RESULT] Centered above Tag 0 – setting (0,0,0) and starting 10 ft search.")
                    tag0_centered = True
                    # Define Tag0 as exact origin
                    x = 0.0
                    y = 0.0
                    theta = 0.0
                    tag0_pose = (0.0, 0.0, 0.0)

                    # Initialize search pattern
                    row_index = 0
                    row_dir = +1
                    search_state = "SEARCH_ROW"
                    center1_state = {"good_frames": 0}
                    origin_no_tag_frames = 0
                    drone.hover()
            else:
                # No Tag0 visible → behave like aruco_continuous: try to drift back to origin
                dist0 = math.sqrt(x * x + y * y)
                if dist0 > POS_TOL:
                     # -------- Time / pose update --------
                    now = time.time()
                    dt = now - last_time
                    last_time = now

                    x, y, theta = update_pose_from_nav(drone, dt, x, y, theta)

                    # Always try to drift back to origin when Tag 0 is not in view
                    print(f"[ORIGIN] x={x:.3f}, y={y:.3f}")

                    move_towards_xy(drone, x, y, 0.0, 0.0, SEARCH_SPEED)
                    origin_no_tag_frames = 0
                else:
                    origin_no_tag_frames += 1
                    if origin_no_tag_frames >= ORIGIN_NO_TAG_FRAMES_THRESH:
                        print("[RESULT] At (0,0) with no Tag0 for many frames → landing.")
                        search_state = "DONE"

        # 3) SEARCH_ROW: move along current row (x direction)
        elif search_state == "SEARCH_ROW":
            # Continuous detection for Tag1 during row motion
            if TAG1_ID in ids_list and tag0_centered:
                centered1, center1_state = center_on_tag(
                    drone,
                    frame,
                    target_id=TAG1_ID,
                    ids=ids,
                    corners=corners,
                    tolerance_px=PX_TOL,
                    speed=CENTER_SPEED,
                    state=center1_state,
                )
                if centered1:
                    print("[RESULT] Centered above Tag 1 during row search – computing transform & landing.")
                    tag1_centered = True
                    tag1_pose = (x, y, theta)
                    search_state = "TRANSFORM_DONE"
                    drone.hover()
            else:
                # No Tag1 currently in view → continue row motion
                if row_dir > 0:
                    target_x = BOX_LEN_M
                else:
                    target_x = 0.0
                target_y = row_index * ROW_STEP_M

                reached = move_towards_xy(drone, x, y, target_x, target_y, SEARCH_SPEED)

                if reached:
                    drone.hover()
                    print(f"[SEARCH] End of row {row_index} at x={x:.2f}, y={y:.2f}")
                    # If next row would exceed box, we’re done searching
                    if (row_index + 1) * ROW_STEP_M > BOX_LEN_M:
                        print("[SEARCH] Finished all rows – no Tag1 found; landing.")
                        search_state = "DONE"
                    else:
                        # Prepare first 90° CW turn at row end
                        cur_yaw = get_yaw_deg(drone)
                        target_yaw = cur_yaw - 90.0
                        search_state = "TURN1"
                        turn_target_yaw = target_yaw % 360.0
                        print(f"[TURN1] Target yaw: {turn_target_yaw:.1f} deg")

        # 4) TURN1: 90° CW at row end
        elif search_state == "TURN1":
            if TAG1_ID in ids_list and tag0_centered:
                # Even during turning, if Tag1 appears, try to center
                centered1, center1_state = center_on_tag(
                    drone, frame, TAG1_ID, ids, corners,
                    tolerance_px=PX_TOL, speed=CENTER_SPEED,
                    state=center1_state,
                )
                if centered1:
                    print("[RESULT] Centered above Tag 1 during TURN1 – computing transform & landing.")
                    tag1_centered = True
                    tag1_pose = (x, y, theta)
                    search_state = "TRANSFORM_DONE"
                    drone.hover()
            else:
                # Continue yawing to target
                done_turn = turn_to_yaw(drone, turn_target_yaw)
                if done_turn:
                    drone.hover()
                    print("[TURN1] Completed first 90° turn.")
                    # Now we step one row in "forward" direction
                    search_state = "STEP_ROW"
                    step_target_y = (row_index + 1) * ROW_STEP_M
                    # We store for clarity
                    row_step_target = step_target_y

        # 5) STEP_ROW: move 1 ft into next row
        elif search_state == "STEP_ROW":
            if TAG1_ID in ids_list and tag0_centered:
                centered1, center1_state = center_on_tag(
                    drone, frame, TAG1_ID, ids, corners,
                    tolerance_px=PX_TOL, speed=CENTER_SPEED,
                    state=center1_state,
                )
                if centered1:
                    print("[RESULT] Centered above Tag 1 during row step – computing transform & landing.")
                    tag1_centered = True
                    tag1_pose = (x, y, theta)
                    search_state = "TRANSFORM_DONE"
                    drone.hover()
            else:
                # Use navdata to target (current x, row_step_target)
                reached = move_towards_xy(drone, x, y, x, row_step_target, STEP_SPEED)
                if reached:
                    drone.hover()
                    print(f"[STEP_ROW] Reached new row y={y:.2f}")
                    # Second 90° CCW turn to face next row direction
                    cur_yaw = get_yaw_deg(drone)
                    target_yaw = cur_yaw - 90.0
                    turn_target_yaw = target_yaw % 360.0
                    search_state = "TURN2"
                    print(f"[TURN2] Target yaw: {turn_target_yaw:.1f} deg")

        # 6) TURN2: second 90° CW turn to align with next row direction
        elif search_state == "TURN2":
            if TAG1_ID in ids_list and tag0_centered:
                centered1, center1_state = center_on_tag(
                    drone, frame, TAG1_ID, ids, corners,
                    tolerance_px=PX_TOL, speed=CENTER_SPEED,
                    state=center1_state,
                )
                if centered1:
                    print("[RESULT] Centered above Tag 1 during TURN2 – computing transform & landing.")
                    tag1_centered = True
                    tag1_pose = (x, y, theta)
                    search_state = "TRANSFORM_DONE"
                    drone.hover()
            else:
                done_turn = turn_to_yaw(drone, turn_target_yaw)
                if done_turn:
                    drone.hover()
                    # Ready for next row
                    row_index += 1
                    row_dir *= -1
                    print(f"[TURN2] Completed second 90° turn. Next row: {row_index}, dir={'+' if row_dir>0 else '-'}x")
                    search_state = "SEARCH_ROW"

        # 7) TRANSFORM_DONE: we have tag1_pose, compute transform & then land
        elif search_state == "TRANSFORM_DONE":
            if tag1_pose is None:
                print("[ERROR] TRANSFORM_DONE reached but tag1_pose is None.")
            else:
                x0, y0, theta0 = tag0_pose  # we defined Tag0 origin as (0,0,0)
                x1, y1, theta1 = tag1_pose

                dx = x1 - x0
                dy = y1 - y0
                dtheta = theta1 - theta0   # in radians

                # Wrap dtheta to [-pi, pi]
                dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
                dtheta_deg = math.degrees(dtheta)

                cos_t = math.cos(dtheta)
                sin_t = math.sin(dtheta)

                print("\n[RESULT] Relative transform from Tag 0 frame to Tag 1 frame:")
                print(f"Δx = {dx:.3f} m, Δy = {dy:.3f} m, Δθ = {dtheta_deg:.2f} deg")
                print("Homogeneous 2D transform T_0^1:")
                print(f"[[ {cos_t:.3f}, {-sin_t:.3f}, {dx:.3f} ],")
                print(f" [ {sin_t:.3f},  {cos_t:.3f}, {dy:.3f} ],")
                print(f" [  0.000,   0.000,  1.000 ]]")

            search_state = "DONE"

        # 8) DONE: break out of main loop and land
        elif search_state == "DONE":
            break

        # ----------------------------------------------------------------
        #  OVERLAYS & WINDOW
        # ----------------------------------------------------------------
        if ret:
            cv2.imshow("Aruco 10ft Search (Tag 1)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                print("[ESC] Landing from main loop.")
                search_state = "DONE"
                break

    # ----------------------------------------------------------------
    #  LAND + BATTERY END
    # ----------------------------------------------------------------
    print("[INFO] Landing…")
    drone.land()
    time.sleep(3.0)

    drone.navdata_ready.wait(timeout=1.0)
    demo = getattr(drone.navdata, "demo", None)
    if demo:
        battery_end = demo.vbat_flying_percentage
        print(f"[BATTERY] End battery: {battery_end}%")
    else:
        print("[BATTERY] End battery: (navdata.demo not available)")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[KEYBOARD] Ctrl+C → emergency exit, closing windows.")
        cv2.destroyAllWindows()
        sys.exit(0)
