import time
import sys
import math
import msvcrt

import cv2
import cv2.aruco as aruco

import pyardrone
from pyardrone import at


# -------------------------------------------------------------------------
#  DRONE CLASS: Navdata + helpers, NO internal video thread
# -------------------------------------------------------------------------
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """High-level helpers + navdata, but no internal video connection."""
    pass


# -------------------------------------------------------------------------
#  CONSTANTS
# -------------------------------------------------------------------------
MANUAL_SPEED = 0.15        # speed for manual WASD
VERT_SPEED   = 0.3         # up/down speed
YAW_SPEED    = 0.3         # yaw speed

CENTER_SPEED = 0.03        # speed used by visual centering
CENTER_TOL_PX = 30         # how close (in pixels) to call it centered
CENTER_GOOD_FRAMES = 3     # consecutive “good” frames to declare centered

# Hysteresis params
ENTER_AUTO_FRAMES = 3      # need this many consecutive frames seeing a tag to enter auto
EXIT_AUTO_FRAMES  = 5      # need this many “lost” frames to drop back to manual

# Pose integration
VEL_SCALE = 1.0 / 1000.0   # navdata.vx, vy are mm/s → m/s (if your navdata uses that)

# ArUco dictionary + parameters
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters_create()


# -------------------------------------------------------------------------
#  POSE INTEGRATION: (x forward, y right) in world frame
# -------------------------------------------------------------------------
def update_pose_from_navdata(drone, x, y, theta, dt):
    """
    Integrate pose using navdata.demo velocities.
    x, y in meters; theta yaw in radians.

    We assume:
      - demo.vx, demo.vy are in mm/s in the DRONE frame
      - psi is yaw in milli-degrees.
    """
    demo = getattr(drone.navdata, "demo", None)
    if not demo or dt <= 0:
        return x, y, theta

    vx_body = demo.vx * VEL_SCALE  # m/s forward
    vy_body = demo.vy * VEL_SCALE  # m/s right

    # yaw (psi) is in milli-degrees
    psi_deg = demo.psi / 1000.0
    yaw = math.radians(psi_deg)

    dx_body = vx_body * dt
    dy_body = vy_body * dt

    dx_world = dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
    dy_world = dx_body * math.sin(yaw) + dy_body * math.cos(yaw)

    x += dx_world
    y += dy_world
    theta = yaw

    return x, y, theta


# -------------------------------------------------------------------------
#  VISUAL CENTERING ON A TAG (image geometry only)
# -------------------------------------------------------------------------
def center_on_tag(drone, frame, target_id, ids, corners,
                  tolerance_px=CENTER_TOL_PX,
                  speed=CENTER_SPEED,
                  state=None):
    """
    Visually center the drone over a given ArUco tag ID using only pixel
    geometry – independent of navdata.

    Returns: (centered_bool, state_dict)
    """
    if state is None:
        state = {"good_frames": 0}

    if ids is None:
        drone.hover()
        state["good_frames"] = 0
        return False, state

    ids_list = ids.flatten().tolist()
    if target_id not in ids_list:
        drone.hover()
        state["good_frames"] = 0
        return False, state

    idx = ids_list.index(target_id)
    tag_corners = corners[idx][0]  # shape (4,2)

    # Compute tag center in image
    cx = tag_corners[:, 0].mean()
    cy = tag_corners[:, 1].mean()

    h, w = frame.shape[:2]
    cx0 = w / 2.0
    cy0 = h / 2.0

    dx = cx0 - cx   # +dx means tag appears to the RIGHT in image
    dy = cy0 - cy   # +dy means tag appears BELOW center in image

    moved = False
    good = state["good_frames"]

    # Horizontal (left/right)
    if abs(dx) > tolerance_px:
        if dx > 0:
            # need to move drone right to push tag left in image
            drone.move(right=speed)
        else:
            drone.move(left=speed)
        moved = True

    # Forward/back (up-down in image)
    elif abs(dy) > tolerance_px:
        if dy > 0:
            # tag below center → move forward
            drone.move(forward=speed)
        else:
            drone.move(backward=speed)
        moved = True

    if moved:
        good = 0
        time.sleep(0.2)   # let the drone respond before next frame
    else:
        drone.hover()
        good += 1

    state["good_frames"] = good
    centered = (good >= CENTER_GOOD_FRAMES)
    return centered, state


# -------------------------------------------------------------------------
#  MAIN
# -------------------------------------------------------------------------
def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    # Ask for navdata demo
    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=5.0)
    if drone.navdata_ready.is_set():
        drone.send(at.CONFIG('general:navdata_demo', True))
        time.sleep(0.1)
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            print(f"[NAVDATA] Battery (start): {demo.vbat_flying_percentage}%")
        else:
            print("[WARN] Demo navdata not populated yet.")
    else:
        print("[WARN] No navdata_ready in 5 seconds.")

    # Flat trim on ground
    print("[INFO] Sending FTRIM...")
    drone.send(at.FTRIM())
    time.sleep(1.0)

    # Open video (bottom camera assumed via ffmpeg switching done elsewhere)
    cap = cv2.VideoCapture("tcp://192.168.1.1:5555")
    if not cap.isOpened():
        print("[ERROR] Could not open video stream.")
        drone.land()
        drone.close()
        return

    print("[INFO] Controls:")
    print("  - w/s/a/d : forward/back/left/right")
    print("  - r/f     : up/down")
    print("  - q/e     : yaw CCW/CW")
    print("  - h       : hover")
    print("  - l       : land & exit")
    print("Auto mode: if Tag 0 or Tag 1 is seen for several frames,")
    print("           the drone enters auto-centering mode for that tag.")
    print("           Losing the tag for several frames returns to manual.")

    # Takeoff
    print("[FLIGHT] Taking off...")
    drone.takeoff()
    is_flying = True
    last_cmd_time = time.time()

    # Pose
    x = 0.0
    y = 0.0
    theta = 0.0      # yaw (rad)
    last_pose_time = time.time()

    # Auto/manual mode and hysteresis counters
    mode = "manual"  # "manual", "auto_tag0", "auto_tag1"
    frames_0_seen = 0
    frames_1_seen = 0
    lost_0_frames = 0
    lost_1_frames = 0

    center_state = None

    # Store poses for transform
    pose_tag0 = None  # (x0, y0, theta0)
    pose_tag1 = None  # (x1, y1, theta1)

    print("[INFO] Starting main loop...")
    while True:
        now = time.time()
        dt = now - last_pose_time
        last_pose_time = now

        # Global land key check (always active)
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key == 'l':
                print("[INPUT] 'l' pressed → landing & exit.")
                break
            # note: DO NOT consume the key for movement here; we handle
            # manual movement in manual branch below.
        # integrate pose from navdata
        x, y, theta = update_pose_from_navdata(drone, x, y, theta, dt)

        # Grab frame
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[WARN] No video frame. Hovering.")
            drone.hover()
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        ids_list = ids.flatten().tolist() if ids is not None else []
        sees0 = (0 in ids_list)
        sees1 = (1 in ids_list)

        # ------------------ HYSTERESIS COUNTERS -------------------
        # Update “seen” counters
        if sees0:
            frames_0_seen += 1
            lost_0_frames = 0
        else:
            frames_0_seen = 0
            if mode == "auto_tag0":
                lost_0_frames += 1

        if sees1:
            frames_1_seen += 1
            lost_1_frames = 0
        else:
            frames_1_seen = 0
            if mode == "auto_tag1":
                lost_1_frames += 1

        # ------------------ MODE TRANSITIONS ----------------------
        if mode == "manual":
            # Prefer Tag 0 over Tag 1 for auto
            if frames_0_seen >= ENTER_AUTO_FRAMES:
                mode = "auto_tag0"
                center_state = {"good_frames": 0}
                print("[MODE] manual → auto_tag0 (Tag 0 locked)")
            elif frames_1_seen >= ENTER_AUTO_FRAMES:
                mode = "auto_tag1"
                center_state = {"good_frames": 0}
                print("[MODE] manual → auto_tag1 (Tag 1 locked)")

        elif mode == "auto_tag0":
            if lost_0_frames >= EXIT_AUTO_FRAMES:
                print("[MODE] auto_tag0 → manual (Tag 0 lost)")
                mode = "manual"
                center_state = None

        elif mode == "auto_tag1":
            if lost_1_frames >= EXIT_AUTO_FRAMES:
                print("[MODE] auto_tag1 → manual (Tag 1 lost)")
                mode = "manual"
                center_state = None

        # ------------------ BEHAVIOR BY MODE ----------------------
        if mode == "auto_tag0":
            # Autonomously center on Tag 0
            centered, center_state = center_on_tag(
                drone,
                frame,
                target_id=0,
                ids=ids,
                corners=corners,
                tolerance_px=CENTER_TOL_PX,
                speed=CENTER_SPEED,
                state=center_state,
            )

            if centered and pose_tag0 is None:
                # Treat current pose as Tag 0 origin
                pose_tag0 = (x, y, theta)
                print("[RESULT] Centered above Tag 0.")
                print(f"[POSE] Tag 0 pose (world): x={x:.3f} m, y={y:.3f} m, theta={math.degrees(theta):.2f} deg")

        elif mode == "auto_tag1":
            # Autonomously center on Tag 1
            centered, center_state = center_on_tag(
                drone,
                frame,
                target_id=1,
                ids=ids,
                corners=corners,
                tolerance_px=CENTER_TOL_PX,
                speed=CENTER_SPEED,
                state=center_state,
            )

            if centered and pose_tag1 is None:
                pose_tag1 = (x, y, theta)
                print("[RESULT] Centered above Tag 1.")
                print(f"[POSE] Tag 1 pose (world): x={x:.3f} m, y={y:.3f} m, theta={math.degrees(theta):.2f} deg")

                # If we also have Tag 0 pose, compute 2D transform from 0 → 1
                if pose_tag0 is not None:
                    x0, y0, th0 = pose_tag0
                    x1, y1, th1 = pose_tag1

                    dx = x1 - x0
                    dy = y1 - y0
                    dth = th1 - th0

                    T_0_1 = [
                        [math.cos(dth), -math.sin(dth), dx],
                        [math.sin(dth),  math.cos(dth), dy],
                        [0.0,            0.0,          1.0],
                    ]

                    print("\n[TRANSFORM] Pose of Tag 1 in Tag 0 frame (approx):")
                    print(f"Δx = {dx:.3f} m, Δy = {dy:.3f} m, Δθ = {math.degrees(dth):.2f} deg")
                    print("Homogeneous 2D T_0^1:")
                    for row in T_0_1:
                        print("  [ " + "  ".join(f"{v: .3f}" for v in row) + " ]")

                print("[INFO] Hovering, then landing.")
                drone.hover()
                time.sleep(1.0)
                break  # exit main loop -> land at the end

        else:
            # ------------------ MANUAL MODE ------------------------
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                last_cmd_time = time.time()

                if is_flying:
                    if key == 'w':
                        drone.move(forward=MANUAL_SPEED)
                    elif key == 's':
                        drone.move(backward=MANUAL_SPEED)
                    elif key == 'a':
                        drone.move(left=MANUAL_SPEED)
                    elif key == 'd':
                        drone.move(right=MANUAL_SPEED)
                    elif key == 'r':
                        drone.move(up=VERT_SPEED)
                    elif key == 'f':
                        drone.move(down=VERT_SPEED)
                    elif key == 'q':
                        drone.move(ccw=YAW_SPEED)
                    elif key == 'e':
                        drone.move(cw=YAW_SPEED)
                    elif key == 'h':
                        drone.hover()

            # small hover if idle
            if time.time() - last_cmd_time > 0.5 and is_flying:
                drone.hover()

        # Show video (for debugging)
        cv2.imshow("AR.Drone ArUco (hysteresis)", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            print("[INPUT] ESC → landing & exit.")
            break

    # ------------------------- SHUTDOWN --------------------------
    print("[INFO] Landing...")
    drone.land()
    time.sleep(2.0)

    # Print final battery
    demo = getattr(drone.navdata, "demo", None)
    if demo:
        print(f"[NAVDATA] Battery (end): {demo.vbat_flying_percentage}%")

    cap.release()
    cv2.destroyAllWindows()
    drone.close()
    print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C → emergency exit & land.")
        cv2.destroyAllWindows()
        sys.exit(0)
