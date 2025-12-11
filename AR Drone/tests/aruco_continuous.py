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
#  CONSTANTS (meters)
# --------------------------------------------------------------------
TARGET_ALT_M = 0.8                     # ≈3 ft altitude
TARGET_ALT_MM = int(TARGET_ALT_M * 1000)

SEARCH_SPEED = 0.035                    # slow search/center speed
POS_TOL = 0.1                         # ~10cm tolerance for origin
PX_TOL = 100

TAG0_ID = 0
ORIGIN_NO_TAG_FRAMES_THRESH = 60*6       # frames at (0,0) with no tag before landing


# --------------------------------------------------------------------
#  POSE INTEGRATION (x forward, y right)
# --------------------------------------------------------------------
def update_pose_from_nav(drone, dt, x, y, theta):
    """
    Integrate navdata velocities to maintain pose (x,y,theta) in meters.

    World frame anchored at takeoff:
      +x_world = forward from tag 0
      +y_world = right   from tag 0

    Drone Demo Data:
      - demo.vx > 0 when moving FORWARD
      - demo.vy > 0 when moving RIGHT
    """
    demo = getattr(drone.navdata, "demo", None)
    if not demo or dt <= 0:
        return x, y, theta

    # Body-frame velocities in m/s
    vx_body = demo.vx / 1000.0  # +forward
    vy_body = demo.vy / 1000.0  # +right

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
#  MOVE TOWARD (0,0) USING NAVDATA (x forward, y right)
# --------------------------------------------------------------------
def move_towards_xy(drone, x_cur, y_cur, x_target, y_target, speed):
    """
    Move drone toward (x_target, y_target) in world frame (x forward, y right).
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
#  TAG CENTERING (image geometry only)
# --------------------------------------------------------------------
def center_on_tag(drone, frame, target_id, ids, corners,
                  tolerance_px=100, speed=SEARCH_SPEED, state=None):
    """
    Visually center the drone over a given ArUco tag ID using only pixel
    geometry – independent of navdata.

    Returns (centered_bool, state_dict).
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

    # Horizontal (left/right)
    if abs(dx) > tolerance_px*2:  #double tolerance for left and right
        if dx > 0:
            drone.move(right=speed)
            time.sleep(0.3)
        else:
            drone.move(left=speed)
            time.sleep(0.3)
        moved = True

    # Forward/back (up/down in image)
    elif abs(dy) > tolerance_px:
        if dy > 0:
            drone.move(forward=speed)
            time.sleep(0.3)
        else:
            drone.move(backward=speed)
            time.sleep(0.3)
        moved = True

    if moved:
        good = 0
    else:
        drone.hover()
        good += 1

    state["good_frames"] = good
    centered = (good >= 5)  # 5 consecutive “good” frames
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
            print(f"[NAVDATA] Battery: {demo.vbat_flying_percentage}%")
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
    center_state = None

    # --------------------------- TAKEOFF ------------------------------------
    drone.send(at.FTRIM())
    time.sleep(1)
    print("[INFO] Takeoff…")
    drone.takeoff()
    time.sleep(2.0)  # let it finish takeoff sequence
    
    # ----------------------------------------------------------------
    #  CLIMB TO TARGET ALTITUDE
    # ----------------------------------------------------------------
    print("[INFO] Climbing to target altitude…")

    while True:
        # Manual land (terminal 'l')
        if msvcrt.kbhit() and msvcrt.getwch().lower() == "l":
            print("[INPUT] 'l' → landing during climb.")
            drone.land()
            cap.release()
            cv2.destroyAllWindows()
            return

        demo = getattr(drone.navdata, "demo", None)
        alt_mm = demo.altitude if demo else 0
        print(f"[ALT] {alt_mm} mm   ", end="\r")

        if alt_mm >= TARGET_ALT_MM:
            print("\n[INFO] Reached target altitude.")
            drone.hover()
            break

        drone.move(up=0.7)
    
    # ----------------------------------------------------------------
    #  SEARCH / CENTER LOOP
    # ----------------------------------------------------------------
    print("[INFO] Starting continuous Tag 0 search & centering…")

    origin_no_tag_frames = 0

    while True:
        last_time = time.time()
        # Manual land (terminal 'l')
        if msvcrt.kbhit() and msvcrt.getwch().lower() == "l":
            print("[INPUT] 'l' → manual landing during search.")
            break

        # Video frame + detection
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
        
        # ----- CASE 1: Tag 0 visible → center & land -----
        if TAG0_ID in ids_list:
            origin_no_tag_frames = 0  # just in case

            centered, center_state = center_on_tag(
                drone,
                frame,
                target_id=TAG0_ID,
                ids=ids,
                corners=corners,
                tolerance_px=PX_TOL,
                speed=SEARCH_SPEED,
                state=center_state,
            )

            if ret:
                cv2.putText(frame, "CENTERING on Tag 0", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if centered:
                print("[RESULT] Centered above Tag 0. Hover, then land.")
                drone.hover()
                break

        # Pose integration
        now = time.time()
        dt = now - last_time
        last_time = now
        x, y, theta = update_pose_from_nav(drone, dt, x, y, theta)

        # ----- CASE 2: Tag 0 not visible -----
        drone.hover()

        # Always try to drift back to origin when Tag 0 is not in view
        dist0 = math.sqrt(x * x + y * y)
        print(f"[ORIGIN] x={x:.3f}, y={y:.3f} (y>0=right), dist={dist0:.3f} m")

        if dist0 > POS_TOL:
            # Go back to (0,0)
            move_towards_xy(drone, x, y, 0.0, 0.0, SEARCH_SPEED)
            origin_no_tag_frames = 0  # only count “no tag” when at origin
            if ret:
                cv2.putText(frame, "Returning to origin (0,0)…", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        else:
            # At (0,0) and still no Tag 0 → increment counter
            origin_no_tag_frames += 1
            if ret:
                cv2.putText(frame, f"At origin, no Tag 0 ({origin_no_tag_frames})",
                            (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            if origin_no_tag_frames >= ORIGIN_NO_TAG_FRAMES_THRESH:
                print("[RESULT] At (0,0) with no Tag 0 for many frames → landing.")
                break

        # Overlay pose info & SHOW WINDOW
        if ret:
            cv2.putText(frame, f"x={x:.2f} m, y={y:.2f} m (y>0=right)",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("Aruco Continuous Search", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                print("[ESC] Landing from search loop.")
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
