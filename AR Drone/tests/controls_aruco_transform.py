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


def main():
    print("[INFO] Connecting to AR.Drone (no video)...")
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
        print("       Make sure ffplay is CLOSED and no other script is")
        print("       connected to tcp://192.168.1.1:5555.")
        return

    # --------------------------- ARUCO SETUP --------------------------------
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    print("[INFO] Takeoff + climb to ~3ft (~1m)...")
    drone.takeoff()
    time.sleep(2.0)

    # Climb upward slowly until altitude ~1 m
    target_alt = 1000   # in millimeters
    climb_duration = 6  # fallback timeout in seconds

    start_climb = time.time()
    while True:
        drone.move(up=0.4)   # climb gently
        time.sleep(0.05)

        demo = getattr(drone.navdata, "demo", None)
        if demo:
            alt = demo.altitude
            print(f"[ALT] {alt} mm", end="\r")
            if alt >= target_alt:
                break

        if time.time() - start_climb > climb_duration:
            print("\n[WARN] Altitude estimate not ready; stopping climb.")
            break

    # Stop vertical movement → hover hold
    drone.hover()
    print("\n[INFO] Hovering at ~3 ft. Manual controls for up to 120s.")
    print("       Keyboard controls (terminal window in focus):")
    print("         w/s/a/d -> forward/back/left/right")
    print("         r/f     -> up/down")
    print("         q/e     -> rotate left/right")
    print("         h       -> hover")
    print("         0       -> record pose at ArUco ID 0 (home)")
    print("         1       -> record pose at ArUco ID 1 (destination)")
    print("         l       -> land & exit")
    print("       ESC in video window also lands & exits.")

    hover_start = time.time()
    last_cmd_time = time.time()
    is_flying = True

    # --------------------- POSE ESTIMATION STATE ---------------------------
    # Pose in "world" frame (we'll treat this as Tag 0 frame after calibration)
    x = 0.0  # meters
    y = 0.0  # meters
    theta = 0.0  # radians (heading)

    last_pose_time = time.time()

    # Store detected IDs each frame so keyboard callbacks can check them
    last_detected_ids = []

    # Recorded poses at tag 0 and tag 1
    pose_tag0 = None  # (x0, y0, theta0)
    pose_tag1 = None  # (x1, y1, theta1)

    # ----------------------------- MAIN LOOP -------------------------------
    while True:
        now = time.time()
        dt = now - last_pose_time
        last_pose_time = now

        # --- Update pose from navdata velocities ---
        demo = getattr(drone.navdata, "demo", None)
        if demo and dt > 0:
            # vx, vy in mm/s in drone BODY frame (x forward, y left)
            vx_body = demo.vx / 1000.0  # m/s
            vy_body = demo.vy / 1000.0  # m/s

            # psi is yaw in millidegrees
            yaw_rad = (demo.psi / 1000.0) * (math.pi / 180.0)

            # Treat psi as our heading; you could also integrate yaw rate, but
            # using fused yaw is simpler:
            theta = yaw_rad

            # Rotate body-frame velocity into WORLD frame using theta
            # World x forward, y left (same as drone at theta=0)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            vx_world = vx_body * cos_t - vy_body * sin_t
            vy_world = vx_body * sin_t + vy_body * cos_t

            x += vx_world * dt
            y += vy_world * dt

        # --- Keyboard controls (terminal must have focus) ---
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            now = time.time()

            # Land & exit
            if key == 'l':
                print("[INPUT] 'l' pressed → landing now.")
                break

            # Record pose for Tag 0 (home)
            if key == '0':
                if 0 in last_detected_ids:
                    pose_tag0 = (x, y, theta)
                    print(f"[POSE] Recorded Tag 0 pose: x={x:.3f} m, y={y:.3f} m, "
                          f"theta={theta:.3f} rad")
                else:
                    print("[POSE] '0' pressed but ArUco ID 0 is not currently detected.")

            # Record pose for Tag 1 (destination)
            if key == '1':
                if 1 in last_detected_ids:
                    pose_tag1 = (x, y, theta)
                    print(f"[POSE] Recorded Tag 1 pose: x={x:.3f} m, y={y:.3f} m, "
                          f"theta={theta:.3f} rad")
                else:
                    print("[POSE] '1' pressed but ArUco ID 1 is not currently detected.")

            if is_flying:
                if key == 'w':
                    print("[CMD] forward")
                    drone.move(forward=0.15)
                    last_cmd_time = now
                elif key == 's':
                    print("[CMD] backward")
                    drone.move(backward=0.15)
                    last_cmd_time = now
                elif key == 'a':
                    print("[CMD] left")
                    drone.move(left=0.15)
                    last_cmd_time = now
                elif key == 'd':
                    print("[CMD] right")
                    drone.move(right=0.15)
                    last_cmd_time = now
                elif key == 'r':
                    print("[CMD] up")
                    drone.move(up=0.3)
                    last_cmd_time = now
                elif key == 'f':
                    print("[CMD] down")
                    drone.move(down=0.3)
                    last_cmd_time = now
                elif key == 'q':
                    print("[CMD] rotate left")
                    drone.move(ccw=0.3)
                    last_cmd_time = now
                elif key == 'e':
                    print("[CMD] rotate right")
                    drone.move(cw=0.3)
                    last_cmd_time = now
                elif key == 'h':
                    print("[CMD] hover")
                    drone.hover()
                    last_cmd_time = now

        # If no movement command for a short time, send hover to keep it stable
        if time.time() - last_cmd_time > 0.5 and is_flying:
            drone.hover()
            last_cmd_time = time.time()

        # --- Auto-stop after 2 min ---
        if time.time() - hover_start >= 120:
            print("[TIMER] 2 minutes elapsed → landing.")
            break

        # --- Read frame from ffmpeg/OpenCV ---
        ret, frame = cap.read()
        if not ret:
            print("[WARN] No video frame (retrying)...")
            time.sleep(0.02)
            continue

        # ------------------------- ARUCO DETECTION -------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params
        )

        last_detected_ids = []
        if ids is not None:
            ids_list = ids.flatten().tolist()
            last_detected_ids = ids_list
            # Draw the green squares around detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"[ARUCO] Detected IDs: {ids_list}")

        # --------------------------- DISPLAY -------------------------------
        # Optionally overlay current pose estimate on the frame
        pose_text = f"x={x:.2f}m, y={y:.2f}m, θ={theta:.2f}rad"
        cv2.putText(frame, pose_text, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("AR.Drone Bottom Camera (Aruco + Pose)", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC in the video window
            print("[INPUT] ESC pressed → landing.")
            break

        time.sleep(0.01)

    # ------------------------------ LAND ----------------------------------
    print("[INFO] Landing...")
    drone.land()
    time.sleep(3.0)

    # Final battery check
    if drone.navdata_ready.is_set():
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            print(f"[NAVDATA] End Battery: {demo.vbat_flying_percentage}%")

    cap.release()
    cv2.destroyAllWindows()

    # ------------------------- TRANSFORM OUTPUT ----------------------------
    if pose_tag0 is not None and pose_tag1 is not None:
        x0, y0, th0 = pose_tag0
        x1, y1, th1 = pose_tag1

        dx = x1 - x0
        dy = y1 - y0
        dtheta = th1 - th0

        cos_d = math.cos(dtheta)
        sin_d = math.sin(dtheta)

        print("\n[RESULT] Relative transform from Tag 0 frame to Tag 1 frame:")
        print(f"Δx = {dx:.3f} m, Δy = {dy:.3f} m, Δθ = {dtheta:.3f} rad")
        print("Homogeneous 2D transform T_0^1:")
        print(f"[[ {cos_d:.3f}, {-sin_d:.3f}, {dx:.3f} ],")
        print(f" [ {sin_d:.3f},  {cos_d:.3f}, {dy:.3f} ],")
        print( " [  0.000,   0.000,  1.000 ]]")

        print("\nThis gives you a straight-line vector and yaw offset that")
        print("you can send as a single path to the Tello from its own home pose.")
    else:
        print("\n[RESULT] Did not record both Tag 0 and Tag 1 poses.")
        print("         Make sure you press '0' over ID 0 and '1' over ID 1 next time.")

    print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C → emergency exit & land.")
        cv2.destroyAllWindows()
        sys.exit(0)
