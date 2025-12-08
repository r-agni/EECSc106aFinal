import time
import sys
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
    print("\n[INFO] Hovering at ~3 ft. You now have manual controls for up to 60s.")
    print("       Keyboard controls (terminal window in focus):")
    print("         w/s/a/d -> forward/back/left/right")
    print("         r/f     -> up/down")
    print("         q/e     -> rotate left/right")
    print("         h       -> hover")
    print("         l       -> land & exit")
    print("       ESC in video window also lands & exits.")

    hover_start = time.time()
    last_cmd_time = time.time()
    is_flying = True

    # ----------------------------- MAIN LOOP -------------------------------
    while True:
        # --- Keyboard controls (terminal must have focus) ---
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            now = time.time()

            # Land & exit
            if key == 'l':
                print("[INPUT] 'l' pressed → landing now.")
                break

            if is_flying:
                if key == 'w':
                    print("[CMD] forward")
                    drone.move(forward=0.25)
                    last_cmd_time = now
                elif key == 's':
                    print("[CMD] backward")
                    drone.move(backward=0.25)
                    last_cmd_time = now
                elif key == 'a':
                    print("[CMD] left")
                    drone.move(left=0.25)
                    last_cmd_time = now
                elif key == 'd':
                    print("[CMD] right")
                    drone.move(right=0.25)
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

        # --- Auto-stop after 1 min ---
        if time.time() - hover_start >= 60:
            print("[TIMER] 1 minute elapsed → landing.")
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

        if ids is not None:
            # This draws the green squares around detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"[ARUCO] Detected IDs: {ids.flatten().tolist()}")

        # --------------------------- DISPLAY -------------------------------
        cv2.imshow("AR.Drone Bottom Camera (Aruco Detection)", frame)
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
    print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C → emergency exit & land.")
        cv2.destroyAllWindows()
        sys.exit(0)
