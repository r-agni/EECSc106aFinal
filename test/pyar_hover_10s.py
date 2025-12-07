import time
import sys
import msvcrt          # Windows-only keyboard input
import cv2

from pyardrone import ARDrone, at  # 'at' not used yet, but useful later


def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDrone()   # uses default host/ports 192.168.1.1, 5554/5555/5556 :contentReference[oaicite:0]{index=0}
    print("[INFO] Connected.")

    # --- Wait for navdata so we can read battery etc. ---
    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=5.0)   # threading.Event from pyardrone 
    if drone.navdata_ready.is_set():
        demo = getattr(drone.navdata, "demo", None)   # NavData.demo option :contentReference[oaicite:2]{index=2}
        if demo is not None:
            print(f"[NAVDATA] Start battery: {demo.vbat_flying_percentage}%")
        else:
            print("[WARN] Navdata ready, but demo data not enabled.")
    else:
        print("[WARN] Navdata not ready after 5s; continuing anyway.")

    # --- Wait briefly for video to become ready (non-blocking safety) ---
    print("[INFO] Waiting for video stream...")
    start_wait = time.time()
    while not getattr(drone, "video_ready", None) or not drone.video_ready.is_set():
        if time.time() - start_wait > 5.0:
            print("[WARN] Video not ready after 5s; will continue without it.")
            break
        time.sleep(0.05)

    print("[INFO] Controls:")
    print("  - Drone hovers for up to 10 seconds.")
    print("  - Press 'l' in the terminal to land immediately.")
    print("  - Press ESC in the video window to emergency land & exit.")

    # --- Takeoff ---
    print("[FLIGHT] Taking off...")
    drone.takeoff()  :contentReference[oaicite:3]{index=3}
    takeoff_time = time.time()

    try:
        while True:
            # Keep hover command refreshed so the drone doesn't drift too much
            drone.hover()  :contentReference[oaicite:4]{index=4}

            # --- Keyboard handling in terminal (non-blocking) ---
            if msvcrt.kbhit():
                ch = msvcrt.getwch().lower()
                if ch == 'l':
                    print("[INPUT] 'l' pressed – landing now.")
                    break

            # --- Auto-land after 10 seconds of flight ---
            if time.time() - takeoff_time >= 10.0:
                print("[TIMER] 10 seconds elapsed – auto-landing.")
                break

            # --- Video display (if available) ---
            frame = getattr(drone, "frame", None)  # VideoMixin.frame, already BGR for OpenCV :contentReference[oaicite:5]{index=5}
            if frame is not None:
                cv2.imshow("AR.Drone Video", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    print("[INPUT] ESC in video window – emergency land & exit.")
                    break

            time.sleep(0.01)

    finally:
        # --- Safe landing & cleanup ---
        print("[FLIGHT] Landing...")
        drone.land()
        time.sleep(3.0)

        # Read end battery if navdata is still coming in
        if drone.navdata_ready.is_set():
            demo = getattr(drone.navdata, "demo", None)
            if demo is not None:
                print(f"[NAVDATA] End battery: {demo.vbat_flying_percentage}%")

        cv2.destroyAllWindows()
        print("[INFO] Done.")
        # When the script exits, pyardrone’s background threads will stop


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C detected, exiting.")
        cv2.destroyAllWindows()
        sys.exit(0)
