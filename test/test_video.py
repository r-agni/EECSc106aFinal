import time
import cv2
from pyardrone import ARDrone

def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDrone()
    print("[INFO] Connected.")

    print("[DEBUG] Has frame attribute:", hasattr(drone, "frame"))
    print("[DEBUG] Has video_ready attribute:", hasattr(drone, "video_ready"))

    if not hasattr(drone, "video_ready"):
        print("[ERROR] pyardrone did NOT enable video support (cv2 not detected?).")
        return

    print("[INFO] Waiting for video stream...")
    start_time = time.time()

    # wait up to 10 seconds for video to be ready
    while not drone.video_ready.is_set():
        if time.time() - start_time > 10:
            print("[ERROR] Video stream never became ready (timeout).")
            return
        time.sleep(0.1)

    print("[INFO] Video stream is ready — showing frames. Press ESC to quit.")

    while True:
        frame = drone.frame

        if frame is not None:
            cv2.imshow("Drone Video Test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
        else:
            print("[WARN] Frame is None — waiting...")
            time.sleep(0.05)

    print("[INFO] Closing.")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
