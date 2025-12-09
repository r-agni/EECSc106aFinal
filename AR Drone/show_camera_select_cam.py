import time
import cv2
import cv2.aruco as aruco

import pyardrone
from pyardrone import at


# --- CONTROL-ONLY DRONE: NO VIDEO CLIENT ---
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """High-level helpers + navdata, but no internal video connection."""
    pass


def main():
    # ---------------- CAMERA SELECTION ----------------
    choice = input("Select camera [f = front, b = bottom] (default: f): ").strip().lower()
    if choice == "b":
        video_channel = 1   # bottom cam
        cam_name = "BOTTOM"
    else:
        video_channel = 0   # front cam
        cam_name = "FRONT"

    print(f"[INFO] Using {cam_name} camera (video:video_channel = {video_channel})")

    # ---------------- CONNECT TO DRONE ----------------
    print("[INFO] Connecting to AR.Drone (no video)...")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=10.0)

    if drone.navdata_ready.is_set():
        drone.send(at.CONFIG("general:navdata_demo", True))
        time.sleep(0.1)
        demo = getattr(drone.navdata, "demo", None)
        if demo:
            print(f"[NAVDATA] Battery: {demo.vbat_flying_percentage}%")
        else:
            print("[WARN] Demo navdata not populated yet.")
    else:
        print("[WARN] No navdata after 10s, continuing anyway.")

    # ---------------- SET CAMERA CHANNEL ----------------
    print(f"[INFO] Switching to {cam_name} camera...")
    drone.send(at.CONFIG("video:video_channel", video_channel))
    time.sleep(0.5)

    # ---------------- OPEN VIDEO STREAM ----------------
    stream_url = "tcp://192.168.1.1:5555"
    print(f"[INFO] Opening video stream: {stream_url}")
    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] OpenCV could not open the video stream.")
        print("       Make sure ffplay and other scripts are CLOSED")
        print("       and that you are connected to the drone's Wi-Fi.")
        return

    # ---------------- ARUCO SETUP (OPTIONAL) ----------------
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    print("[INFO] Video stream opened. Press ESC in the window to quit.")

    # ---------------- MAIN LOOP ----------------
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to read frame, retrying...")
            time.sleep(0.05)
            continue

        # ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            # uncomment if you want to spam IDs to console:
            # print("[ARUCO] Detected IDs:", ids.flatten().tolist())

        window_title = f"AR.Drone {cam_name} Camera"
        cv2.imshow(window_title, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("[INFO] ESC pressed, exiting.")
            break

    # ---------------- CLEANUP ----------------
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Done (drone stays in its current state — no takeoff/land here).")


if __name__ == "__main__":
    main()
