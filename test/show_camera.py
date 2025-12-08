import cv2
import cv2.aruco as aruco
import time
from pyardrone import ARDrone, at

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDrone()
    print("[INFO] Connected.")

    # Wait a bit so navdata/state is sane
    drone.navdata_ready.wait(timeout=3.0)

    # According to AR.Drone SDK, video:video_channel:
    # 0 = front camera
    # 1 = bottom camera
    # 2/3 = mixed / picture-in-picture modes (varies by firmware)
    print("[INFO] Switching to BOTTOM camera (video:video_channel = 1)")
    drone.send(at.CONFIG('video:video_channel', 1))

    # give the drone time to apply the config before we exit
    time.sleep(1.0)
    
    stream_url = "tcp://192.168.1.1:5555"
    print(f"[INFO] Opening video stream: {stream_url}")

    # Try forcing the FFmpeg backend explicitly
    cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] OpenCV could not open the video stream.")
        return

    print("[INFO] Video stream opened. Press ESC in the window to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to read frame, retrying...")
            time.sleep(0.05)
            continue
        
        # inside the loop, after you read `frame`:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
        
        cv2.imshow("AR.Drone Camera (whichever is currently selected)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("[INFO] ESC pressed, exiting.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
