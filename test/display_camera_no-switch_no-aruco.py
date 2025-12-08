import cv2
import time
import cv2.aruco as aruco

def main():
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
