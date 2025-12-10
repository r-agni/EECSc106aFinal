import time
import msvcrt  # Windows keyboard input
import cv2
import numpy as np
import pyardrone
from pyardrone import at
import cv2.aruco as aruco
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    #High-level helpers + navdata, but no internal video connection.
    pass

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
param_markers =  cv2.aruco.DetectorParameters()
detection = cv2.aruco.ArucoDetector(marker_dict, param_markers)

def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None

horiz_duration = 3.0   # seconds for ~2m sideways
forward_duration = 1.5 # seconds for ~1m forward
def current_segment_duration(seg_name):
    if seg_name in ("RIGHT", "LEFT"):
        return horiz_duration
    else:
        return forward_duration

def main():
    print("=" * 60)
    print("  AR DRONE - pathing in a box")
    print("=" * 60)
    video_channel = 1   # bottom cam
    cam_name = "BOTTOM"
        # ---------------- CONNECT TO DRONE ----------------
    print("[INFO] Connecting to AR.Drone (no internal video)...")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    print("[INFO] Waiting for navdata...")
    drone.navdata_ready.wait(timeout=10.0)

    if drone.navdata_ready.is_set():
        # Enable navdata demo so battery etc. are easy to read
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

    if cap is not None and not cap.isOpened():
        print("[!] Could not open video stream. Check stream URL / connection.")
        drone.close()
        return

    print("[*] Press 'q' in the console or OpenCV window to quit.")
    print("[*] Showing camera feed and wall detection overlay...")

    is_flying = False
    speed = 0.05        
    segments = ["RIGHT", "FORWARD", "LEFT", "FORWARD"]
    segment_index = 0
    #segment_start_time = time.time()

    drone.send(at.FTRIM())
    time.sleep(1)
    print("TAKEOFF")
    drone.takeoff()
    is_flying = True
    time.sleep(3)
    print("[OK] Airborne!")

    segment_start_time = time.time()
    try:
        while True:
            if cap is not None:
                ret, frame = cap.read()
                if not ret:
                    print("[!] Failed to read frame from stream.")
                    time.sleep(0.1)
                    continue

                # Resize for smoother display
                frame = cv2.resize(frame, (640, 360))
            else:
                print("No video Capturing object.")
                time.sleep(0.5)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detection.detectMarkers(gray)
            tag1_seen = False
            if ids is not None:
                ids_flat = ids.flatten()
                if 1 in ids_flat:
                    tag1_seen = True
                    aruco.drawDetectedMarkers(frame, corners, ids)

            if tag1_seen:
                print("[INFO] ArUco tag ID 1 detected! Landing...")
                drone.hover()
                time.sleep(0.5)
                drone.land()
                is_flying = False
                time.sleep(3)
                break  # exit main loop

            # --- Zig-zag path logic ---
            now = time.time()
            current_seg = segments[segment_index]
            seg_dur = current_segment_duration(current_seg)

            # If current segment time elapsed, switch to the next segment
            if now - segment_start_time > seg_dur:
                segment_index = (segment_index + 1) % len(segments)
                segment_start_time = now
                current_seg = segments[segment_index]
                print(f"[PATH] Switching to segment: {current_seg}")

            # Send movement command for current segment
            if current_seg == "RIGHT":
                drone.move(right=speed)
            elif current_seg == "LEFT":
                drone.move(left=speed)
            elif current_seg == "FORWARD":
                drone.move(forward=speed)

            window_title = f"AR.Drone {cam_name} Camera"
            cv2.imshow(window_title, frame)

            key = get_key()
            key_cv = cv2.waitKey(1) & 0xFF
            if key == 'q' or key_cv == ord('q'):
                print("[*] Quit requested")
                break

    except KeyboardInterrupt:
            print("\n[!] Keyboard interrupt")

    finally:
        try:
            drone.hover()
            time.sleep(0.3)
        except:
            pass
        if is_flying:
            print("\n[*] Landing before exit...")
            drone.land()
            time.sleep(3)

        print("[*] Closing connection...")
        drone.close()
        cv2.destroyAllWindows()
        print("[*] Goodbye!")

if __name__ == "__main__":
    main()