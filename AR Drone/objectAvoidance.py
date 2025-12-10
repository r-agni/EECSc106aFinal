import time
import msvcrt  # Windows keyboard input
import cv2
import numpy as np

import pyardrone
from pyardrone import at
#from pyardrone import ARDrone


class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    #High-level helpers + navdata, but no internal video connection.
    pass


def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None


def detect_wall(frame, edge_thresh=0.08):
    """
    Idea:
    - Convert to grayscale, blur, run Canny edges.
    - Look only at the center region of the image.
    - If the percentage of edge pixels in that center region
      exceeds edge_thresh, we say there's a 'wall' in front.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    h, w = edges.shape
    roi_frac = 0.45  #45% of frame
    cx_center = w // 2
    cy_center = h // 2
    half_w = int(w * roi_frac / 2)
    half_h = int(h * roi_frac / 2)
    cx1, cx2 = cx_center - half_w, cx_center + half_w
    cy1, cy2 = cy_center - half_h, cy_center + half_h
    center_roi = edges[cy1:cy2, cx1:cx2]

    edge_density = float(np.count_nonzero(center_roi)) / center_roi.size

    #is_wall = edge_density > edge_thresh
    edge_count = np.count_nonzero(center_roi)
    #print(edge_count)
    #is_wall = edge_count < 1500
    if edge_count > 770:
        is_wall = 0 #safe
    elif 570 < edge_count <= 770:
        is_wall = 1 #caution
    else:
        is_wall = 2 #wall
    return is_wall, edge_density, (cx1, cy1, cx2, cy2), edges

def go_around(is_wall, drone, speed):
        if is_wall == 1:
            print("Going left. Cautious")
            drone.move(left = speed / 2)
        else:
            print("Going left. Wall")
            drone.move(left = speed)

def go_forward(is_wall, drone, speed):
    if is_wall == 0:
        print("Going forward: safe")
        drone.move(forward = speed)
    else:
        print("Going forward: cautious")
        drone.move(forward = speed / 2)

def main():
    print("=" * 60)
    print("  AR DRONE - Object Avoidance")
    print("=" * 60)

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
    print("[INFO] Connecting to AR.Drone (no internal video)...")
    drone = ARDroneNoVideo()
    #drone = ARDrone()
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
    completed = False
    speed = 0.05        
    last_cmd_time = time.time()
    print("TAKEOFF")
    drone.takeoff()
    is_flying = True
    time.sleep(3)
    print("[OK] Airborne!")
    
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

            is_wall, edge_density, (cx1, cy1, cx2, cy2), edges = detect_wall(frame)

            # Draw ROI
            cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (0, 255, 0), 2)

            if is_wall == 2:
                cv2.putText(frame,
                            "WALL AHEAD!",
                            (50, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.2,
                            (0, 0, 255),
                            3)
                print("WALL.")
                drone.hover()
                go_around(is_wall, drone, speed)

            elif is_wall == 1:
                cv2.putText(frame,
                            "Caution! Wall close",
                            (50, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.2,
                            (0, 0, 255),
                            3)
                print("Caution!")
                go_around(is_wall, drone, speed)
                go_forward(is_wall, drone, speed)
            else:
                print("safe")
                go_forward(is_wall, drone, speed)

            # Show edge density for debugging
            cv2.putText(frame,
                        f"edge_density={edge_density:.3f}",
                        (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2)

            # --- Display stream ---
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
