import time
import msvcrt  # Windows keyboard input
import cv2
import numpy as np
import pyardrone
from pyardrone import at
import cv2.aruco as aruco

import matplotlib.pyplot as plt
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    #High-level helpers + navdata, but no internal video connection.
    pass

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
param_markers =  cv2.aruco.DetectorParameters()
detection = cv2.aruco.ArucoDetector(marker_dict, param_markers)

speed = 0.05
ROW_SPACING_M = 0.02

SCALE_X = 0.15
SCALE_Y = 0.73

def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None

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
    #-------------- initializations + Takeoff -------------
    is_flying = False        
    segments = ["RIGHT", "FORWARD", "LEFT", "FORWARD"]
    segment_index = 0

    drone.send(at.FTRIM())
    time.sleep(1)
    print("TAKEOFF")
    drone.takeoff()
    is_flying = True
    time.sleep(3.5)
    print("[OK] Airborne!")

    T_RIGHT   = 5.0   # seconds per right leg
    T_LEFT    = 5.5   # seconds per left leg
    T_FORWARD = 1.8   # seconds per forward leg

    est_x = 0.0
    est_y = 0.0
    path_x = [est_x]
    path_y = [est_y]
    prev_v_right = 0.0
    prev_v_forward = 0.0
    alpha = 0.3
    DRIFT_COMP = 0.01
    last_pose_time = time.time()
    segment_start_time = time.time()
    start_found = False
    try:
        #Waits to find start Tag0
        while not start_found:
            ret, frame = cap.read()
            if not ret:
                print("[!] Failed to read frame from stream.")
                time.sleep(0.1)
                continue

            frame = cv2.resize(frame, (640, 360))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detection.detectMarkers(gray)

            if ids is not None:
                ids_flat = ids.flatten()
                if 0 in ids_flat:
                    # Draw and confirm
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    print("[INFO] Start tag ID 0 detected. Zeroing origin at current pose (0,0).")
                    # Zero the odom here
                    est_x_r = 0.0
                    est_y_r = 0.0
                    path_x = [est_x]
                    path_y = [est_y]
                    last_pose_time = time.time()
                    #segment_start_time = time.time()
                    start_found = True
            # Just hover while searching for start tag
            drone.hover()
            window_title = f"AR.Drone {cam_name} Camera"
            cv2.imshow(window_title, frame)

            key = get_key()
            key_cv = cv2.waitKey(1) & 0xFF
            if key == 'q' or key_cv == ord('q'):
                print("[*] Quit requested during start-tag search.")
                raise KeyboardInterrupt
        #Searches for start tag1
        segment_start_time = time.time()

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

            # --- Zig-zag path logic + time integration ---
            now = time.time()
            dt = now - last_pose_time
            if dt > 0.25:
                dt = 0.0
            last_pose_time = now
            current_seg = segments[segment_index]

            if tag1_seen:
                print("[INFO] ArUco tag ID 1 detected! Landing...")
                drone.hover()
                time.sleep(0.5)
                drone.land()
                is_flying = False
                time.sleep(3)
                break  # exit main loop

            # Send movement command for current segment + estimate x and y
            if current_seg == "RIGHT":
                drone.move(right=speed)
                # PATH TRACKING: +x direction
            elif current_seg == "LEFT":
                drone.move(left=speed)
                # PATH TRACKING: -x direction
            elif current_seg == "FORWARD":
                drone.move(forward=speed, left = DRIFT_COMP)
                # PATH TRACKING: +y direction

            demo = getattr(drone.navdata, "demo", None)
            if demo is not None:
                raw_vx = float(demo.vx)  # forward/back
                raw_vy = float(demo.vy)  # left/right

                # Auto-detect units: if values are large, assume mm/s, else m/s
                if abs(raw_vx) > 20.0 or abs(raw_vy) > 20.0:
                    vel_scale = 1.0 / 1000.0   # mm/s -> m/s
                else:
                    vel_scale = 1.0            # already m/s

                vx = raw_vx * vel_scale
                vy = raw_vy * vel_scale
                # World frame: +x = right, +y = forward
                v_forward = -vx          # forward
                v_right = vy          # right (assuming +vy is right in your setup)

                # Smooth + removing small velocities
                if abs(v_right) < 0.025:
                    v_right = 0.0
                if abs(v_forward) < 0.025:
                    v_forward = 0.0
                v_right   = alpha * v_right   + (1 - alpha) * prev_v_right
                v_forward = alpha * v_forward + (1 - alpha) * prev_v_forward
                prev_v_right   = v_right
                prev_v_forward = v_forward
                # Integrate + scale
                est_x_r += v_right * dt
                est_y_r += v_forward * dt
                est_x = SCALE_X * est_x_r
                est_y = SCALE_Y * est_y_r

            path_x.append(est_x)
            path_y.append(est_y)

            #------- NEW switching states based off distance ----------
            now = time.time()
            elapsed = now - segment_start_time
            if current_seg == "RIGHT" and elapsed >= T_RIGHT:
                segment_index = (segment_index + 1) % len(segments)
                current_seg = segments[segment_index]
                segment_start_time = now
                print(f"[PATH] Switching to segment: {current_seg} (time-based RIGHT)")
                drone.hover()
                time.sleep(0.7)
                last_pose_time = time.time()
                continue

            elif current_seg == "LEFT" and elapsed >= T_LEFT:
                segment_index = (segment_index + 1) % len(segments)
                current_seg = segments[segment_index]
                segment_start_time = now
                print(f"[PATH] Switching to segment: {current_seg} (time-based LEFT)")
                drone.hover()
                time.sleep(0.7)
                last_pose_time = time.time()
                continue

            elif current_seg == "FORWARD" and elapsed >= T_FORWARD:
                segment_index = (segment_index + 1) % len(segments)
                current_seg = segments[segment_index]
                segment_start_time = now
                print(f"[PATH] Switching to segment: {current_seg} (time-based FORWARD)")
                drone.hover()
                time.sleep(0.7)
                last_pose_time = time.time()
                continue

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

        #-------------------------
        #plotting
        #------------------------
        print(f"[PATH] NavData Drone Path:")
        print(f"       x = {est_x:.3f} m (right +, left -)")
        print(f"       y = {est_y:.3f} m (forward +, back -)")

        plt.figure()
        plt.plot(path_x, path_y, marker='o')
        plt.scatter([0], [0], s=80)          # start
        plt.scatter([est_x], [est_y], s=80)  # end
        plt.text(0, 0, " Start (0,0)", va='bottom', ha='left')
        plt.text(est_x, est_y, " End", va='bottom', ha='left')
        plt.xlabel("x (m)  [right = +]")
        plt.ylabel("y (m)  [forward = +]")
        plt.title("Estimated Drone Path (Navdata-based Dead Reckoning)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    main()