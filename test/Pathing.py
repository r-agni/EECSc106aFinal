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
SCALE_X = 1.0
SCALE_Y = 1.0

def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None

def segment_reached(current_seg, est_x, est_y, leg_start_x, leg_start_y, horiz_target, forward_target):
    """
    Return True if the current zig-zag segment has reached its target
    distance, based on navdata-estimated position.
    """
    dx = est_x - leg_start_x
    dy = est_y - leg_start_y
    if current_seg == "RIGHT":
        return dx >= horiz_target
    elif current_seg == "LEFT":
        return dx <= -horiz_target
    elif current_seg == "FORWARD":
        return dy >= forward_target
    else:
        return False

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
    time.sleep(3)
    print("[OK] Airborne!")

    est_x = 0.0
    est_y = 0.0
    path_x = [est_x]
    path_y = [est_y]
    prev_v_right = 0.0
    prev_v_forward = 0.0
    alpha = 0.3
    DRIFT_COMP = 0.02
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
                    est_x = 0.0
                    est_y = 0.0
                    path_x = [est_x]
                    path_y = [est_y]
                    leg_start_x = est_x
                    leg_start_y = est_y
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
        leg_start_x = est_x
        leg_start_y = est_y
        horiz_target   = 1.0   # ≈ 2 m sideways
        forward_target = 0.20   # ≈ 0.10 m forward
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
                v_forward = vx          # forward
                v_right   = vy          # right (assuming +vy is right in your setup)

                # Smooth + removing small velocities
                if abs(v_right) < 0.025:
                    v_right = 0.0
                if abs(v_forward) < 0.025:
                    v_forward = 0.0
                v_right   = alpha * v_right   + (1 - alpha) * prev_v_right
                v_forward = alpha * v_forward + (1 - alpha) * prev_v_forward
                prev_v_right   = v_right
                prev_v_forward = v_forward
                # Integrate
                est_x += SCALE_X * v_right * dt
                est_y += SCALE_Y * v_forward * dt

            path_x.append(est_x)
            path_y.append(est_y)

            #------- NEW switching states based off distance ----------
            if current_seg == "FORWARD":
                MAX_SEG_TIME = 2.5   # seconds (shorter)
            else:
                MAX_SEG_TIME = 6.0   # seconds (longer for 2m sideways)
            if segment_reached(current_seg, est_x, est_y, leg_start_x, leg_start_y, horiz_target, forward_target):
                # move to next segment
                segment_index = (segment_index + 1) % len(segments)
                current_seg = segments[segment_index]
                segment_start_time = now

                # reset leg start pose for the next segment
                leg_start_x = est_x
                leg_start_y = est_y
                print(f"[PATH] Switching to segment: {current_seg} (distance reached)")

                # brief hover to stabilize
                drone.hover()
                time.sleep(1.0)
                last_pose_time = time.time()
                continue
            #fail-safe
            if time.time() - segment_start_time > MAX_SEG_TIME:
                segment_index = (segment_index + 1) % len(segments)
                current_seg = segments[segment_index]
                segment_start_time = time.time()
                leg_start_x = est_x
                leg_start_y = est_y
                print(f"[PATH] Switching to segment (via Fail safe): {current_seg} (timeout)")

                drone.hover()
                time.sleep(1.0)
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