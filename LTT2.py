import cv2
import cv2.aruco as aruco
import numpy as np

"""This script uses a Logitech webcam to detect ArUco tags 0 (target position)
and 1 (drone tag). For each frame:

- Detects ArUco markers.
- Estimates their pose using known tag sizes and camera intrinsics.
- Computes the relative transform ^0 T_1 (Tag 1 in Tag 0 frame) whenever both
  tags are visible.
- Logs the trajectory of Tag 1 in Tag 0's frame over time.
"""

# ---------------------------------------------------------------------------
# ArUco dictionary and detector
# ---------------------------------------------------------------------------
marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
param_markers = cv2.aruco.DetectorParameters()
detection = cv2.aruco.ArucoDetector(marker_dict, param_markers)


# ---------------------------------------------------------------------------
# Helper functions for transformation calculation
# ---------------------------------------------------------------------------
def rvec_tvec_to_T(rvec, tvec):
    """Convert OpenCV rvec, tvec to 4x4 homogeneous transform ^C T_tag."""
    R, _ = cv2.Rodrigues(rvec)
    t = np.array(tvec, dtype=np.float32).reshape(3, 1)

    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3:4] = t
    return T


def invert_T(T):
    """Invert a rigid 4x4 transform."""
    R = T[:3, :3]
    t = T[:3, 3:4]
    T_inv = np.eye(4, dtype=np.float32)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3:4] = -R.T @ t
    return T_inv


# ---------------------------------------------------------------------------
# Tag sizes (in meters)
# ---------------------------------------------------------------------------
tag_length = 0.1      # 10 cm for tag ID 0
tag_length_D = 0.025  # 2.5 cm for tag ID 1
marker_sizes = {
    0: tag_length,
    1: tag_length_D,
}

# ---------------------------------------------------------------------------
# Camera intrinsics (from calibrator.py)
# ---------------------------------------------------------------------------
fx = 806.5926129911874
fy = 794.9765471001547
cx = 559.8894214553991
cy = 272.6579404675458

camera_matrix = np.array([
    [fx,   0.0, cx],
    [0.0,  fy, cy],
    [0.0,  0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.array(
    [0.28992605, -2.14313702, -0.02123785, 0.03814953, 4.11487117],
    dtype=np.float32
)

# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------
camera_index = 1
cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Could not open camera.")
    exit()

# Make sure these match your calibration resolution/orientation
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)

trajectory_tag1_in_tag0 = []
T_0_1_final = None

print("Press 'q' to quit.\n")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame.")
        break

    if frame is not None:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detection.detectMarkers(gray_frame)

        # Poses for tag 0 and tag 1 (in camera frame)
        rvec_0 = tvec_0 = None
        rvec_1 = tvec_1 = None

        if ids is not None and len(corners) > 0:
            # Draw all detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, corner in enumerate(corners):
                tag_id = int(ids[i][0])

                # Only care about tags we know the size for
                if tag_id not in marker_sizes:
                    continue

                marker_length = marker_sizes[tag_id]

                # Compute center for text overlay (corner is usually shape (1,4,2))
                center_x = int(corner[0][:, 0].mean())
                center_y = int(corner[0][:, 1].mean())
                print(f"Marker ID: {tag_id}, Position: (x: {center_x}, y: {center_y})")

                # Estimate pose for THIS marker only, with its correct physical size
                # Pass [corner] so the function sees a single-marker list
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    [corner],
                    marker_length,
                    camera_matrix,
                    dist_coeffs
                )

                # rvecs, tvecs shape: (1, 1, 3) typically -> take [0][0]
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Save for relative transform computation later
                if tag_id == 0:
                    rvec_0, tvec_0 = rvec, tvec
                elif tag_id == 1:
                    rvec_1, tvec_1 = rvec, tvec

                # Predict distance from camera
                distance_m = np.linalg.norm(tvec)

                # Draw coordinate axes on the marker
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    marker_length * 0.5
                )

                # Text overlay with distance
                text = f"ID {tag_id}: {distance_m:.2f} m"
                cv2.putText(
                    frame,
                    text,
                    (center_x - 80, center_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA
                )

            # After processing all markers in this frame, if both tags are visible,
            # compute Tag 1 pose in Tag 0 frame: ^0 T_1 = (^C T_0)^(-1) * (^C T_1)
            if (rvec_0 is not None) and (rvec_1 is not None):
                T_cam_0 = rvec_tvec_to_T(rvec_0, tvec_0)  # ^C T_0
                T_cam_1 = rvec_tvec_to_T(rvec_1, tvec_1)  # ^C T_1

                T_0_1 = invert_T(T_cam_0) @ T_cam_1      # ^0 T_1
                T_0_1_final = T_0_1.copy()

                # Translation of Tag 1 in Tag 0 frame
                p_0_1 = T_0_1[:3, 3].copy()
                trajectory_tag1_in_tag0.append(p_0_1)

    cv2.imshow("Webcam: ", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# ---------------------------------------------------------------------------
# Print results
# ---------------------------------------------------------------------------
print("\n=== Trajectory of Tag 1 in Tag 0 frame ===")
if len(trajectory_tag1_in_tag0) == 0:
    print("No frames where both tags 0 and 1 were visible at the same time.")
else:
    for i, p in enumerate(trajectory_tag1_in_tag0):
        x, y, z = p
        print(f"{i:03d}: x = {x:.3f} m, y = {y:.3f} m, z = {z:.3f} m")

    if T_0_1_final is not None:
        print("\n=== Final transform ^0 T_1 (Tag 1 in Tag 0 frame) ===")
        print(T_0_1_final)
