"""
A_id = detect_tag()
T_C0_A = aruco_pose(A_id)          # tag A -> camera at start
reset_odometry_frame()             # define C0

while flying:
    update_odometry()              # maintain T_Ct_C0

    ids = detect_tags()
    if exists id != A_id:
        B_id = choose_first(ids != A_id)
        T_C1_B = aruco_pose(B_id)  # tag B -> camera now
        T_C1_C0 = odom_transform() # camera now -> camera at start

        T_B_A = inverse(T_C1_B) * T_C1_C0 * T_C0_A
        return T_B_A

"""

# Minimal example: compute transform from first seen ArUco tag A to next different tag B
# Requires:
#   - OpenCV with aruco
#   - Camera intrinsics (K) and distortion (dist)
#   - Marker size in meters
#   - Some odometry/VIO that can return T_Ct_C0 (camera now in start-camera frame)

import cv2
import numpy as np

MARKER_SIZE = 0.10  # meters (change!)
A_ID = None

# ---- Fill these with your calibrated values ----
K = np.array([
    [600.0, 0.0, 320.0],
    [0.0, 600.0, 240.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64)
dist = np.zeros((5, 1), dtype=np.float64)
# -----------------------------------------------

def T_from_rvec_tvec(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T

def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

# ---- ODOM PLACEHOLDERS ----
def reset_odometry_frame():
    """
    Call your system's reset / set-origin here.
    After this, odom should report T_Ct_C0.
    """
    pass

def get_T_Ct_C0():
    """
    Return 4x4 transform of camera-now relative to camera-at-start.
    Replace with real VIO/SLAM/odometry output.
    """
    return np.eye(4, dtype=np.float64)
# ---------------------------

def detect_poses(gray, dictionary, params):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return {}, corners, ids

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, K, dist)
    poses = {}
    for i, tag_id in enumerate(ids.flatten()):
        poses[int(tag_id)] = (rvecs[i], tvecs[i])
    return poses, corners, ids

def main():
    global A_ID

    cap = cv2.VideoCapture(0)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()

    T_C0_A = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        poses, _, _ = detect_poses(gray, dictionary, params)

        # 1) Acquire first tag as origin A
        if A_ID is None and len(poses) > 0:
            A_ID = next(iter(poses.keys()))
            rvecA, tvecA = poses[A_ID]
            T_C0_A = T_from_rvec_tvec(rvecA, tvecA)  # tag A -> camera at start
            reset_odometry_frame()
            print(f"Origin tag A = {A_ID}")

        # 2) Later, find first different tag B and compute transform
        elif A_ID is not None and T_C0_A is not None:
            other_ids = [tid for tid in poses.keys() if tid != A_ID]
            if other_ids:
                B_ID = other_ids[0]
                rvecB, tvecB = poses[B_ID]
                T_C1_B = T_from_rvec_tvec(rvecB, tvecB)  # tag B -> camera now
                T_C1_C0 = get_T_Ct_C0()                  # camera now -> camera start

                # Transform mapping points in A frame into B frame:
                T_B_A = invert_T(T_C1_B) @ T_C1_C0 @ T_C0_A

                print(f"Destination tag B = {B_ID}")
                print("T_B_A =")
                np.set_printoptions(precision=4, suppress=True)
                print(T_B_A)

                break

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
