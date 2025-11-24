import cv2
import cv2.aruco as aruco
import numpy as np

"""#This is code to have our logitech cam detect aruco tags 0 (our target position) and 1 (drone tag).
If it spots either tag, it will use pinhole camera methods to predict the distance between the cam and tag
It will also process the Transformation matrices between both tags for every frame both are in view, and will
return a transform matrix of Tag1 in the Tag0 frame."""

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
param_markers =  cv2.aruco.DetectorParameters()
detection = cv2.aruco.ArucoDetector(marker_dict, param_markers)

#helper functions for transformation calculation
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


tag_length = 0.1 #10 cm
tag_length_D = 0.025 # 2.5 cm
marker_sizes = {
    0: tag_length,
    1:tag_length_D
}
#Values from calibratior.py:
fx = 806.5926129911874
fy = 794.9765471001547
cx = 559.8894214553991
cy = 272.6579404675458
camera_matrix = np.array([
    [fx,    0.0, cx],   # fx,  0,  cx
    [   0.0, fy, cy],   # 0,  fy,  cy
    [   0.0,    0.0,   1.0]
], dtype=np.float32)
dist_coeffs = np.array([ 0.28992605, -2.14313702, -0.02123785,  0.03814953,  4.11487117], dtype=np.float32)

camera_index = 1
cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Could not open camera.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080) 
trajectory_tag1_in_tag0 = []
T_0_1_final = None 

while (True):
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame.")
        break

    if frame is not None:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detection.detectMarkers(gray_frame)

        rvec_0 = tvec_0 = None
        rvec_1 = tvec_1 = None
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids) 
            for i, corner in enumerate(corners):
                # Get the center of the marker
                center_x = int(corner[0][:, 0].mean())
                center_y = int(corner[0][:, 1].mean())
                print(f"Marker ID: {ids[i][0]}, Position: (x: {center_x}, y: {center_y})")

                tag_id = int(ids[i][0])
                if tag_id not in marker_sizes:
                    continue
                marker_length = marker_sizes[tag_id]           
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    corners,
                    marker_length,
                    camera_matrix,
                    dist_coeffs
                 )
                
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                if tag_id == 0:
                    rvec_0, tvec_0 = rvec, tvec
                elif tag_id == 1:
                    rvec_1, tvec_1 = rvec, tvec

                #Predict distance from camera
                distance_m = np.linalg.norm(tvec)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_length * 0.5)
                text = f"ID {ids[i][0]}: {distance_m:.2f} m"
                cv2.putText(frame, text, (center_x - 80, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                
                #Find Transform matrix
                if (rvec_0 is not None) and (rvec_1 is not None):
                    T_cam_0 = rvec_tvec_to_T(rvec_0, tvec_0)   # ^C T_0
                    T_cam_1 = rvec_tvec_to_T(rvec_1, tvec_1)   # ^C T_1

                    # ^0 T_1 = (^C T_0)^(-1) * (^C T_1)
                    T_0_1 = invert_T(T_cam_0) @ T_cam_1
                    T_0_1_final = T_0_1.copy()

                    p_0_1 = T_0_1[:3, 3].copy()
                    trajectory_tag1_in_tag0.append(p_0_1)

    cv2.imshow("Webcam: ", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

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