import cv2
import cv2.aruco as aruco
import numpy as np

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
param_markers =  cv2.aruco.DetectorParameters()
detection = cv2.aruco.ArucoDetector(marker_dict, param_markers)

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
#dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
dist_coeffs = np.array([ 0.28992605, -2.14313702, -0.02123785,  0.03814953,  4.11487117], dtype=np.float32)

camera_index = 1
cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Could not open camera.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080) 

while (True):
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame.")
        break

    if frame is not None:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detection.detectMarkers(gray_frame)
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
                distance_m = np.linalg.norm(tvec)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_length * 0.5)
                text = f"ID {ids[i][0]}: {distance_m:.2f} m"
                cv2.putText(frame, text, (center_x - 80, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Webcam: ", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()