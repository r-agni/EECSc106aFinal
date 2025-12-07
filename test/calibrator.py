import cv2
import numpy as np

# ==== YOUR BOARD SETTINGS ===================================
# 11 squares across, 8 squares down -> 10 x 7 inner corners
checkerboard_dims = (10, 7)   # (cols, rows) of INNER corners
square_size = 0.015           # 1.5 cm in meters
# ============================================================

# Camera index: change to 0 if needed
camera_index = 1

cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

# Set your target resolution (width, height)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    raise SystemExit

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30, 0.001)

# Prepare template of 3D object points for one board
objp = np.zeros((checkerboard_dims[0] * checkerboard_dims[1], 3), np.float32)
objp[:, :2] = np.indices(checkerboard_dims).T.reshape(-1, 2)
objp *= square_size

objpoints = []   # 3D points in world
imgpoints = []   # 2D image points

print("Live calibration running...")
print("Instructions:")
print("  - Move the checkerboard around the view (tilt, rotate, vary distance).")
print("  - When corners are outlined in GREEN, press 'c' to capture that view.")
print("  - Capture at least 10–20 views from different poses.")
print("  - Press 'q' to finish and run calibration.\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame from camera.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret_corners, corners = cv2.findChessboardCorners(
        gray, checkerboard_dims, None
    )

    display = frame.copy()

    if ret_corners:
        # Refine corners
        corners_subpix = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
        )
        cv2.drawChessboardCorners(display, checkerboard_dims, corners_subpix, ret_corners)

        cv2.putText(display, "Checkerboard found - press 'c' to capture",
                    (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(display, "Show the 10x7 inner-corner checkerboard to the camera",
                    (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.putText(display, f"Captured views: {len(objpoints)}",
                (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Live Calibration", display)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c') and ret_corners:
        # Save this view
        objpoints.append(objp.copy())
        imgpoints.append(corners_subpix)
        print(f"[CAPTURED] View #{len(objpoints)}")

    elif key == ord('q'):
        print("Quitting capture loop...")
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 5:
    print(f"\nERROR: Only {len(objpoints)} valid views captured.")
    print("Capture at least 10–20 views with 'c' before pressing 'q'.")
    raise SystemExit

# Use size from the last frame we processed
h, w = gray.shape[:2]

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, (w, h), None, None
)

print("\n=== CAMERA CALIBRATION RESULTS ===")
print("Image resolution used: {} x {}".format(w, h))
print("\nCamera matrix K:")
print(camera_matrix)

print("\nfx =", camera_matrix[0, 0])
print("fy =", camera_matrix[1, 1])
print("cx =", camera_matrix[0, 2])
print("cy =", camera_matrix[1, 2])

print("\nDistortion coefficients (k1, k2, p1, p2, k3,...):")
print(dist_coeffs.ravel())
