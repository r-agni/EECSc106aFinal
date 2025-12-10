"""
ARuco Marker Detector for AR Drone

This module handles ARuco marker detection using the bottom camera.
Specifically designed to find ARuco tag ID 1 for navigation.
"""

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector:
    """ARuco marker detector using OpenCV."""

    def __init__(self, dictionary=aruco.DICT_4X4_50, target_id=1):
        """
        Initialize ARuco detector.

        Args:
            dictionary: ARuco dictionary type (default: DICT_4X4_50)
            target_id: Target ARuco marker ID to search for
        """
        self.target_id = target_id
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = aruco.DetectorParameters()

        print(f"[ARUCO] Initialized detector for ID {target_id}")
        print(f"[ARUCO] Dictionary: {dictionary}")

    def detect(self, frame):
        """
        Detect ARuco markers in frame.

        Args:
            frame: OpenCV BGR image frame

        Returns:
            list of dict with keys:
                - 'id': Marker ID
                - 'corners': 4x2 array of corner coordinates
                - 'center': (cx, cy) center point
                - 'area': Marker area in pixels
        """
        # Convert to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        markers = []

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]

                # Calculate center
                cx = int(np.mean(marker_corners[:, 0]))
                cy = int(np.mean(marker_corners[:, 1]))

                # Calculate area
                # Using cross product for quadrilateral area
                x = marker_corners[:, 0]
                y = marker_corners[:, 1]
                area = 0.5 * abs(
                    (x[0]*y[1] - x[1]*y[0]) +
                    (x[1]*y[2] - x[2]*y[1]) +
                    (x[2]*y[3] - x[3]*y[2]) +
                    (x[3]*y[0] - x[0]*y[3])
                )

                markers.append({
                    'id': int(marker_id),
                    'corners': marker_corners,
                    'center': (cx, cy),
                    'area': area
                })

        return markers

    def find_target(self, frame):
        """
        Find the target ARuco marker.

        Args:
            frame: OpenCV BGR image frame

        Returns:
            dict with marker info if found, None otherwise
        """
        markers = self.detect(frame)

        for marker in markers:
            if marker['id'] == self.target_id:
                return marker

        return None

    def is_target_found(self, frame):
        """
        Check if target marker is in frame.

        Args:
            frame: OpenCV BGR image frame

        Returns:
            bool: True if target found
        """
        return self.find_target(frame) is not None

    def draw_detections(self, frame, markers=None):
        """
        Draw detected ARuco markers on frame.

        Args:
            frame: OpenCV BGR image
            markers: List of marker dicts (if None, will detect)

        Returns:
            Frame with drawn markers
        """
        if markers is None:
            markers = self.detect(frame)

        annotated_frame = frame.copy()

        if not markers:
            return annotated_frame

        # Prepare data for OpenCV drawing function
        corners_list = [np.array([m['corners']], dtype=np.float32) for m in markers]
        ids_list = np.array([[m['id']] for m in markers], dtype=np.int32)

        # Draw detected markers
        aruco.drawDetectedMarkers(annotated_frame, corners_list, ids_list)

        # Add additional info for each marker
        for marker in markers:
            cx, cy = marker['center']

            # Highlight target marker differently
            if marker['id'] == self.target_id:
                color = (0, 255, 0)  # Green for target
                thickness = 3
                label = f"TARGET ID {marker['id']}"
            else:
                color = (0, 255, 255)  # Yellow for others
                thickness = 2
                label = f"ID {marker['id']}"

            # Draw center point
            cv2.circle(annotated_frame, (cx, cy), 5, color, -1)

            # Draw label with background
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated_frame,
                         (cx - label_size[0]//2, cy - 25),
                         (cx + label_size[0]//2, cy - 5),
                         color, -1)
            cv2.putText(annotated_frame, label,
                       (cx - label_size[0]//2, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        return annotated_frame

    def get_detection_quality(self, marker):
        """
        Assess quality of marker detection.

        Args:
            marker: Marker dict from detect()

        Returns:
            str: 'excellent', 'good', 'poor'
        """
        # Quality based on marker area (larger = closer = better)
        area = marker['area']

        if area > 5000:
            return 'excellent'
        elif area > 2000:
            return 'good'
        else:
            return 'poor'

    def get_marker_distance_estimate(self, marker, known_size_mm=100):
        """
        Estimate distance to marker based on apparent size.

        Args:
            marker: Marker dict from detect()
            known_size_mm: Real-world marker size in millimeters

        Returns:
            float: Estimated distance in millimeters (rough approximation)
        """
        # Very rough estimation based on area
        # Assumes marker is square and directly below camera
        marker_pixel_size = np.sqrt(marker['area'])

        # Focal length approximation for AR Drone camera
        # This is a rough estimate and should be calibrated
        focal_length_pixels = 200

        # Distance = (real_size * focal_length) / pixel_size
        distance_mm = (known_size_mm * focal_length_pixels) / marker_pixel_size

        return distance_mm

    def get_summary(self, markers):
        """
        Get human-readable summary of detections.

        Args:
            markers: List of marker dicts from detect()

        Returns:
            str summary
        """
        if not markers:
            return "No ARuco markers detected"

        target_found = any(m['id'] == self.target_id for m in markers)

        summary = f"Detected {len(markers)} marker(s)"
        if target_found:
            summary += f" - TARGET ID {self.target_id} FOUND!"
        else:
            summary += f" (target ID {self.target_id} not found)"

        return summary
