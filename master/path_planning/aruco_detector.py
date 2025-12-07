"""
ArUco tag detection for Tello drone camera.

Adapted from LTT2.py to work with Tello's 320x240 video stream.
Detects ArUco markers and estimates their 3D pose and world position.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from .config import PathPlanningConfig
from .utils import camera_frame_to_world


class ArucoDetector:
    """ArUco marker detection and pose estimation for Tello camera"""

    def __init__(self, config: PathPlanningConfig):
        """
        Initialize ArUco detector.

        Args:
            config: PathPlanningConfig instance
        """
        self.config = config

        # Initialize ArUco dictionary and detector
        aruco_dict_name = getattr(cv2.aruco, config.ARUCO_DICT)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Marker sizes (meters)
        self.marker_sizes = config.get_marker_sizes()

        # Camera calibration
        self.camera_matrix = config.CAMERA_MATRIX
        self.dist_coeffs = config.DIST_COEFFS

        # Detection tracking
        self.tags_found = {0: False, 1: False}
        self.tag_detections = {0: [], 1: []}  # Store multiple detections for confidence
        self.frame_counter = 0

    def detect_tags(self, frame: np.ndarray, drone_position: Dict, drone_yaw: float) -> List[Dict]:
        """
        Detect ArUco tags in frame and estimate their world positions.

        Args:
            frame: BGR image from Tello camera (320x240)
            drone_position: Current drone position dict {x, y, z} in cm
            drone_yaw: Current drone yaw in degrees

        Returns:
            List of detected tag dictionaries with keys:
            - tag_id: ArUco marker ID
            - distance_m: Distance from camera to tag in meters
            - tvec: Translation vector (camera frame)
            - rvec: Rotation vector
            - world_position: Estimated world position {x, y, z} in cm
            - confidence: Detection confidence (based on detection count)
            - timestamp: Detection timestamp
        """
        self.frame_counter += 1

        # Only process every Nth frame for performance
        if self.frame_counter % self.config.DETECTION_FRAME_INTERVAL != 0:
            return []

        if frame is None:
            return []

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        detected_tags = []

        if ids is not None and len(corners) > 0:
            for i, corner in enumerate(corners):
                tag_id = int(ids[i][0])

                # Only care about tags we know the size for (ID 0 and 1)
                if tag_id not in self.marker_sizes:
                    continue

                marker_length = self.marker_sizes[tag_id]

                # Estimate pose for this marker
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    [corner],
                    marker_length,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Calculate distance from camera
                distance_m = np.linalg.norm(tvec)

                # Transform from camera frame to world frame
                world_pos = camera_frame_to_world(tvec, rvec, drone_position, drone_yaw)

                # Add to detection history
                self.tag_detections[tag_id].append({
                    'timestamp': time.time(),
                    'distance': distance_m,
                    'world_position': world_pos
                })

                # Calculate confidence based on number of detections
                detection_count = len(self.tag_detections[tag_id])
                confidence = min(1.0, detection_count / 5.0)  # Max confidence after 5 detections

                # Mark as found if confidence is high enough
                if confidence >= self.config.MIN_DETECTION_CONFIDENCE:
                    self.tags_found[tag_id] = True

                detected_tags.append({
                    'tag_id': tag_id,
                    'distance_m': distance_m,
                    'tvec': tvec.tolist(),
                    'rvec': rvec.tolist(),
                    'world_position': world_pos,
                    'confidence': confidence,
                    'timestamp': time.time(),
                    'drone_position': drone_position.copy(),
                    'drone_yaw': drone_yaw
                })

        return detected_tags

    def get_confirmed_tag_positions(self) -> Dict[int, Dict]:
        """
        Get world positions of tags that have been confirmed with high confidence.

        Returns:
            Dict mapping tag_id to world position dict {x, y, z}
        """
        confirmed_positions = {}

        for tag_id in [0, 1]:
            if self.tags_found[tag_id] and len(self.tag_detections[tag_id]) > 0:
                # Average the last N detections for better accuracy
                recent_detections = self.tag_detections[tag_id][-10:]

                avg_x = sum(d['world_position']['x'] for d in recent_detections) / len(recent_detections)
                avg_y = sum(d['world_position']['y'] for d in recent_detections) / len(recent_detections)
                avg_z = sum(d['world_position']['z'] for d in recent_detections) / len(recent_detections)

                confirmed_positions[tag_id] = {
                    'x': avg_x,
                    'y': avg_y,
                    'z': avg_z
                }

        return confirmed_positions

    def both_tags_found(self) -> bool:
        """
        Check if both start and finish tags have been found with sufficient confidence.

        Returns:
            bool: True if both tags confirmed
        """
        return self.tags_found[0] and self.tags_found[1]

    def get_tag_status(self) -> Dict:
        """
        Get current detection status for both tags.

        Returns:
            Dict with tag detection statistics
        """
        return {
            'tag_0_found': self.tags_found[0],
            'tag_1_found': self.tags_found[1],
            'tag_0_detections': len(self.tag_detections[0]),
            'tag_1_detections': len(self.tag_detections[1]),
            'both_found': self.both_tags_found()
        }

    def draw_detections(self, frame: np.ndarray, detected_tags: List[Dict]) -> np.ndarray:
        """
        Draw ArUco markers and information on frame for visualization.

        Args:
            frame: Input frame
            detected_tags: List of detected tag dicts from detect_tags()

        Returns:
            Frame with drawn markers and text
        """
        annotated_frame = frame.copy()

        if not detected_tags:
            return annotated_frame

        for tag_info in detected_tags:
            tag_id = tag_info['tag_id']
            distance_m = tag_info['distance_m']
            confidence = tag_info['confidence']

            # Draw text overlay
            status = "CONFIRMED" if confidence >= self.config.MIN_DETECTION_CONFIDENCE else f"{confidence*100:.0f}%"
            text = f"ID {tag_id}: {distance_m:.2f}m [{status}]"

            # Position text in corner based on tag ID
            if tag_id == 0:
                position = (10, 30)
                color = (255, 0, 0)  # Blue for start
            else:
                position = (10, 60)
                color = (0, 165, 255)  # Orange for finish

            cv2.putText(
                annotated_frame,
                text,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA
            )

        # Draw overall status
        status_text = f"Tags Found: {self.tags_found[0]}, {self.tags_found[1]}"
        cv2.putText(
            annotated_frame,
            status_text,
            (10, annotated_frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0) if self.both_tags_found() else (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        return annotated_frame

    def reset(self):
        """Reset detection state for new mission"""
        self.tags_found = {0: False, 1: False}
        self.tag_detections = {0: [], 1: []}
        self.frame_counter = 0
