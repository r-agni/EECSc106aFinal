"""
Obstacle detection and dimension estimation using YOLOv8.

This module detects objects in video frames, estimates their distance using
the pinhole camera model, and classifies their threat level.
"""

import cv2
import numpy as np
from typing import List, Dict, Optional
from ultralytics import YOLO

from .config import Config


class ObstacleDetector:
    """Real-time object detection and distance estimation"""

    def __init__(self, config: Config = Config()):
        """
        Initialize obstacle detector with YOLOv8 model.

        Args:
            config: Configuration object with detection parameters
        """
        self.config = config
        self.frame_count = 0

        # Load YOLOv8 nano model (will download on first run)
        print(f"[*] Loading YOLO model: {config.YOLO_MODEL}")
        self.model = YOLO(config.YOLO_MODEL)
        print("[+] YOLO model loaded successfully")

        self.focal_length = config.FOCAL_LENGTH_PX
        self.object_heights = config.OBJECT_HEIGHTS

    def detect_objects(self, frame: np.ndarray) -> List[Dict]:
        """
        Run YOLO detection on frame.

        Args:
            frame: Input image (BGR format from OpenCV)

        Returns:
            List of detected objects with bounding boxes and confidence
        """
        results = self.model(frame, conf=self.config.DETECTION_CONFIDENCE, verbose=False)

        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract box data
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                detection = {
                    "bbox": {
                        "x": int(x1),
                        "y": int(y1),
                        "w": int(x2 - x1),
                        "h": int(y2 - y1),
                    },
                    "class": class_name,
                    "confidence": confidence,
                    "class_id": class_id
                }
                detections.append(detection)

        return detections

    def estimate_distance(self, bbox_height: int, object_class: str) -> float:
        """
        Estimate distance to object using pinhole camera model.

        Formula: distance = (known_height * focal_length) / pixel_height

        Args:
            bbox_height: Height of bounding box in pixels
            object_class: Class name of detected object

        Returns:
            Estimated distance in meters
        """
        if bbox_height == 0:
            return 999.0  # Very far away

        # Get known height for this object class
        known_height = self.object_heights.get(object_class, self.object_heights["generic"])

        # Calculate distance using pinhole camera model
        distance_m = (known_height * self.focal_length) / bbox_height

        return distance_m

    def estimate_width(self, bbox_width: int, distance_m: float) -> float:
        """
        Estimate real-world width of object.

        Args:
            bbox_width: Width of bounding box in pixels
            distance_m: Distance to object in meters

        Returns:
            Estimated width in meters
        """
        if self.focal_length == 0:
            return 0.5  # Default guess

        width_m = (bbox_width * distance_m) / self.focal_length
        return width_m

    def determine_position(self, bbox_center_x: int) -> str:
        """
        Classify object position as left, center, or right.

        Args:
            bbox_center_x: X coordinate of bounding box center

        Returns:
            Position string: "left", "center", or "right"
        """
        if bbox_center_x < self.config.ZONE_LEFT_BOUNDARY:
            return "left"
        elif bbox_center_x > self.config.ZONE_RIGHT_BOUNDARY:
            return "right"
        else:
            return "center"

    def assess_threat_level(self, distance_m: float, position: str) -> str:
        """
        Determine threat level based on distance and position.

        Args:
            distance_m: Distance to obstacle in meters
            position: Position of obstacle ("left", "center", "right")

        Returns:
            Threat level: "high", "medium", or "low"
        """
        # Center obstacles are more threatening
        if position == "center":
            if distance_m < self.config.THREAT_DISTANCE_HIGH:
                return "high"
            elif distance_m < self.config.THREAT_DISTANCE_MEDIUM:
                return "medium"
            else:
                return "low"
        else:
            # Side obstacles need to be closer to be threats
            if distance_m < self.config.THREAT_DISTANCE_HIGH * 0.7:
                return "medium"
            else:
                return "low"

    def process_frame(self, frame: Optional[np.ndarray]) -> List[Dict]:
        """
        Main processing pipeline - detect objects and estimate dimensions.

        Args:
            frame: Input video frame (BGR format)

        Returns:
            List of obstacle dictionaries with full information
        """
        if frame is None:
            return []

        self.frame_count += 1

        # Skip frames for performance
        if self.frame_count % self.config.PROCESS_EVERY_N_FRAMES != 0:
            return []

        # Detect objects
        detections = self.detect_objects(frame)

        # Process each detection
        obstacles = []
        for i, det in enumerate(detections):
            bbox = det["bbox"]
            bbox_height = bbox["h"]
            bbox_width = bbox["w"]
            bbox_center_x = bbox["x"] + bbox_width // 2

            # Estimate distance and dimensions
            distance_m = self.estimate_distance(bbox_height, det["class"])
            width_m = self.estimate_width(bbox_width, distance_m)
            position = self.determine_position(bbox_center_x)
            threat_level = self.assess_threat_level(distance_m, position)

            obstacle = {
                "id": i,
                "class": det["class"],
                "confidence": det["confidence"],
                "bbox": bbox,
                "distance_m": round(distance_m, 2),
                "width_m": round(width_m, 2),
                "position": position,
                "threat_level": threat_level
            }

            obstacles.append(obstacle)

            # Log detection if enabled
            if self.config.LOG_DETECTIONS and threat_level in ["high", "medium"]:
                print(f"[DETECTION] {det['class']} @ {distance_m:.2f}m, "
                      f"pos={position}, threat={threat_level}")

        return obstacles

    def calibrate_focal_length(self, known_distance_m: float, known_height_m: float,
                               measured_pixel_height: int):
        """
        Calibrate focal length using a known object at measured distance.

        Args:
            known_distance_m: Measured distance to object in meters
            known_height_m: Known real-world height of object in meters
            measured_pixel_height: Measured height in pixels from video frame

        Returns:
            Calibrated focal length in pixels
        """
        self.focal_length = (measured_pixel_height * known_distance_m) / known_height_m
        print(f"[+] Focal length calibrated: {self.focal_length:.1f} pixels")
        return self.focal_length

    def draw_detections(self, frame: np.ndarray, obstacles: List[Dict]) -> np.ndarray:
        """
        Draw bounding boxes and labels on frame for visualization.

        Args:
            frame: Input frame
            obstacles: List of detected obstacles

        Returns:
            Frame with drawn detections
        """
        frame_copy = frame.copy()

        for obs in obstacles:
            bbox = obs["bbox"]
            x, y, w, h = bbox["x"], bbox["y"], bbox["w"], bbox["h"]

            # Color based on threat level
            if obs["threat_level"] == "high":
                color = (0, 0, 255)  # Red
            elif obs["threat_level"] == "medium":
                color = (0, 165, 255)  # Orange
            else:
                color = (0, 255, 0)  # Green

            # Draw bounding box
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = f"{obs['class']} {obs['distance_m']:.1f}m"
            cv2.putText(frame_copy, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return frame_copy
