"""
YOLO-based Obstacle Detector for AR Drone

This module handles obstacle detection using YOLOv8 on the front camera feed.
It estimates distance based on bounding box size and classifies obstacles
as "close" (requires avoidance) or "far" (can ignore).
"""

import cv2
import numpy as np


class ObstacleDetector:
    """YOLO-based obstacle detection using front camera."""

    def __init__(self, model_path='yolov8n.pt', confidence_threshold=0.5,
                 close_threshold=0.1, frame_size=(640, 480)):
        """
        Initialize YOLO obstacle detector.

        Args:
            model_path: Path to YOLO model file
            confidence_threshold: Minimum confidence for detection (0.0-1.0)
            close_threshold: Area ratio threshold for "close" obstacles
            frame_size: Expected frame dimensions (width, height)
        """
        self.confidence_threshold = confidence_threshold
        self.close_threshold = close_threshold
        self.frame_size = frame_size
        self.model = None
        self.model_loaded = False

        # Try to load YOLO model
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.model_loaded = True
            print(f"[YOLO] Model loaded successfully: {model_path}")
        except ImportError:
            print("[WARN] ultralytics package not found. Install with: pip install ultralytics")
            print("[WARN] Obstacle detection will be disabled")
        except Exception as e:
            print(f"[WARN] Could not load YOLO model: {e}")
            print("[WARN] Obstacle detection will be disabled")

    def detect(self, frame):
        """
        Detect obstacles in frame using YOLO.

        Args:
            frame: OpenCV BGR image frame

        Returns:
            list of dict with keys:
                - 'bbox': (x1, y1, x2, y2) bounding box coordinates
                - 'confidence': Detection confidence (0.0-1.0)
                - 'class_id': COCO class ID
                - 'class_name': COCO class name
                - 'area_ratio': Bbox area / frame area
                - 'distance': 'close' or 'far'
                - 'center': (cx, cy) center point
        """
        if not self.model_loaded or self.model is None:
            return []

        try:
            # Run YOLO inference
            results = self.model(frame, verbose=False)
            obstacles = []

            frame_area = frame.shape[0] * frame.shape[1]

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    conf = float(box.conf[0])
                    if conf < self.confidence_threshold:
                        continue

                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    bbox = (int(x1), int(y1), int(x2), int(y2))

                    # Calculate bbox properties
                    bbox_width = x2 - x1
                    bbox_height = y2 - y1
                    bbox_area = bbox_width * bbox_height
                    area_ratio = bbox_area / frame_area

                    # Center point
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Distance estimation based on size
                    # Large bbox = close object (requires avoidance)
                    # Small bbox = far object (can ignore)
                    distance_category = "close" if area_ratio > self.close_threshold else "far"

                    # Get class info
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id] if hasattr(result, 'names') else str(class_id)

                    obstacles.append({
                        'bbox': bbox,
                        'confidence': conf,
                        'class_id': class_id,
                        'class_name': class_name,
                        'area_ratio': area_ratio,
                        'distance': distance_category,
                        'center': (cx, cy),
                        'width': bbox_width,
                        'height': bbox_height
                    })

            return obstacles

        except Exception as e:
            print(f"[ERROR] YOLO detection failed: {e}")
            return []

    def get_close_obstacles(self, obstacles):
        """
        Filter for close obstacles that require avoidance.

        Args:
            obstacles: List of obstacle dicts from detect()

        Returns:
            List of close obstacles only
        """
        return [obs for obs in obstacles if obs['distance'] == 'close']

    def draw_detections(self, frame, obstacles):
        """
        Draw obstacle bounding boxes and labels on frame.

        Args:
            frame: OpenCV BGR image
            obstacles: List of obstacle dicts from detect()

        Returns:
            Frame with drawn detections
        """
        annotated_frame = frame.copy()

        for obs in obstacles:
            x1, y1, x2, y2 = obs['bbox']

            # Color based on distance
            if obs['distance'] == 'close':
                color = (0, 0, 255)  # Red for close obstacles
                thickness = 3
            else:
                color = (0, 255, 255)  # Yellow for far obstacles
                thickness = 2

            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)

            # Draw label
            label = f"{obs['class_name']} {obs['confidence']:.2f} ({obs['distance']})"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

            # Background for text
            cv2.rectangle(annotated_frame,
                         (x1, y1 - label_size[1] - 10),
                         (x1 + label_size[0], y1),
                         color, -1)

            # Text
            cv2.putText(annotated_frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw center point
            cx, cy = obs['center']
            cv2.circle(annotated_frame, (cx, cy), 5, color, -1)

        return annotated_frame

    def should_avoid(self, obstacles):
        """
        Determine if drone should avoid obstacles.

        Args:
            obstacles: List of obstacle dicts from detect()

        Returns:
            (should_avoid: bool, direction: str or None)
            direction can be 'left', 'right', 'up', or None
        """
        close_obs = self.get_close_obstacles(obstacles)

        if not close_obs:
            return False, None

        # Analyze obstacle positions
        # If obstacle is in center, suggest avoidance direction
        frame_width = self.frame_size[0]
        frame_center_x = frame_width / 2

        # Find largest close obstacle
        largest_obs = max(close_obs, key=lambda x: x['area_ratio'])
        obs_center_x = largest_obs['center'][0]

        # Suggest direction based on obstacle position
        if obs_center_x < frame_center_x:
            # Obstacle on left, go right
            direction = 'right'
        else:
            # Obstacle on right, go left
            direction = 'left'

        return True, direction

    def get_summary(self, obstacles):
        """
        Get human-readable summary of detected obstacles.

        Args:
            obstacles: List of obstacle dicts from detect()

        Returns:
            str summary
        """
        if not obstacles:
            return "No obstacles detected"

        close_count = len(self.get_close_obstacles(obstacles))
        far_count = len(obstacles) - close_count

        summary = f"Detected {len(obstacles)} obstacle(s): "
        summary += f"{close_count} close, {far_count} far"

        return summary
