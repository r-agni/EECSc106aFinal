"""
Video stream interface for obstacle detection.

This module provides access to the drone's video stream and
prepares frames for object detection.
"""

import cv2
import numpy as np
from queue import Empty
from typing import Optional


class VideoProcessor:
    """Interface to access and process video frames from drone"""

    def __init__(self, video_handler):
        """
        Initialize video processor.

        Args:
            video_handler: VideoStreamHandler instance from tello_server
        """
        self.video_handler = video_handler

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        Get the most recent frame from video stream.

        Returns:
            Latest frame as numpy array (BGR format), or None if unavailable
        """
        try:
            frame = self.video_handler.frame_queue.get_nowait()
            return frame
        except Empty:
            return None

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for object detection.

        Args:
            frame: Input frame in BGR format

        Returns:
            Preprocessed frame (YOLO expects BGR, so no conversion needed)
        """
        # YOLOv8 handles BGR input natively, so minimal preprocessing needed
        return frame

    def resize_frame(self, frame: np.ndarray, width: int, height: int) -> np.ndarray:
        """
        Resize frame to specified dimensions.

        Args:
            frame: Input frame
            width: Target width
            height: Target height

        Returns:
            Resized frame
        """
        return cv2.resize(frame, (width, height))

    def draw_camera_zones(self, frame: np.ndarray, zone_left: float, zone_right: float) -> np.ndarray:
        """
        Draw vertical lines showing left/center/right zones.

        Args:
            frame: Input frame
            zone_left: X coordinate of left boundary
            zone_right: X coordinate of right boundary

        Returns:
            Frame with zone lines drawn
        """
        frame_copy = frame.copy()
        height = frame.shape[0]

        # Draw zone boundaries
        cv2.line(frame_copy, (int(zone_left), 0), (int(zone_left), height), (255, 255, 0), 1)
        cv2.line(frame_copy, (int(zone_right), 0), (int(zone_right), height), (255, 255, 0), 1)

        # Add labels
        cv2.putText(frame_copy, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame_copy, "CENTER", (int(zone_left) + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame_copy, "RIGHT", (int(zone_right) + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return frame_copy

    def add_status_text(self, frame: np.ndarray, text: str, position: tuple = (10, 60),
                        color: tuple = (0, 255, 0)) -> np.ndarray:
        """
        Add status text overlay to frame.

        Args:
            frame: Input frame
            text: Text to display
            position: (x, y) position for text
            color: BGR color tuple

        Returns:
            Frame with text overlay
        """
        frame_copy = frame.copy()
        cv2.putText(frame_copy, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame_copy
