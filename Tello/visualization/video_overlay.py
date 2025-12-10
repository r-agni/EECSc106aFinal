"""
Video overlay handler for obstacle detection visualization.

Draws bounding boxes, labels, and threat indicators on the video stream.
"""

import cv2
import numpy as np
from typing import List, Dict, Optional


class VideoOverlay:
    """
    Adds obstacle detection overlays to video stream.

    Works with both OpenCV window and HTTP stream.
    """

    def __init__(self, video_handler, obstacle_detector):
        """
        Initialize video overlay handler.

        Args:
            video_handler: VideoStreamHandler instance
            obstacle_detector: ObstacleDetector instance
        """
        self.video_handler = video_handler
        self.detector = obstacle_detector
        self.show_overlays = True

    def process_frame_with_overlays(self, frame: np.ndarray) -> np.ndarray:
        """
        Process frame and add obstacle detection overlays.

        Args:
            frame: Input video frame

        Returns:
            Frame with overlays drawn
        """
        if frame is None or not self.show_overlays:
            return frame

        # Detect obstacles
        obstacles = self.detector.process_frame(frame)

        if not obstacles:
            return frame

        # Draw detections on frame
        return self.detector.draw_detections(frame, obstacles)

    def enable(self):
        """Enable overlays."""
        self.show_overlays = True

    def disable(self):
        """Disable overlays."""
        self.show_overlays = False

    def toggle(self):
        """Toggle overlays on/off."""
        self.show_overlays = not self.show_overlays
        return self.show_overlays
