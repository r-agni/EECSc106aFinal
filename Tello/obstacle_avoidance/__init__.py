"""
Path Planning and Obstacle Avoidance System for TeloDrone

This package provides autonomous navigation capabilities with real-time
obstacle detection and avoidance for the DJI Tello drone.

Main components:
- obstacle_detector: YOLOv8-based object detection and distance estimation
- circumvent: Obstacle avoidance maneuvers with position compensation
- path_executor: Main path following coordinator with dead reckoning
- video_processor: Video stream interface
- config: Configuration parameters
"""

__version__ = "1.0.0"
__author__ = "TeloDrone Team"

from .config import Config
from .path_executor import PathExecutor
from .obstacle_detector import ObstacleDetector
from .video_processor import VideoProcessor
from .circumvent import ObstacleCircumvention

__all__ = [
    "Config",
    "PathExecutor",
    "ObstacleDetector",
    "VideoProcessor",
    "ObstacleCircumvention"
]
