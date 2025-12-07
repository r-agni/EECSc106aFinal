"""
Configuration parameters for path planning system.

Loads settings from environment variables with sensible defaults.
"""

import os
import numpy as np
from dotenv import load_dotenv


class PathPlanningConfig:
    """Centralized configuration for path planning system"""

    def __init__(self, env_path=None):
        """
        Initialize configuration from environment variables.

        Args:
            env_path: Optional path to .env file. If None, looks in current directory.
        """
        if env_path:
            load_dotenv(env_path)
        else:
            # Try to load .env from path_planning directory
            env_file = os.path.join(os.path.dirname(__file__), '.env')
            if os.path.exists(env_file):
                load_dotenv(env_file)

        # ==================== SEARCH AREA CONFIGURATION ====================
        self.SEARCH_AREA_WIDTH_M = float(os.getenv('SEARCH_AREA_WIDTH_M', 5.0))
        self.SEARCH_AREA_HEIGHT_M = float(os.getenv('SEARCH_AREA_HEIGHT_M', 5.0))
        self.SEARCH_ALTITUDE_M = float(os.getenv('SEARCH_ALTITUDE_M', 1.2))

        # ==================== GRID SEARCH PARAMETERS ====================
        self.GRID_CELL_SIZE_M = float(os.getenv('GRID_CELL_SIZE_M', 1.5))
        self.GRID_PATTERN = os.getenv('GRID_PATTERN', 'lawnmower')
        self.WAYPOINT_HOVER_TIME_SEC = float(os.getenv('WAYPOINT_HOVER_TIME_SEC', 2.0))
        self.ENABLE_ROTATION_SCAN = os.getenv('ENABLE_ROTATION_SCAN', 'true').lower() == 'true'
        self.ROTATION_SCAN_STEPS = int(os.getenv('ROTATION_SCAN_STEPS', 4))

        # ==================== ARUCO DETECTION ====================
        self.ARUCO_DICT = os.getenv('ARUCO_DICT', 'DICT_4X4_50')
        self.TAG_0_SIZE_M = float(os.getenv('TAG_0_SIZE_M', 0.1))  # 10 cm
        self.TAG_1_SIZE_M = float(os.getenv('TAG_1_SIZE_M', 0.025))  # 2.5 cm
        self.MIN_DETECTION_CONFIDENCE = float(os.getenv('MIN_DETECTION_CONFIDENCE', 0.8))
        self.DETECTION_FRAME_INTERVAL = int(os.getenv('DETECTION_FRAME_INTERVAL', 3))

        # ==================== TELLO CAMERA CALIBRATION ====================
        # Estimated intrinsics for Tello 320x240 camera
        # FOV: ~82.6° horizontal
        fx = float(os.getenv('TELLO_FX', 290.0))
        fy = float(os.getenv('TELLO_FY', 290.0))
        cx = float(os.getenv('TELLO_CX', 160.0))
        cy = float(os.getenv('TELLO_CY', 120.0))

        self.CAMERA_MATRIX = np.array([
            [fx,   0.0, cx],
            [0.0,  fy,  cy],
            [0.0,  0.0, 1.0]
        ], dtype=np.float32)

        k1 = float(os.getenv('TELLO_K1', 0.0))
        k2 = float(os.getenv('TELLO_K2', 0.0))
        p1 = float(os.getenv('TELLO_P1', 0.0))
        p2 = float(os.getenv('TELLO_P2', 0.0))
        k3 = float(os.getenv('TELLO_K3', 0.0))

        self.DIST_COEFFS = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        self.CAMERA_WIDTH = 320
        self.CAMERA_HEIGHT = 240

        # ==================== PATH OPTIMIZATION ====================
        self.OPTIMIZATION_ALGORITHM = os.getenv('OPTIMIZATION_ALGORITHM', 'astar')
        self.OBSTACLE_PENALTY_WEIGHT = float(os.getenv('OBSTACLE_PENALTY_WEIGHT', 2.0))
        self.SMOOTHING_ENABLED = os.getenv('SMOOTHING_ENABLED', 'true').lower() == 'true'
        self.MIN_WAYPOINT_DISTANCE_CM = float(os.getenv('MIN_WAYPOINT_DISTANCE_CM', 30))

        # Grid discretization for A*
        self.GRID_CELL_SIZE_CM = float(os.getenv('GRID_CELL_SIZE_CM', 20))  # 20cm cells

        # Movement speed estimates for time calculation
        self.DRONE_SPEED_M_S = float(os.getenv('DRONE_SPEED_M_S', 0.5))  # 0.5 m/s
        self.COMMAND_OVERHEAD_SEC = float(os.getenv('COMMAND_OVERHEAD_SEC', 0.3))
        self.ROTATION_SPEED_DEG_S = float(os.getenv('ROTATION_SPEED_DEG_S', 180.0))

        # ==================== SAFETY & TIMEOUTS ====================
        self.MAX_SEARCH_TIME_SEC = int(os.getenv('MAX_SEARCH_TIME_SEC', 540))  # 9 minutes
        self.MIN_BATTERY_FOR_SEARCH = int(os.getenv('MIN_BATTERY_FOR_SEARCH', 60))
        self.EXPLORATION_TIMEOUT_SEC = int(os.getenv('EXPLORATION_TIMEOUT_SEC', 600))  # 10 min
        self.REQUIRE_USER_CONFIRMATION = os.getenv('REQUIRE_USER_CONFIRMATION', 'true').lower() == 'true'

        # ==================== VISUALIZATION ====================
        self.SAVE_VISUALIZATION = os.getenv('SAVE_VISUALIZATION', 'true').lower() == 'true'
        self.SHOW_LIVE_PLOT = os.getenv('SHOW_LIVE_PLOT', 'true').lower() == 'true'
        self.PLOT_DPI = int(os.getenv('PLOT_DPI', 150))
        self.PLOT_FORMAT = os.getenv('PLOT_FORMAT', 'png')

        # ==================== LOGGING ====================
        self.LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO')
        self.SAVE_MOVEMENT_LOG = os.getenv('SAVE_MOVEMENT_LOG', 'true').lower() == 'true'
        self.LOG_DIRECTORY = os.getenv('LOG_DIRECTORY', './logs/path_planning/')

    def get_marker_sizes(self):
        """
        Get dictionary mapping tag IDs to their physical sizes.

        Returns:
            Dict[int, float]: Tag ID -> size in meters
        """
        return {
            0: self.TAG_0_SIZE_M,
            1: self.TAG_1_SIZE_M,
        }

    def get_search_area_cm(self):
        """
        Get search area dimensions in centimeters.

        Returns:
            Tuple[float, float]: (width_cm, height_cm)
        """
        return (
            self.SEARCH_AREA_WIDTH_M * 100,
            self.SEARCH_AREA_HEIGHT_M * 100
        )

    def __str__(self):
        """String representation of configuration"""
        return f"""PathPlanningConfig:
  Search Area: {self.SEARCH_AREA_WIDTH_M}m x {self.SEARCH_AREA_HEIGHT_M}m
  Grid Cell Size: {self.GRID_CELL_SIZE_M}m
  Search Altitude: {self.SEARCH_ALTITUDE_M}m
  ArUco Dict: {self.ARUCO_DICT}
  Tag Sizes: {self.TAG_0_SIZE_M}m, {self.TAG_1_SIZE_M}m
  Camera: {self.CAMERA_WIDTH}x{self.CAMERA_HEIGHT}
  Optimization: {self.OPTIMIZATION_ALGORITHM}
  Max Search Time: {self.MAX_SEARCH_TIME_SEC}s
"""
