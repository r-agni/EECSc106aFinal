"""
Configuration parameters for path planning and obstacle avoidance system.
"""

import os


class Config:
    """Centralized configuration for all path planning parameters"""

    # ==================== CAMERA PARAMETERS ====================
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    FOCAL_LENGTH_PX = 700  # Initial estimate, calibrate for accuracy

    # ==================== DETECTION PARAMETERS ====================
    # YOLO model path - using local file in obstacle_avoidance folder
    YOLO_MODEL = os.path.join(os.path.dirname(__file__), "yolov8n.pt")
    DETECTION_CONFIDENCE = 0.5  # Minimum confidence threshold
    PROCESS_EVERY_N_FRAMES = 2  # Process every 2nd frame for performance

    # Known object heights (meters) for distance estimation
    OBJECT_HEIGHTS = {
        "person": 1.7,
        "chair": 0.9,
        "dining table": 0.75,
        "couch": 0.85,
        "potted plant": 0.6,
        "bottle": 0.25,
        "cup": 0.12,
        "book": 0.25,
        "laptop": 0.02,
        "cell phone": 0.15,
        "vase": 0.3,
        "backpack": 0.4,
        "handbag": 0.3,
        "suitcase": 0.6,
        "tv": 0.8,
        "generic": 1.0  # Default for unknown objects
    }

    # ==================== THREAT ASSESSMENT ====================
    THREAT_DISTANCE_HIGH = 1.5  # meters - immediate threat, must avoid
    THREAT_DISTANCE_MEDIUM = 2.5  # meters - monitor closely
    THREAT_DISTANCE_LOW = 4.0  # meters - aware but ignore

    # Camera zone boundaries (divide frame into thirds)
    ZONE_LEFT_BOUNDARY = CAMERA_WIDTH / 3
    ZONE_RIGHT_BOUNDARY = 2 * CAMERA_WIDTH / 3

    # ==================== CIRCUMVENTION PARAMETERS ====================
    LATERAL_DODGE_DISTANCE = 80  # cm - left/right dodge distance
    VERTICAL_DODGE_DISTANCE = 60  # cm - up/down dodge distance
    FORWARD_PASS_DISTANCE = 150  # cm - default distance to pass obstacle
    CLEARANCE_MARGIN = 30  # cm - extra safety margin

    # ==================== NAVIGATION PARAMETERS ====================
    WAYPOINT_TOLERANCE = 20  # cm - distance to consider "at waypoint"
    MAX_STEP_DISTANCE = 100  # cm - maximum single movement step
    MIN_STEP_DISTANCE = 20  # cm - minimum movement (Tello limitation)

    # ==================== SAFETY PARAMETERS ====================
    MIN_BATTERY_PERCENT = 20  # Auto-land below this
    MIN_BATTERY_WARNING = 30  # Warning threshold
    MAX_FLIGHT_TIME = 600  # seconds - 10 minutes max
    EMERGENCY_STOP_DISTANCE = 0.5  # meters - immediate emergency stop

    # ==================== LOGGING ====================
    VERBOSE_LOGGING = True
    LOG_DETECTIONS = True
    LOG_POSITION_UPDATES = False  # Can be noisy, disable for cleaner output
