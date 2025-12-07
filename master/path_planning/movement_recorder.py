"""
Movement recording system for exploration phase.

Records ALL movements, positions, rotations, obstacles, and ArUco detections
during the exploration phase for later path optimization.
"""

import json
import time
import uuid
import os
from typing import Dict, List, Optional
from datetime import datetime
from .config import PathPlanningConfig


class MovementRecorder:
    """Records complete movement history during exploration"""

    def __init__(self, config: PathPlanningConfig):
        """
        Initialize movement recorder.

        Args:
            config: PathPlanningConfig instance
        """
        self.config = config
        self.mission_id = str(uuid.uuid4())[:8]
        self.start_time = None
        self.end_time = None

        # Recording state
        self.movements = []
        self.aruco_detections = []
        self.obstacles_encountered = []

        # Current movement being recorded
        self.current_movement = None
        self.sequence_id = 0

        # Statistics
        self.total_distance_cm = 0.0
        self.obstacle_avoidance_count = 0
        self.start_battery = None
        self.end_battery = None

    def start_mission(self, search_area_width_m: float, search_area_height_m: float, battery_percent: int):
        """
        Start recording a new mission.

        Args:
            search_area_width_m: Search area width in meters
            search_area_height_m: Search area height in meters
            battery_percent: Starting battery percentage
        """
        self.start_time = time.time()
        self.start_battery = battery_percent
        self.search_area = {
            'width_m': search_area_width_m,
            'height_m': search_area_height_m
        }

        print(f"[RECORDER] Mission {self.mission_id} started at {datetime.now().strftime('%H:%M:%S')}")
        print(f"[RECORDER] Search area: {search_area_width_m}m x {search_area_height_m}m")
        print(f"[RECORDER] Starting battery: {battery_percent}%")

    def record_movement_start(self, command: str, params: Dict, position_before: Dict, yaw_before: float):
        """
        Record the start of a movement command.

        Args:
            command: Command name (e.g., 'move_forward', 'rotate_cw')
            params: Command parameters (e.g., {'distance': 100})
            position_before: Position dict {x, y, z} before movement
            yaw_before: Yaw angle before movement
        """
        self.current_movement = {
            'sequence_id': self.sequence_id,
            'timestamp': time.time(),
            'command': command,
            'params': params.copy(),
            'position_before': position_before.copy(),
            'yaw_before': yaw_before,
            'position_after': None,  # Will be filled in record_movement_end
            'yaw_after': None,
            'duration_sec': None,
            'success': None,
            'type': 'planned',  # Will be changed to 'obstacle_avoidance' if needed
            'obstacle_triggered': None
        }

    def record_movement_end(self, success: bool, position_after: Dict, yaw_after: float, duration: float):
        """
        Record the end of a movement command.

        Args:
            success: Whether command succeeded
            position_after: Position dict {x, y, z} after movement
            yaw_after: Yaw angle after movement
            duration: Command duration in seconds
        """
        if self.current_movement is None:
            print("[RECORDER WARNING] record_movement_end called without start")
            return

        self.current_movement['position_after'] = position_after.copy()
        self.current_movement['yaw_after'] = yaw_after
        self.current_movement['duration_sec'] = duration
        self.current_movement['success'] = success

        # Calculate distance traveled
        if self.current_movement['command'] in ['move_forward', 'move_back', 'move_left', 'move_right', 'move_up', 'move_down']:
            pos_before = self.current_movement['position_before']
            pos_after = self.current_movement['position_after']
            import math
            distance = math.sqrt(
                (pos_after['x'] - pos_before['x'])**2 +
                (pos_after['y'] - pos_before['y'])**2 +
                (pos_after['z'] - pos_before['z'])**2
            )
            self.total_distance_cm += distance

        # Add to movements list
        self.movements.append(self.current_movement)
        self.sequence_id += 1
        self.current_movement = None

    def mark_as_obstacle_avoidance(self, obstacle_info: Dict, avoidance_type: str):
        """
        Mark current/last movement as obstacle avoidance maneuver.

        Args:
            obstacle_info: Dict with obstacle detection info
            avoidance_type: Type of avoidance (e.g., 'lateral_dodge_left')
        """
        if self.current_movement is not None:
            # Current movement being recorded
            self.current_movement['type'] = 'obstacle_avoidance'
            self.current_movement['obstacle_triggered'] = obstacle_info.copy()
        elif len(self.movements) > 0:
            # Mark most recent movement
            self.movements[-1]['type'] = 'obstacle_avoidance'
            self.movements[-1]['obstacle_triggered'] = obstacle_info.copy()

        # Record obstacle encounter
        self.obstacles_encountered.append({
            'timestamp': time.time(),
            'at_position': obstacle_info.get('position', {}),
            'obstacle': obstacle_info,
            'avoidance_action': avoidance_type,
            'sequence_id': self.sequence_id - 1
        })

        self.obstacle_avoidance_count += 1

    def record_aruco_detection(self, tag_info: Dict):
        """
        Record ArUco tag detection.

        Args:
            tag_info: Tag detection dict from ArucoDetector.detect_tags()
        """
        self.aruco_detections.append(tag_info.copy())

    def end_mission(self, battery_percent: int):
        """
        End mission recording.

        Args:
            battery_percent: Ending battery percentage
        """
        self.end_time = time.time()
        self.end_battery = battery_percent

        duration = self.end_time - self.start_time
        battery_used = self.start_battery - self.end_battery

        print(f"\n[RECORDER] Mission {self.mission_id} ended")
        print(f"[RECORDER] Duration: {duration:.1f}s ({duration/60:.1f} min)")
        print(f"[RECORDER] Total movements: {len(self.movements)}")
        print(f"[RECORDER] Distance traveled: {self.total_distance_cm:.0f} cm")
        print(f"[RECORDER] Obstacle avoidances: {self.obstacle_avoidance_count}")
        print(f"[RECORDER] ArUco detections: {len(self.aruco_detections)}")
        print(f"[RECORDER] Battery used: {battery_used}%")

    def get_movement_log(self) -> Dict:
        """
        Get complete movement log.

        Returns:
            Dict containing all recorded data
        """
        duration = 0
        if self.start_time and self.end_time:
            duration = self.end_time - self.start_time

        # Find unique tag detections (get confirmed positions)
        unique_tags_found = list(set(d['tag_id'] for d in self.aruco_detections))

        return {
            'mission_id': self.mission_id,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'search_area': self.search_area,

            'movements': self.movements,
            'aruco_detections': self.aruco_detections,
            'obstacles_encountered': self.obstacles_encountered,

            'statistics': {
                'total_distance_traveled_cm': self.total_distance_cm,
                'total_commands_executed': len(self.movements),
                'obstacle_avoidance_count': self.obstacle_avoidance_count,
                'exploration_duration_sec': duration,
                'tags_found': unique_tags_found,
                'tag_detection_count': len(self.aruco_detections),
                'battery_consumed_percent': (self.start_battery or 0) - (self.end_battery or 0)
            }
        }

    def save_to_file(self, filename: Optional[str] = None) -> str:
        """
        Save movement log to JSON file.

        Args:
            filename: Optional filename. If None, auto-generates based on mission ID.

        Returns:
            str: Path to saved file
        """
        if not self.config.SAVE_MOVEMENT_LOG:
            print("[RECORDER] Logging disabled in config, skipping save")
            return ""

        # Create log directory if it doesn't exist
        os.makedirs(self.config.LOG_DIRECTORY, exist_ok=True)

        # Generate filename
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"mission_{self.mission_id}_{timestamp}.json"

        filepath = os.path.join(self.config.LOG_DIRECTORY, filename)

        # Get log data
        log_data = self.get_movement_log()

        # Save to file
        with open(filepath, 'w') as f:
            json.dump(log_data, f, indent=2)

        print(f"[RECORDER] Movement log saved to: {filepath}")
        return filepath

    def get_statistics_summary(self) -> str:
        """
        Get formatted statistics summary.

        Returns:
            str: Formatted summary string
        """
        stats = self.get_movement_log()['statistics']

        summary = f"""
========== EXPLORATION STATISTICS ==========
Mission ID: {self.mission_id}
Duration: {stats['exploration_duration_sec']:.1f}s ({stats['exploration_duration_sec']/60:.1f} min)
Total Distance: {stats['total_distance_traveled_cm']:.0f} cm ({stats['total_distance_traveled_cm']/100:.1f} m)
Commands Executed: {stats['total_commands_executed']}
Obstacle Avoidances: {stats['obstacle_avoidance_count']}
ArUco Detections: {stats['tag_detection_count']}
Tags Found: {stats['tags_found']}
Battery Used: {stats['battery_consumed_percent']}%
==========================================
"""
        return summary

    def reset(self):
        """Reset recorder for new mission"""
        self.mission_id = str(uuid.uuid4())[:8]
        self.start_time = None
        self.end_time = None
        self.movements = []
        self.aruco_detections = []
        self.obstacles_encountered = []
        self.current_movement = None
        self.sequence_id = 0
        self.total_distance_cm = 0.0
        self.obstacle_avoidance_count = 0
        self.start_battery = None
        self.end_battery = None
