"""
Obstacle circumvention logic with position compensation.

This module generates and executes avoidance maneuvers that ensure the drone
returns to the exact position it would have been at if no obstacle existed.
"""

import time
from typing import List, Tuple, Dict

from .config import Config


class ObstacleCircumvention:
    """
    Generate and execute obstacle avoidance maneuvers.

    CRITICAL: All maneuvers are designed to return drone to exact planned path position.
    """

    def __init__(self, drone_controller, position_estimator, config: Config = Config()):
        """
        Initialize circumvention system.

        Args:
            drone_controller: DroneController instance from tello_server
            position_estimator: PositionEstimator instance for tracking
            config: Configuration object
        """
        self.drone = drone_controller
        self.position = position_estimator
        self.config = config
        self.state = "IDLE"

    def assess_threat(self, obstacle: Dict) -> str:
        """
        Determine if obstacle requires avoidance.

        Args:
            obstacle: Obstacle dictionary with distance and position

        Returns:
            Threat level: "high", "medium", or "low"
        """
        return obstacle.get("threat_level", "low")

    def calculate_pass_distance(self, obstacle_distance_m: float, obstacle_width_m: float) -> int:
        """
        Calculate how far forward to move to clear obstacle.

        Args:
            obstacle_distance_m: Distance to obstacle in meters
            obstacle_width_m: Width of obstacle in meters

        Returns:
            Pass distance in centimeters
        """
        # Convert to cm
        obstacle_dist_cm = obstacle_distance_m * 100
        obstacle_width_cm = obstacle_width_m * 100

        # Need to travel far enough to clear obstacle + margin
        # Use obstacle distance + half its width + clearance margin
        pass_distance = obstacle_dist_cm + (obstacle_width_cm / 2) + self.config.CLEARANCE_MARGIN

        # Clamp to reasonable bounds
        pass_distance = max(100, min(pass_distance, 300))  # 1m to 3m

        return int(pass_distance)

    def plan_lateral_dodge(self, obstacle: Dict) -> List[Tuple[str, int]]:
        """
        Generate mathematically balanced left/right dodge maneuver.

        The maneuver ensures that lateral movement cancels out perfectly:
        - Move LEFT by distance D
        - Move FORWARD to pass obstacle
        - Move RIGHT by distance D  -> Net lateral movement = 0!

        Args:
            obstacle: Obstacle dictionary

        Returns:
            List of (command, distance) tuples
        """
        # Determine dodge direction based on obstacle position
        position = obstacle.get("position", "center")

        if position == "left":
            # Obstacle on left, dodge right
            dodge_dir = "move_right"
            return_dir = "move_left"
        else:
            # Obstacle in center or right, dodge left
            dodge_dir = "move_left"
            return_dir = "move_right"

        # Calculate pass distance
        pass_dist = self.calculate_pass_distance(
            obstacle["distance_m"],
            obstacle["width_m"]
        )

        # Create balanced maneuver
        maneuver = [
            (dodge_dir, self.config.LATERAL_DODGE_DISTANCE),
            ("move_forward", pass_dist),
            (return_dir, self.config.LATERAL_DODGE_DISTANCE),
        ]

        if self.config.VERBOSE_LOGGING:
            print(f"[MANEUVER] Lateral dodge: {dodge_dir[5:].upper()} {self.config.LATERAL_DODGE_DISTANCE}cm, "
                  f"FWD {pass_dist}cm, {return_dir[5:].upper()} {self.config.LATERAL_DODGE_DISTANCE}cm")

        return maneuver

    def plan_vertical_dodge(self, obstacle: Dict) -> List[Tuple[str, int]]:
        """
        Generate mathematically balanced up/down dodge maneuver.

        The maneuver ensures that vertical movement cancels out perfectly:
        - Move UP by distance D
        - Move FORWARD to pass obstacle
        - Move DOWN by distance D  -> Net vertical movement = 0!

        Args:
            obstacle: Obstacle dictionary

        Returns:
            List of (command, distance) tuples
        """
        # Calculate pass distance
        pass_dist = self.calculate_pass_distance(
            obstacle["distance_m"],
            obstacle["width_m"]
        )

        # Create balanced maneuver
        maneuver = [
            ("move_up", self.config.VERTICAL_DODGE_DISTANCE),
            ("move_forward", pass_dist),
            ("move_down", self.config.VERTICAL_DODGE_DISTANCE),
        ]

        if self.config.VERBOSE_LOGGING:
            print(f"[MANEUVER] Vertical dodge: UP {self.config.VERTICAL_DODGE_DISTANCE}cm, "
                  f"FWD {pass_dist}cm, DOWN {self.config.VERTICAL_DODGE_DISTANCE}cm")

        return maneuver

    def select_maneuver(self, obstacle: Dict) -> List[Tuple[str, int]]:
        """
        Choose appropriate avoidance maneuver based on obstacle characteristics.

        Args:
            obstacle: Obstacle dictionary

        Returns:
            List of (command, distance) tuples
        """
        # For most cases, use lateral dodge
        # Could implement smarter selection based on obstacle height/type later
        return self.plan_lateral_dodge(obstacle)

    def execute_with_verification(self, maneuver: List[Tuple[str, int]]) -> bool:
        """
        Execute avoidance maneuver and verify position compensation.

        Args:
            maneuver: List of (command, distance) tuples

        Returns:
            True if successful, False otherwise
        """
        self.state = "EXECUTING_DODGE"

        # Record position before maneuver
        pos_before = self.position.get_position().copy()

        print(f"[CIRCUMVENT] Executing {len(maneuver)}-step maneuver...")

        # Execute all movements in sequence
        for i, (command, distance) in enumerate(maneuver, 1):
            print(f"  Step {i}/{len(maneuver)}: {command} {distance}cm")

            success, msg = self.drone.send_command(command, distance=distance)

            if not success:
                print(f"[!] Maneuver failed at step {i}: {msg}")
                self.state = "IDLE"
                return False

            # Update position estimate
            self.position.update_from_command(command, {"distance": distance})

            # Allow drone to stabilize between movements
            time.sleep(0.8)

        # Verify position after maneuver
        pos_after = self.position.get_position()

        # Calculate position error (Y and Z should be unchanged)
        y_error = abs(pos_after["y"] - pos_before["y"])
        z_error = abs(pos_after["z"] - pos_before["z"])
        x_progress = pos_after["x"] - pos_before["x"]

        print(f"[POSITION] Forward progress: {x_progress:.1f}cm, "
              f"Y drift: {y_error:.1f}cm, Z drift: {z_error:.1f}cm")

        # Warn if significant drift detected
        if y_error > 10 or z_error > 10:
            print(f"[!] WARNING: Position drift exceeds tolerance!")
            print(f"    Expected Y drift: 0cm, Actual: {y_error:.1f}cm")
            print(f"    Expected Z drift: 0cm, Actual: {z_error:.1f}cm")

        self.state = "IDLE"
        print("[+] Circumvention maneuver completed")

        return True

    def execute_maneuver(self, obstacle: Dict) -> bool:
        """
        Main entry point - assess threat and execute appropriate maneuver.

        Args:
            obstacle: Obstacle dictionary

        Returns:
            True if maneuver executed successfully, False otherwise
        """
        # Assess threat level
        threat = self.assess_threat(obstacle)

        if threat != "high":
            if self.config.VERBOSE_LOGGING:
                print(f"[CIRCUMVENT] Obstacle threat level '{threat}' - no action needed")
            return True

        # Select and execute maneuver
        print(f"\n[!] HIGH THREAT: {obstacle['class']} at {obstacle['distance_m']:.2f}m, "
              f"position {obstacle['position']}")

        maneuver = self.select_maneuver(obstacle)

        return self.execute_with_verification(maneuver)

    def emergency_stop(self):
        """Execute emergency stop (hover in place)"""
        print("[!!!] EMERGENCY STOP")
        self.state = "EMERGENCY"
        # Drone will naturally hover when no commands sent
        time.sleep(1.0)
        self.state = "IDLE"
