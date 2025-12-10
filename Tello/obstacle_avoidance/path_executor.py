"""
Path execution coordinator with obstacle avoidance and dead reckoning.

This module orchestrates autonomous navigation, continuously monitoring for
obstacles and executing circumvention maneuvers while tracking position.
"""

import math
import time
from typing import List, Tuple, Dict, Optional

from .config import Config
from .video_processor import VideoProcessor
from .obstacle_detector import ObstacleDetector
from .circumvent import ObstacleCircumvention


class PositionEstimator:
    """Track drone position using dead reckoning with yaw compensation"""

    def __init__(self):
        self.pos = {"x": 0.0, "y": 0.0, "z": 0.0}  # cm from origin
        self.yaw = 0.0  # degrees from telemetry (0° = North/Forward)

    def get_position(self) -> Dict[str, float]:
        """Get current position estimate"""
        return self.pos.copy()

    def reset_position(self, x: float = 0, y: float = 0, z: float = 0):
        """Reset position to specified coordinates"""
        self.pos = {"x": x, "y": y, "z": z}
        print(f"[POSITION] Reset to ({x:.1f}, {y:.1f}, {z:.1f})")

    def update_from_command(self, command: str, params: Dict):
        """
        Update position after movement command - accounts for yaw.

        Args:
            command: Movement command string
            params: Command parameters (e.g., {"distance": 50})
        """
        dist = params.get("distance", 0)

        if command == "move_forward":
            # Move in current heading direction
            self.pos["x"] += dist * math.cos(math.radians(self.yaw))
            self.pos["y"] += dist * math.sin(math.radians(self.yaw))

        elif command == "move_back":
            # Move opposite to current heading
            self.pos["x"] -= dist * math.cos(math.radians(self.yaw))
            self.pos["y"] -= dist * math.sin(math.radians(self.yaw))

        elif command == "move_left":
            # Move perpendicular left to current heading
            self.pos["x"] += dist * math.cos(math.radians(self.yaw - 90))
            self.pos["y"] += dist * math.sin(math.radians(self.yaw - 90))

        elif command == "move_right":
            # Move perpendicular right to current heading
            self.pos["x"] += dist * math.cos(math.radians(self.yaw + 90))
            self.pos["y"] += dist * math.sin(math.radians(self.yaw + 90))

        elif command == "move_up":
            self.pos["z"] += dist

        elif command == "move_down":
            self.pos["z"] -= dist

        elif command == "rotate_cw":
            self.yaw = (self.yaw + params.get("degrees", 0)) % 360

        elif command == "rotate_ccw":
            self.yaw = (self.yaw - params.get("degrees", 0)) % 360

    def update_yaw(self, yaw_from_telemetry: float):
        """
        Update yaw from drone's IMU - more accurate than dead reckoning.

        Args:
            yaw_from_telemetry: Yaw angle in degrees from drone
        """
        self.yaw = yaw_from_telemetry

    def distance_to(self, target_pos: Tuple[float, float, float]) -> float:
        """
        Calculate 3D distance to target position.

        Args:
            target_pos: Target (x, y, z) coordinates in cm

        Returns:
            Distance in cm
        """
        dx = target_pos[0] - self.pos["x"]
        dy = target_pos[1] - self.pos["y"]
        dz = target_pos[2] - self.pos["z"]
        return math.sqrt(dx**2 + dy**2 + dz**2)


class PathExecutor:
    """Main path execution coordinator with obstacle avoidance"""

    def __init__(self, drone_controller, state_manager, video_handler, config: Config = Config()):
        """
        Initialize path executor.

        Args:
            drone_controller: DroneController instance from tello_server
            state_manager: StateManager instance from tello_server
            video_handler: VideoStreamHandler instance from tello_server
            config: Configuration object
        """
        self.drone = drone_controller
        self.state = state_manager
        self.config = config

        # Initialize components
        self.position = PositionEstimator()
        self.video_proc = VideoProcessor(video_handler)
        self.detector = ObstacleDetector(config)
        self.circumvent = ObstacleCircumvention(drone_controller, self.position, config)

        self.running = False
        self.start_time = None

    def safety_check(self) -> bool:
        """
        Check safety conditions (battery, flight time, etc.).

        Returns:
            True if safe to continue, False otherwise
        """
        state = self.state.get_state()

        # Check battery
        battery = state.get("battery", 0)
        if battery < self.config.MIN_BATTERY_PERCENT:
            print(f"\n[!] CRITICAL: Battery at {battery}% - Emergency landing!")
            self.drone.send_command("land")
            return False
        elif battery < self.config.MIN_BATTERY_WARNING:
            print(f"[!] WARNING: Battery low ({battery}%)")

        # Check flight time
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > self.config.MAX_FLIGHT_TIME:
                print(f"\n[!] Max flight time exceeded ({elapsed:.0f}s) - Landing!")
                self.drone.send_command("land")
                return False

        return True

    def at_waypoint(self, waypoint: Tuple[float, float, float]) -> bool:
        """
        Check if drone is at waypoint.

        Args:
            waypoint: Target (x, y, z) coordinates

        Returns:
            True if within tolerance of waypoint
        """
        distance = self.position.distance_to(waypoint)
        return distance < self.config.WAYPOINT_TOLERANCE

    def find_highest_threat(self, obstacles: List[Dict]) -> Optional[Dict]:
        """
        Find the obstacle with highest threat level.

        Args:
            obstacles: List of detected obstacles

        Returns:
            Highest threat obstacle, or None if no threats
        """
        high_threats = [obs for obs in obstacles if obs["threat_level"] == "high"]
        if high_threats:
            # Return closest high threat
            return min(high_threats, key=lambda x: x["distance_m"])

        medium_threats = [obs for obs in obstacles if obs["threat_level"] == "medium"]
        if medium_threats:
            return min(medium_threats, key=lambda x: x["distance_m"])

        return None

    def move_toward_waypoint(self, target: Tuple[float, float, float]) -> bool:
        """
        Execute one movement step toward target waypoint.

        Args:
            target: Target (x, y, z) coordinates

        Returns:
            True if movement successful
        """
        current = self.position.get_position()

        # Calculate deltas
        dx = target[0] - current["x"]
        dy = target[1] - current["y"]
        dz = target[2] - current["z"]

        # Prioritize vertical movement first (safer)
        if abs(dz) > self.config.WAYPOINT_TOLERANCE:
            distance = min(abs(dz), self.config.MAX_STEP_DISTANCE)
            distance = max(distance, self.config.MIN_STEP_DISTANCE)

            if dz > 0:
                command = "move_up"
            else:
                command = "move_down"

            success, msg = self.drone.send_command(command, distance=int(distance))
            if success:
                self.position.update_from_command(command, {"distance": int(distance)})
                time.sleep(0.5)
            return success

        # Then handle horizontal movement
        # Calculate distance and angle to target
        horizontal_dist = math.sqrt(dx**2 + dy**2)

        if horizontal_dist > self.config.WAYPOINT_TOLERANCE:
            # Calculate angle to target
            target_angle = math.degrees(math.atan2(dy, dx))
            angle_diff = target_angle - self.position.yaw

            # Normalize angle to [-180, 180]
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360

            # If we need to turn significantly, rotate first
            if abs(angle_diff) > 15:  # 15 degree threshold
                rotation = min(abs(angle_diff), 90)  # Max 90 degrees per step
                rotation = max(rotation, 30)  # Minimum 30 degrees to avoid too many small rotations

                print(f"[ROTATE] Current yaw: {self.position.yaw:.1f}°, Target angle: {target_angle:.1f}°, "
                      f"Need to rotate: {angle_diff:.1f}°, Will rotate: {rotation:.1f}°")

                if angle_diff > 0:
                    success, msg = self.drone.send_command("rotate_ccw", degrees=int(rotation))
                    if success:
                        self.position.update_from_command("rotate_ccw", {"degrees": int(rotation)})
                else:
                    success, msg = self.drone.send_command("rotate_cw", degrees=int(rotation))
                    if success:
                        self.position.update_from_command("rotate_cw", {"degrees": int(rotation)})

                # Wait longer for rotation to complete and IMU to stabilize
                time.sleep(1.0)
                return success

            # Move forward toward target
            distance = min(horizontal_dist, self.config.MAX_STEP_DISTANCE)
            distance = max(distance, self.config.MIN_STEP_DISTANCE)

            success, msg = self.drone.send_command("move_forward", distance=int(distance))
            if success:
                self.position.update_from_command("move_forward", {"distance": int(distance)})
                time.sleep(0.5)
            return success

        return True

    def execute_path(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """
        Main path following with obstacle avoidance.

        Args:
            waypoints: List of (x, y, z) waypoint coordinates in cm

        Returns:
            True if path completed successfully, False otherwise
        """
        if not waypoints:
            print("[!] No waypoints provided")
            return False

        self.running = True
        self.start_time = time.time()

        print("\n" + "="*60)
        print("PATH EXECUTION STARTED")
        print("="*60)
        print(f"Total waypoints: {len(waypoints)}")
        print(f"Path: {waypoints}")
        print("="*60 + "\n")

        try:
            # First waypoint - takeoff if needed
            if waypoints[0][2] > 0:
                print("[*] Taking off...")
                success, msg = self.drone.send_command("takeoff")
                if not success:
                    print(f"[!] Takeoff failed: {msg}")
                    return False
                print("[*] Waiting for IMU to stabilize after takeoff...")
                time.sleep(5)  # Wait for takeoff to complete AND IMU to stabilize
                print("[+] IMU stable, ready for navigation")
                self.position.reset_position(0, 0, waypoints[0][2])

            # Navigate to each waypoint
            for i, waypoint in enumerate(waypoints):
                if not self.running:
                    print("\n[!] Path execution stopped")
                    break

                print(f"\n{'='*60}")
                print(f"WAYPOINT {i+1}/{len(waypoints)}: ({waypoint[0]:.0f}, {waypoint[1]:.0f}, {waypoint[2]:.0f})")
                print(f"{'='*60}")

                # Special handling for landing waypoint (z=0)
                if waypoint[2] == 0:
                    print("[*] Landing...")
                    self.drone.send_command("land")
                    time.sleep(3)
                    print("[+] Landed successfully")
                    continue

                # Navigate to waypoint with obstacle avoidance
                while not self.at_waypoint(waypoint):
                    if not self.safety_check():
                        print("\n[!] Safety check failed - aborting")
                        return False

                    # Update yaw from telemetry (only if telemetry is non-zero)
                    # This prevents overwriting our dead-reckoning yaw with stale telemetry
                    state = self.state.get_state()
                    telemetry_yaw = state.get("orientation", {}).get("yaw", 0)
                    # Only trust telemetry if it's significantly different (IMU has caught up)
                    if abs(telemetry_yaw - self.position.yaw) > 5 or abs(self.position.yaw) < 1:
                        self.position.update_yaw(telemetry_yaw)

                    # Get current position
                    current_pos = self.position.get_position()
                    distance_remaining = self.position.distance_to(waypoint)

                    if self.config.LOG_POSITION_UPDATES:
                        print(f"[POS] ({current_pos['x']:.1f}, {current_pos['y']:.1f}, {current_pos['z']:.1f}) "
                              f"-> Target: {distance_remaining:.1f}cm away")

                    # Check for obstacles
                    frame = self.video_proc.get_latest_frame()
                    if frame is not None:
                        obstacles = self.detector.process_frame(frame)

                        if obstacles:
                            threat = self.find_highest_threat(obstacles)
                            if threat and threat["threat_level"] == "high":
                                print(f"\n[!] OBSTACLE DETECTED: {threat['class']} at {threat['distance_m']:.2f}m")
                                print(f"    Position: {threat['position']}, Threat: {threat['threat_level']}")

                                # Execute avoidance maneuver
                                if not self.circumvent.execute_maneuver(threat):
                                    print("[!] Circumvention failed")
                                    return False

                                # After avoidance, continue to next iteration
                                continue

                    # No obstacles - move toward waypoint
                    if not self.move_toward_waypoint(waypoint):
                        print("[!] Movement failed")
                        return False

                    time.sleep(0.1)  # Small delay between movements

                print(f"[+] Reached waypoint {i+1}/{len(waypoints)}")

            # Path completed
            print("\n" + "="*60)
            print("PATH EXECUTION COMPLETED SUCCESSFULLY")
            print("="*60)
            elapsed = time.time() - self.start_time
            print(f"Total time: {elapsed:.1f}s")
            print(f"Final position: ({current_pos['x']:.1f}, {current_pos['y']:.1f}, {current_pos['z']:.1f})")

            return True

        except KeyboardInterrupt:
            print("\n[!] Keyboard interrupt - Emergency landing")
            self.drone.send_command("land")
            return False

        except Exception as e:
            print(f"\n[!] Error during path execution: {e}")
            import traceback
            traceback.print_exc()
            print("[!] Emergency landing")
            self.drone.send_command("land")
            return False

        finally:
            self.running = False

    def stop(self):
        """Stop path execution"""
        print("\n[*] Stopping path execution...")
        self.running = False
