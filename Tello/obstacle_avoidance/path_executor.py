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
from tello_commands import TelloSafeCommands


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
            # With 0° pointing along +Y axis: x = dist * sin(yaw), y = dist * cos(yaw)
            self.pos["x"] += dist * math.sin(math.radians(self.yaw))
            self.pos["y"] += dist * math.cos(math.radians(self.yaw))

        elif command == "move_back":
            # Move opposite to current heading
            self.pos["x"] -= dist * math.sin(math.radians(self.yaw))
            self.pos["y"] -= dist * math.cos(math.radians(self.yaw))

        elif command == "move_left":
            # Move perpendicular left to current heading
            self.pos["x"] += dist * math.sin(math.radians(self.yaw - 90))
            self.pos["y"] += dist * math.cos(math.radians(self.yaw - 90))

        elif command == "move_right":
            # Move perpendicular right to current heading
            self.pos["x"] += dist * math.sin(math.radians(self.yaw + 90))
            self.pos["y"] += dist * math.cos(math.radians(self.yaw + 90))

        elif command == "move_up":
            self.pos["z"] += dist

        elif command == "move_down":
            self.pos["z"] -= dist

        elif command == "rotate_cw":
            # Disabled: yaw tracking unreliable, trust rotation command without verification
            # self.yaw = (self.yaw + params.get("degrees", 0)) % 360
            pass

        elif command == "rotate_ccw":
            # Disabled: yaw tracking unreliable, trust rotation command without verification
            # self.yaw = (self.yaw - params.get("degrees", 0)) % 360
            pass

    def update_yaw(self, yaw_from_telemetry: float):
        """
        Update yaw from external source.

        Args:
            yaw_from_telemetry: Yaw angle in degrees
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

    def __init__(self, tello, video_handler, config: Config = Config()):
        """
        Initialize path executor.

        Args:
            tello: DJITelloPy Tello instance
            video_handler: VideoStreamHandler instance
            config: Configuration object
        """
        self.tello = tello
        self.config = config

        # Initialize components
        self.position = PositionEstimator()
        self.video_proc = VideoProcessor(video_handler)
        self.detector = ObstacleDetector(config)
        self.circumvent = ObstacleCircumvention(tello, self.position, config)

        self.running = False
        self.start_time = None
        self.waypoint_callback = None  # Callback for waypoint updates
        self.rotated_for_waypoint = False  # Track if we've rotated for current waypoint

    def set_waypoint_callback(self, callback):
        """Set callback function for waypoint progress updates"""
        self.waypoint_callback = callback

    def safety_check(self) -> bool:
        """
        Check safety conditions (battery, flight time, etc.).

        Returns:
            True if safe to continue, False otherwise
        """
        # Check battery
        try:
            battery = self.tello.get_battery()
        except:
            battery = 100  # Assume OK if can't read

        if battery < self.config.MIN_BATTERY_PERCENT:
            print(f"\n[!] CRITICAL: Battery at {battery}% - Emergency landing!")
            self.tello.land()
            return False
        elif battery < self.config.MIN_BATTERY_WARNING:
            print(f"[!] WARNING: Battery low ({battery}%)")

        # Check flight time
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > self.config.MAX_FLIGHT_TIME:
                print(f"\n[!] Max flight time exceeded ({elapsed:.0f}s) - Landing!")
                self.tello.land()
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
            # Use very slow vertical movements for better stability
            distance = min(abs(dz), 20)  # Max 20cm per step (slow for stability)
            distance = max(distance, self.config.MIN_STEP_DISTANCE)

            try:
                if dz > 0:
                    TelloSafeCommands.move_up(self.tello, int(distance))
                else:
                    TelloSafeCommands.move_down(self.tello, int(distance))
                self.position.update_from_command("move_up" if dz > 0 else "move_down", {"distance": int(distance)})
                return True
            except Exception as e:
                print(f"[!] Vertical movement failed: {e}")
                return False

        # Then handle horizontal movement
        # Calculate distance and angle to target
        horizontal_dist = math.sqrt(dx**2 + dy**2)

        if horizontal_dist > self.config.WAYPOINT_TOLERANCE:
            # Rotate once at the start to face target, then just move forward
            # Only rotate if we haven't already rotated for this waypoint
            if not self.rotated_for_waypoint:
                # Calculate angle to target
                # Adjust coordinate frame: 0° should point along +Y axis (forward), not +X axis
                # Standard atan2(dy, dx) gives 0° for +X, but we want 0° for +Y
                # So we use atan2(dx, dy) which gives 0° for +Y, 90° for +X
                target_angle = math.degrees(math.atan2(dx, dy))
                angle_diff = target_angle - self.position.yaw

                # Normalize angle to [-180, 180] using robust modulo arithmetic
                angle_diff = ((angle_diff + 180) % 360) - 180

                # Rotate to face target - trust it works, don't verify
                ROTATION_THRESHOLD = 5  # degrees - rotate even for small angles
                if abs(angle_diff) > ROTATION_THRESHOLD:
                    print(f"[ROTATE] Rotating {angle_diff:.1f}° to face target")
                    try:
                        if angle_diff > 0:
                            TelloSafeCommands.rotate_counter_clockwise(self.tello, int(abs(angle_diff)))
                        else:
                            TelloSafeCommands.rotate_clockwise(self.tello, int(abs(angle_diff)))
                        
                        # Update estimated yaw and mark rotation complete
                        self.position.yaw = target_angle
                        print(f"[ROTATE] Rotation complete, now facing target")
                    except Exception as e:
                        print(f"[!] Rotation failed: {e}")
                        return False
                
                # Mark that we've rotated for this waypoint
                self.rotated_for_waypoint = True

            # Move forward toward target
            # Use slow horizontal movements for better accuracy
            distance = min(horizontal_dist, 20)  # Max 20cm per step
            distance = max(distance, self.config.MIN_STEP_DISTANCE)

            try:
                TelloSafeCommands.move_forward(self.tello, int(distance))
                self.position.update_from_command("move_forward", {"distance": int(distance)})
                return True
            except Exception as e:
                print(f"[!] Forward movement failed: {e}")
                return False

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

        # Notify dashboard of waypoints
        if self.waypoint_callback:
            self.waypoint_callback(waypoints, 0)

        # Initialize current_pos to avoid undefined variable errors
        current_pos = self.position.get_position()

        try:
            # First waypoint - takeoff if needed
            if waypoints[0][2] > 0:
                print("\n" + "="*60)
                print("PRE-TAKEOFF DIAGNOSTICS")
                print("="*60)

                # Get current drone state
                try:
                    battery = self.tello.get_battery()
                    height = self.tello.get_height()
                    temp = self.tello.get_temperature()
                    print(f"[STATE] Battery: {battery}%")
                    print(f"[STATE] Height: {height}cm")
                    print(f"[STATE] Temperature: {temp}°C")

                    # Check if drone is already flying
                    if height > 10:
                        print(f"[WARN] Drone reports non-zero height: {height}cm")
                        print("[WARN] Drone may already be airborne or sensor issue")
                except Exception as e:
                    print(f"[WARN] Could not read some telemetry: {e}")

                print(f"\n[PLAN] Target takeoff altitude: {waypoints[0][2]}cm")
                print("="*60 + "\n")

                print("[*] Initiating takeoff sequence...")
                try:
                    self.tello.takeoff()
                    print("[+] Takeoff command accepted")
                except Exception as e:
                    print(f"\n[!] TAKEOFF FAILED: {e}")
                    print("[!] Possible reasons:")
                    print("    1. Drone hardware issue (motors, propellers)")
                    print("    2. Battery too low to take off")
                    print("    3. Drone not on flat surface")
                    print("    4. Communication timeout")
                    print("    5. Drone in error state - try restarting")

                    # Additional diagnostics
                    print("\n[DIAG] Post-failure drone state:")
                    try:
                        battery = self.tello.get_battery()
                        height = self.tello.get_height()
                        print(f"[DIAG] Battery: {battery}%")
                        print(f"[DIAG] Height: {height}cm")
                    except:
                        print("[DIAG] Could not read drone state")
                    return False
                print("\n[*] Waiting for drone to stabilize after takeoff...")
                print("[INFO] Monitoring drone stabilization...")

                # Monitor stabilization with progress
                for i in range(5, 0, -1):
                    time.sleep(1)
                    try:
                        height = self.tello.get_height()
                        battery = self.tello.get_battery()
                        print(f"[{6-i}/5] Height: {height}cm, Battery: {battery}%, {i}s remaining...")
                    except:
                        print(f"[{6-i}/5] {i}s remaining...")

                print("[+] Stabilization complete")

                # Verify we're actually airborne
                try:
                    final_height = self.tello.get_height()
                    print(f"\n[VERIFY] Final height after stabilization: {final_height}cm")

                    if final_height < 20:
                        print(f"[WARN] Height is lower than expected ({final_height}cm)")
                        print("[WARN] Drone may not have taken off properly")
                        print("[WARN] Proceeding anyway, but watch for issues...")
                    else:
                        print(f"[SUCCESS] Drone confirmed airborne at {final_height}cm")
                except:
                    print("[WARN] Could not verify height after takeoff")

                print("[+] Ready for navigation")
                print("="*60 + "\n")
                self.position.reset_position(0, 0, waypoints[0][2])

            # Navigate to each waypoint
            for i, waypoint in enumerate(waypoints):
                if not self.running:
                    print("\n[!] Path execution stopped")
                    break

                # Reset rotation flag for new waypoint
                self.rotated_for_waypoint = False

                # Notify waypoint progress
                if self.waypoint_callback:
                    self.waypoint_callback(waypoints, i)

                print(f"\n{'='*60}")
                print(f"WAYPOINT {i+1}/{len(waypoints)}: ({waypoint[0]:.0f}, {waypoint[1]:.0f}, {waypoint[2]:.0f})")
                print(f"{'='*60}")

                # Special handling for landing waypoint (z=0)
                if waypoint[2] == 0:
                    print("[*] Landing...")
                    try:
                        self.tello.land()
                        time.sleep(3)
                        print("[+] Landed successfully")
                    except Exception as e:
                        print(f"[!] Landing failed: {e}")
                    continue

                # Navigate to waypoint with obstacle avoidance
                while not self.at_waypoint(waypoint):
                    if not self.safety_check():
                        print("\n[!] Safety check failed - aborting")
                        return False

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
            try:
                self.tello.land()
            except:
                pass
            return False

        except Exception as e:
            print(f"\n[!] Error during path execution: {e}")
            import traceback
            traceback.print_exc()
            print("[!] Emergency landing")
            try:
                self.tello.land()
            except:
                pass
            return False

        finally:
            self.running = False

    def stop(self):
        """Stop path execution"""
        print("\n[*] Stopping path execution...")
        self.running = False
