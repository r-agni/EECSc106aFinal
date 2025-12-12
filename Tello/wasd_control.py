"""
WASD manual control mode for Tello drone with live video streaming and dashboard.

Usage:
    python wasd_control.py

Controls:
    W/S - Move forward/backward
    A/D - Move left/right
    Q/E - Rotate counter-clockwise/clockwise
    R/F - Move up/down
    T - Takeoff
    L - Land
    SPACE - Emergency stop
    ESC - Quit

This will:
    1. Connect to Tello drone
    2. Start video streaming (browser at http://localhost:8080)
    3. Display live navigation map at http://localhost:8081
    4. Allow manual control via WASD keys
    5. Track position using dead reckoning on the dashboard
"""

import sys
import time
import argparse
import math
import keyboard
import threading
from djitellopy import Tello
from drone_wrapper import TelloConnection, VideoStreamHandler
from obstacle_avoidance.path_executor import PositionEstimator
from obstacle_avoidance.config import Config
from obstacle_avoidance.obstacle_detector import ObstacleDetector
from obstacle_avoidance.video_processor import VideoProcessor
from visualization.web_dashboard import WebDashboard


class WASDController:
    """WASD manual control with live video and dashboard"""

    def __init__(self, enable_video_stream: bool = True, http_port: int = 8080,
                 enable_live_map: bool = True):
        """
        Initialize WASD controller.

        Args:
            enable_video_stream: Enable HTTP video streaming
            http_port: Port for HTTP video stream
            enable_live_map: Enable live navigation map visualization
        """
        self.config = Config()
        self.tello = None
        self.video_handler = None
        self.enable_video_stream = enable_video_stream
        self.http_port = http_port
        self.detector = None
        self.video_proc = None
        self.live_map = None
        self.enable_live_map = enable_live_map
        self.position = PositionEstimator()
        self.running = False
        self.in_flight = False

        # Movement settings
        self.move_distance = 30  # cm per keypress
        self.rotate_angle = 30   # degrees per keypress

    def connect(self) -> bool:
        """
        Connect to drone and start video stream.

        Returns:
            True if successful, False otherwise
        """
        print("\n" + "="*60)
        print("TELLO WASD MANUAL CONTROL")
        print("="*60)

        # Connect to WiFi
        print("\n[STEP 1] Connecting to Tello WiFi...")
        if not TelloConnection.connect_wifi():
            print("[!] WiFi connection failed")
            return False

        # Connect to drone
        print("\n[STEP 2] Connecting to drone...")
        try:
            self.tello = Tello()
            self.tello.connect()
            print("[+] Drone connected")
        except Exception as e:
            print(f"[!] Drone connection failed: {e}")
            return False

        # Check battery
        try:
            battery = self.tello.get_battery()
            if battery < 30:
                print(f"[!] Battery too low ({battery}%) - need at least 30%")
                return False
            print(f"[+] Battery: {battery}%")
        except:
            print("[WARN] Could not read battery")

        # Start video stream
        print("\n[STEP 3] Starting video stream...")
        self.tello.streamon()
        self.video_handler = VideoStreamHandler(self.tello, enable_http_stream=self.enable_video_stream, http_port=self.http_port)
        self.video_handler.start_stream()
        time.sleep(3)  # Wait for stream to stabilize

        # Initialize components
        print("\n[STEP 4] Initializing control system...")
        self.detector = ObstacleDetector(self.config)
        self.video_proc = VideoProcessor(self.video_handler)

        # Setup video overlays for bounding boxes
        detected_obstacles = []  # Track latest detections for overlay

        def overlay_processor(frame):
            """Draw bounding boxes on video frames"""
            # Run detection (throttled by detector config)
            obstacles = self.detector.process_frame(frame)
            if obstacles:
                detected_obstacles.clear()
                detected_obstacles.extend(obstacles)

            # Draw bounding boxes
            if detected_obstacles:
                frame = self.detector.draw_detections(frame, detected_obstacles)

            return frame

        # Attach overlay processor to video handler
        self.video_handler.set_overlay_processor(overlay_processor)
        print("[+] Video overlays enabled - bounding boxes will appear on stream")

        # Initialize web dashboard if enabled
        if self.enable_live_map:
            dashboard_port = self.video_handler.http_port + 1
            self.live_map = WebDashboard(
                target_x=0,
                target_y=0,
                target_theta=0,
                port=dashboard_port,
                video_port=self.video_handler.http_port
            )
            self.live_map.start()
            print(f"\n[DASHBOARD] Web dashboard available at http://localhost:{dashboard_port}")
            print(f"[DASHBOARD] Video stream on http://localhost:{self.video_handler.http_port}/stream")
            time.sleep(1)  # Let server start

        print("\n" + "="*60)
        print("SYSTEM READY")
        print("="*60)
        print(f"Video stream: http://localhost:{self.video_handler.http_port}")
        if self.enable_live_map:
            print(f"Dashboard: http://localhost:{dashboard_port}")
        print("\nControls:")
        print("  W/S - Forward/Backward")
        print("  A/D - Left/Right")
        print("  Q/E - Rotate CCW/CW")
        print("  R/F - Up/Down")
        print("  T   - Takeoff")
        print("  L   - Land")
        print("  SPACE - Emergency Stop")
        print("  ESC - Quit")
        print("="*60 + "\n")

        return True

    def update_dashboard(self):
        """Update dashboard with current position"""
        if self.live_map:
            pos = self.position.get_position()
            self.live_map.update_position(
                pos['x'], pos['y'], pos['z'],
                self.position.yaw
            )

    def handle_movement(self, command: str, **params):
        """Execute movement and update position tracking"""
        try:
            # Map commands to DJITelloPy methods
            if command == "move_forward":
                self.tello.move_forward(params.get("distance", 30))
            elif command == "move_back":
                self.tello.move_back(params.get("distance", 30))
            elif command == "move_left":
                self.tello.move_left(params.get("distance", 30))
            elif command == "move_right":
                self.tello.move_right(params.get("distance", 30))
            elif command == "move_up":
                self.tello.move_up(params.get("distance", 30))
            elif command == "move_down":
                self.tello.move_down(params.get("distance", 30))
            elif command == "rotate_ccw":
                self.tello.rotate_counter_clockwise(params.get("degrees", 30))
            elif command == "rotate_cw":
                self.tello.rotate_clockwise(params.get("degrees", 30))
            else:
                print(f"[!] Unknown command: {command}")
                return False

            self.position.update_from_command(command, params)
            self.update_dashboard()
            pos = self.position.get_position()
            print(f"[POS] ({pos['x']:.0f}, {pos['y']:.0f}, {pos['z']:.0f}) cm, Yaw: {self.position.yaw:.0f}°")
            return True
        except Exception as e:
            print(f"[!] Command failed: {e}")
            return False

    def start_control_loop(self):
        """Main control loop handling keyboard input"""
        self.running = True

        print("\n[*] Control loop started. Press keys to control drone...")
        print("[*] Press T to takeoff, L to land, ESC to quit\n")

        while self.running:
            try:
                # Check for keyboard input
                if keyboard.is_pressed('esc'):
                    print("\n[*] ESC pressed - Exiting...")
                    break

                elif keyboard.is_pressed('space'):
                    print("\n[!] EMERGENCY STOP")
                    try:
                        self.tello.emergency()
                    except:
                        pass
                    self.in_flight = False
                    time.sleep(0.5)

                elif keyboard.is_pressed('t'):
                    if not self.in_flight:
                        print("[*] Taking off...")
                        try:
                            self.tello.takeoff()
                            self.in_flight = True
                            self.position.reset_position(0, 0, 100)  # Assume 100cm altitude after takeoff
                            if self.live_map:
                                self.live_map.set_status("MANUAL CONTROL")
                            self.update_dashboard()
                            print("[+] Takeoff successful")
                        except Exception as e:
                            print(f"[!] Takeoff failed: {e}")
                        time.sleep(3)  # Wait for takeoff to complete
                    else:
                        print("[!] Already in flight")
                        time.sleep(0.3)

                elif keyboard.is_pressed('l'):
                    if self.in_flight:
                        print("[*] Landing...")
                        try:
                            self.tello.land()
                            self.in_flight = False
                            self.position.reset_position(self.position.pos['x'],
                                                        self.position.pos['y'],
                                                        0)
                            self.update_dashboard()
                            if self.live_map:
                                self.live_map.set_status("LANDED")
                            print("[+] Landed")
                        except Exception as e:
                            print(f"[!] Land failed: {e}")
                        time.sleep(3)  # Wait for landing to complete
                    else:
                        print("[!] Not in flight")
                        time.sleep(0.3)

                elif self.in_flight:
                    # Movement controls (only when in flight)
                    if keyboard.is_pressed('w'):
                        print("[→] Moving forward...")
                        self.handle_movement("move_forward", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('s'):
                        print("[←] Moving backward...")
                        self.handle_movement("move_back", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('a'):
                        print("[←] Moving left...")
                        self.handle_movement("move_left", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('d'):
                        print("[→] Moving right...")
                        self.handle_movement("move_right", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('r'):
                        print("[↑] Moving up...")
                        self.handle_movement("move_up", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('f'):
                        print("[↓] Moving down...")
                        self.handle_movement("move_down", distance=self.move_distance)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('q'):
                        print("[↶] Rotating counter-clockwise...")
                        self.handle_movement("rotate_ccw", degrees=self.rotate_angle)
                        time.sleep(0.3)

                    elif keyboard.is_pressed('e'):
                        print("[↷] Rotating clockwise...")
                        self.handle_movement("rotate_cw", degrees=self.rotate_angle)
                        time.sleep(0.3)

                # Small delay to prevent CPU spinning
                time.sleep(0.05)

            except KeyboardInterrupt:
                print("\n[!] Keyboard interrupt - Exiting...")
                break

        self.running = False

    def cleanup(self):
        """Clean shutdown"""
        print("\n[CLEANUP] Shutting down...")

        # Land if still in flight
        if self.in_flight:
            print("[!] Drone still in flight - landing...")
            try:
                self.tello.land()
                time.sleep(3)
            except:
                pass

        if self.live_map:
            self.live_map.stop()
        if self.video_handler:
            self.video_handler.stop_stream()
        if self.tello:
            try:
                self.tello.streamoff()
            except:
                pass
            try:
                self.tello.end()
            except:
                pass
        print("[+] Shutdown complete")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="WASD manual control for Tello drone with live video and dashboard"
    )
    parser.add_argument("--no-stream", action="store_true", help="Disable HTTP video streaming")
    parser.add_argument("--no-map", action="store_true", help="Disable live navigation map")
    parser.add_argument("--port", type=int, default=8080, help="HTTP streaming port (default: 8080)")

    args = parser.parse_args()

    # Create controller
    controller = WASDController(
        enable_video_stream=not args.no_stream,
        http_port=args.port,
        enable_live_map=not args.no_map
    )

    try:
        # Connect to drone
        if not controller.connect():
            print("[!] Connection failed - exiting")
            return 1

        # Start control loop
        controller.start_control_loop()

        return 0

    except KeyboardInterrupt:
        print("\n\n[!] Keyboard interrupt - Shutting down!")
        return 1

    except Exception as e:
        print(f"\n[!] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        controller.cleanup()


if __name__ == "__main__":
    sys.exit(main())
