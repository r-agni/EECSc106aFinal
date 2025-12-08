"""
AR Drone Command Line Interface with Debug Output

A comprehensive CLI for controlling the AR Drone with real-time status
and extensive debug information.
"""

import time
import sys
import threading
import cv2
from datetime import datetime
from typing import Optional
from drone_api import DroneAPI


class DroneCLI:
    """Command-line interface for AR Drone control with debug output."""

    def __init__(self):
        """Initialize the CLI."""
        self.drone: Optional[DroneAPI] = None
        self.running = False
        self.video_thread = None
        self.status_thread = None
        self.show_video = True  # Auto-enable video by default
        self.debug_mode = True
        self.command_count = 0
        self.current_camera = 0  # 0 = front, 1 = bottom

    def debug(self, message: str, level: str = "INFO"):
        """Print debug message with timestamp."""
        if self.debug_mode:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            prefix = {
                "INFO": "[INFO]",
                "DEBUG": "[DEBUG]",
                "WARN": "[WARN]",
                "ERROR": "[ERROR]",
                "SUCCESS": "[SUCCESS]",
                "COMMAND": "[CMD]"
            }.get(level, "[INFO]")
            print(f"{timestamp} {prefix} {message}")

    def connect(self):
        """Connect to the drone."""
        self.debug("=" * 70, "INFO")
        self.debug("Starting AR Drone Connection...", "INFO")
        self.debug("=" * 70, "INFO")

        try:
            self.debug("Creating DroneAPI instance...", "DEBUG")
            self.drone = DroneAPI()

            self.debug("Connection established!", "SUCCESS")
            self.debug(f"Host: {self.drone.host}", "DEBUG")
            self.debug(f"Navigation Port: {self.drone.nav_port}", "DEBUG")
            self.debug(f"Video Port: {self.drone.video_port}", "DEBUG")
            self.debug(f"AT Command Port: {self.drone.at_port}", "DEBUG")

            # Check video availability
            self.debug("Checking video stream availability...", "DEBUG")
            if self.drone.wait_for_video(timeout=5.0):
                self.debug("Video stream is READY!", "SUCCESS")
                self.debug("Video will be displayed automatically in a separate window", "INFO")
            else:
                self.debug("Video stream NOT available (continuing anyway)", "WARN")
                self.show_video = False

            # Get initial status
            self.debug("Reading initial drone state...", "DEBUG")
            self.print_detailed_status()

            self.debug("=" * 70, "SUCCESS")
            self.debug("Drone ready for commands!", "SUCCESS")
            self.debug("=" * 70, "SUCCESS")

        except Exception as e:
            self.debug(f"Connection failed: {e}", "ERROR")
            raise

    def print_detailed_status(self):
        """Print detailed drone status with all telemetry."""
        if not self.drone:
            self.debug("Drone not connected!", "ERROR")
            return

        self.debug("-" * 70, "INFO")
        self.debug("DETAILED DRONE STATUS", "INFO")
        self.debug("-" * 70, "INFO")

        try:
            # Battery
            battery = self.drone.get_battery_percentage()
            if battery is not None:
                self.debug(f"Battery Level: {battery}%", "INFO")
                if battery < 20:
                    self.debug("LOW BATTERY WARNING!", "WARN")
            else:
                self.debug("Battery Level: N/A", "DEBUG")

            # Altitude
            altitude = self.drone.get_altitude()
            if altitude is not None:
                self.debug(f"Altitude: {altitude} mm ({altitude/1000.0:.2f} m)", "INFO")
            else:
                self.debug("Altitude: N/A", "DEBUG")

            # Flight state
            is_flying = self.drone.is_flying()
            is_emergency = self.drone.is_emergency()
            self.debug(f"Flying: {is_flying}", "INFO")
            self.debug(f"Emergency State: {is_emergency}", "INFO")

            # Rotation
            rotation = self.drone.get_rotation()
            if rotation:
                self.debug(f"Pitch: {rotation['pitch']:.2f}°", "DEBUG")
                self.debug(f"Roll: {rotation['roll']:.2f}°", "DEBUG")
                self.debug(f"Yaw: {rotation['yaw']:.2f}°", "DEBUG")

            # Velocity
            velocity = self.drone.get_velocity()
            if velocity:
                self.debug(f"Velocity X: {velocity['vx']:.1f} mm/s", "DEBUG")
                self.debug(f"Velocity Y: {velocity['vy']:.1f} mm/s", "DEBUG")
                self.debug(f"Velocity Z: {velocity['vz']:.1f} mm/s", "DEBUG")

            # Video status
            video_ready = self.drone.is_video_ready()
            self.debug(f"Video Stream: {'READY' if video_ready else 'NOT READY'}", "INFO")

        except Exception as e:
            self.debug(f"Error reading status: {e}", "ERROR")

        self.debug("-" * 70, "INFO")

    def execute_command(self, command: str, args: list):
        """Execute a drone command with debug output."""
        self.command_count += 1
        self.debug(f"Command #{self.command_count}: {command} {args}", "COMMAND")

        try:
            # Flight commands
            if command == "takeoff":
                self.debug("Initiating takeoff sequence...", "DEBUG")
                self.debug("Checking pre-takeoff status...", "DEBUG")
                self.debug(f"Is flying: {self.drone.is_flying()}", "DEBUG")
                self.debug(f"Is emergency: {self.drone.is_emergency()}", "DEBUG")

                self.drone.takeoff()
                self.debug("Takeoff command sent!", "SUCCESS")
                self.debug("Waiting 5 seconds for drone to stabilize...", "DEBUG")

                for i in range(10):
                    time.sleep(0.5)
                    if i % 2 == 0:
                        altitude = self.drone.get_altitude()
                        if altitude:
                            self.debug(f"Takeoff progress - Altitude: {altitude} mm", "DEBUG")

                self.debug("Drone should be airborne now", "INFO")
                self.debug(f"Is flying: {self.drone.is_flying()}", "DEBUG")

            elif command == "land":
                self.debug("Initiating landing sequence...", "DEBUG")
                self.debug(f"Current altitude: {self.drone.get_altitude()} mm", "DEBUG")

                self.drone.land()
                self.debug("Land command sent!", "SUCCESS")
                self.debug("Waiting 3 seconds for drone to land...", "DEBUG")

                for i in range(6):
                    time.sleep(0.5)
                    if i % 2 == 0:
                        altitude = self.drone.get_altitude()
                        if altitude:
                            self.debug(f"Landing progress - Altitude: {altitude} mm", "DEBUG")

                self.debug("Drone should be on ground now", "INFO")
                self.debug(f"Is flying: {self.drone.is_flying()}", "DEBUG")

            elif command == "hover":
                self.debug("Sending hover command...", "DEBUG")
                self.drone.hover()
                self.debug("Hover command sent!", "SUCCESS")

            elif command == "emergency":
                self.debug("!!! EMERGENCY STOP !!!", "WARN")
                self.drone.emergency()
                self.debug("Emergency command sent - motors cut!", "WARN")

            elif command == "reset":
                self.debug("Resetting emergency state...", "DEBUG")
                self.drone.reset_emergency()
                self.debug("Reset command sent!", "SUCCESS")

            # Movement commands
            elif command == "forward":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving forward at speed {speed}...", "DEBUG")
                rotation = self.drone.get_rotation()
                if rotation:
                    self.debug(f"Current heading (yaw): {rotation['yaw']:.2f}°", "DEBUG")
                self.drone.move_forward(speed)
                self.debug("Forward command sent!", "SUCCESS")
                self.debug("Note: Command sends once - hold key or use 'move' for continuous", "INFO")

            elif command == "backward":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving backward at speed {speed}...", "DEBUG")
                self.drone.move_backward(speed)
                self.debug("Backward command sent!", "SUCCESS")

            elif command == "left":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving left at speed {speed}...", "DEBUG")
                self.drone.move_left(speed)
                self.debug("Left command sent!", "SUCCESS")

            elif command == "right":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving right at speed {speed}...", "DEBUG")
                self.drone.move_right(speed)
                self.debug("Right command sent!", "SUCCESS")

            elif command == "up":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving up at speed {speed}...", "DEBUG")
                self.drone.move_up(speed)
                self.debug("Up command sent!", "SUCCESS")

            elif command == "down":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Moving down at speed {speed}...", "DEBUG")
                self.drone.move_down(speed)
                self.debug("Down command sent!", "SUCCESS")

            elif command == "cw" or command == "clockwise":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Rotating clockwise at speed {speed}...", "DEBUG")
                self.drone.rotate_clockwise(speed)
                self.debug("Clockwise rotation command sent!", "SUCCESS")

            elif command == "ccw" or command == "counterclockwise":
                speed = float(args[0]) if args else 0.5
                self.debug(f"Rotating counter-clockwise at speed {speed}...", "DEBUG")
                self.drone.rotate_counterclockwise(speed)
                self.debug("Counter-clockwise rotation command sent!", "SUCCESS")

            # Movement with duration
            elif command == "move":
                direction = args[0] if args else "forward"
                duration = float(args[1]) if len(args) > 1 else 1.0
                speed = float(args[2]) if len(args) > 2 else 0.5

                self.debug(f"Moving {direction} for {duration}s at speed {speed}...", "DEBUG")

                movement_map = {
                    "forward": self.drone.move_forward,
                    "backward": self.drone.move_backward,
                    "left": self.drone.move_left,
                    "right": self.drone.move_right,
                    "up": self.drone.move_up,
                    "down": self.drone.move_down,
                }

                if direction in movement_map:
                    end_time = time.time() + duration
                    steps = 0
                    start_altitude = self.drone.get_altitude()
                    self.debug(f"Starting altitude: {start_altitude} mm", "DEBUG")

                    while time.time() < end_time:
                        movement_map[direction](speed)
                        time.sleep(0.05)
                        steps += 1

                        # Log progress every 20 steps (~1 second)
                        if steps % 20 == 0:
                            current_alt = self.drone.get_altitude()
                            velocity = self.drone.get_velocity()
                            self.debug(f"Step {steps}: Alt={current_alt}mm, " +
                                     (f"Vel={velocity}" if velocity else "Vel=N/A"), "DEBUG")

                    end_altitude = self.drone.get_altitude()
                    self.debug(f"Movement complete! Executed {steps} steps", "SUCCESS")
                    self.debug(f"End altitude: {end_altitude} mm (delta: {end_altitude - start_altitude if start_altitude and end_altitude else 'N/A'} mm)", "DEBUG")
                    self.drone.hover()
                else:
                    self.debug(f"Unknown direction: {direction}", "ERROR")

            # Calibration
            elif command == "trim" or command == "flattrim":
                self.debug("Performing flat trim calibration...", "DEBUG")
                self.debug("Make sure drone is on a FLAT surface!", "WARN")
                self.drone.flat_trim()
                self.debug("Flat trim command sent!", "SUCCESS")

            elif command == "calibrate":
                device = int(args[0]) if args else 0
                self.debug(f"Calibrating magnetometer (device {device})...", "DEBUG")
                self.drone.calibrate_magnetometer(device)
                self.debug("Calibration command sent!", "SUCCESS")

            # Configuration
            elif command == "config":
                if len(args) >= 2:
                    key = args[0]
                    value = args[1]
                    self.debug(f"Setting config: {key} = {value}", "DEBUG")
                    self.drone.set_config(key, value)
                    self.debug("Config command sent!", "SUCCESS")
                else:
                    self.debug("Usage: config <key> <value>", "ERROR")

            elif command == "maxalt":
                altitude = int(args[0]) if args else 2000
                self.debug(f"Setting max altitude to {altitude} mm", "DEBUG")
                self.drone.set_max_altitude(altitude)
                self.debug("Max altitude set!", "SUCCESS")

            elif command == "outdoor":
                enabled = args[0].lower() in ['true', '1', 'yes'] if args else True
                self.debug(f"Setting outdoor mode: {enabled}", "DEBUG")
                self.drone.enable_outdoor_mode(enabled)
                self.debug("Outdoor mode configured!", "SUCCESS")

            # Status
            elif command == "status":
                self.print_detailed_status()

            elif command == "battery":
                battery = self.drone.get_battery_percentage()
                self.debug(f"Battery: {battery}%", "INFO")

            # Video
            elif command == "video":
                action = args[0] if args else "start"
                if action == "start":
                    self.debug("Starting video display...", "DEBUG")
                    self.debug(f"Video ready status: {self.drone.is_video_ready()}", "DEBUG")
                    self.show_video = True
                    self.debug("Video display enabled (showing in separate window)", "SUCCESS")
                elif action == "stop":
                    self.debug("Stopping video display...", "DEBUG")
                    self.show_video = False
                    cv2.destroyAllWindows()
                    self.debug("Video display disabled", "SUCCESS")
                elif action == "save":
                    filename = args[1] if len(args) > 1 else f"frame_{int(time.time())}.jpg"
                    self.debug(f"Saving frame to {filename}...", "DEBUG")
                    frame = self.drone.get_frame()
                    if frame is not None:
                        self.debug(f"Frame dimensions: {frame.shape}", "DEBUG")
                    if self.drone.save_frame(filename):
                        self.debug(f"Frame saved to {filename}!", "SUCCESS")
                    else:
                        self.debug("Failed to save frame (video not ready?)", "ERROR")
                elif action == "info":
                    self.debug("Video stream information:", "INFO")
                    self.debug(f"  Video ready: {self.drone.is_video_ready()}", "INFO")
                    camera_name = "Front" if self.current_camera == 0 else "Bottom"
                    self.debug(f"  Active camera: {camera_name} (ID: {self.current_camera})", "INFO")
                    frame = self.drone.get_frame()
                    if frame is not None:
                        self.debug(f"  Frame shape: {frame.shape}", "INFO")
                        self.debug(f"  Frame dtype: {frame.dtype}", "INFO")
                    else:
                        self.debug("  No frame available", "WARN")
                elif action == "switch":
                    camera_id = int(args[1]) if len(args) > 1 else (1 if self.current_camera == 0 else 0)
                    camera_names = {0: "Front", 1: "Bottom", 2: "Front (Small)", 3: "Bottom (Small)"}
                    self.debug(f"Switching to camera {camera_id} ({camera_names.get(camera_id, 'Unknown')})...", "DEBUG")
                    self.drone.switch_camera(camera_id)
                    self.current_camera = camera_id
                    self.debug(f"Camera switched! Now using {camera_names.get(camera_id, 'Unknown')} camera", "SUCCESS")
                elif action == "front":
                    self.debug("Switching to front camera...", "DEBUG")
                    self.drone.set_front_camera()
                    self.current_camera = 0
                    self.debug("Now using FRONT camera", "SUCCESS")
                elif action == "bottom":
                    self.debug("Switching to bottom camera...", "DEBUG")
                    self.drone.set_bottom_camera()
                    self.current_camera = 1
                    self.debug("Now using BOTTOM camera", "SUCCESS")

            # Sequences
            elif command == "square":
                side_duration = float(args[0]) if args else 2.0
                speed = float(args[1]) if len(args) > 1 else 0.4
                self.debug(f"Executing square pattern (side={side_duration}s, speed={speed})...", "DEBUG")

                for i in range(4):
                    self.debug(f"Flying side {i+1} of 4...", "INFO")
                    end_time = time.time() + side_duration
                    while time.time() < end_time:
                        self.drone.move_forward(speed)
                        time.sleep(0.05)

                    self.debug(f"Hovering...", "INFO")
                    for _ in range(10):
                        self.drone.hover()
                        time.sleep(0.05)

                    self.debug(f"Rotating 90 degrees...", "INFO")
                    end_time = time.time() + 1.0
                    while time.time() < end_time:
                        self.drone.rotate_clockwise(0.5)
                        time.sleep(0.05)

                self.debug("Square pattern complete!", "SUCCESS")

            # Help
            elif command == "help":
                self.print_help()

            # Debug toggle
            elif command == "debug":
                action = args[0] if args else "toggle"
                if action == "on":
                    self.debug_mode = True
                    self.debug("Debug mode enabled", "SUCCESS")
                elif action == "off":
                    self.debug_mode = False
                    print("Debug mode disabled")
                else:
                    self.debug_mode = not self.debug_mode
                    self.debug(f"Debug mode: {'ON' if self.debug_mode else 'OFF'}", "SUCCESS")

            # Exit
            elif command == "quit" or command == "exit":
                self.debug("Exiting CLI...", "INFO")
                return False

            else:
                self.debug(f"Unknown command: {command}", "ERROR")
                self.debug("Type 'help' for available commands", "INFO")

        except Exception as e:
            self.debug(f"Command execution error: {e}", "ERROR")
            import traceback
            if self.debug_mode:
                self.debug(traceback.format_exc(), "DEBUG")

        return True

    def video_loop(self):
        """Background thread for video display."""
        self.debug("Video display thread started", "DEBUG")
        frame_count = 0
        last_debug = time.time()

        while self.running:
            if self.show_video and self.drone:
                try:
                    # Update window title with camera info
                    camera_name = "FRONT" if self.current_camera == 0 else "BOTTOM"
                    window_title = f"AR.Drone Live Feed - {camera_name} Camera [ESC to close]"

                    key = self.drone.display_video(window_title)

                    if key == 27:  # ESC
                        self.debug("ESC pressed in video window - closing video", "INFO")
                        self.show_video = False
                        cv2.destroyAllWindows()

                    frame_count += 1

                    # Debug frame count every 5 seconds
                    if time.time() - last_debug > 5.0:
                        fps = frame_count / 5.0
                        self.debug(f"Video streaming: {fps:.1f} FPS ({frame_count} frames) - {camera_name} camera", "DEBUG")
                        frame_count = 0
                        last_debug = time.time()

                except Exception as e:
                    self.debug(f"Video display error: {e}", "ERROR")
                    time.sleep(0.1)
            time.sleep(0.03)

        cv2.destroyAllWindows()
        self.debug("Video display thread stopped", "DEBUG")

    def print_help(self):
        """Print help message."""
        help_text = """
╔════════════════════════════════════════════════════════════════════╗
║                   AR DRONE COMMAND LINE INTERFACE                  ║
╚════════════════════════════════════════════════════════════════════╝

FLIGHT COMMANDS:
  takeoff                    - Take off and hover
  land                       - Land at current position
  hover                      - Maintain current position
  emergency                  - Emergency stop (cuts motors!)
  reset                      - Reset emergency state

MOVEMENT COMMANDS:
  forward [speed]            - Move forward (speed: 0.0-1.0, default: 0.5)
  backward [speed]           - Move backward
  left [speed]               - Move left
  right [speed]              - Move right
  up [speed]                 - Move up
  down [speed]               - Move down
  cw [speed]                 - Rotate clockwise
  ccw [speed]                - Rotate counter-clockwise

  move <dir> [duration] [speed]  - Move in direction for duration seconds
                                   Example: move forward 2.0 0.5

CALIBRATION:
  trim                       - Flat trim calibration (on flat surface!)
  calibrate [device]         - Calibrate magnetometer

CONFIGURATION:
  config <key> <value>       - Set configuration parameter
  maxalt <mm>                - Set max altitude in millimeters
  outdoor <true/false>       - Enable/disable outdoor mode

STATUS & INFO:
  status                     - Show detailed drone status
  battery                    - Show battery level
  help                       - Show this help message

VIDEO:
  video start                - Start video display (auto-enabled by default)
  video stop                 - Stop video display (press ESC in window)
  video save [filename]      - Save current frame to file
  video info                 - Show video stream information and stats
  video front                - Switch to front-facing camera
  video bottom               - Switch to bottom-facing camera
  video switch [0-3]         - Switch camera (0=front, 1=bottom, 2/3=small)

SEQUENCES:
  square [side_time] [speed] - Fly in square pattern

UTILITY:
  debug <on/off>             - Toggle debug output
  quit / exit                - Exit CLI

NOTES:
  - Speed values range from 0.0 (stopped) to 1.0 (full speed)
  - Always do 'trim' before first flight
  - Monitor battery level regularly
  - Use 'emergency' only when necessary (drone will fall!)
  - Video stream displays automatically in separate window
  - Press ESC in video window to close it
  - Debug output shows timestamps, commands, and telemetry
  - AR Drone has 2 cameras: Front (default) and Bottom
  - Use 'video front' or 'video bottom' to switch cameras
  - Window title shows which camera is active

╚════════════════════════════════════════════════════════════════════╝
"""
        print(help_text)

    def run(self):
        """Run the CLI main loop."""
        self.debug("Starting Drone CLI...", "INFO")

        try:
            # Connect to drone
            self.connect()

            # Start video thread
            self.running = True
            self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
            self.video_thread.start()
            self.debug("Background video thread started", "DEBUG")

            # Print welcome message
            print("\n" + "="*70)
            print("  AR DRONE COMMAND LINE INTERFACE")
            print("  Type 'help' for available commands")
            print("  Type 'quit' to exit")
            if self.show_video:
                print("  Video stream is ACTIVE in separate window")
            print("="*70 + "\n")

            # Main command loop
            while True:
                try:
                    # Get command from user
                    user_input = input("drone> ").strip()

                    if not user_input:
                        continue

                    # Parse command
                    parts = user_input.split()
                    command = parts[0].lower()
                    args = parts[1:]

                    self.debug(f"Parsing input: '{user_input}'", "DEBUG")
                    self.debug(f"Command: '{command}', Args: {args}", "DEBUG")

                    # Execute command
                    if not self.execute_command(command, args):
                        break

                except KeyboardInterrupt:
                    self.debug("\nKeyboard interrupt detected", "WARN")
                    break
                except EOFError:
                    self.debug("\nEOF detected", "WARN")
                    break
                except Exception as e:
                    self.debug(f"Error in main loop: {e}", "ERROR")

        finally:
            # Cleanup
            self.debug("Cleaning up...", "INFO")
            self.running = False

            if self.video_thread:
                self.debug("Waiting for video thread to stop...", "DEBUG")
                self.video_thread.join(timeout=2.0)

            if self.drone:
                self.debug("Closing drone connection...", "DEBUG")
                self.drone.close()

            self.debug("CLI shutdown complete", "SUCCESS")
            print("\nGoodbye!\n")


def main():
    """Main entry point."""
    print("""
    ╔══════════════════════════════════════════════════════════════╗
    ║                                                              ║
    ║              AR DRONE COMMAND LINE INTERFACE                 ║
    ║                    With Debug Output                         ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """)

    cli = DroneCLI()

    try:
        cli.run()
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
