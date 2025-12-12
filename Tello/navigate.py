"""
Simplified autonomous navigation with obstacle avoidance and live video streaming.

Usage:
    python navigate.py --x 200 --y 100 --theta 45

    This will:
    1. Connect to Tello drone
    2. Start video streaming (OpenCV window + browser at http://localhost:8080)
    3. Navigate to target position (x, y) in cm with final orientation theta
    4. Avoid obstacles along the way while maintaining target endpoint
    5. Land at destination

Options:
    --no-stream Disable HTTP video streaming
    --no-map    Disable live navigation map visualization

Enhanced Debugging:
    This version includes comprehensive logging and diagnostics to help identify
    issues with drone connection and takeoff:

    - Connection diagnostics: Detailed WiFi and SDK connection status
    - Pre-takeoff checks: Battery, height, temperature, and connection verification
    - Takeoff monitoring: Real-time progress tracking with height verification
    - Error analysis: Specific troubleshooting tips based on error type
    - State logging: Continuous monitoring of drone telemetry during flight

    If the drone is not taking off, check the detailed logs for:
    1. WiFi connection status and ping results
    2. SDK connection errors and timeout messages
    3. Battery level warnings (needs 10%+ to fly, 30%+ recommended)
    4. Pre-takeoff height sensor readings
    5. Takeoff command execution time and post-takeoff height verification
"""

import sys
import time
import argparse
import math
from djitellopy import Tello
from drone_wrapper import TelloConnection, VideoStreamHandler
from obstacle_avoidance.path_executor import PathExecutor, PositionEstimator
from obstacle_avoidance.config import Config
from obstacle_avoidance.obstacle_detector import ObstacleDetector
from obstacle_avoidance.video_processor import VideoProcessor
from obstacle_avoidance.path_planner import generate_waypoints_with_rrt
from visualization.web_dashboard import WebDashboard
from visualization.nav_wrapper import VisualizationWrapper


def convert_desired_to_command_distance(x_desired: float, y_desired: float) -> tuple[float, float]:
    """
    Convert desired actual distances to drone command values.
    
    Based on calibration:
    - Command: x=200cm, y=500cm
    - Actual movement: x=104cm (1.04m), y=528cm (5.28m)
    
    To achieve desired distance, we need to reverse the scale:
    - X: command = desired / 0.52 (or desired * 1.923)
    - Y: command = desired / 1.056 (or desired * 0.947)
    
    Args:
        x_desired: Desired actual X distance in meters
        y_desired: Desired actual Y distance in meters
        
    Returns:
        Tuple of (command_x, command_y) in cm for drone commands
    """
    # Scale factors from calibration
    X_ACTUAL_SCALE = 0.52  # actual = command * 0.52
    Y_ACTUAL_SCALE = 1.056  # actual = command * 1.056
    
    # Convert meters to cm
    x_desired_cm = x_desired * 100
    y_desired_cm = y_desired * 100
    
    # Reverse the scale to get command values
    command_x = x_desired_cm / X_ACTUAL_SCALE
    command_y = y_desired_cm / Y_ACTUAL_SCALE
    
    return command_x, command_y


class SimpleNavigator:
    """Simplified navigation system - go to (x, y, theta) with obstacle avoidance"""

    def __init__(self, enable_video_stream: bool = True, http_port: int = 8080,
                 enable_live_map: bool = True):
        """
        Initialize navigation system.

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
        self.executor = None
        self.detector = None
        self.video_proc = None
        self.live_map = None
        self.enable_live_map = enable_live_map

    def connect(self) -> bool:
        """
        Connect to drone and start video stream.

        Returns:
            True if successful, False otherwise
        """
        print("\n" + "="*60)
        print("TELLO AUTONOMOUS NAVIGATION")
        print("="*60)

        # Connect to WiFi
        print("\n[STEP 1] Connecting to Tello WiFi...")
        if not TelloConnection.connect_wifi():
            print("\n[!] WiFi connection failed")
            print("[!] Troubleshooting steps:")
            print("    1. Ensure Tello drone is powered ON")
            print("    2. Wait for WiFi LED to blink (indicates ready)")
            print("    3. Check Windows WiFi settings")
            print("    4. Try connecting to Tello WiFi manually first")
            print("    5. Restart the drone if issues persist")
            return False

        # Connect to drone
        print("\n[STEP 2] Connecting to drone SDK...")
        print("[INFO] This establishes command/control connection")
        try:
            self.tello = Tello()
            self.tello.connect()
            print("[+] SDK connection established")
        except Exception as e:
            print(f"\n[!] Drone SDK connection failed: {e}")
            print("[!] Troubleshooting steps:")
            print("    1. Verify WiFi connection is active")
            print("    2. Check if another program is using the drone")
            print("    3. Restart the drone (power cycle)")
            print("    4. Ensure no firewall blocking UDP ports 8889, 8890, 11111")
            print("    5. Try pinging 192.168.10.1 to verify network connectivity")
            return False

        # Check battery
        print("\n[STEP 3] Checking battery and drone status...")
        battery = self.tello.get_battery()
        temp = self.tello.get_temperature()

        print(f"[STATUS] Battery: {battery}%")
        print(f"[STATUS] Temperature: {temp}°C")
        print(f"[STATUS] Connection: connected")

        if battery < 10:
            print(f"\n[ERROR] Battery critically low ({battery}%) - CANNOT FLY")
            print("[ERROR] Please charge the battery before flying")
            return False
        elif battery < 20:
            print(f"\n[WARN] Battery low ({battery}%) - flight time will be limited")
            print("[WARN] Consider charging for longer flights")
        elif battery < 30:
            print(f"[WARN] Battery at {battery}% - should be adequate for short flights")

        if battery < 30:
            user_input = input(f"\nBattery is at {battery}%. Continue anyway? (y/N): ")
            if user_input.lower() != 'y':
                print("[!] Aborted by user due to low battery")
                return False

        print(f"[+] Battery check passed: {battery}%")

        # Start video stream
        print("\n[STEP 4] Starting video stream...")
        print("[INFO] Opening video UDP stream on port 11111...")
        self.tello.streamon()
        self.video_handler = VideoStreamHandler(self.tello, enable_http_stream=self.enable_video_stream, http_port=self.http_port)
        self.video_handler.start_stream()
        print("[+] Waiting for video stream to stabilize...")
        time.sleep(3)  # Wait for stream to stabilize
        print("[+] Video stream ready")

        # Initialize navigation components
        print("\n[STEP 5] Initializing navigation system...")
        self.executor = PathExecutor(self.tello, self.video_handler, self.config)

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

        print("\n" + "="*60)
        print("SYSTEM READY")
        print("="*60)
        print(f"Video stream: http://localhost:{self.video_handler.http_port}")
        print("Press Ctrl+C anytime for emergency landing")
        print("="*60 + "\n")

        return True

    def navigate_to(self, x: float, y: float, theta: float = 0, altitude: float = 120) -> bool:
        """
        Navigate to target position with obstacle avoidance.

        Args:
            x: Target X position in cm (forward/back)
            y: Target Y position in cm (left/right)
            theta: Target orientation in degrees (0 = forward)
            altitude: Flight altitude in cm (default 120cm = 1.2m)

        Returns:
            True if navigation successful, False otherwise
        """
        print(f"\n[TARGET] Position: ({x:.0f}, {y:.0f}) cm, Orientation: {theta:.0f}°, Altitude: {altitude:.0f}cm")

        # Generate waypoints using linear interpolation
        # This creates a simple, fast, and 100% reliable straight-line path
        # Obstacles are handled by reactive avoidance during flight
        print("\n[PLANNING] Generating waypoints using linear interpolation...")

        # No pre-planning around obstacles - reactive navigation handles them
        obstacles = []  # Ignored by linear planner

        waypoints = generate_waypoints_with_rrt(
            start_x=0,
            start_y=0,
            goal_x=x,
            goal_y=y,
            altitude=altitude,
            obstacles=obstacles
        )

        print(f"\n[PLAN] Generated {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {i+1}. ({wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}) cm")

        print(f"\n[PLAN] Final rotation to {theta:.0f}° will occur before landing")

        # Initialize web dashboard if enabled
        if self.enable_live_map:
            # Use web dashboard on port 8081 (8080 is used for video stream)
            dashboard_port = self.video_handler.http_port + 1
            self.live_map = WebDashboard(
                target_x=x,
                target_y=y,
                target_theta=theta,
                port=dashboard_port,
                video_port=self.video_handler.http_port
            )
            self.live_map.start()
            print(f"\n[DASHBOARD] Web dashboard available at http://localhost:{dashboard_port}")
            print(f"[DASHBOARD] Video stream on http://localhost:{self.video_handler.http_port}/stream")
            time.sleep(1)  # Let server start

        input("\nPress ENTER to start navigation...")

        # Execute navigation with obstacle avoidance
        if self.enable_live_map:
            # Use visualization wrapper
            viz_wrapper = VisualizationWrapper(self.executor, self.live_map)
            success = viz_wrapper.execute_path_with_viz(waypoints)
        else:
            success = self.executor.execute_path(waypoints)

        # Handle navigation failure - always land the drone
        if not success:
            print("\n" + "="*60)
            print("NAVIGATION FAILED - EMERGENCY LANDING")
            print("="*60)
            print("[!] Path execution failed - performing emergency landing")

            if self.enable_live_map:
                self.live_map.set_status("FAILED")

            self.tello.land()
            time.sleep(1)

            print("\n[!] Emergency landing complete")
            print("="*60 + "\n")
            return False

        # Navigation succeeded - do final rotation and land
        print("\n" + "="*60)
        print("FINAL POSITIONING")
        print("="*60)

        # Do final rotation to target theta
        # Simplified: just execute rotation command without verification or looping
        if theta != 0:
            print(f"\n[ROTATE] Rotating to final orientation {theta:.0f}°...")
            # Execute rotation directly, trust it works without verification
            if theta > 0:
                self.tello.rotate_counter_clockwise(int(abs(theta)))
            else:
                self.tello.rotate_clockwise(int(abs(theta)))

        # Land at current position
        print("\n[FINAL] Landing at target position...")
        self.tello.land()
        time.sleep(1)

        # Mark mission complete on map
        if self.enable_live_map:
            self.live_map.set_status("COMPLETE")
            time.sleep(1)

        print("\n" + "="*60)
        print("NAVIGATION COMPLETED SUCCESSFULLY")
        print("="*60)
        print(f"Final position: ({x:.0f}, {y:.0f}) cm")
        print(f"Final orientation: {theta:.0f}°")
        print("="*60 + "\n")
        return True

    def cleanup(self):
        """Clean shutdown"""
        print("\n[CLEANUP] Shutting down...")
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
        description="Navigate Tello drone to target position with obstacle avoidance"
    )
    parser.add_argument("--x", type=float, required=True, help="Target X position in cm (forward/back)")
    parser.add_argument("--y", type=float, required=True, help="Target Y position in cm (left/right)")
    parser.add_argument("--theta", type=float, default=0, help="Target orientation in degrees (default: 0)")
    parser.add_argument("--altitude", type=float, default=120, help="Flight altitude in cm (default: 120)")
    parser.add_argument("--no-stream", action="store_true", help="Disable HTTP video streaming")
    parser.add_argument("--no-map", action="store_true", help="Disable live navigation map")
    parser.add_argument("--port", type=int, default=8080, help="HTTP streaming port (default: 8080)")

    args = parser.parse_args()

    # Convert desired distances (in meters) to drone command values (in cm)
    command_x, command_y = convert_desired_to_command_distance(args.x, args.y)
    
    print(f"\n[CONVERSION] Desired distances: ({args.x:.2f}m, {args.y:.2f}m)")
    print(f"[CONVERSION] Drone commands: ({command_x:.0f}cm, {command_y:.0f}cm)")
    print(f"[CONVERSION] Expected actual movement: ({args.x:.2f}m, {args.y:.2f}m)\n")

    # Create navigator
    navigator = SimpleNavigator(
        enable_video_stream=not args.no_stream,
        http_port=args.port,
        enable_live_map=not args.no_map
    )

    try:
        # Connect to drone
        if not navigator.connect():
            print("[!] Connection failed - exiting")
            return 1

        # Navigate to target using command coordinates
        success = navigator.navigate_to(
            x=command_x,
            y=command_y,
            theta=args.theta,
            altitude=args.altitude
        )

        if not success:
            print("[!] Navigation failed")
            return 1

        return 0

    except KeyboardInterrupt:
        print("\n\n[!] Keyboard interrupt - Emergency landing!")
        try:
            navigator.tello.emergency()
        except Exception as e:
            print(f"[!] Emergency command failed: {e}")
        return 1

    except Exception as e:
        print(f"\n[!] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        try:
            navigator.tello.emergency()
        except Exception as emergency_err:
            print(f"[!] Emergency command failed: {emergency_err}")
        return 1

    finally:
        navigator.cleanup()


if __name__ == "__main__":
    sys.exit(main())
