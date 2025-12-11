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
"""

import sys
import time
import argparse
import math
from tello_server import DroneController, StateManager, VideoStreamHandler
from obstacle_avoidance.path_executor import PathExecutor, PositionEstimator
from obstacle_avoidance.config import Config
from obstacle_avoidance.obstacle_detector import ObstacleDetector
from obstacle_avoidance.video_processor import VideoProcessor
from obstacle_avoidance.path_planner import generate_waypoints_with_rrt
from pid_controller import PositionController
from visualization.web_dashboard import WebDashboard
from visualization.nav_wrapper import VisualizationWrapper


def manual_control_mode(navigator, altitude: float = 120) -> int:
    """
    Manual WASD control mode with dashboard visualization.
    
    Args:
        navigator: SimpleNavigator instance
        altitude: Flight altitude in cm
        
    Returns:
        Exit code (0 for success, 1 for error)
    """
    print("\n" + "="*60)
    print("MANUAL WASD CONTROL MODE")
    print("="*60)
    print("Controls:")
    print("  W - Move forward 30cm")
    print("  A - Move left 30cm")
    print("  S - Move backward 30cm")
    print("  D - Move right 30cm")
    print("  Q - Rotate counter-clockwise 30°")
    print("  E - Rotate clockwise 30°")
    print("  R - Move up 30cm")
    print("  F - Move down 30cm")
    print("  T - Takeoff")
    print("  L - Land")
    print("  ESC - Emergency stop")
    print("="*60 + "\n")
    
    # Initialize web dashboard if enabled
    if navigator.enable_live_map:
        dashboard_port = navigator.video_handler.http_port + 1
        navigator.live_map = WebDashboard(
            target_x=0,
            target_y=0,
            target_theta=0,
            port=dashboard_port,
            video_port=navigator.video_handler.http_port
        )
        navigator.live_map.start()
        navigator.live_map.set_status("MANUAL CONTROL")
        print(f"[DASHBOARD] Web dashboard available at http://localhost:{dashboard_port}")
        print(f"[DASHBOARD] Video stream on http://localhost:{navigator.video_handler.http_port}/stream")
        time.sleep(1)
    
    input("Press ENTER to takeoff...")
    
    # Takeoff
    print("[*] Taking off...")
    success, msg = navigator.drone_ctrl.send_command("takeoff")
    if not success:
        print(f"[!] Takeoff failed: {msg}")
        return 1
    
    time.sleep(5)  # Wait for takeoff to stabilize
    navigator.executor.position.reset_position(0, 0, altitude)
    
    # Update dashboard with initial position
    if navigator.enable_live_map:
        navigator.live_map.update_position(0, 0, altitude, 0)
    
    print("[+] Drone in the air - WASD controls active")
    print("[*] Use OpenCV window for keyboard input")
    
    landed = False
    
    try:
        while not landed:
            # Get keyboard input from video window
            key = navigator.video_handler.get_keyboard_input()
            
            if key:
                pos = navigator.executor.position.get_position()
                
                # Process key and execute command
                if key == 'w':
                    print("[WASD] Moving forward 30cm")
                    navigator.drone_ctrl.send_command("move_forward", distance=30)
                    navigator.executor.position.update_from_command("move_forward", {"distance": 30})
                    
                elif key == 'a':
                    print("[WASD] Moving left 30cm")
                    navigator.drone_ctrl.send_command("move_left", distance=30)
                    navigator.executor.position.update_from_command("move_left", {"distance": 30})
                    
                elif key == 's':
                    print("[WASD] Moving backward 30cm")
                    navigator.drone_ctrl.send_command("move_back", distance=30)
                    navigator.executor.position.update_from_command("move_back", {"distance": 30})
                    
                elif key == 'd':
                    print("[WASD] Moving right 30cm")
                    navigator.drone_ctrl.send_command("move_right", distance=30)
                    navigator.executor.position.update_from_command("move_right", {"distance": 30})
                    
                elif key == 'q':
                    print("[WASD] Rotating counter-clockwise 30°")
                    navigator.drone_ctrl.send_command("rotate_ccw", degrees=30)
                    navigator.executor.position.update_from_command("rotate_ccw", {"degrees": 30})
                    
                elif key == 'e':
                    print("[WASD] Rotating clockwise 30°")
                    navigator.drone_ctrl.send_command("rotate_cw", degrees=30)
                    navigator.executor.position.update_from_command("rotate_cw", {"degrees": 30})
                    
                elif key == 'r':
                    print("[WASD] Moving up 30cm")
                    navigator.drone_ctrl.send_command("move_up", distance=30)
                    navigator.executor.position.update_from_command("move_up", {"distance": 30})
                    
                elif key == 'f':
                    print("[WASD] Moving down 30cm")
                    navigator.drone_ctrl.send_command("move_down", distance=30)
                    navigator.executor.position.update_from_command("move_down", {"distance": 30})
                    
                elif key == 'l':
                    print("[WASD] Landing...")
                    navigator.drone_ctrl.send_command("land")
                    landed = True
                    
                elif key == '\x1b':  # ESC key
                    print("[WASD] Emergency stop!")
                    navigator.drone_ctrl.send_command("emergency")
                    landed = True
                
                # Update dashboard after each movement
                if navigator.enable_live_map and not landed:
                    pos = navigator.executor.position.get_position()
                    navigator.live_map.update_position(
                        pos['x'], pos['y'], pos['z'],
                        navigator.executor.position.yaw
                    )
                    print(f"[POS] ({pos['x']:.1f}, {pos['y']:.1f}, {pos['z']:.1f}) cm, Yaw: {navigator.executor.position.yaw:.1f}°")
            
            time.sleep(0.05)  # Small delay to prevent CPU overload
        
        if navigator.enable_live_map:
            navigator.live_map.set_status("LANDED")
            time.sleep(2)
        
        print("\n[+] Manual control session ended")
        return 0
        
    except KeyboardInterrupt:
        print("\n[!] Keyboard interrupt - Emergency landing")
        navigator.drone_ctrl.send_command("land")
        return 1


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
        self.state_mgr = StateManager()
        self.drone_ctrl = DroneController(self.state_mgr)
        self.video_handler = VideoStreamHandler(enable_http_stream=enable_video_stream, http_port=http_port)
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
        if not self.drone_ctrl.connect_wifi():
            print("[!] WiFi connection failed")
            return False

        # Connect to drone
        print("\n[STEP 2] Connecting to drone...")
        if not self.drone_ctrl.connect_drone():
            print("[!] Drone connection failed")
            return False

        # Check battery
        battery = self.state_mgr.get_state().get("battery", 0)
        if battery < 30:
            print(f"[!] Battery too low ({battery}%) - need at least 30%")
            return False
        print(f"[+] Battery: {battery}%")

        # Start video stream
        print("\n[STEP 3] Starting video stream...")
        self.drone_ctrl.start_video_stream()
        self.video_handler.start_stream()
        time.sleep(3)  # Wait for stream to stabilize

        # Initialize navigation components
        print("\n[STEP 4] Initializing navigation system...")
        self.executor = PathExecutor(self.drone_ctrl, self.state_mgr, self.video_handler, self.config)
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

        if success:
            # Use PID controller for precise positioning at target
            print("\n" + "="*60)
            print("PID POSITION CORRECTION")
            print("="*60)
            print("Using PID control for precise landing at target...")

            # Update visualization state
            if self.enable_live_map:
                self.live_map.set_status("PID POSITIONING")

            # Initialize PID controller
            pid_controller = PositionController(
                self.executor.position,
                self.drone_ctrl,
                self.state_mgr
            )

            # Wrap PID to update visualization
            if self.enable_live_map:
                pid_success = self._pid_with_viz(pid_controller, x, y, altitude, theta)
            else:
                pid_success = pid_controller.move_to_target(x, y, altitude, theta)

            # Update visualization state
            if self.enable_live_map:
                self.live_map.set_status("NAVIGATING")

            if pid_success:
                print("\n[PID] ✓ Precise position achieved!")

                # Now land
                print("\n[FINAL] Landing at target position...")
                self.drone_ctrl.send_command("land")
                time.sleep(3)

                # Mark mission complete on map
                if self.enable_live_map:
                    self.live_map.set_status("COMPLETE")
                    time.sleep(2)  # Let user see final state

                print("\n" + "="*60)
                print("NAVIGATION COMPLETED SUCCESSFULLY")
                print("="*60)
                print(f"Final position: ({x:.0f}, {y:.0f}) cm")
                print(f"Final orientation: {theta:.0f}°")
                print("="*60 + "\n")
                return True
            else:
                print("\n[PID] ! Position correction incomplete, landing anyway...")
                self.drone_ctrl.send_command("land")
                time.sleep(3)

                if self.enable_live_map:
                    self.live_map.set_status("INCOMPLETE")
                    time.sleep(2)

                return False

        return success

    def _pid_with_viz(self, pid_controller, x, y, z, yaw) -> bool:
        """PID control with visualization updates."""
        # Hook into PID's execute_control_step to update viz
        original_execute = pid_controller.execute_control_step

        def wrapped_execute(outputs):
            # Update position on map before executing
            pos = pid_controller.position_est.get_position()
            self.live_map.update_position(
                pos['x'], pos['y'], pos['z'],
                pid_controller.position_est.yaw
            )

            # Execute movement
            result = original_execute(outputs)

            # Update position after executing
            pos = pid_controller.position_est.get_position()
            self.live_map.update_position(
                pos['x'], pos['y'], pos['z'],
                pid_controller.position_est.yaw
            )

            return result

        # Replace method temporarily
        pid_controller.execute_control_step = wrapped_execute

        try:
            return pid_controller.move_to_target(x, y, z, yaw)
        finally:
            # Restore original
            pid_controller.execute_control_step = original_execute

    def cleanup(self):
        """Clean shutdown"""
        print("\n[CLEANUP] Shutting down...")
        if self.live_map:
            self.live_map.stop()
        if self.video_handler:
            self.video_handler.stop_stream()
        if self.drone_ctrl:
            self.drone_ctrl.stop_video_stream()
            self.drone_ctrl.disconnect()
        print("[+] Shutdown complete")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Navigate Tello drone to target position with obstacle avoidance"
    )
    parser.add_argument("--x", type=float, help="Target X position in cm (forward/back)")
    parser.add_argument("--y", type=float, help="Target Y position in cm (left/right)")
    parser.add_argument("--theta", type=float, default=0, help="Target orientation in degrees (default: 0)")
    parser.add_argument("--altitude", type=float, default=120, help="Flight altitude in cm (default: 120)")
    parser.add_argument("--no-stream", action="store_true", help="Disable HTTP video streaming")
    parser.add_argument("--no-map", action="store_true", help="Disable live navigation map")
    parser.add_argument("--port", type=int, default=8080, help="HTTP streaming port (default: 8080)")
    parser.add_argument("--no_telemetry", action="store_true", help="Disable telemetry sync after rotations (prevents infinite loops)")
    parser.add_argument("--wasd", action="store_true", help="Enable manual WASD control mode with dashboard visualization")

    args = parser.parse_args()

    # Create navigator
    navigator = SimpleNavigator(
        enable_video_stream=not args.no_stream,
        http_port=args.port,
        enable_live_map=not args.no_map
    )

    # Apply telemetry configuration
    if args.no_telemetry:
        navigator.config.ENABLE_TELEMETRY_SYNC = False
        print("[CONFIG] Telemetry sync disabled - using pure dead reckoning for rotations")

    try:
        # Connect to drone
        if not navigator.connect():
            print("[!] Connection failed - exiting")
            return 1

        # Check if WASD mode is enabled
        if args.wasd:
            print("\n[MODE] Manual WASD control mode activated")
            return manual_control_mode(navigator, args.altitude)

        # Validate required arguments for navigation mode
        if args.x is None or args.y is None:
            print("[!] Error: --x and --y are required for navigation mode")
            print("    Use --wasd for manual control mode (no coordinates needed)")
            return 1

        # Navigate to target
        success = navigator.navigate_to(
            x=args.x,
            y=args.y,
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
            navigator.drone_ctrl.send_command("emergency")
        except:
            pass
        return 1

    except Exception as e:
        print(f"\n[!] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        try:
            navigator.drone_ctrl.send_command("emergency")
        except:
            pass
        return 1

    finally:
        navigator.cleanup()


if __name__ == "__main__":
    sys.exit(main())
