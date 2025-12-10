"""
Main Orchestrator - Grid Search ARuco Detection with Obstacle Avoidance

This is the main script that orchestrates all modules:
- Path Planning: Grid search pattern
- ARuco Detection: Finding target marker ID 1 using bottom camera
- Obstacle Detection: YOLO-based obstacle avoidance using front camera
- PID Control: Smooth navigation towards detected marker
- Visualization: Real-time grid display

The script switches between cameras intermittently since both cannot stream simultaneously.
"""

import time
import sys
import cv2
import msvcrt
import numpy as np

# AR Drone library
import pyardrone
from pyardrone import at

# Import our custom modules
from aruco_detection import ArucoDetector, TransformCalculator
from object_detection import ObstacleDetector
from pid_controller import PIDController
from path_planning import GridSearchPlanner, GridVisualizer


# ==================== DRONE WRAPPER ====================
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """AR Drone without internal video connection."""
    pass


# ==================== CONFIGURATION ====================
class Config:
    """Mission configuration parameters."""

    # Target settings
    TARGET_ARUCO_ID = 1
    TARGET_ALTITUDE_MM = 1000  # 1 meter

    # Grid search settings
    GRID_SIZE = 5
    CELL_SIZE_M = 0.5
    SEARCH_PATTERN = 'snake'  # 'snake', 'spiral', 'outward'

    # Camera settings
    CAMERA_SWITCH_INTERVAL_S = 2.0  # Switch every 2 seconds
    STREAM_URL = "tcp://192.168.1.1:5555"
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480

    # PID settings
    PID_X_GAINS = (0.3, 0.0, 0.1)
    PID_Y_GAINS = (0.3, 0.0, 0.1)
    PID_OUTPUT_LIMITS = (-0.3, 0.3)

    # Navigation settings
    ARRIVAL_THRESHOLD_M = 0.15
    WAYPOINT_DWELL_TIME_S = 3.0  # Time to search each cell

    # YOLO settings
    YOLO_MODEL_PATH = 'yolov8n.pt'
    YOLO_CONFIDENCE = 0.5
    OBSTACLE_CLOSE_THRESHOLD = 0.1  # Area ratio threshold

    # Mission settings
    MAX_FLIGHT_TIME_S = 300  # 5 minutes max
    HOVER_COMMAND_INTERVAL_S = 0.5


# ==================== MISSION STATES ====================
class MissionState:
    """Mission state machine states."""
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    CLIMB = "CLIMB"
    SEARCH = "SEARCH"
    NAVIGATE_TO_TARGET = "NAVIGATE_TO_TARGET"
    ARRIVED = "ARRIVED"
    LAND = "LAND"
    COMPLETE = "COMPLETE"


# ==================== MAIN ORCHESTRATOR ====================
class MissionOrchestrator:
    """Main orchestrator coordinating all mission components."""

    def __init__(self):
        """Initialize mission orchestrator."""
        print("="*70)
        print("  GRID SEARCH - ARUCO TAG DETECTION WITH OBSTACLE AVOIDANCE")
        print("="*70)

        # Initialize drone
        print("\n[INIT] Connecting to AR.Drone...")
        self.drone = ARDroneNoVideo()
        print("[INIT] Connected.")

        # Wait for navdata
        print("[INIT] Waiting for navigation data...")
        self.drone.navdata_ready.wait(timeout=5.0)

        if self.drone.navdata_ready.is_set():
            self.drone.send(at.CONFIG('general:navdata_demo', True))
            time.sleep(0.1)
            demo = getattr(self.drone.navdata, "demo", None)
            if demo:
                print(f"[INIT] Battery: {demo.vbat_flying_percentage}%")

        # Initialize modules
        print("\n[INIT] Initializing modules...")
        self.aruco_detector = ArucoDetector(target_id=Config.TARGET_ARUCO_ID)
        self.obstacle_detector = ObstacleDetector(
            model_path=Config.YOLO_MODEL_PATH,
            confidence_threshold=Config.YOLO_CONFIDENCE,
            close_threshold=Config.OBSTACLE_CLOSE_THRESHOLD
        )
        self.transform_calc = TransformCalculator(
            frame_width=Config.FRAME_WIDTH,
            frame_height=Config.FRAME_HEIGHT
        )
        self.planner = GridSearchPlanner(
            grid_size=Config.GRID_SIZE,
            cell_size_m=Config.CELL_SIZE_M,
            pattern=Config.SEARCH_PATTERN
        )
        self.visualizer = GridVisualizer(
            grid_size=Config.GRID_SIZE,
            cell_size_m=Config.CELL_SIZE_M
        )

        # PID controllers
        self.pid_x = PIDController(*Config.PID_X_GAINS, output_limits=Config.PID_OUTPUT_LIMITS)
        self.pid_y = PIDController(*Config.PID_Y_GAINS, output_limits=Config.PID_OUTPUT_LIMITS)

        # State variables
        self.state = MissionState.INIT
        self.current_camera = 'bottom'  # Start with bottom for ARuco
        self.last_camera_switch = time.time()
        self.mission_start_time = time.time()
        self.last_hover_time = time.time()

        # Waypoint tracking
        self.current_waypoint = None
        self.waypoint_start_time = None

        # Target tracking
        self.target_transform = None
        self.drone_position = np.array([0.0, 0.0])  # x, y in meters
        self.drone_yaw = 0.0

        # Video stream
        print(f"[INIT] Opening video stream: {Config.STREAM_URL}")
        self.cap = cv2.VideoCapture(Config.STREAM_URL, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open video stream")

        # Frame buffers for both cameras
        self.bottom_frame = None
        self.front_frame = None
        self.last_bottom_frame = None
        self.last_front_frame = None

        print("\n[INIT] Initialization complete!")
        print("\n[CONTROLS]")
        print("  l - Land and exit")
        print("  ESC in video window - Exit")

    def run(self):
        """Main mission loop."""
        try:
            # Takeoff sequence
            self._takeoff()

            # Main mission loop
            while True:
                # Check for user input
                if self._check_user_exit():
                    break

                # Check mission timeout
                if time.time() - self.mission_start_time > Config.MAX_FLIGHT_TIME_S:
                    print("\n[TIMEOUT] Max flight time reached")
                    break

                # Handle camera switching
                self._handle_camera_switching()

                # Read video frame
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.02)
                    continue

                # Process based on current camera and save to appropriate buffer
                if self.current_camera == 'bottom':
                    self.bottom_frame = self._process_aruco_detection(frame.copy())
                    self.last_bottom_frame = self.bottom_frame
                else:  # front camera
                    self.front_frame = self._process_obstacle_detection(frame.copy())
                    self.last_front_frame = self.front_frame

                # Execute mission state machine
                self._execute_state_machine()

                # Periodic hover command
                self._send_periodic_hover()

                # Update visualization
                demo = getattr(self.drone.navdata, "demo", None)
                if demo:
                    self.drone_yaw = demo.psi / 1000.0

                self.visualizer.update(self.planner, self.drone_yaw)

                # Display both video feeds
                exit_requested = self._display_dual_feeds()
                if exit_requested:
                    print("\n[INPUT] ESC pressed")
                    break

                time.sleep(0.01)

            # Landing sequence
            self._land()

        except KeyboardInterrupt:
            print("\n[INTERRUPT] Ctrl+C received")
            self._emergency_land()

        finally:
            self._cleanup()

    def _takeoff(self):
        """Execute takeoff sequence."""
        print("\n[MISSION] Starting takeoff sequence...")
        self.state = MissionState.TAKEOFF

        # Switch to bottom camera
        self.drone.send(at.CONFIG('video:video_channel', 1))
        time.sleep(0.5)

        # Takeoff
        self.drone.takeoff()
        time.sleep(2.0)

        # Climb to target altitude
        print(f"[MISSION] Climbing to {Config.TARGET_ALTITUDE_MM}mm...")
        self.state = MissionState.CLIMB

        start_climb = time.time()
        while True:
            self.drone.move(up=0.4)
            time.sleep(0.05)

            demo = getattr(self.drone.navdata, "demo", None)
            if demo:
                alt = demo.altitude
                print(f"[ALT] {alt} mm", end="\r")
                if alt >= Config.TARGET_ALTITUDE_MM:
                    break

            if time.time() - start_climb > 8:
                print("\n[WARN] Altitude timeout")
                break

        self.drone.hover()
        print(f"\n[MISSION] Hovering at target altitude")
        print("\n[MISSION] Starting grid search...")
        self.state = MissionState.SEARCH
        self.last_hover_time = time.time()

    def _handle_camera_switching(self):
        """Handle intermittent camera switching."""
        if time.time() - self.last_camera_switch > Config.CAMERA_SWITCH_INTERVAL_S:
            if self.current_camera == 'bottom':
                # Switch to front for obstacle detection
                self.current_camera = 'front'
                self.drone.send(at.CONFIG('video:video_channel', 0))
                # print("[CAMERA] → FRONT (obstacle detection)")
            else:
                # Switch to bottom for ARuco detection
                self.current_camera = 'bottom'
                self.drone.send(at.CONFIG('video:video_channel', 1))
                # print("[CAMERA] → BOTTOM (ARuco search)")

            self.last_camera_switch = time.time()
            time.sleep(0.3)  # Wait for camera switch

    def _process_aruco_detection(self, frame):
        """Process ARuco detection on bottom camera."""
        # Detect ARuco markers
        target_marker = self.aruco_detector.find_target(frame)

        if target_marker:
            # Draw detection
            frame = self.aruco_detector.draw_detections(frame, [target_marker])

            # Calculate transform
            demo = getattr(self.drone.navdata, "demo", None)
            altitude = demo.altitude if demo else Config.TARGET_ALTITUDE_MM

            self.target_transform = self.transform_calc.marker_to_drone_transform(
                target_marker['center'],
                altitude,
                self.drone_yaw
            )

            # Update grid
            current_row, current_col = self.planner.drone_cell
            self.planner.mark_target_found(current_row, current_col)

            # Transition to navigation state
            if self.state == MissionState.SEARCH:
                print(f"\n[ARUCO] Target ID {Config.TARGET_ARUCO_ID} FOUND!")
                print(f"[TRANSFORM] {self.transform_calc.get_transform_summary(self.target_transform)}")
                self.state = MissionState.NAVIGATE_TO_TARGET
                self.pid_x.reset()
                self.pid_y.reset()
        else:
            # Just detect all markers for display
            markers = self.aruco_detector.detect(frame)
            if markers:
                frame = self.aruco_detector.draw_detections(frame, markers)

        # Add camera label
        cv2.putText(frame, "BOTTOM CAMERA - ARUCO DETECTION", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return frame

    def _process_obstacle_detection(self, frame):
        """Process obstacle detection on front camera."""
        obstacles = self.obstacle_detector.detect(frame)

        if obstacles:
            # Draw obstacles
            frame = self.obstacle_detector.draw_detections(frame, obstacles)

            # Check for close obstacles
            should_avoid, direction = self.obstacle_detector.should_avoid(obstacles)

            if should_avoid:
                print(f"[OBSTACLE] Close obstacle detected! Suggest: {direction}")
                # In future: implement avoidance maneuver

                # Add warning overlay
                cv2.putText(frame, f"WARNING: OBSTACLE - GO {direction.upper()}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Add camera label
        cv2.putText(frame, "FRONT CAMERA - OBSTACLE DETECTION", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 165, 0), 2)

        return frame

    def _execute_state_machine(self):
        """Execute mission state machine."""
        if self.state == MissionState.SEARCH:
            self._execute_search()

        elif self.state == MissionState.NAVIGATE_TO_TARGET:
            self._execute_navigation()

        elif self.state == MissionState.ARRIVED:
            self._execute_arrived()

    def _execute_search(self):
        """Execute grid search state."""
        # Get or advance to next waypoint
        if self.current_waypoint is None or self._is_waypoint_complete():
            self.current_waypoint = self.planner.get_next_waypoint()

            if self.current_waypoint is None:
                print("\n[SEARCH] Grid search complete - target not found")
                self.state = MissionState.LAND
                return

            # Mark waypoint and update visualization
            row, col = self.current_waypoint
            self.planner.mark_exploring(row, col)
            self.planner.update_drone_position(row, col)
            self.waypoint_start_time = time.time()

            print(f"[SEARCH] Waypoint ({row}, {col}) - {self.planner.get_state_summary()}")

        # Hover at current waypoint (camera will scan for ARuco)
        self.drone.hover()
        self.last_hover_time = time.time()

    def _execute_navigation(self):
        """Execute navigation to target state."""
        if self.target_transform is None:
            # Lost target, return to search
            print("[NAV] Target lost, resuming search")
            self.state = MissionState.SEARCH
            return

        # Check if arrived
        if self.target_transform['distance_m'] < Config.ARRIVAL_THRESHOLD_M:
            print(f"\n[NAV] Arrived at target!")
            self.state = MissionState.ARRIVED
            self.drone.hover()
            return

        # Calculate PID errors
        errors = self.transform_calc.calculate_pid_errors(self.target_transform)

        # Update PID controllers
        control_x = self.pid_x.update(errors['error_x'])
        control_y = self.pid_y.update(errors['error_y'])

        # Send movement command
        # Note: control mapping depends on drone's coordinate system
        # Assuming: x = left/right, y = forward/backward
        self.drone.move(
            left=-control_x if control_x > 0 else 0,
            right=control_x if control_x < 0 else 0,
            forward=-control_y if control_y > 0 else 0,
            backward=control_y if control_y < 0 else 0
        )
        self.last_hover_time = time.time()

        print(f"[NAV] Distance: {self.target_transform['distance_m']:.3f}m, "
              f"Control: X={control_x:.2f} Y={control_y:.2f}", end='\r')

    def _execute_arrived(self):
        """Execute arrived at target state."""
        # Hover for a moment
        self.drone.hover()
        time.sleep(2)

        # Output final position
        demo = getattr(self.drone.navdata, "demo", None)
        if demo:
            self.drone_yaw = demo.psi / 1000.0

        print("\n" + "="*70)
        print("  TARGET REACHED!")
        print("="*70)
        print(f"[FINAL] Position: x={self.drone_position[0]:.3f}m, "
              f"y={self.drone_position[1]:.3f}m, yaw={self.drone_yaw:.1f}°")
        print("="*70)

        self.state = MissionState.LAND

    def _is_waypoint_complete(self):
        """Check if current waypoint search is complete."""
        if self.waypoint_start_time is None:
            return True

        # Waypoint complete after dwell time
        if time.time() - self.waypoint_start_time > Config.WAYPOINT_DWELL_TIME_S:
            row, col = self.current_waypoint
            self.planner.mark_explored(row, col)
            return True

        return False

    def _send_periodic_hover(self):
        """Send periodic hover command to maintain connection."""
        if time.time() - self.last_hover_time > Config.HOVER_COMMAND_INTERVAL_S:
            if self.state in [MissionState.SEARCH, MissionState.ARRIVED]:
                self.drone.hover()
                self.last_hover_time = time.time()

    def _display_dual_feeds(self):
        """
        Display both camera feeds side-by-side.

        Returns:
            bool: True if exit requested (ESC pressed)
        """
        # Use last known frames if current frame not available
        bottom_display = self.last_bottom_frame if self.last_bottom_frame is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        front_display = self.last_front_frame if self.last_front_frame is not None else np.zeros((480, 640, 3), dtype=np.uint8)

        # If we have current frames, use them
        if self.bottom_frame is not None:
            bottom_display = self.bottom_frame
        if self.front_frame is not None:
            front_display = self.front_frame

        # Add "LIVE" indicator to currently active camera
        active_color = (0, 255, 0)
        inactive_color = (128, 128, 128)

        if self.current_camera == 'bottom':
            cv2.circle(bottom_display, (620, 20), 10, active_color, -1)
            cv2.putText(bottom_display, "LIVE", (570, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, active_color, 2)
            cv2.circle(front_display, (620, 20), 10, inactive_color, -1)
        else:
            cv2.circle(front_display, (620, 20), 10, active_color, -1)
            cv2.putText(front_display, "LIVE", (570, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, active_color, 2)
            cv2.circle(bottom_display, (620, 20), 10, inactive_color, -1)

        # Add state information to bottom feed
        state_text = f"STATE: {self.state}"
        cv2.putText(bottom_display, state_text, (10, bottom_display.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Add progress info to front feed
        progress = self.planner.get_progress()
        progress_text = f"Progress: {progress['percent_complete']:.0f}%"
        cv2.putText(front_display, progress_text, (10, front_display.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Resize to make them fit better (optional)
        # Combine side-by-side
        combined = np.hstack([bottom_display, front_display])

        # Add separator line
        cv2.line(combined, (640, 0), (640, 480), (255, 255, 255), 2)

        # Add title bar
        title_bar = np.zeros((60, combined.shape[1], 3), dtype=np.uint8)
        cv2.putText(title_bar, "AR DRONE DUAL CAMERA VIEW", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

        # Battery and time info
        demo = getattr(self.drone.navdata, "demo", None)
        if demo:
            battery_text = f"Battery: {demo.vbat_flying_percentage}%"
            cv2.putText(title_bar, battery_text, (combined.shape[1] - 200, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Combine title with feeds
        final_display = np.vstack([title_bar, combined])

        # Display
        cv2.imshow("AR Drone - Dual Camera Feed", final_display)

        # Check for ESC key
        key = cv2.waitKey(1) & 0xFF
        return key == 27  # ESC

    def _check_user_exit(self):
        """Check for user exit command."""
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key == 'l':
                print("\n[INPUT] 'l' pressed - landing")
                return True
        return False

    def _land(self):
        """Execute landing sequence."""
        print("\n[MISSION] Landing...")
        self.state = MissionState.LAND
        self.drone.land()
        time.sleep(3.0)
        self.state = MissionState.COMPLETE

        # Final report
        self._print_mission_report()

    def _emergency_land(self):
        """Emergency landing."""
        print("\n[EMERGENCY] Emergency landing...")
        try:
            self.drone.land()
            time.sleep(3.0)
        except:
            pass

    def _print_mission_report(self):
        """Print final mission report."""
        print("\n" + "="*70)
        print("  MISSION SUMMARY")
        print("="*70)

        progress = self.planner.get_progress()
        print(f"[SEARCH] Explored: {progress['explored_cells']}/{progress['total_cells']} cells "
              f"({progress['percent_complete']:.1f}%)")

        if self.planner.target_found:
            print(f"[SUCCESS] ARuco tag {Config.TARGET_ARUCO_ID} found at {self.planner.target_cell}")
            print(f"[POSITION] Final: x={self.drone_position[0]:.3f}m, "
                  f"y={self.drone_position[1]:.3f}m, yaw={self.drone_yaw:.1f}°")
        else:
            print(f"[INCOMPLETE] ARuco tag {Config.TARGET_ARUCO_ID} not found")

        demo = getattr(self.drone.navdata, "demo", None)
        if demo:
            print(f"[BATTERY] Final: {demo.vbat_flying_percentage}%")

        flight_time = time.time() - self.mission_start_time
        print(f"[TIME] Flight duration: {flight_time:.1f}s")

        print("="*70)

    def _cleanup(self):
        """Cleanup resources."""
        print("\n[CLEANUP] Closing resources...")
        self.cap.release()
        cv2.destroyAllWindows()
        self.visualizer.close()
        print("[CLEANUP] Done.")


# ==================== ENTRY POINT ====================
def main():
    """Main entry point."""
    try:
        orchestrator = MissionOrchestrator()
        orchestrator.run()
    except Exception as e:
        print(f"\n[ERROR] Mission failed: {e}")
        import traceback
        traceback.print_exc()
        cv2.destroyAllWindows()
        sys.exit(1)


if __name__ == "__main__":
    main()
