"""
Mission controller for autonomous ArUco tag discovery and optimal path execution.

Orchestrates the complete 3-phase mission:
1. Exploration: Search area to find both ArUco tags
2. Optimization: Calculate optimal path from start to finish
3. Execution: Execute optimal path after user confirmation
"""

import time
import sys
from typing import Optional, Dict
from .config import PathPlanningConfig
from .aruco_detector import ArucoDetector
from .search_planner import SearchPlanner
from .movement_recorder import MovementRecorder
from .path_optimizer import PathOptimizer
from .visualizer import PathVisualizer

# Import from existing obstacle_avoidance system
sys.path.append('..')
from obstacle_avoidance import PathExecutor, ObstacleDetector, VideoProcessor, Config as ObstacleConfig


class MissionController:
    """Main orchestrator for path planning mission"""

    def __init__(self, drone_controller, state_manager, video_handler, config: Optional[PathPlanningConfig] = None):
        """
        Initialize mission controller.

        Args:
            drone_controller: DroneController instance from tello_server
            state_manager: StateManager instance from tello_server
            video_handler: VideoStreamHandler instance from tello_server
            config: Optional PathPlanningConfig (creates default if None)
        """
        self.drone = drone_controller
        self.state = state_manager
        self.video_handler = video_handler

        # Configuration
        self.config = config if config else PathPlanningConfig()

        # Initialize components
        self.aruco_detector = ArucoDetector(self.config)
        self.search_planner = SearchPlanner(self.config)
        self.movement_recorder = MovementRecorder(self.config)
        self.path_optimizer = PathOptimizer(self.config)
        self.visualizer = PathVisualizer(self.config)

        # Use existing obstacle avoidance system
        obstacle_config = ObstacleConfig()
        self.path_executor = PathExecutor(
            drone_controller,
            state_manager,
            video_handler,
            obstacle_config
        )

        # Video processor for frame access
        self.video_proc = VideoProcessor(video_handler)

        # Obstacle detector for parallel detection
        self.obstacle_detector = ObstacleDetector(obstacle_config)

        # Mission state
        self.mission_active = False
        self.tags_found = {0: False, 1: False}
        self.tag_positions = {}

        # Wrap PathExecutor to record movements
        self._wrap_path_executor()

        print("[MISSION CONTROLLER] Initialized")
        print(f"[MISSION CONTROLLER] Search area: {self.config.SEARCH_AREA_WIDTH_M}m x {self.config.SEARCH_AREA_HEIGHT_M}m")

    def _wrap_path_executor(self):
        """Wrap PathExecutor's command execution to record movements"""
        original_send_command = self.drone.send_command

        def wrapped_send_command(command, **params):
            # Get current state before command
            position_before = self.path_executor.position.get_position()
            yaw_before = self.path_executor.position.yaw

            # Record movement start
            self.movement_recorder.record_movement_start(
                command, params, position_before, yaw_before
            )

            # Execute command
            start_time = time.time()
            success, msg = original_send_command(command, **params)
            duration = time.time() - start_time

            # Get state after command
            position_after = self.path_executor.position.get_position()
            yaw_after = self.path_executor.position.yaw

            # Record movement end
            self.movement_recorder.record_movement_end(
                success, position_after, yaw_after, duration
            )

            return success, msg

        # Apply wrapper
        self.drone.send_command = wrapped_send_command

    def run_mission(self) -> bool:
        """
        Run complete path planning mission.

        Returns:
            bool: True if mission completed successfully
        """
        print("\n" + "="*60)
        print("  ARUCO TAG DISCOVERY & PATH PLANNING MISSION")
        print("="*60)

        # Pre-flight checks
        if not self._preflight_checks():
            return False

        # PHASE 1: Exploration
        print("\n[PHASE 1] EXPLORATION")
        if not self._exploration_phase():
            print("[MISSION] Exploration failed!")
            return False

        # PHASE 2: Optimization
        print("\n[PHASE 2] PATH OPTIMIZATION")
        optimal_path = self._optimization_phase()
        if not optimal_path:
            print("[MISSION] Path optimization failed!")
            return False

        # PHASE 3: Visualization & User Confirmation
        print("\n[PHASE 3] VISUALIZATION")
        self._visualization_phase(optimal_path)

        # User confirmation
        if self.config.REQUIRE_USER_CONFIRMATION:
            response = input("\n[MISSION] Execute optimal path? (y/n): ")
            if response.lower() != 'y':
                print("[MISSION] User cancelled execution")
                return True  # Still successful, just not executed

        # PHASE 4: Execute Optimal Path
        print("\n[PHASE 4] OPTIMAL PATH EXECUTION")
        if not self._execution_phase(optimal_path):
            print("[MISSION] Execution failed!")
            return False

        print("\n[MISSION] Mission completed successfully!")
        return True

    def _preflight_checks(self) -> bool:
        """Perform pre-flight safety checks"""
        print("\n[PREFLIGHT] Running safety checks...")

        # Check battery
        state = self.state.get_state()
        battery = state.get('battery', 0)

        if battery < self.config.MIN_BATTERY_FOR_SEARCH:
            print(f"[PREFLIGHT] FAIL: Battery too low ({battery}% < {self.config.MIN_BATTERY_FOR_SEARCH}%)")
            return False

        print(f"[PREFLIGHT] Battery: {battery}% ✓")

        # Check connection
        connection_status = state.get('connection_status', 'disconnected')
        if connection_status != 'connected':
            print(f"[PREFLIGHT] FAIL: Drone not connected ({connection_status})")
            return False

        print(f"[PREFLIGHT] Connection: {connection_status} ✓")
        print("[PREFLIGHT] All checks passed ✓")
        return True

    def _exploration_phase(self) -> bool:
        """
        Phase 1: Explore search area to find both ArUco tags.

        Returns:
            bool: True if both tags found
        """
        # Generate search waypoints
        waypoints = self.search_planner.generate_waypoints_with_rotation_scan()

        print(f"[EXPLORATION] Generated {len(waypoints)} waypoints")

        # Estimate time
        estimated_time = self.search_planner.estimate_exploration_time(waypoints)
        print(f"[EXPLORATION] Estimated time: {estimated_time:.1f}s ({estimated_time/60:.1f} min)")

        if estimated_time > self.config.MAX_SEARCH_TIME_SEC:
            print(f"[EXPLORATION WARNING] Estimated time exceeds limit!")

        # Get current battery
        battery = self.state.get_state().get('battery', 0)

        # Start recording
        self.movement_recorder.start_mission(
            self.config.SEARCH_AREA_WIDTH_M,
            self.config.SEARCH_AREA_HEIGHT_M,
            battery
        )

        # Takeoff
        print("[EXPLORATION] Taking off...")
        success, _ = self.drone.send_command("takeoff")
        if not success:
            print("[EXPLORATION] Takeoff failed!")
            return False

        time.sleep(3)  # Wait for stabilization

        # Update position to search altitude
        self.path_executor.position.pos['z'] = self.config.SEARCH_ALTITUDE_M * 100

        # Execute waypoints
        self.mission_active = True
        mission_start = time.time()

        for idx, waypoint in enumerate(waypoints):
            # Check timeout
            if time.time() - mission_start > self.config.EXPLORATION_TIMEOUT_SEC:
                print("[EXPLORATION] Timeout reached!")
                break

            # Check if both tags found (early termination)
            if self.aruco_detector.both_tags_found():
                print(f"\n[EXPLORATION] Both tags found! Early termination at waypoint {idx}/{len(waypoints)}")
                break

            # Execute waypoint
            if waypoint['type'] == 'move':
                self._execute_move_waypoint(waypoint)
            elif waypoint['type'] == 'rotate':
                self._execute_rotation(waypoint)

            # Process frames for ArUco + obstacle detection
            self._process_detection_frame()

        # Land
        print("\n[EXPLORATION] Landing...")
        self.drone.send_command("land")
        time.sleep(3)

        # End recording
        final_battery = self.state.get_state().get('battery', 0)
        self.movement_recorder.end_mission(final_battery)

        # Save movement log
        self.movement_recorder.save_to_file()

        # Check if both tags found
        if self.aruco_detector.both_tags_found():
            self.tag_positions = self.aruco_detector.get_confirmed_tag_positions()
            print(f"[EXPLORATION] Success! Both tags located:")
            print(f"  Tag 0 (start): {self.tag_positions[0]}")
            print(f"  Tag 1 (finish): {self.tag_positions[1]}")
            return True
        else:
            tag_status = self.aruco_detector.get_tag_status()
            print(f"[EXPLORATION] Failed to find both tags: {tag_status}")
            return False

    def _execute_move_waypoint(self, waypoint: Dict):
        """Execute movement to waypoint"""
        target_pos = waypoint['position']  # (x, y, z) tuple in cm
        current_pos = self.path_executor.position.get_position()

        # Calculate movement needed
        dx = target_pos[0] - current_pos['x']
        dy = target_pos[1] - current_pos['y']
        dz = target_pos[2] - current_pos['z']

        # Move to position (using PathExecutor for obstacle avoidance)
        waypoint_list = [(target_pos[0], target_pos[1], target_pos[2])]
        self.path_executor.execute_path(waypoint_list)

        # Hover for scanning
        hover_time = waypoint.get('hover_time', 0)
        if hover_time > 0:
            time.sleep(hover_time)

    def _execute_rotation(self, waypoint: Dict):
        """Execute rotation for scanning"""
        angle = waypoint['angle']

        if angle > 0:
            command = "rotate_cw"
        else:
            command = "rotate_ccw"
            angle = abs(angle)

        self.drone.send_command(command, degrees=int(angle))
        time.sleep(0.5)  # Brief pause after rotation

    def _process_detection_frame(self):
        """Process current frame for ArUco and obstacle detection"""
        frame = self.video_proc.get_latest_frame()
        if frame is None:
            return

        # Get current drone state
        current_pos = self.path_executor.position.get_position()
        current_yaw = self.path_executor.position.yaw

        # ArUco detection
        detected_tags = self.aruco_detector.detect_tags(frame, current_pos, current_yaw)

        # Record detections
        for tag_info in detected_tags:
            self.movement_recorder.record_aruco_detection(tag_info)

        # Note: Obstacle detection is handled automatically by PathExecutor

    def _optimization_phase(self) -> Optional[Dict]:
        """
        Phase 2: Calculate optimal path from start to finish tag.

        Returns:
            Dict with optimal path info, or None if failed
        """
        if 0 not in self.tag_positions or 1 not in self.tag_positions:
            print("[OPTIMIZATION] Missing tag positions!")
            return None

        start_pos = self.tag_positions[0]
        goal_pos = self.tag_positions[1]
        movement_log = self.movement_recorder.get_movement_log()

        # Run A* optimization
        optimal_path = self.path_optimizer.calculate_optimal_path(
            start_pos, goal_pos, movement_log
        )

        return optimal_path

    def _visualization_phase(self, optimal_path: Dict):
        """Phase 3: Visualize exploration vs optimal path"""
        movement_log = self.movement_recorder.get_movement_log()

        self.visualizer.visualize_paths(
            movement_log,
            optimal_path,
            self.tag_positions
        )

        # Print statistics summary
        print(self.movement_recorder.get_statistics_summary())

    def _execution_phase(self, optimal_path: Dict) -> bool:
        """
        Phase 4: Execute the optimal path.

        Args:
            optimal_path: Optimal path dict from optimizer

        Returns:
            bool: True if execution successful
        """
        if not optimal_path or 'waypoints' not in optimal_path:
            print("[EXECUTION] No valid optimal path!")
            return False

        waypoints = optimal_path['waypoints']
        print(f"[EXECUTION] Executing {len(waypoints)} waypoints...")

        # Takeoff
        print("[EXECUTION] Taking off...")
        success, _ = self.drone.send_command("takeoff")
        if not success:
            print("[EXECUTION] Takeoff failed!")
            return False

        time.sleep(3)

        # Reset position estimator
        self.path_executor.position.reset_position()

        # Execute optimal path using PathExecutor
        try:
            self.path_executor.execute_path(waypoints)
        except Exception as e:
            print(f"[EXECUTION] Error during execution: {e}")
            self.drone.send_command("land")
            return False

        # Land at goal
        print("[EXECUTION] Landing at goal...")
        self.drone.send_command("land")
        time.sleep(3)

        print("[EXECUTION] Optimal path executed successfully!")
        return True

    def stop(self):
        """Emergency stop mission"""
        self.mission_active = False
        print("[MISSION] Emergency stop!")
        self.drone.send_command("emergency")
