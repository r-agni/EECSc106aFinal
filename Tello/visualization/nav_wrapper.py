"""
Navigation wrapper that feeds data to live map visualization.

Wraps PathExecutor and PID controller to update visualization in real-time.
"""

import time
import math
from typing import List, Tuple, Dict, Optional


class VisualizationWrapper:
    """
    Wraps navigation components to feed live data to visualization.

    Intercepts position updates and obstacle detections to update the map.
    """

    def __init__(self, path_executor, live_map):
        """
        Initialize visualization wrapper.

        Args:
            path_executor: PathExecutor instance
            live_map: LiveNavigationMap instance
        """
        self.executor = path_executor
        self.live_map = live_map
        self.last_update = time.time()
        self.update_interval = 0.1  # Update map every 100ms

    def execute_path_with_viz(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """
        Execute path while updating visualization.

        Args:
            waypoints: List of (x, y, z) waypoints

        Returns:
            True if successful
        """
        # Update status (web dashboard doesn't need planned path)
        self.live_map.set_status("NAVIGATING")

        # Setup waypoint tracking callback
        def waypoint_update_callback(waypoints, current_idx):
            if hasattr(self.live_map, 'update_waypoints'):
                self.live_map.update_waypoints(waypoints, current_idx)

        self.executor.set_waypoint_callback(waypoint_update_callback)

        # Wrap the executor's movement functions
        original_move = self.executor.move_toward_waypoint
        self.executor.move_toward_waypoint = self._wrapped_move_toward_waypoint

        # Store reference to original for calling
        self._original_move = original_move

        try:
            # Execute path (this will use our wrapped function)
            success = self.executor.execute_path(waypoints)
            return success

        finally:
            # Restore original function
            self.executor.move_toward_waypoint = original_move
            self.live_map.set_status("READY")

    def _wrapped_move_toward_waypoint(self, target: Tuple[float, float, float]) -> bool:
        """Wrapper that updates visualization before/after movement."""

        # Update position on map
        pos = self.executor.position.get_position()
        self.live_map.update_position(
            pos['x'], pos['y'], pos['z'],
            self.executor.position.yaw
        )

        # Check for obstacles and add to map
        frame = self.executor.video_proc.get_latest_frame()
        if frame is not None:
            obstacles = self.executor.detector.process_frame(frame)

            for obs in obstacles:
                # Only add obstacles that are somewhat threatening
                if obs['distance_m'] < 4.0:  # Within 4 meters
                    # Calculate obstacle position
                    obs_x, obs_y = self._estimate_obstacle_position(
                        obs['distance_m'],
                        obs['position']  # "left", "center", "right"
                    )

                    self.live_map.add_obstacle(
                        x=obs_x,
                        y=obs_y,
                        distance_m=obs['distance_m'],
                        width_m=obs['width_m'],
                        threat_level=obs['threat_level'],
                        obj_class=obs['class']
                    )

        # Execute actual movement
        result = self._original_move(target)

        # Update position again after movement
        pos = self.executor.position.get_position()
        self.live_map.update_position(
            pos['x'], pos['y'], pos['z'],
            self.executor.position.yaw
        )

        return result

    def _estimate_obstacle_position(self, distance_m: float,
                                   position: str) -> Tuple[float, float]:
        """
        Estimate obstacle position in global coordinates.

        Args:
            distance_m: Distance to obstacle in meters
            position: "left", "center", or "right"

        Returns:
            (x, y) position in cm
        """
        current_pos = self.executor.position.get_position()
        current_yaw = self.executor.position.yaw

        # Bearing offset based on camera position
        bearing_offset = {
            'left': -20,    # 20 degrees left
            'center': 0,    # Straight ahead
            'right': 20     # 20 degrees right
        }.get(position, 0)

        bearing = current_yaw + bearing_offset
        distance_cm = distance_m * 100

        obs_x = current_pos['x'] + distance_cm * math.cos(math.radians(bearing))
        obs_y = current_pos['y'] + distance_cm * math.sin(math.radians(bearing))

        return obs_x, obs_y

    def update_position_continuous(self):
        """Continuously update position (call in navigation loop)."""
        current_time = time.time()
        if current_time - self.last_update >= self.update_interval:
            pos = self.executor.position.get_position()
            self.live_map.update_position(
                pos['x'], pos['y'], pos['z'],
                self.executor.position.yaw
            )
            self.last_update = current_time
