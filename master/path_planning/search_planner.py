"""
Search pattern planning for ArUco tag discovery.

Generates grid-based waypoints for systematic exploration of search area.
"""

import math
from typing import List, Tuple, Dict
from .config import PathPlanningConfig


class SearchPlanner:
    """Generates search waypoints for systematic area exploration"""

    def __init__(self, config: PathPlanningConfig):
        """
        Initialize search planner.

        Args:
            config: PathPlanningConfig instance
        """
        self.config = config

        # Search area dimensions (convert meters to cm)
        self.area_width_cm, self.area_height_cm = config.get_search_area_cm()
        self.grid_cell_size_cm = config.GRID_CELL_SIZE_M * 100
        self.search_altitude_cm = config.SEARCH_ALTITUDE_M * 100

    def generate_grid_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Generate grid waypoints covering the search area.

        Uses lawnmower pattern (sweep left-to-right, step forward, repeat).

        Returns:
            List of waypoint tuples (x, y, z) in cm
        """
        waypoints = []

        # Calculate number of grid cells
        num_x = math.ceil(self.area_width_cm / self.grid_cell_size_cm)
        num_y = math.ceil(self.area_height_cm / self.grid_cell_size_cm)

        print(f"[SEARCH PLANNER] Generating {num_x}x{num_y} grid ({num_x*num_y} waypoints)")
        print(f"[SEARCH PLANNER] Area: {self.area_width_cm/100:.1f}m x {self.area_height_cm/100:.1f}m")
        print(f"[SEARCH PLANNER] Grid spacing: {self.grid_cell_size_cm/100:.1f}m")

        # Generate lawnmower pattern
        for y_idx in range(num_y):
            y = y_idx * self.grid_cell_size_cm

            if y_idx % 2 == 0:
                # Even rows: sweep left to right
                x_range = range(num_x)
            else:
                # Odd rows: sweep right to left (more efficient)
                x_range = range(num_x - 1, -1, -1)

            for x_idx in x_range:
                x = x_idx * self.grid_cell_size_cm
                waypoints.append((x, y, self.search_altitude_cm))

        return waypoints

    def generate_waypoints_with_rotation_scan(self) -> List[Dict]:
        """
        Generate waypoints with rotation scan commands.

        At each grid position, adds rotation commands to scan all directions.

        Returns:
            List of waypoint dicts with keys:
            - type: 'move' or 'rotate'
            - position: (x, y, z) for move commands
            - angle: rotation angle for rotate commands
            - hover_time: hover duration for move commands
        """
        grid_waypoints = self.generate_grid_waypoints()
        waypoints_with_scans = []

        if not self.config.ENABLE_ROTATION_SCAN:
            # Just return move waypoints without rotation
            for pos in grid_waypoints:
                waypoints_with_scans.append({
                    'type': 'move',
                    'position': pos,
                    'hover_time': self.config.WAYPOINT_HOVER_TIME_SEC
                })
            return waypoints_with_scans

        # Add rotation scans at each grid position
        rotation_step = 360 / self.config.ROTATION_SCAN_STEPS

        for idx, pos in enumerate(grid_waypoints):
            # Move to grid position
            waypoints_with_scans.append({
                'type': 'move',
                'position': pos,
                'hover_time': self.config.WAYPOINT_HOVER_TIME_SEC,
                'waypoint_id': idx
            })

            # Add rotation scan (4 rotations of 90 degrees)
            for step in range(self.config.ROTATION_SCAN_STEPS):
                waypoints_with_scans.append({
                    'type': 'rotate',
                    'angle': rotation_step,
                    'waypoint_id': idx,
                    'scan_step': step
                })

        return waypoints_with_scans

    def generate_spiral_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Generate spiral pattern waypoints (alternative to grid).

        Starts at center and spirals outward.

        Returns:
            List of waypoint tuples (x, y, z) in cm
        """
        waypoints = []
        center_x = self.area_width_cm / 2
        center_y = self.area_height_cm / 2

        # Spiral parameters
        step = self.grid_cell_size_cm
        max_radius = max(center_x, center_y)

        x, y = center_x, center_y
        dx, dy = step, 0
        segment_length = 1

        waypoints.append((x, y, self.search_altitude_cm))

        while math.sqrt((x - center_x)**2 + (y - center_y)**2) < max_radius:
            for _ in range(2):  # Two segments per spiral level
                for _ in range(segment_length):
                    x += dx
                    y += dy

                    # Check if still within bounds
                    if 0 <= x <= self.area_width_cm and 0 <= y <= self.area_height_cm:
                        waypoints.append((x, y, self.search_altitude_cm))

                # Rotate direction
                dx, dy = -dy, dx

            segment_length += 1

        return waypoints

    def estimate_exploration_time(self, waypoints: List) -> float:
        """
        Estimate time to complete exploration of all waypoints.

        Args:
            waypoints: List of waypoint dicts or tuples

        Returns:
            float: Estimated time in seconds
        """
        total_time = 0.0

        # Movement time between waypoints
        prev_pos = (0, 0, 0)  # Start position
        movement_count = 0

        for wp in waypoints:
            if isinstance(wp, dict):
                if wp['type'] == 'move':
                    current_pos = wp['position']
                    distance_cm = math.sqrt(
                        (current_pos[0] - prev_pos[0])**2 +
                        (current_pos[1] - prev_pos[1])**2 +
                        (current_pos[2] - prev_pos[2])**2
                    )
                    distance_m = distance_cm / 100.0

                    # Time = distance / speed + overhead + hover time
                    total_time += distance_m / self.config.DRONE_SPEED_M_S
                    total_time += self.config.COMMAND_OVERHEAD_SEC
                    total_time += wp.get('hover_time', 0)

                    prev_pos = current_pos
                    movement_count += 1

                elif wp['type'] == 'rotate':
                    # Rotation time
                    angle = wp['angle']
                    total_time += abs(angle) / self.config.ROTATION_SPEED_DEG_S
                    total_time += self.config.COMMAND_OVERHEAD_SEC
            else:
                # Simple tuple waypoint
                current_pos = wp
                distance_cm = math.sqrt(
                    (current_pos[0] - prev_pos[0])**2 +
                    (current_pos[1] - prev_pos[1])**2 +
                    (current_pos[2] - prev_pos[2])**2
                )
                distance_m = distance_cm / 100.0

                total_time += distance_m / self.config.DRONE_SPEED_M_S
                total_time += self.config.COMMAND_OVERHEAD_SEC

                prev_pos = current_pos
                movement_count += 1

        print(f"[SEARCH PLANNER] Estimated exploration time: {total_time:.1f}s ({total_time/60:.1f} min)")
        print(f"[SEARCH PLANNER] Total waypoints: {movement_count}")

        return total_time

    def get_search_coverage_info(self) -> Dict:
        """
        Get information about search coverage.

        Returns:
            Dict with coverage statistics
        """
        num_x = math.ceil(self.area_width_cm / self.grid_cell_size_cm)
        num_y = math.ceil(self.area_height_cm / self.grid_cell_size_cm)

        return {
            'area_width_m': self.area_width_cm / 100.0,
            'area_height_m': self.area_height_cm / 100.0,
            'grid_cell_size_m': self.grid_cell_size_cm / 100.0,
            'grid_cells_x': num_x,
            'grid_cells_y': num_y,
            'total_waypoints': num_x * num_y,
            'search_altitude_m': self.search_altitude_cm / 100.0,
            'pattern': self.config.GRID_PATTERN,
            'rotation_scan_enabled': self.config.ENABLE_ROTATION_SCAN
        }
