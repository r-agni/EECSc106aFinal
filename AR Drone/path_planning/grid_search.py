"""
Grid Search Path Planner

Implements a systematic grid-based search pattern for finding ARuco markers.
Uses a snake-like pattern to efficiently cover the search area.
"""

import numpy as np


class GridSearchPlanner:
    """
    Grid-based search pattern generator with multiple search strategies.
    """

    def __init__(self, grid_size=5, cell_size_m=0.5, pattern='snake'):
        """
        Initialize grid search planner.

        Args:
            grid_size: Number of cells per side (NxN grid)
            cell_size_m: Size of each cell in meters
            pattern: Search pattern ('snake', 'spiral', 'outward')
        """
        self.grid_size = grid_size
        self.cell_size_m = cell_size_m
        self.pattern = pattern

        # Generate waypoint sequence
        self.waypoints = self._generate_pattern(pattern)
        self.current_waypoint_idx = 0

        # Grid state map
        # 0 = unexplored, 1 = exploring, 2 = explored, 3 = target found
        self.grid_map = np.zeros((grid_size, grid_size), dtype=int)

        # Drone position (grid coordinates)
        self.drone_start_cell = (grid_size // 2, grid_size // 2)
        self.drone_cell = list(self.drone_start_cell)

        # Target position (when found)
        self.target_cell = None
        self.target_found = False

        print(f"[GRID] Initialized {grid_size}x{grid_size} grid with {pattern} pattern")
        print(f"[GRID] Cell size: {cell_size_m}m, Total waypoints: {len(self.waypoints)}")

    def _generate_pattern(self, pattern):
        """Generate waypoint sequence based on pattern type."""
        if pattern == 'snake':
            return self._generate_snake_pattern()
        elif pattern == 'spiral':
            return self._generate_spiral_pattern()
        elif pattern == 'outward':
            return self._generate_outward_pattern()
        else:
            raise ValueError(f"Unknown pattern: {pattern}")

    def _generate_snake_pattern(self):
        """
        Generate snake/boustrophedon search pattern.
        Moves left-to-right on even rows, right-to-left on odd rows.
        """
        waypoints = []
        for row in range(self.grid_size):
            if row % 2 == 0:
                # Left to right
                for col in range(self.grid_size):
                    waypoints.append((row, col))
            else:
                # Right to left
                for col in range(self.grid_size - 1, -1, -1):
                    waypoints.append((row, col))
        return waypoints

    def _generate_spiral_pattern(self):
        """Generate spiral search pattern from outside to center."""
        waypoints = []
        top, bottom = 0, self.grid_size - 1
        left, right = 0, self.grid_size - 1

        while top <= bottom and left <= right:
            # Top row (left to right)
            for col in range(left, right + 1):
                waypoints.append((top, col))
            top += 1

            # Right column (top to bottom)
            for row in range(top, bottom + 1):
                waypoints.append((row, right))
            right -= 1

            # Bottom row (right to left)
            if top <= bottom:
                for col in range(right, left - 1, -1):
                    waypoints.append((bottom, col))
                bottom -= 1

            # Left column (bottom to top)
            if left <= right:
                for row in range(bottom, top - 1, -1):
                    waypoints.append((row, left))
                left += 1

        return waypoints

    def _generate_outward_pattern(self):
        """Generate outward spiral pattern from center."""
        waypoints = []
        center = self.grid_size // 2
        waypoints.append((center, center))

        for radius in range(1, (self.grid_size + 1) // 2 + 1):
            # Top edge
            row = max(0, center - radius)
            for col in range(max(0, center - radius), min(self.grid_size, center + radius + 1)):
                if (row, col) not in waypoints and 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    waypoints.append((row, col))

            # Bottom edge
            row = min(self.grid_size - 1, center + radius)
            for col in range(max(0, center - radius), min(self.grid_size, center + radius + 1)):
                if (row, col) not in waypoints and 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    waypoints.append((row, col))

            # Left edge
            col = max(0, center - radius)
            for row in range(max(0, center - radius), min(self.grid_size, center + radius + 1)):
                if (row, col) not in waypoints and 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    waypoints.append((row, col))

            # Right edge
            col = min(self.grid_size - 1, center + radius)
            for row in range(max(0, center - radius), min(self.grid_size, center + radius + 1)):
                if (row, col) not in waypoints and 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    waypoints.append((row, col))

        return waypoints

    def get_next_waypoint(self):
        """
        Get next waypoint in search sequence.

        Returns:
            (row, col) or None if search complete
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            return None

        waypoint = self.waypoints[self.current_waypoint_idx]
        self.current_waypoint_idx += 1
        return waypoint

    def get_current_waypoint(self):
        """Get current waypoint without advancing."""
        if self.current_waypoint_idx == 0:
            return None
        return self.waypoints[self.current_waypoint_idx - 1]

    def peek_next_waypoint(self):
        """Preview next waypoint without advancing."""
        if self.current_waypoint_idx >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_idx]

    def mark_exploring(self, row, col):
        """Mark cell as currently being explored."""
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            self.grid_map[row, col] = 1

    def mark_explored(self, row, col):
        """Mark cell as fully explored."""
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            if self.grid_map[row, col] != 3:  # Don't override target found
                self.grid_map[row, col] = 2

    def mark_target_found(self, row, col):
        """Mark cell where target was found."""
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            self.grid_map[row, col] = 3
            self.target_cell = (row, col)
            self.target_found = True
            print(f"[GRID] Target found at cell ({row}, {col})")

    def update_drone_position(self, row, col):
        """Update current drone grid position."""
        self.drone_cell = [row, col]

    def cell_to_world(self, row, col):
        """
        Convert grid cell to world coordinates (meters from start).

        Args:
            row, col: Grid cell coordinates

        Returns:
            (x_m, y_m): Position in meters
        """
        # Calculate offset from starting cell
        start_row, start_col = self.drone_start_cell
        row_offset = row - start_row
        col_offset = col - start_col

        # Convert to meters
        x_m = col_offset * self.cell_size_m
        y_m = row_offset * self.cell_size_m

        return x_m, y_m

    def world_to_cell(self, x_m, y_m):
        """
        Convert world coordinates to grid cell.

        Args:
            x_m, y_m: Position in meters from start

        Returns:
            (row, col): Grid cell coordinates
        """
        start_row, start_col = self.drone_start_cell

        # Convert meters to cell offset
        col_offset = round(x_m / self.cell_size_m)
        row_offset = round(y_m / self.cell_size_m)

        # Calculate absolute cell
        row = start_row + row_offset
        col = start_col + col_offset

        # Clamp to grid bounds
        row = max(0, min(row, self.grid_size - 1))
        col = max(0, min(col, self.grid_size - 1))

        return row, col

    def get_progress(self):
        """
        Get search progress statistics.

        Returns:
            dict with 'explored_cells', 'total_cells', 'percent_complete'
        """
        explored_count = np.sum(self.grid_map >= 2)  # Explored or target found
        total_cells = self.grid_size * self.grid_size
        percent = (explored_count / total_cells) * 100

        return {
            'explored_cells': int(explored_count),
            'total_cells': total_cells,
            'percent_complete': percent,
            'target_found': self.target_found
        }

    def is_search_complete(self):
        """Check if search is complete (all waypoints visited or target found)."""
        return self.target_found or (self.current_waypoint_idx >= len(self.waypoints))

    def reset(self):
        """Reset search state."""
        self.current_waypoint_idx = 0
        self.grid_map = np.zeros((self.grid_size, self.grid_size), dtype=int)
        self.drone_cell = list(self.drone_start_cell)
        self.target_cell = None
        self.target_found = False
        print("[GRID] Search state reset")

    def get_state_summary(self):
        """Get human-readable state summary."""
        progress = self.get_progress()
        current_wp = self.get_current_waypoint()

        summary = (
            f"Grid {self.grid_size}x{self.grid_size} | "
            f"Progress: {progress['percent_complete']:.1f}% "
            f"({progress['explored_cells']}/{progress['total_cells']}) | "
            f"Waypoint: {self.current_waypoint_idx}/{len(self.waypoints)}"
        )

        if self.target_found:
            summary += f" | TARGET FOUND at {self.target_cell}"

        return summary
