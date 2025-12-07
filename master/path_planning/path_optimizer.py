"""
Path optimization using A* algorithm with time-based cost function.

Calculates the most optimal path from start tag to finish tag considering:
- Distance (shortest)
- Number of movements (fewest)
- Obstacles encountered (avoid obstacle-heavy areas)
Priority: Shortest TIME (balance of all factors)
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from queue import PriorityQueue
from .config import PathPlanningConfig
from .utils import euclidean_distance, euclidean_distance_2d, calculate_heading_change, calculate_path_length


class PathOptimizer:
    """A* path optimization with time-based cost function"""

    def __init__(self, config: PathPlanningConfig):
        """
        Initialize path optimizer.

        Args:
            config: PathPlanningConfig instance
        """
        self.config = config
        self.obstacle_heatmap = None
        self.grid_size_cm = config.GRID_CELL_SIZE_CM

    def calculate_optimal_path(
        self,
        start_pos: Dict,
        goal_pos: Dict,
        movement_log: Dict
    ) -> Dict:
        """
        Calculate optimal path from start to goal using A* search.

        Args:
            start_pos: Start position dict {x, y, z} in cm
            goal_pos: Goal position dict {x, y, z} in cm
            movement_log: Complete exploration movement log

        Returns:
            Dict with keys:
            - waypoints: List of (x, y, z) tuples
            - total_distance_cm: Total path distance
            - estimated_time_sec: Estimated execution time
            - waypoint_count: Number of waypoints
            - cost_breakdown: Dict with distance, rotation, obstacle costs
        """
        print(f"\n[OPTIMIZER] Calculating optimal path...")
        print(f"[OPTIMIZER] Start: ({start_pos['x']:.0f}, {start_pos['y']:.0f})")
        print(f"[OPTIMIZER] Goal: ({goal_pos['x']:.0f}, {goal_pos['y']:.0f})")

        # Build obstacle heatmap from exploration data
        self.obstacle_heatmap = self._build_obstacle_heatmap(movement_log)

        # Run A* search
        waypoints = self._astar_search(start_pos, goal_pos)

        if waypoints is None:
            print("[OPTIMIZER ERROR] No path found!")
            return None

        print(f"[OPTIMIZER] Raw path: {len(waypoints)} waypoints")

        # Smooth path if enabled
        if self.config.SMOOTHING_ENABLED:
            waypoints = self._smooth_path(waypoints)
            print(f"[OPTIMIZER] Smoothed path: {len(waypoints)} waypoints")

        # Calculate statistics
        total_distance = calculate_path_length(waypoints)
        estimated_time = self._estimate_execution_time(waypoints)

        # Calculate cost breakdown
        distance_cost = total_distance / 100.0 / self.config.DRONE_SPEED_M_S
        rotation_cost = self._estimate_rotation_cost(waypoints)
        obstacle_penalty = self._calculate_obstacle_penalty(waypoints)

        print(f"[OPTIMIZER] Optimal path distance: {total_distance:.0f} cm")
        print(f"[OPTIMIZER] Estimated time: {estimated_time:.1f}s ({estimated_time/60:.1f} min)")
        print(f"[OPTIMIZER] Waypoint count: {len(waypoints)}")

        return {
            'waypoints': waypoints,
            'total_distance_cm': total_distance,
            'estimated_time_sec': estimated_time,
            'waypoint_count': len(waypoints),
            'cost_breakdown': {
                'distance_cost': distance_cost,
                'rotation_cost': rotation_cost,
                'obstacle_penalty': obstacle_penalty
            }
        }

    def _build_obstacle_heatmap(self, movement_log: Dict) -> np.ndarray:
        """
        Build 2D heatmap of obstacle encounter frequency.

        Args:
            movement_log: Movement log dict

        Returns:
            2D numpy array with obstacle penalties
        """
        # Determine grid size from search area
        search_area = movement_log['search_area']
        width_cm = search_area['width_m'] * 100
        height_cm = search_area['height_m'] * 100

        grid_width = int(width_cm / self.grid_size_cm) + 1
        grid_height = int(height_cm / self.grid_size_cm) + 1

        heatmap = np.zeros((grid_height, grid_width), dtype=np.float32)

        # Add penalty for each obstacle encounter
        for obstacle_event in movement_log.get('obstacles_encountered', []):
            pos = obstacle_event.get('at_position', {})

            if not pos:
                continue

            # Convert to grid coordinates
            grid_x = int(pos.get('x', 0) / self.grid_size_cm)
            grid_y = int(pos.get('y', 0) / self.grid_size_cm)

            # Bounds check
            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                # Add Gaussian penalty in radius around obstacle
                radius = int(50 / self.grid_size_cm)  # 50cm radius
                self._add_gaussian_penalty(
                    heatmap,
                    grid_x,
                    grid_y,
                    radius,
                    self.config.OBSTACLE_PENALTY_WEIGHT
                )

        print(f"[OPTIMIZER] Built obstacle heatmap: {grid_width}x{grid_height} grid")
        print(f"[OPTIMIZER] Obstacle encounters: {len(movement_log.get('obstacles_encountered', []))}")

        return heatmap

    def _add_gaussian_penalty(self, heatmap, center_x, center_y, radius, weight):
        """Add Gaussian penalty around a point"""
        height, width = heatmap.shape

        for y in range(max(0, center_y - radius), min(height, center_y + radius + 1)):
            for x in range(max(0, center_x - radius), min(width, center_x + radius + 1)):
                distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                if distance <= radius:
                    penalty = weight * math.exp(-(distance**2) / (2 * (radius / 3)**2))
                    heatmap[y, x] += penalty

    def _astar_search(self, start_pos: Dict, goal_pos: Dict) -> Optional[List[Tuple]]:
        """
        A* pathfinding algorithm.

        Args:
            start_pos: Start position dict {x, y, z}
            goal_pos: Goal position dict {x, y, z}

        Returns:
            List of (x, y, z) waypoint tuples, or None if no path found
        """
        # Convert positions to grid coordinates
        start_grid = self._world_to_grid(start_pos)
        goal_grid = self._world_to_grid(goal_pos)

        # Priority queue: (f_score, counter, position)
        open_set = PriorityQueue()
        counter = 0
        open_set.put((0, counter, start_grid))

        # Track paths
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}

        # For tie-breaking
        in_open_set = {start_grid}

        while not open_set.empty():
            _, _, current = open_set.get()
            in_open_set.discard(current)

            # Check if we reached goal
            if self._distance_2d(current, goal_grid) < 2:  # Within 2 grid cells
                # Reconstruct path
                return self._reconstruct_path(came_from, current, start_pos, goal_pos)

            # Explore neighbors
            for neighbor in self._get_neighbors(current):
                # Calculate cost
                move_cost = self._movement_cost(current, neighbor)
                obstacle_penalty = self._get_obstacle_penalty(neighbor)

                tentative_g = g_score[current] + move_cost + obstacle_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_grid)
                    f_score[neighbor] = f

                    if neighbor not in in_open_set:
                        counter += 1
                        open_set.put((f, counter, neighbor))
                        in_open_set.add(neighbor)

        # No path found
        return None

    def _world_to_grid(self, pos: Dict) -> Tuple[int, int]:
        """Convert world position to grid coordinates"""
        return (
            int(pos['x'] / self.grid_size_cm),
            int(pos['y'] / self.grid_size_cm)
        )

    def _grid_to_world(self, grid_pos: Tuple[int, int], z: float = 120.0) -> Tuple[float, float, float]:
        """Convert grid coordinates to world position"""
        return (
            grid_pos[0] * self.grid_size_cm,
            grid_pos[1] * self.grid_size_cm,
            z
        )

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbor grid cells (8-connected)"""
        x, y = pos
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy

                # Bounds check
                if self.obstacle_heatmap is not None:
                    height, width = self.obstacle_heatmap.shape
                    if 0 <= nx < width and 0 <= ny < height:
                        neighbors.append((nx, ny))
                else:
                    neighbors.append((nx, ny))

        return neighbors

    def _movement_cost(self, from_pos: Tuple, to_pos: Tuple) -> float:
        """Calculate time cost of moving between grid positions"""
        distance_grid = math.sqrt(
            (to_pos[0] - from_pos[0])**2 +
            (to_pos[1] - from_pos[1])**2
        )
        distance_cm = distance_grid * self.grid_size_cm
        distance_m = distance_cm / 100.0

        # Time = distance / speed + command overhead
        time_cost = distance_m / self.config.DRONE_SPEED_M_S + self.config.COMMAND_OVERHEAD_SEC
        return time_cost

    def _get_obstacle_penalty(self, grid_pos: Tuple[int, int]) -> float:
        """Get obstacle penalty at grid position"""
        if self.obstacle_heatmap is None:
            return 0.0

        x, y = grid_pos
        height, width = self.obstacle_heatmap.shape

        if 0 <= x < width and 0 <= y < height:
            return self.obstacle_heatmap[y, x]

        return 0.0

    def _heuristic(self, pos: Tuple, goal: Tuple) -> float:
        """Heuristic function (estimated time to goal)"""
        distance_grid = self._distance_2d(pos, goal)
        distance_cm = distance_grid * self.grid_size_cm
        distance_m = distance_cm / 100.0

        # Straight-line time estimate
        return distance_m / self.config.DRONE_SPEED_M_S

    def _distance_2d(self, pos1: Tuple, pos2: Tuple) -> float:
        """2D Euclidean distance between grid positions"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def _reconstruct_path(
        self,
        came_from: Dict,
        current: Tuple,
        start_pos: Dict,
        goal_pos: Dict
    ) -> List[Tuple]:
        """Reconstruct path from A* search result"""
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        # Convert grid coordinates to world coordinates
        world_path = [self._grid_to_world(p) for p in path]

        # Ensure start and goal are exact
        world_path[0] = (start_pos['x'], start_pos['y'], start_pos.get('z', 120.0))
        world_path[-1] = (goal_pos['x'], goal_pos['y'], goal_pos.get('z', 120.0))

        return world_path

    def _smooth_path(self, waypoints: List[Tuple]) -> List[Tuple]:
        """Remove redundant waypoints using line-of-sight"""
        if len(waypoints) <= 2:
            return waypoints

        smoothed = [waypoints[0]]
        i = 0

        while i < len(waypoints) - 1:
            # Try to skip ahead as far as possible
            j = len(waypoints) - 1

            while j > i + 1:
                if self._is_line_clear(waypoints[i], waypoints[j]):
                    smoothed.append(waypoints[j])
                    i = j
                    break
                j -= 1
            else:
                # Couldn't skip, move to next
                i += 1
                if i < len(waypoints):
                    smoothed.append(waypoints[i])

        # Ensure goal is included
        if smoothed[-1] != waypoints[-1]:
            smoothed.append(waypoints[-1])

        return smoothed

    def _is_line_clear(self, pos1: Tuple, pos2: Tuple) -> bool:
        """Check if straight line between positions avoids high obstacles"""
        # Sample points along line
        num_samples = int(euclidean_distance_2d(
            {'x': pos1[0], 'y': pos1[1]},
            {'x': pos2[0], 'y': pos2[1]}
        ) / self.grid_size_cm)

        num_samples = max(2, num_samples)

        for i in range(num_samples + 1):
            t = i / num_samples
            x = pos1[0] + t * (pos2[0] - pos1[0])
            y = pos1[1] + t * (pos2[1] - pos1[1])

            grid_pos = (int(x / self.grid_size_cm), int(y / self.grid_size_cm))
            penalty = self._get_obstacle_penalty(grid_pos)

            # Reject if penalty too high
            if penalty > self.config.OBSTACLE_PENALTY_WEIGHT:
                return False

        return True

    def _estimate_execution_time(self, waypoints: List[Tuple]) -> float:
        """Estimate total execution time for path"""
        total_time = 0.0

        for i in range(len(waypoints) - 1):
            # Movement time
            distance_cm = euclidean_distance(waypoints[i], waypoints[i + 1])
            distance_m = distance_cm / 100.0
            total_time += distance_m / self.config.DRONE_SPEED_M_S
            total_time += self.config.COMMAND_OVERHEAD_SEC

        # Add rotation time estimate
        total_time += self._estimate_rotation_cost(waypoints)

        return total_time

    def _estimate_rotation_cost(self, waypoints: List[Tuple]) -> float:
        """Estimate total rotation time"""
        if len(waypoints) < 3:
            return 0.0

        total_rotation_time = 0.0

        for i in range(1, len(waypoints) - 1):
            prev = {'x': waypoints[i-1][0], 'y': waypoints[i-1][1]}
            current = {'x': waypoints[i][0], 'y': waypoints[i][1]}
            next_wp = {'x': waypoints[i+1][0], 'y': waypoints[i+1][1]}

            angle_change = abs(calculate_heading_change(prev, current, next_wp))
            rotation_time = angle_change / self.config.ROTATION_SPEED_DEG_S

            total_rotation_time += rotation_time

        return total_rotation_time

    def _calculate_obstacle_penalty(self, waypoints: List[Tuple]) -> float:
        """Calculate total obstacle penalty along path"""
        total_penalty = 0.0

        for waypoint in waypoints:
            grid_pos = self._world_to_grid({'x': waypoint[0], 'y': waypoint[1]})
            total_penalty += self._get_obstacle_penalty(grid_pos)

        return total_penalty
