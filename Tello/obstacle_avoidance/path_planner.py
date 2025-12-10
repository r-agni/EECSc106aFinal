"""
RRT (Rapidly-exploring Random Tree) path planner for drone navigation.

Generates waypoints from start to goal while avoiding known obstacles.
"""

import math
import random
from typing import List, Tuple, Optional, Set
import numpy as np


class Node:
    """Node in the RRT tree"""

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z
        self.parent: Optional[Node] = None
        self.cost = 0.0

    def position(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def distance_to(self, other: 'Node') -> float:
        """Euclidean distance to another node"""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)


class Obstacle:
    """Obstacle representation for collision checking"""

    def __init__(self, x: float, y: float, radius: float, height: float = 200):
        """
        Args:
            x, y: Center position in cm
            radius: Obstacle radius in cm
            height: Obstacle height in cm (default 200cm = 2m)
        """
        self.x = x
        self.y = y
        self.radius = radius
        self.height = height

    def collides_with_point(self, x: float, y: float, z: float, safety_margin: float = 30) -> bool:
        """
        Check if point collides with obstacle (with safety margin).

        Args:
            x, y, z: Point position in cm
            safety_margin: Additional clearance in cm
        """
        # Check height - only collide if drone is below obstacle top
        if z > self.height:
            return False

        # Check horizontal distance
        dx = x - self.x
        dy = y - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        return dist < (self.radius + safety_margin)

    def collides_with_line(self, x1: float, y1: float, z1: float,
                           x2: float, y2: float, z2: float,
                           safety_margin: float = 30) -> bool:
        """
        Check if line segment collides with obstacle.

        Uses point-to-line distance formula.
        """
        # Sample points along the line
        num_samples = 10
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            z = z1 + t * (z2 - z1)

            if self.collides_with_point(x, y, z, safety_margin):
                return True

        return False


class RRTPlanner:
    """RRT path planner for generating waypoints"""

    def __init__(self,
                 max_iterations: int = 500,
                 step_size: float = 50.0,  # cm
                 goal_sample_rate: float = 0.1,
                 goal_threshold: float = 30.0,  # cm
                 bounds: Tuple[float, float, float, float] = (-500, 500, -500, 500)):
        """
        Initialize RRT planner.

        Args:
            max_iterations: Maximum iterations for tree growth
            step_size: Maximum distance for each branch extension (cm)
            goal_sample_rate: Probability of sampling goal (0-1)
            goal_threshold: Distance to consider goal reached (cm)
            bounds: (min_x, max_x, min_y, max_y) search space bounds in cm
        """
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.goal_threshold = goal_threshold
        self.bounds = bounds

        self.tree: List[Node] = []
        self.obstacles: List[Obstacle] = []

    def set_obstacles(self, obstacles: List[Tuple[float, float, float]]):
        """
        Set obstacle list from detected obstacles.

        Args:
            obstacles: List of (x, y, radius_cm) tuples
        """
        self.obstacles = []
        for obs in obstacles:
            x, y, radius = obs
            self.obstacles.append(Obstacle(x, y, radius))

    def is_collision_free(self, x1: float, y1: float, z1: float,
                          x2: float, y2: float, z2: float) -> bool:
        """Check if line segment is collision-free"""
        for obstacle in self.obstacles:
            if obstacle.collides_with_line(x1, y1, z1, x2, y2, z2):
                return False
        return True

    def sample_random_point(self, goal: Node) -> Node:
        """
        Sample a random point in search space.

        With probability goal_sample_rate, return goal.
        Otherwise, return random point in bounds.
        """
        if random.random() < self.goal_sample_rate:
            return goal

        min_x, max_x, min_y, max_y = self.bounds
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        z = goal.z  # Keep altitude constant for 2D planning

        return Node(x, y, z)

    def find_nearest_node(self, point: Node) -> Node:
        """Find nearest node in tree to given point"""
        nearest = self.tree[0]
        min_dist = point.distance_to(nearest)

        for node in self.tree[1:]:
            dist = point.distance_to(node)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        return nearest

    def steer(self, from_node: Node, to_node: Node) -> Node:
        """
        Create new node by extending from_node toward to_node.

        Extension is limited by step_size.
        """
        dist = from_node.distance_to(to_node)

        if dist < self.step_size:
            # Close enough - return to_node
            return Node(to_node.x, to_node.y, to_node.z)

        # Extend by step_size in direction of to_node
        ratio = self.step_size / dist
        x = from_node.x + ratio * (to_node.x - from_node.x)
        y = from_node.y + ratio * (to_node.y - from_node.y)
        z = from_node.z + ratio * (to_node.z - from_node.z)

        return Node(x, y, z)

    def extract_path(self, goal_node: Node) -> List[Tuple[float, float, float]]:
        """
        Extract path from start to goal by backtracking through tree.

        Returns path in reverse order (goal to start), so we reverse it.
        """
        path = []
        current = goal_node

        while current is not None:
            path.append(current.position())
            current = current.parent

        path.reverse()  # Start to goal order
        return path

    def simplify_path(self, path: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Simplify path by removing unnecessary waypoints.

        Uses greedy approach: try to skip waypoints if direct connection is collision-free.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Try to find furthest reachable waypoint
            furthest_idx = current_idx + 1

            for i in range(current_idx + 2, len(path)):
                x1, y1, z1 = path[current_idx]
                x2, y2, z2 = path[i]

                if self.is_collision_free(x1, y1, z1, x2, y2, z2):
                    furthest_idx = i
                else:
                    break  # Can't skip any more

            simplified.append(path[furthest_idx])
            current_idx = furthest_idx

        return simplified

    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float]) -> Optional[List[Tuple[float, float, float]]]:
        """
        Plan path from start to goal using RRT.

        Args:
            start: (x, y, z) start position in cm
            goal: (x, y, z) goal position in cm

        Returns:
            List of waypoints [(x, y, z), ...] or None if no path found
        """
        # Initialize tree with start node
        start_node = Node(start[0], start[1], start[2])
        goal_node = Node(goal[0], goal[1], goal[2])

        self.tree = [start_node]

        print(f"[RRT] Planning path from {start} to {goal}")
        print(f"[RRT] Obstacles: {len(self.obstacles)}")

        for iteration in range(self.max_iterations):
            # Sample random point
            random_point = self.sample_random_point(goal_node)

            # Find nearest node in tree
            nearest_node = self.find_nearest_node(random_point)

            # Steer toward random point
            new_node = self.steer(nearest_node, random_point)

            # Check if path to new node is collision-free
            if self.is_collision_free(nearest_node.x, nearest_node.y, nearest_node.z,
                                     new_node.x, new_node.y, new_node.z):
                # Add to tree
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + nearest_node.distance_to(new_node)
                self.tree.append(new_node)

                # Check if we reached goal
                if new_node.distance_to(goal_node) < self.goal_threshold:
                    print(f"[RRT] Path found in {iteration} iterations!")

                    # Extract and simplify path
                    path = self.extract_path(new_node)
                    simplified_path = self.simplify_path(path)

                    print(f"[RRT] Original waypoints: {len(path)}, Simplified: {len(simplified_path)}")
                    return simplified_path

        print(f"[RRT] No path found after {self.max_iterations} iterations")
        return None


def generate_waypoints_with_rrt(start_x: float, start_y: float,
                                 goal_x: float, goal_y: float,
                                 altitude: float,
                                 obstacles: List[Tuple[float, float, float]] = None) -> List[Tuple[float, float, float]]:
    """
    Generate waypoints using RRT algorithm.

    Args:
        start_x, start_y: Start position in cm
        goal_x, goal_y: Goal position in cm
        altitude: Flight altitude in cm
        obstacles: List of (x, y, radius) obstacle tuples in cm

    Returns:
        List of waypoints including takeoff and landing
    """
    planner = RRTPlanner(
        max_iterations=500,
        step_size=50.0,  # 50cm steps
        goal_sample_rate=0.15,  # 15% chance to sample goal
        goal_threshold=30.0,  # 30cm goal tolerance
        bounds=(-600, 600, -600, 600)  # 6m x 6m search space
    )

    # Set obstacles if provided
    if obstacles:
        planner.set_obstacles(obstacles)

    # Plan path at altitude
    start = (start_x, start_y, altitude)
    goal = (goal_x, goal_y, altitude)

    path = planner.plan(start, goal)

    if path is None:
        # Fallback to simple direct path if RRT fails
        print("[RRT] Planning failed - using direct path")
        return [
            (start_x, start_y, altitude),
            (goal_x, goal_y, altitude),
            (goal_x, goal_y, 0)
        ]

    # Add takeoff at start and landing at end
    waypoints = [(start_x, start_y, altitude)]  # Takeoff
    waypoints.extend(path[1:])  # RRT path (skip duplicate start)
    waypoints.append((goal_x, goal_y, 0))  # Landing

    return waypoints
