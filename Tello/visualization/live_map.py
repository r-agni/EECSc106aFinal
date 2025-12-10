"""
Real-time navigation map visualization.

Displays live updating map showing:
- Current drone position and orientation
- Target position
- Detected obstacles (with threat levels)
- Planned path
- Optimal path (updated as obstacles detected)
- Trajectory history
"""

import threading
import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend (better for Windows)
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrow, Circle, Rectangle, Wedge
from typing import List, Tuple, Dict, Optional
import math


class LiveNavigationMap:
    """
    Real-time visualization of drone navigation.

    Shows position, obstacles, path planning, and orientation on a live updating map.
    """

    def __init__(self, target_x: float, target_y: float, target_theta: float,
                 map_size: Tuple[float, float] = (600, 600),
                 update_rate: float = 0.1):
        """
        Initialize live map visualization.

        Args:
            target_x: Target X position in cm
            target_y: Target Y position in cm
            target_theta: Target orientation in degrees
            map_size: (width, height) of map in cm
            update_rate: Map update rate in seconds (0.1 = 10 FPS)
        """
        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        self.map_width, self.map_height = map_size

        # State tracking
        self.current_pos = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.trajectory = []  # List of (x, y) positions
        self.obstacles = []  # List of obstacle dicts
        self.planned_path = []  # Waypoints from path executor
        self.optimal_path = []  # A* optimal path (updates with obstacles)

        # Navigation state
        self.is_navigating = False
        self.is_pid_correcting = False
        self.mission_complete = False

        # Thread control
        self.running = False
        self.update_rate = update_rate
        self.lock = threading.Lock()

        # Matplotlib setup
        self.fig, self.ax = None, None
        self.animation = None

    def start(self):
        """Start the live map visualization in a separate thread."""
        self.running = True

        # Create visualization thread
        self.viz_thread = threading.Thread(target=self._run_visualization, daemon=True)
        self.viz_thread.start()

        time.sleep(0.5)  # Give time for window to open
        print(f"[VIZ] Live map started - showing path to ({self.target_x:.0f}, {self.target_y:.0f})")

    def stop(self):
        """Stop the visualization."""
        self.running = False
        if self.viz_thread:
            self.viz_thread.join(timeout=2.0)
        plt.close('all')
        print("[VIZ] Live map stopped")

    def update_position(self, x: float, y: float, z: float, yaw: float):
        """Update current drone position."""
        with self.lock:
            self.current_pos = {"x": x, "y": y, "z": z, "yaw": yaw}
            self.trajectory.append((x, y))

            # Limit trajectory history to last 200 points
            if len(self.trajectory) > 200:
                self.trajectory.pop(0)

    def add_obstacle(self, x: float, y: float, distance_m: float,
                    width_m: float, threat_level: str, obj_class: str):
        """
        Add detected obstacle to map.

        Args:
            x: Obstacle X position in cm (estimated from drone position)
            y: Obstacle Y position in cm
            distance_m: Distance to obstacle in meters
            width_m: Obstacle width in meters
            threat_level: "high", "medium", or "low"
            obj_class: Object class name (e.g., "person", "chair")
        """
        with self.lock:
            # Check if obstacle already exists nearby (avoid duplicates)
            for obs in self.obstacles:
                dx = abs(obs['x'] - x)
                dy = abs(obs['y'] - y)
                if dx < 30 and dy < 30:  # Within 30cm
                    # Update existing obstacle
                    obs['distance_m'] = distance_m
                    obs['threat_level'] = threat_level
                    obs['last_seen'] = time.time()
                    return

            # Add new obstacle
            self.obstacles.append({
                'x': x,
                'y': y,
                'distance_m': distance_m,
                'width_m': width_m,
                'threat_level': threat_level,
                'class': obj_class,
                'last_seen': time.time()
            })

            # Remove stale obstacles (not seen in 10 seconds)
            current_time = time.time()
            self.obstacles = [obs for obs in self.obstacles
                            if current_time - obs['last_seen'] < 10.0]

    def update_planned_path(self, waypoints: List[Tuple[float, float, float]]):
        """Update planned waypoints from path executor."""
        with self.lock:
            self.planned_path = [(wp[0], wp[1]) for wp in waypoints]

    def update_optimal_path(self, path_points: List[Tuple[float, float]]):
        """Update optimal path (A* or similar)."""
        with self.lock:
            self.optimal_path = path_points

    def set_navigation_state(self, is_navigating: bool = False,
                           is_pid: bool = False, complete: bool = False):
        """Update navigation state flags."""
        with self.lock:
            self.is_navigating = is_navigating
            self.is_pid_correcting = is_pid
            self.mission_complete = complete

    def _calculate_obstacle_position(self, distance_m: float,
                                    bearing_offset: float = 0) -> Tuple[float, float]:
        """
        Calculate obstacle position in global coordinates.

        Args:
            distance_m: Distance to obstacle in meters
            bearing_offset: Bearing offset from drone heading in degrees

        Returns:
            (x, y) position in cm
        """
        distance_cm = distance_m * 100
        bearing = self.current_pos['yaw'] + bearing_offset

        obs_x = self.current_pos['x'] + distance_cm * math.cos(math.radians(bearing))
        obs_y = self.current_pos['y'] + distance_cm * math.sin(math.radians(bearing))

        return obs_x, obs_y

    def _run_visualization(self):
        """Main visualization loop (runs in thread)."""
        # Setup matplotlib in non-blocking mode
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Drone Navigation Map')

        # Manual update loop instead of FuncAnimation (better for threading)
        while self.running:
            try:
                with self.lock:
                    data = {
                        'current_pos': self.current_pos.copy(),
                        'trajectory': self.trajectory.copy(),
                        'obstacles': self.obstacles.copy(),
                        'planned_path': self.planned_path.copy(),
                        'optimal_path': self.optimal_path.copy(),
                        'is_navigating': self.is_navigating,
                        'is_pid': self.is_pid_correcting,
                        'complete': self.mission_complete
                    }

                self._draw_map(data)

                # Update display
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

                time.sleep(self.update_rate)

            except Exception as e:
                print(f"[VIZ] Error updating map: {e}")
                break

        plt.close(self.fig)

    def _draw_map(self, data: Dict):
        """Draw all map elements."""
        self.ax.clear()

        # Set map bounds (centered on midpoint between start and target)
        center_x = self.target_x / 2
        center_y = self.target_y / 2

        # Dynamic bounds based on target
        margin = 100  # cm
        x_min = min(-margin, center_x - self.map_width/2)
        x_max = max(self.target_x + margin, center_x + self.map_width/2)
        y_min = min(-margin, center_y - self.map_height/2)
        y_max = max(self.target_y + margin, center_y + self.map_height/2)

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.set_xlabel('X Position (cm)', fontsize=10)
        self.ax.set_ylabel('Y Position (cm)', fontsize=10)

        # Title with status
        status = "MISSION COMPLETE" if data['complete'] else \
                "PID CORRECTING" if data['is_pid'] else \
                "NAVIGATING" if data['is_navigating'] else "READY"
        self.ax.set_title(f'Live Navigation Map - Status: {status}',
                         fontsize=12, fontweight='bold')

        # 1. Draw start position (origin)
        self.ax.plot(0, 0, 'go', markersize=15, label='Start', zorder=5)
        self.ax.add_patch(Circle((0, 0), 10, color='green', alpha=0.3))

        # 2. Draw target position
        self.ax.plot(self.target_x, self.target_y, 'r*', markersize=20,
                    label=f'Target ({self.target_x:.0f}, {self.target_y:.0f})', zorder=5)

        # Target orientation indicator
        arrow_len = 40
        target_dx = arrow_len * math.cos(math.radians(self.target_theta))
        target_dy = arrow_len * math.sin(math.radians(self.target_theta))
        self.ax.arrow(self.target_x, self.target_y, target_dx, target_dy,
                     head_width=15, head_length=10, fc='red', ec='red',
                     alpha=0.6, zorder=4)

        # 3. Draw planned path (initial waypoints)
        if data['planned_path']:
            path_x = [p[0] for p in data['planned_path']]
            path_y = [p[1] for p in data['planned_path']]
            self.ax.plot(path_x, path_y, 'b--', linewidth=1, alpha=0.4,
                        label='Planned Path', zorder=2)
            # Waypoint markers
            self.ax.plot(path_x, path_y, 'bs', markersize=6, alpha=0.4, zorder=2)

        # 4. Draw optimal path (A* updated with obstacles)
        if data['optimal_path']:
            opt_x = [p[0] for p in data['optimal_path']]
            opt_y = [p[1] for p in data['optimal_path']]
            self.ax.plot(opt_x, opt_y, 'c-', linewidth=2, alpha=0.7,
                        label='Optimal Path', zorder=3)

        # 5. Draw obstacles
        for obs in data['obstacles']:
            # Obstacle circle
            color = {'high': 'red', 'medium': 'orange', 'low': 'yellow'}.get(
                obs['threat_level'], 'gray')
            radius = obs['width_m'] * 50  # Convert to cm and scale

            circle = Circle((obs['x'], obs['y']), radius,
                          color=color, alpha=0.3, zorder=3)
            self.ax.add_patch(circle)

            # Obstacle center
            self.ax.plot(obs['x'], obs['y'], 'x', color=color,
                        markersize=8, markeredgewidth=2, zorder=4)

            # Threat zone for high threats
            if obs['threat_level'] == 'high':
                threat_circle = Circle((obs['x'], obs['y']), radius * 1.5,
                                      color='red', alpha=0.1, linestyle='--',
                                      fill=False, linewidth=2, zorder=2)
                self.ax.add_patch(threat_circle)

        # 6. Draw trajectory (path traveled)
        if data['trajectory']:
            traj_x = [p[0] for p in data['trajectory']]
            traj_y = [p[1] for p in data['trajectory']]
            self.ax.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.6,
                        label='Actual Path', zorder=4)

        # 7. Draw current drone position and orientation
        curr = data['current_pos']

        # Drone position
        self.ax.plot(curr['x'], curr['y'], 'bo', markersize=12,
                    label=f"Drone ({curr['x']:.0f}, {curr['y']:.0f})", zorder=6)

        # Drone orientation (heading arrow)
        arrow_length = 50
        dx = arrow_length * math.cos(math.radians(curr['yaw']))
        dy = arrow_length * math.sin(math.radians(curr['yaw']))

        arrow = FancyArrow(curr['x'], curr['y'], dx, dy,
                          width=8, head_width=20, head_length=15,
                          fc='blue', ec='blue', zorder=6)
        self.ax.add_patch(arrow)

        # Drone field of view cone (camera view)
        fov_angle = 60  # degrees
        fov_distance = 150  # cm
        wedge = Wedge((curr['x'], curr['y']), fov_distance,
                     curr['yaw'] - fov_angle/2, curr['yaw'] + fov_angle/2,
                     fc='blue', alpha=0.1, zorder=1)
        self.ax.add_patch(wedge)

        # 8. Add info text box
        info_text = self._generate_info_text(data)
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                    fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    family='monospace')

        # Legend
        self.ax.legend(loc='upper right', fontsize=8, framealpha=0.9)

    def _generate_info_text(self, data: Dict) -> str:
        """Generate info text for display."""
        curr = data['current_pos']

        # Calculate distance to target
        dx = self.target_x - curr['x']
        dy = self.target_y - curr['y']
        dist_to_target = math.sqrt(dx**2 + dy**2)

        # Angle error
        angle_error = abs(self.target_theta - curr['yaw'])
        if angle_error > 180:
            angle_error = 360 - angle_error

        # Obstacle count by threat
        high_threats = sum(1 for o in data['obstacles'] if o['threat_level'] == 'high')
        med_threats = sum(1 for o in data['obstacles'] if o['threat_level'] == 'medium')
        low_threats = sum(1 for o in data['obstacles'] if o['threat_level'] == 'low')

        info = f"""NAVIGATION INFO
───────────────────
Position: ({curr['x']:.1f}, {curr['y']:.1f})
Altitude: {curr['z']:.1f} cm
Heading:  {curr['yaw']:.1f}°

Target: ({self.target_x:.0f}, {self.target_y:.0f}, {self.target_theta:.0f}°)
Distance: {dist_to_target:.1f} cm
Angle Δ:  {angle_error:.1f}°

Obstacles Detected:
  🔴 High:   {high_threats}
  🟠 Medium: {med_threats}
  🟡 Low:    {low_threats}

Path Points: {len(data['trajectory'])}
"""
        return info
