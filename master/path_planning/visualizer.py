"""
Path visualization system.

Creates dual-panel comparison of exploration path vs optimal path with statistics.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch
import numpy as np
from typing import Dict, List, Tuple
from .config import PathPlanningConfig


class PathVisualizer:
    """Visualizes exploration and optimal paths"""

    def __init__(self, config: PathPlanningConfig):
        """
        Initialize visualizer.

        Args:
            config: PathPlanningConfig instance
        """
        self.config = config

    def visualize_paths(
        self,
        movement_log: Dict,
        optimal_path: Dict,
        tag_positions: Dict[int, Dict]
    ):
        """
        Create comprehensive visualization of exploration vs optimal path.

        Args:
            movement_log: Complete exploration movement log
            optimal_path: Optimized path dict from PathOptimizer
            tag_positions: Dict mapping tag_id to world position {x, y, z}
        """
        # Create figure with two panels
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

        # Get search area
        area = movement_log['search_area']
        area_width_cm = area['width_m'] * 100
        area_height_cm = area['height_m'] * 100

        # === LEFT PANEL: Exploration Path ===
        self._draw_exploration_panel(ax1, movement_log, tag_positions, area_width_cm, area_height_cm)

        # === RIGHT PANEL: Optimal Path ===
        self._draw_optimal_panel(ax2, movement_log, optimal_path, tag_positions, area_width_cm, area_height_cm)

        # === STATISTICS TABLE ===
        self._add_statistics_table(fig, movement_log, optimal_path)

        plt.tight_layout(rect=[0, 0.15, 1, 1])

        # Save if configured
        if self.config.SAVE_VISUALIZATION:
            filename = f"path_planning_result.{self.config.PLOT_FORMAT}"
            plt.savefig(filename, dpi=self.config.PLOT_DPI, bbox_inches='tight')
            print(f"[VISUALIZER] Saved visualization to: {filename}")

        # Show if configured
        if self.config.SHOW_LIVE_PLOT:
            plt.show()
        else:
            plt.close()

    def _draw_exploration_panel(
        self,
        ax,
        movement_log: Dict,
        tag_positions: Dict,
        area_width_cm: float,
        area_height_cm: float
    ):
        """Draw exploration phase visualization"""
        ax.set_title("Exploration Phase", fontsize=16, fontweight='bold')
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        # Draw search area boundary
        ax.add_patch(Rectangle(
            (0, 0),
            area_width_cm,
            area_height_cm,
            fill=False,
            edgecolor='black',
            linewidth=2,
            label='Search Area'
        ))

        # Extract exploration path from movements
        exploration_positions = []
        for movement in movement_log['movements']:
            pos_after = movement.get('position_after', {})
            if pos_after:
                exploration_positions.append((pos_after['x'], pos_after['y']))

        if exploration_positions:
            xs = [p[0] for p in exploration_positions]
            ys = [p[1] for p in exploration_positions]

            # Draw exploration path
            ax.plot(xs, ys, 'r-', linewidth=2, alpha=0.7, label='Exploration Path')
            ax.scatter(xs, ys, c='red', s=20, alpha=0.5)

            # Mark start position
            ax.scatter([xs[0]], [ys[0]], c='green', s=100, marker='o',
                      label='Start Position', edgecolors='black', linewidths=2)

        # Draw obstacle encounter zones
        for obs in movement_log.get('obstacles_encountered', []):
            pos = obs.get('at_position', {})
            if pos:
                ax.add_patch(Circle(
                    (pos['x'], pos['y']),
                    30,  # 30cm radius
                    color='red',
                    alpha=0.3
                ))
                # Add obstacle label
                obstacle_class = obs.get('obstacle', {}).get('class', 'unknown')
                ax.text(
                    pos['x'],
                    pos['y'],
                    obstacle_class,
                    fontsize=8,
                    ha='center',
                    va='center'
                )

        # Draw ArUco tag positions
        for tag_id, tag_pos in tag_positions.items():
            if tag_id == 0:
                # Start tag (blue square)
                ax.add_patch(Rectangle(
                    (tag_pos['x'] - 10, tag_pos['y'] - 10),
                    20,
                    20,
                    color='blue',
                    label='Start Tag (ID 0)'
                ))
                ax.text(tag_pos['x'], tag_pos['y'] + 30, 'START',
                       fontsize=10, ha='center', fontweight='bold', color='blue')
            elif tag_id == 1:
                # Finish tag (orange circle)
                ax.add_patch(Circle(
                    (tag_pos['x'], tag_pos['y']),
                    10,
                    color='orange',
                    label='Finish Tag (ID 1)'
                ))
                ax.text(tag_pos['x'], tag_pos['y'] + 30, 'FINISH',
                       fontsize=10, ha='center', fontweight='bold', color='orange')

        ax.legend(loc='upper right', fontsize=9)

    def _draw_optimal_panel(
        self,
        ax,
        movement_log: Dict,
        optimal_path: Dict,
        tag_positions: Dict,
        area_width_cm: float,
        area_height_cm: float
    ):
        """Draw optimal path visualization"""
        ax.set_title("Optimal Path", fontsize=16, fontweight='bold')
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        # Draw search area boundary
        ax.add_patch(Rectangle(
            (0, 0),
            area_width_cm,
            area_height_cm,
            fill=False,
            edgecolor='black',
            linewidth=2
        ))

        # Draw obstacle zones (faded)
        for obs in movement_log.get('obstacles_encountered', []):
            pos = obs.get('at_position', {})
            if pos:
                ax.add_patch(Circle(
                    (pos['x'], pos['y']),
                    30,
                    color='red',
                    alpha=0.1
                ))

        # Draw optimal path
        if optimal_path and 'waypoints' in optimal_path:
            waypoints = optimal_path['waypoints']
            opt_xs = [p[0] for p in waypoints]
            opt_ys = [p[1] for p in waypoints]

            ax.plot(opt_xs, opt_ys, 'g-', linewidth=3, alpha=0.8, label='Optimal Path')
            ax.scatter(opt_xs, opt_ys, c='green', s=50, marker='D', edgecolors='black', linewidths=1)

            # Add waypoint numbers
            for i, (x, y) in enumerate(zip(opt_xs, opt_ys)):
                ax.text(x + 15, y + 15, str(i + 1), fontsize=10, fontweight='bold')

        # Draw ArUco tag positions
        for tag_id, tag_pos in tag_positions.items():
            if tag_id == 0:
                ax.add_patch(Rectangle(
                    (tag_pos['x'] - 10, tag_pos['y'] - 10),
                    20,
                    20,
                    color='blue',
                    label='Start'
                ))
                ax.text(tag_pos['x'], tag_pos['y'] + 30, 'START',
                       fontsize=10, ha='center', fontweight='bold', color='blue')
            elif tag_id == 1:
                ax.add_patch(Circle(
                    (tag_pos['x'], tag_pos['y']),
                    10,
                    color='orange',
                    label='Finish'
                ))
                ax.text(tag_pos['x'], tag_pos['y'] + 30, 'FINISH',
                       fontsize=10, ha='center', fontweight='bold', color='orange')

        ax.legend(loc='upper right', fontsize=9)

    def _add_statistics_table(self, fig, movement_log: Dict, optimal_path: Dict):
        """Add statistics comparison table"""
        stats = movement_log['statistics']

        if optimal_path:
            optimal_dist = optimal_path.get('total_distance_cm', 0)
            optimal_time = optimal_path.get('estimated_time_sec', 0)
            optimal_waypoints = optimal_path.get('waypoint_count', 0)

            distance_saved = stats['total_distance_traveled_cm'] - optimal_dist
            time_saved = stats['exploration_duration_sec'] - optimal_time
        else:
            optimal_dist = 0
            optimal_time = 0
            optimal_waypoints = 0
            distance_saved = 0
            time_saved = 0

        stats_text = f"""
EXPLORATION STATISTICS:
  Total Distance: {stats['total_distance_traveled_cm']:.0f} cm ({stats['total_distance_traveled_cm']/100:.1f} m)
  Total Commands: {stats['total_commands_executed']}
  Obstacles Avoided: {stats['obstacle_avoidance_count']}
  Duration: {stats['exploration_duration_sec']:.1f} sec ({stats['exploration_duration_sec']/60:.1f} min)
  Battery Used: {stats['battery_consumed_percent']}%
  Tags Found: {stats['tags_found']}

OPTIMAL PATH:
  Distance: {optimal_dist:.0f} cm ({optimal_dist/100:.1f} m)
  Waypoints: {optimal_waypoints}
  Estimated Time: {optimal_time:.1f} sec ({optimal_time/60:.1f} min)

IMPROVEMENT:
  Distance Saved: {distance_saved:.0f} cm ({distance_saved/100:.1f} m) [{(distance_saved/stats['total_distance_traveled_cm']*100):.1f}%]
  Time Saved: {time_saved:.1f} sec ({time_saved/60:.1f} min) [{(time_saved/stats['exploration_duration_sec']*100):.1f}%]
"""

        fig.text(
            0.5, 0.02,
            stats_text,
            ha='center',
            fontsize=10,
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )

    def plot_simple_path(self, waypoints: List[Tuple], title: str = "Path"):
        """
        Simple path visualization for testing/debugging.

        Args:
            waypoints: List of (x, y, z) tuples
            title: Plot title
        """
        fig, ax = plt.subplots(figsize=(10, 10))

        xs = [p[0] for p in waypoints]
        ys = [p[1] for p in waypoints]

        ax.plot(xs, ys, 'b-', linewidth=2, marker='o')
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.tight_layout()
        plt.show()
