"""
Grid Visualization

Real-time matplotlib visualization of grid search progress,
drone position, and detected ARuco markers.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow
import numpy as np


class GridVisualizer:
    """Real-time grid search visualization using matplotlib."""

    def __init__(self, grid_size=5, cell_size_m=0.5):
        """
        Initialize grid visualizer.

        Args:
            grid_size: Grid dimensions (NxN)
            cell_size_m: Cell size in meters
        """
        self.grid_size = grid_size
        self.cell_size_m = cell_size_m

        # Create figure
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

        # Color scheme
        self.colors = {
            'unexplored': 'white',
            'exploring': 'lightcoral',
            'explored': 'lightblue',
            'target': 'lime',
            'drone': 'red',
            'aruco': 'blue',
            'path': 'orange'
        }

        print("[VIZ] Grid visualizer initialized")

    def update(self, planner, drone_yaw_deg=0.0, show_path=True):
        """
        Update visualization with current state.

        Args:
            planner: GridSearchPlanner instance
            drone_yaw_deg: Drone yaw angle in degrees
            show_path: Show planned path
        """
        self.ax.clear()

        # Draw grid cells
        for row in range(planner.grid_size):
            for col in range(planner.grid_size):
                cell_state = planner.grid_map[row, col]

                # Determine color
                if cell_state == 0:
                    color = self.colors['unexplored']
                    alpha = 0.3
                elif cell_state == 1:
                    color = self.colors['exploring']
                    alpha = 0.7
                elif cell_state == 2:
                    color = self.colors['explored']
                    alpha = 0.6
                elif cell_state == 3:
                    color = self.colors['target']
                    alpha = 0.9

                # Draw cell
                rect = Rectangle((col, row), 1, 1,
                               linewidth=1.5,
                               edgecolor='black',
                               facecolor=color,
                               alpha=alpha)
                self.ax.add_patch(rect)

                # Add cell coordinates
                self.ax.text(col + 0.5, row + 0.5,
                           f'({row},{col})',
                           ha='center', va='center',
                           fontsize=8, color='gray', alpha=0.5)

        # Draw planned path
        if show_path and planner.current_waypoint_idx < len(planner.waypoints):
            remaining_waypoints = planner.waypoints[planner.current_waypoint_idx:]
            if remaining_waypoints:
                path_rows = [wp[0] + 0.5 for wp in remaining_waypoints]
                path_cols = [wp[1] + 0.5 for wp in remaining_waypoints]
                self.ax.plot(path_cols, path_rows,
                           'o--', color=self.colors['path'],
                           alpha=0.4, linewidth=2, markersize=4,
                           label='Planned Path')

        # Draw drone position with heading indicator
        drone_row, drone_col = planner.drone_cell
        drone_x = drone_col + 0.5
        drone_y = drone_row + 0.5

        # Drone circle
        drone_circle = Circle((drone_x, drone_y), 0.25,
                             color=self.colors['drone'],
                             alpha=0.8,
                             zorder=10,
                             label='Drone')
        self.ax.add_patch(drone_circle)

        # Heading arrow
        arrow_length = 0.3
        arrow_dx = arrow_length * np.sin(np.radians(drone_yaw_deg))
        arrow_dy = -arrow_length * np.cos(np.radians(drone_yaw_deg))

        arrow = FancyArrow(drone_x, drone_y, arrow_dx, arrow_dy,
                          width=0.1, head_width=0.2, head_length=0.15,
                          color='darkred', zorder=11)
        self.ax.add_patch(arrow)

        # Draw ARuco target marker if found
        if planner.target_cell is not None:
            target_row, target_col = planner.target_cell
            target_x = target_col + 0.5
            target_y = target_row + 0.5

            # Target marker
            target_circle = Circle((target_x, target_y), 0.15,
                                  color=self.colors['aruco'],
                                  alpha=0.9,
                                  zorder=9,
                                  label='ARuco Tag 1')
            self.ax.add_patch(target_circle)

            # Target label
            self.ax.text(target_x, target_y - 0.5,
                       'TARGET',
                       ha='center', va='top',
                       fontsize=10, fontweight='bold',
                       color=self.colors['aruco'],
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        # Configure axes
        self.ax.set_xlim(-0.5, planner.grid_size + 0.5)
        self.ax.set_ylim(-0.5, planner.grid_size + 0.5)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()  # Row 0 at top

        # Labels and title
        self.ax.set_xlabel(f'Column (Cell size: {self.cell_size_m}m)', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Row', fontsize=12, fontweight='bold')

        # Title with progress
        progress = planner.get_progress()
        title = f"Grid Search - ARuco Tag Detection\n"
        title += f"Progress: {progress['percent_complete']:.1f}% "
        title += f"({progress['explored_cells']}/{progress['total_cells']} cells)"

        if planner.target_found:
            title += " | TARGET FOUND!"

        self.ax.set_title(title, fontsize=14, fontweight='bold')

        # Grid
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.set_xticks(range(planner.grid_size + 1))
        self.ax.set_yticks(range(planner.grid_size + 1))

        # Legend
        self.ax.legend(loc='upper right', fontsize=10)

        # Redraw
        plt.pause(0.01)

    def draw_overlay_text(self, text, position='bottom'):
        """
        Draw overlay text on visualization.

        Args:
            text: Text to display
            position: 'top', 'bottom', 'left', 'right'
        """
        if position == 'bottom':
            self.ax.text(0.5, -0.02, text,
                        transform=self.ax.transAxes,
                        ha='center', va='top',
                        fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))

    def save_figure(self, filepath):
        """
        Save current visualization to file.

        Args:
            filepath: Output file path (e.g., 'grid_search.png')
        """
        self.fig.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"[VIZ] Saved visualization to {filepath}")

    def close(self):
        """Close visualization window."""
        plt.close(self.fig)
        print("[VIZ] Visualization closed")

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.close()
        except:
            pass
