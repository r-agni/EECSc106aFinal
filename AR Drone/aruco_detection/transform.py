"""
Transform Calculator for ARuco Markers

Calculates spatial transforms from detected ARuco marker positions
to drone coordinates, enabling navigation towards markers.
"""

import numpy as np


class TransformCalculator:
    """Calculate transforms from ARuco detections to drone coordinates."""

    def __init__(self, camera_fov_h_deg=92.0, camera_fov_v_deg=52.0,
                 frame_width=640, frame_height=480):
        """
        Initialize transform calculator.

        Args:
            camera_fov_h_deg: Horizontal field of view in degrees
            camera_fov_v_deg: Vertical field of view in degrees
            frame_width: Frame width in pixels
            frame_height: Frame height in pixels
        """
        self.camera_fov_h_deg = camera_fov_h_deg
        self.camera_fov_v_deg = camera_fov_v_deg
        self.frame_width = frame_width
        self.frame_height = frame_height

        # Convert FOV to radians
        self.fov_h_rad = np.radians(camera_fov_h_deg)
        self.fov_v_rad = np.radians(camera_fov_v_deg)

    def pixel_to_world_offset(self, pixel_x, pixel_y, altitude_mm):
        """
        Convert pixel coordinates to world offset from drone.

        Args:
            pixel_x: X pixel coordinate
            pixel_y: Y pixel coordinate
            altitude_mm: Drone altitude in millimeters

        Returns:
            (x_offset_m, y_offset_m): Offset in meters
        """
        # Calculate pixel offset from frame center
        center_x = self.frame_width / 2
        center_y = self.frame_height / 2

        pixel_offset_x = pixel_x - center_x
        pixel_offset_y = pixel_y - center_y

        # Convert altitude to meters
        altitude_m = altitude_mm / 1000.0

        # Calculate world dimensions at current altitude
        # Using simple pinhole camera model
        world_width_m = 2 * altitude_m * np.tan(self.fov_h_rad / 2)
        world_height_m = 2 * altitude_m * np.tan(self.fov_v_rad / 2)

        # Convert pixel offset to world offset
        x_offset_m = (pixel_offset_x / self.frame_width) * world_width_m
        y_offset_m = (pixel_offset_y / self.frame_height) * world_height_m

        return x_offset_m, y_offset_m

    def marker_to_drone_transform(self, marker_center, altitude_mm, drone_yaw_deg=0.0):
        """
        Calculate full transform from marker to drone.

        Args:
            marker_center: (cx, cy) marker center in pixels
            altitude_mm: Drone altitude in millimeters
            drone_yaw_deg: Drone yaw angle in degrees

        Returns:
            dict with keys:
                - 'x_offset_m': X offset in meters (positive = marker is right)
                - 'y_offset_m': Y offset in meters (positive = marker is forward)
                - 'distance_m': Straight-line distance to marker
                - 'angle_deg': Angle to marker from drone heading
        """
        cx, cy = marker_center

        # Get pixel to world offset
        x_offset_m, y_offset_m = self.pixel_to_world_offset(cx, cy, altitude_mm)

        # Calculate distance
        distance_m = np.sqrt(x_offset_m**2 + y_offset_m**2)

        # Calculate angle to marker (relative to drone heading)
        # Note: OpenCV has Y-axis pointing down, so we flip y_offset
        angle_rad = np.arctan2(x_offset_m, -y_offset_m)
        angle_deg = np.degrees(angle_rad)

        # Adjust for drone yaw
        absolute_angle_deg = (angle_deg + drone_yaw_deg) % 360

        return {
            'x_offset_m': x_offset_m,
            'y_offset_m': y_offset_m,
            'distance_m': distance_m,
            'angle_deg': angle_deg,
            'absolute_angle_deg': absolute_angle_deg
        }

    def get_navigation_command(self, transform, distance_threshold_m=0.2):
        """
        Convert transform to navigation command.

        Args:
            transform: Transform dict from marker_to_drone_transform()
            distance_threshold_m: Distance threshold for "arrived"

        Returns:
            dict with keys:
                - 'status': 'arrived', 'navigate', or 'search'
                - 'direction': 'forward', 'backward', 'left', 'right', or None
                - 'magnitude': 0.0-1.0 movement strength
        """
        distance = transform['distance_m']

        # Check if arrived
        if distance < distance_threshold_m:
            return {
                'status': 'arrived',
                'direction': None,
                'magnitude': 0.0
            }

        # Calculate direction based on offsets
        x_offset = transform['x_offset_m']
        y_offset = transform['y_offset_m']

        # Determine primary direction
        if abs(x_offset) > abs(y_offset):
            # Move left/right
            direction = 'right' if x_offset > 0 else 'left'
            magnitude = min(abs(x_offset), 1.0)
        else:
            # Move forward/backward
            # Note: y_offset positive means marker is forward (due to flipped axis)
            direction = 'backward' if y_offset > 0 else 'forward'
            magnitude = min(abs(y_offset), 1.0)

        return {
            'status': 'navigate',
            'direction': direction,
            'magnitude': magnitude
        }

    def calculate_pid_errors(self, transform):
        """
        Calculate PID errors from transform.

        Args:
            transform: Transform dict from marker_to_drone_transform()

        Returns:
            dict with 'error_x', 'error_y' for PID controllers
        """
        # Error is the negative of offset (we want to minimize offset)
        return {
            'error_x': -transform['x_offset_m'],
            'error_y': -transform['y_offset_m']
        }

    def update_drone_position(self, current_pos, movement_m):
        """
        Update drone position estimate based on movement.

        Args:
            current_pos: (x, y, yaw) current position
            movement_m: (dx, dy) movement in meters

        Returns:
            (x, y, yaw) updated position
        """
        x, y, yaw = current_pos
        dx, dy = movement_m

        # Simple dead reckoning update
        new_x = x + dx
        new_y = y + dy

        return (new_x, new_y, yaw)

    def get_transform_summary(self, transform):
        """
        Get human-readable transform summary.

        Args:
            transform: Transform dict from marker_to_drone_transform()

        Returns:
            str summary
        """
        summary = (
            f"Distance: {transform['distance_m']:.3f}m, "
            f"Offset: X={transform['x_offset_m']:.3f}m Y={transform['y_offset_m']:.3f}m, "
            f"Angle: {transform['angle_deg']:.1f}°"
        )
        return summary
