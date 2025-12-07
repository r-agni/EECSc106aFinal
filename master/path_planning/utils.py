"""
Utility functions for path planning system.

Provides coordinate transforms, distance calculations, and geometric helpers.
"""

import math
import numpy as np
import cv2


def euclidean_distance(pos1, pos2):
    """
    Calculate 3D Euclidean distance between two positions.

    Args:
        pos1: First position (x, y, z) or dict with x, y, z keys
        pos2: Second position (x, y, z) or dict with x, y, z keys

    Returns:
        float: Distance in same units as input
    """
    if isinstance(pos1, dict):
        x1, y1, z1 = pos1['x'], pos1['y'], pos1['z']
    else:
        x1, y1, z1 = pos1[0], pos1[1], pos1[2]

    if isinstance(pos2, dict):
        x2, y2, z2 = pos2['x'], pos2['y'], pos2['z']
    else:
        x2, y2, z2 = pos2[0], pos2[1], pos2[2]

    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def euclidean_distance_2d(pos1, pos2):
    """
    Calculate 2D Euclidean distance (ignoring Z).

    Args:
        pos1: First position (x, y, z) or dict with x, y keys
        pos2: Second position (x, y, z) or dict with x, y keys

    Returns:
        float: Distance in same units as input
    """
    if isinstance(pos1, dict):
        x1, y1 = pos1['x'], pos1['y']
    else:
        x1, y1 = pos1[0], pos1[1]

    if isinstance(pos2, dict):
        x2, y2 = pos2['x'], pos2['y']
    else:
        x2, y2 = pos2[0], pos2[1]

    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def calculate_heading(from_pos, to_pos):
    """
    Calculate heading angle from one position to another.

    Args:
        from_pos: Starting position dict with x, y keys
        to_pos: Target position dict with x, y keys

    Returns:
        float: Heading angle in degrees (0° = East, 90° = North)
    """
    dx = to_pos['x'] - from_pos['x']
    dy = to_pos['y'] - from_pos['y']
    return math.degrees(math.atan2(dy, dx))


def calculate_heading_change(prev_pos, current_pos, next_pos):
    """
    Calculate the change in heading angle when moving through three positions.

    Args:
        prev_pos: Previous position dict
        current_pos: Current position dict
        next_pos: Next position dict

    Returns:
        float: Heading change in degrees (-180 to 180)
    """
    heading_in = calculate_heading(prev_pos, current_pos)
    heading_out = calculate_heading(current_pos, next_pos)

    change = heading_out - heading_in

    # Normalize to -180 to 180
    while change > 180:
        change -= 360
    while change < -180:
        change += 360

    return change


def rvec_tvec_to_transform(rvec, tvec):
    """
    Convert OpenCV rvec and tvec to 4x4 homogeneous transformation matrix.

    Args:
        rvec: Rotation vector from cv2.aruco.estimatePoseSingleMarkers
        tvec: Translation vector from cv2.aruco.estimatePoseSingleMarkers

    Returns:
        np.ndarray: 4x4 transformation matrix
    """
    R, _ = cv2.Rodrigues(rvec)
    t = np.array(tvec, dtype=np.float32).reshape(3, 1)

    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3:4] = t
    return T


def invert_transform(T):
    """
    Invert a 4x4 rigid transformation matrix.

    Args:
        T: 4x4 homogeneous transformation matrix

    Returns:
        np.ndarray: Inverted transformation matrix
    """
    R = T[:3, :3]
    t = T[:3, 3:4]

    T_inv = np.eye(4, dtype=np.float32)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3:4] = -R.T @ t
    return T_inv


def camera_frame_to_world(tvec, rvec, drone_position, drone_yaw):
    """
    Transform ArUco tag position from camera frame to world frame.

    Args:
        tvec: Translation vector from camera to tag (meters)
        rvec: Rotation vector of tag
        drone_position: Drone position dict {x, y, z} in cm
        drone_yaw: Drone yaw in degrees

    Returns:
        dict: Tag world position {x, y, z} in cm
    """
    # Tag position in camera frame (meters)
    tag_cam_x = tvec[0]
    tag_cam_y = tvec[1]
    tag_cam_z = tvec[2]

    # Tello camera points forward, slightly downward
    # Camera frame: X right, Y down, Z forward
    # Drone body frame: X forward, Y left, Z up

    # Transform from camera frame to drone body frame
    # Assuming camera mounted facing forward, level
    tag_drone_x = tag_cam_z  # Forward
    tag_drone_y = -tag_cam_x  # Left (camera right is drone left)
    tag_drone_z = -tag_cam_y  # Up (camera down is drone up)

    # Rotate by drone yaw to get world frame
    yaw_rad = math.radians(drone_yaw)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    # Convert meters to cm
    tag_drone_x_cm = tag_drone_x * 100
    tag_drone_y_cm = tag_drone_y * 100
    tag_drone_z_cm = tag_drone_z * 100

    # Rotate and translate to world frame
    world_x = drone_position['x'] + (tag_drone_x_cm * cos_yaw - tag_drone_y_cm * sin_yaw)
    world_y = drone_position['y'] + (tag_drone_x_cm * sin_yaw + tag_drone_y_cm * cos_yaw)
    world_z = tag_drone_z_cm  # Assume tag is on ground (z=0), so just use relative height

    return {
        'x': world_x,
        'y': world_y,
        'z': 0.0  # Assume tags are on ground
    }


def world_to_grid(world_pos, grid_cell_size_cm):
    """
    Convert world position to grid coordinates.

    Args:
        world_pos: Position dict {x, y, z} in cm
        grid_cell_size_cm: Grid cell size in cm

    Returns:
        Tuple[int, int]: (grid_x, grid_y)
    """
    grid_x = int(world_pos['x'] / grid_cell_size_cm)
    grid_y = int(world_pos['y'] / grid_cell_size_cm)
    return grid_x, grid_y


def grid_to_world(grid_x, grid_y, grid_cell_size_cm, z=0):
    """
    Convert grid coordinates to world position.

    Args:
        grid_x: Grid X coordinate
        grid_y: Grid Y coordinate
        grid_cell_size_cm: Grid cell size in cm
        z: Z coordinate in cm (default 0)

    Returns:
        dict: World position {x, y, z} in cm
    """
    return {
        'x': grid_x * grid_cell_size_cm,
        'y': grid_y * grid_cell_size_cm,
        'z': z
    }


def add_gaussian_penalty(heatmap, center_x, center_y, radius, weight):
    """
    Add Gaussian penalty to obstacle heatmap around a point.

    Args:
        heatmap: 2D numpy array to modify
        center_x: Center X coordinate
        center_y: Center Y coordinate
        radius: Radius of influence in grid cells
        weight: Penalty weight at center
    """
    height, width = heatmap.shape
    y, x = np.ogrid[:height, :width]

    # Gaussian falloff
    distance = np.sqrt((x - center_x)**2 + (y - center_y)**2)
    mask = distance <= radius
    penalty = weight * np.exp(-(distance**2) / (2 * (radius / 3)**2))

    heatmap[mask] += penalty[mask]


def normalize_angle(angle_deg):
    """
    Normalize angle to -180 to 180 range.

    Args:
        angle_deg: Angle in degrees

    Returns:
        float: Normalized angle
    """
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def is_point_in_bounds(point, min_x, max_x, min_y, max_y):
    """
    Check if point is within rectangular bounds.

    Args:
        point: Position dict with x, y keys
        min_x, max_x, min_y, max_y: Boundary coordinates

    Returns:
        bool: True if point is within bounds
    """
    return min_x <= point['x'] <= max_x and min_y <= point['y'] <= max_y


def calculate_path_length(waypoints):
    """
    Calculate total path length from list of waypoints.

    Args:
        waypoints: List of position dicts or tuples

    Returns:
        float: Total path length in same units as waypoints
    """
    total_length = 0.0
    for i in range(len(waypoints) - 1):
        total_length += euclidean_distance(waypoints[i], waypoints[i + 1])
    return total_length
