"""
We have:
- A correct high-level pipeline:
- detect start tag
- explore
- build obstacle grid
- detect goal tag
- plan A* path
- follow waypoints

Must replace:
1. DroneInterface
- camera frame getters
- velocity control
- pose estimate
2. Obstacle localization
- The placeholder is intentionally conservative and dumb.
- Best upgrades:
- add a tiny ToF sensor
- use monocular depth
- use a lightweight object detector and inflate safety zones
3. Frame correctness
- A production version would maintain:
- Tag-A anchored world frame
- T_A_C via odom + tag resets
- This scaffold simplifies that to keep it readable.
"""


"""
AR Drone mapping + ArUco goal finding + A* path planning
Assumptions:
- Drone has TWO cameras: front + bottom
- You have some way to command planar motion and read a rough pose estimate
- Bottom camera sees ArUco tags on the floor
- Front camera can detect obstacles (we provide a simple placeholder)

What you must replace:
- DroneInterface implementation (camera frames, takeoff/land, planar velocity/position control)
- detect_obstacles_front() with your real method (depth, ML, stereo, etc.)

This script focuses on architecture + mapping + planning logic.
"""

import time
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import cv2


# -----------------------------
# Config
# -----------------------------

FT_TO_M = 0.3048
WORLD_SIZE_M = 10 * FT_TO_M  # 10ft x 10ft ~ 3.048m
GRID_RES_M = 0.10            # 10cm cells
GRID_W = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))
GRID_H = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))

TAG_ID_START = 1
TAG_ID_GOAL = 2
MARKER_SIZE_M = 0.10  # <-- set your real tag size

# conservative inflation radius for obstacle safety (cells)
INFLATION_RADIUS_CELLS = 1

# exploration speed & timing placeholders
EXPLORATION_STEP_TIME = 0.5  # seconds per motion command burst


# -----------------------------
# Minimal drone abstraction
# -----------------------------

class DroneInterface:
    """
    Replace this with your real drone SDK wrapper.
    Required capabilities:
      - two camera frames
      - takeoff/land
      - simple planar motion commands
      - OPTIONAL: rough pose estimate in a local frame
    """

    def connect(self):
        pass

    def takeoff(self):
        pass

    def land(self):
        pass

    def get_front_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from front camera."""
        return None

    def get_bottom_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from bottom camera."""
        return None

    def get_local_pose_xytheta(self) -> Tuple[float, float, float]:
        """
        Return (x, y, yaw) estimate in meters/radians in a LOCAL odom frame.
        This can be VIO, optical flow + IMU fusion, etc.
        If you don't have this, you can still run exploration open-loop,
        but mapping accuracy will suffer.
        """
        return (0.0, 0.0, 0.0)

    def command_velocity_xy_yaw(self, vx: float, vy: float, yaw_rate: float):
        """
        Command planar velocity in m/s and yaw rate in rad/s for a short burst.
        """
        pass

    def stop(self):
        self.command_velocity_xy_yaw(0.0, 0.0, 0.0)


# -----------------------------
# Occupancy grid
# -----------------------------

class OccupancyGrid2D:
    """
    0 = unknown/free-ish, 1 = occupied
    We keep it simple for an MVP.
    """
    def __init__(self, width: int, height: int, res: float):
        self.width = width
        self.height = height
        self.res = res
        self.grid = np.zeros((height, width), dtype=np.uint8)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int(x / self.res)
        cy = int(y / self.res)
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        x = (cx + 0.5) * self.res
        y = (cy + 0.5) * self.res
        return x, y

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def mark_occupied_world(self, x: float, y: float):
        cx, cy = self.world_to_cell(x, y)
        if self.in_bounds(cx, cy):
            self.grid[cy, cx] = 1

    def inflate_obstacles(self, radius_cells: int):
        if radius_cells <= 0:
            return
        kernel_size = 2 * radius_cells + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        self.grid = cv2.dilate(self.grid, kernel, iterations=1)


# -----------------------------
# ArUco helpers
# -----------------------------

def detect_aruco_poses(frame_bgr, K, dist):
    """
    Returns dict: tag_id -> (rvec, tvec) where pose is tag->camera.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    poses = {}

    if ids is None:
        return poses

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE_M, K, dist
    )

    for i, tid in enumerate(ids.flatten()):
        poses[int(tid)] = (rvecs[i], tvecs[i])

    return poses


def T_from_rvec_tvec(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# -----------------------------
# Obstacle detection (placeholder)
# -----------------------------

def detect_obstacles_front(frame_bgr) -> List[Tuple[float, float]]:
    """
    Placeholder: returns obstacle points in the DRONE LOCAL frame (x_forward, y_left).
    YOU SHOULD REPLACE THIS.

    Without depth, camera-only obstacle localization is hard.
    For an MVP you can:
      - Use a lightweight ML detector + assume a fixed distance band
      - Or use monocular depth
      - Or add a small ToF sensor
    """
    # naive heuristic: treat large close-looking blobs near image center as "obstacle ahead"
    h, w = frame_bgr.shape[:2]
    roi = frame_bgr[int(h*0.35):int(h*0.85), int(w*0.25):int(w*0.75)]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 120)

    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = sum(cv2.contourArea(c) for c in cnts)

    # If lots of edge activity, assume something in front at ~0.6m
    obstacles = []
    if area > 2000:
        # obstacle point 0.6m forward, centered
        obstacles.append((0.6, 0.0))
    return obstacles


# -----------------------------
# Simple A* on grid
# -----------------------------

@dataclass
class Node:
    cx: int
    cy: int
    g: float
    f: float
    parent: Optional["Node"]


def astar(grid: OccupancyGrid2D, start_xy, goal_xy) -> Optional[List[Tuple[float, float]]]:
    sx, sy = start_xy
    gx, gy = goal_xy

    scx, scy = grid.world_to_cell(sx, sy)
    gcx, gcy = grid.world_to_cell(gx, gy)

    if not grid.in_bounds(scx, scy) or not grid.in_bounds(gcx, gcy):
        return None
    if grid.grid[scy, scx] == 1 or grid.grid[gcy, gcx] == 1:
        return None

    def h(cx, cy):
        return abs(cx - gcx) + abs(cy - gcy)

    open_set = {}
    closed = set()

    start = Node(scx, scy, g=0.0, f=h(scx, scy), parent=None)
    open_set[(scx, scy)] = start

    neighbors = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        current = min(open_set.values(), key=lambda n: n.f)
        key = (current.cx, current.cy)

        if key == (gcx, gcy):
            # reconstruct
            path_cells = []
            n = current
            while n:
                path_cells.append((n.cx, n.cy))
                n = n.parent
            path_cells.reverse()
            return [grid.cell_to_world(cx, cy) for cx, cy in path_cells]

        del open_set[key]
        closed.add(key)

        for dx, dy in neighbors:
            ncx, ncy = current.cx + dx, current.cy + dy
            nkey = (ncx, ncy)
            if not grid.in_bounds(ncx, ncy):
                continue
            if grid.grid[ncy, ncx] == 1:
                continue
            if nkey in closed:
                continue

            step = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
            ng = current.g + step

            if nkey not in open_set or ng < open_set[nkey].g:
                nf = ng + h(ncx, ncy)
                open_set[nkey] = Node(ncx, ncy, g=ng, f=nf, parent=current)

    return None


# -----------------------------
# Exploration pattern (lawnmower)
# -----------------------------

def generate_lawnmower_waypoints(size_m: float, step_m: float) -> List[Tuple[float, float]]:
    """
    Generate coverage waypoints in the Tag-A world frame.
    Assumes origin (0,0) at Tag A.
    Covers a square [0, size] x [0, size].
    """
    wps = []
    y = 0.2  # small offset from edge
    direction = 1

    while y < size_m - 0.2:
        if direction == 1:
            wps.append((0.2, y))
            wps.append((size_m - 0.2, y))
        else:
            wps.append((size_m - 0.2, y))
            wps.append((0.2, y))
        direction *= -1
        y += step_m

    return wps


# -----------------------------
# Main logic
# -----------------------------

def main():
    # --- Camera calibration placeholders ---
    # Replace with your real calibrated intrinsics for BOTH cameras if they differ.
    K = np.array([
        [600.0, 0.0, 320.0],
        [0.0, 600.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)

    drone = DroneInterface()
    drone.connect()
    drone.takeoff()

    grid = OccupancyGrid2D(GRID_W, GRID_H, GRID_RES_M)

    # 1) Find/start on Tag 1 using bottom camera
    T_C0_A = None
    start_world_xy = (0.0, 0.0)

    print("Searching for start tag #1 with bottom camera...")
    while T_C0_A is None:
        frame_bot = drone.get_bottom_frame()
        if frame_bot is None:
            time.sleep(0.05)
            continue

        poses = detect_aruco_poses(frame_bot, K, dist)
        if TAG_ID_START in poses:
            rvecA, tvecA = poses[TAG_ID_START]
            T_C0_A = T_from_rvec_tvec(rvecA, tvecA)  # A -> C0
            print("Start tag found. Setting Tag #1 as origin.")
        else:
            # tiny hover / micro adjust if needed
            drone.stop()
            time.sleep(0.05)

    # 2) Generate exploration waypoints in A-frame
    explore_wps = generate_lawnmower_waypoints(WORLD_SIZE_M, step_m=0.4)

    goal_world_xy = None

    # 3) Explore: update obstacles, scan for Tag 2
    for wp in explore_wps:
        # (A) Move roughly toward waypoint (very simple controller)
        # You should replace this with a proper position controller.
        for _ in range(5):
            # read pose
            x, y, yaw = drone.get_local_pose_xytheta()
            # NOTE: Here we assume local odom frame is close to A-frame for MVP simplicity.
            # A proper version would compute/maintain T_A_C from tag fixes + odom.
            dx = wp[0] - x
            dy = wp[1] - y

            dist_xy = math.hypot(dx, dy)
            if dist_xy < 0.15:
                drone.stop()
                break

            # simple proportional velocity
            vx = 0.4 * np.clip(dx, -0.3, 0.3)
            vy = 0.4 * np.clip(dy, -0.3, 0.3)
            drone.command_velocity_xy_yaw(vx, vy, 0.0)
            time.sleep(EXPLORATION_STEP_TIME)

            # (B) obstacle update from front camera
            frame_front = drone.get_front_frame()
            if frame_front is not None:
                obs_local = detect_obstacles_front(frame_front)
                # transform local obstacle point into world
                # For MVP: assume drone yaw small & local frame aligned with world
                # Replace with proper rotation by yaw:
                for ox_fwd, oy_left in obs_local:
                    # local -> world approx
                    ox = x + ox_fwd
                    oy = y + oy_left
                    grid.mark_occupied_world(ox, oy)

            # (C) check for tag #2 with bottom camera
            frame_bot = drone.get_bottom_frame()
            if frame_bot is not None:
                poses = detect_aruco_poses(frame_bot, K, dist)
                if TAG_ID_GOAL in poses:
                    # This tvec is in camera coords; for a floor tag, we mainly need XY in A-frame.
                    # For MVP: approximate goal at current drone position (x,y).
                    goal_world_xy = (x, y)
                    print("Goal tag #2 detected during exploration!")
                    drone.stop()
                    break

        if goal_world_xy is not None:
            break

    if goal_world_xy is None:
        print("Did not find tag #2 during coverage. Landing.")
        drone.land()
        return

    # 4) Inflate obstacles for safety
    grid.inflate_obstacles(INFLATION_RADIUS_CELLS)

    # 5) Plan path A -> B
    path = astar(grid, start_world_xy, goal_world_xy)
    if path is None or len(path) < 2:
        print("No valid path found. Landing.")
        drone.land()
        return

    print(f"Planned path with {len(path)} waypoints.")

    # 6) Follow path (simple waypoint follower)
    for wx, wy in path:
        for _ in range(8):
            x, y, yaw = drone.get_local_pose_xytheta()
            dx = wx - x
            dy = wy - y
            d = math.hypot(dx, dy)

            if d < 0.10:
                drone.stop()
                break

            vx = 0.5 * np.clip(dx, -0.25, 0.25)
            vy = 0.5 * np.clip(dy, -0.25, 0.25)
            drone.command_velocity_xy_yaw(vx, vy, 0.0)
            time.sleep(0.25)

    drone.stop()
    print("Arrived near goal (estimated). Landing.")
    drone.land()


if __name__ == "__main__":
    main()
