"""
AR Drone mapping + ArUco goal finding + A* path planning with:
- 4-connected grid (only X/Y moves)
- Turn-penalizing A* (minimize direction changes)
- Tall obstacles: treated as hard 2D walls with inflation
- Path compressed into straight segments (fewest direction switches for that path)

Assumptions:
- Drone has TWO cameras: front + bottom
- You can command planar velocity and read rough pose estimate (x, y, yaw)
- Bottom camera sees ArUco tags on the floor (Tag 0 = start, Tag 1 = goal)
- Front camera can detect obstacles (placeholder here)

You must still:
- Calibrate front & bottom cameras and plug in intrinsics
- Improve obstacle detection as needed
"""

import time
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import numpy as np
import cv2
import cv2.aruco as aruco
import pyardrone
from pyardrone import at
import msvcrt  # Windows keyboard input


# =============================
# Drone Connection / Interface
# =============================

class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    # High-level helpers + navdata, but no internal video connection.
    pass


def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None


class DroneInterface:
    """
    AR.Drone wrapper:
      - connect, takeoff, land
      - get navdata-based local pose (x, y, yaw)
      - get frames from front & bottom cameras
      - send planar velocity commands
    """

    def __init__(self):
        self.drone = ARDroneNoVideo()
        self.is_flying = False

        # simple dead-reckoning pose
        self.est_x = 0.0
        self.est_y = 0.0
        self.last_pose_time = time.time()

        # simple video handling (reuse capture per channel)
        self.video_cap: Optional[cv2.VideoCapture] = None
        self.current_video_channel: Optional[int] = None
        self.stream_url = "tcp://192.168.1.1:5555"

    def connect(self):
        print("[INFO] Connecting to AR.Drone...")
        # pyardrone init implicitly connects
        print("[INFO] Connected.")
        print("[INFO] Waiting for navdata...")
        self.drone.navdata_ready.wait(timeout=10.0)
        if self.drone.navdata_ready.is_set():
            self.drone.send(at.CONFIG("general:navdata_demo", True))
            time.sleep(0.1)
            demo = getattr(self.drone.navdata, "demo", None)
            if demo:
                print(f"[NAVDATA] Battery: {demo.vbat_flying_percentage}%")
            else:
                print("[WARN] Demo navdata not populated yet.")
        else:
            print("[WARN] No navdata after 10s, continuing anyway.")

    def takeoff(self):
        if not self.is_flying:
            self.drone.send(at.FTRIM())
            time.sleep(1)
            print("[INFO] TAKEOFF")
            self.drone.takeoff()
            self.is_flying = True
            time.sleep(3)
            print("[OK] Airborne!")

    def land(self):
        if self.is_flying:
            print("[INFO] Landing...")
            self.drone.land()
            time.sleep(3)
            self.is_flying = False

    def _ensure_video_channel(self, channel: int, label: str) -> Optional[cv2.VideoCapture]:
        """
        Ensure we have a VideoCapture open for the requested AR.Drone video channel.
        channel: 0 = front, 1 = bottom
        """
        if self.current_video_channel != channel or self.video_cap is None:
            # switch channel
            print(f"[INFO] Switching to {label} camera (channel {channel})...")
            self.drone.send(at.CONFIG("video:video_channel", channel))
            time.sleep(0.5)

            # (re)open stream
            if self.video_cap is not None:
                self.video_cap.release()
            print(f"[INFO] Opening {label} video stream: {self.stream_url}")
            cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                print(f"[!] Could not open {label} video stream.")
                cap.release()
                self.video_cap = None
                self.current_video_channel = None
                return None
            self.video_cap = cap
            self.current_video_channel = channel

        return self.video_cap

    def _get_frame_from_channel(self, channel: int, label: str) -> Optional[np.ndarray]:
        cap = self._ensure_video_channel(channel, label)
        if cap is None:
            return None
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"[!] Failed to read frame from {label} stream.")
            return None
        return frame

    def get_front_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from front camera."""
        return self._get_frame_from_channel(channel=0, label="FRONT")

    def get_bottom_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from bottom camera."""
        return self._get_frame_from_channel(channel=1, label="BOTTOM")

    def reset_pose(self):
        """Reset odometry-based pose estimate (e.g., when Tag 0 is first seen)."""
        self.est_x = 0.0
        self.est_y = 0.0
        self.last_pose_time = time.time()

    def get_local_pose_xytheta(self) -> Tuple[float, float, float]:
        """
        Return (x, y, yaw) estimate in meters/radians in a LOCAL odom frame.
        Integrates navdata velocities; expect drift over time.
        """
        demo = getattr(self.drone.navdata, "demo", None)
        if demo is None:
            return (self.est_x, self.est_y, 0.0)

        # velocities mm/s -> m/s
        vx = getattr(demo, "vx", 0.0) / 1000.0
        vy = getattr(demo, "vy", 0.0) / 1000.0

        # yaw angle (deg)
        if hasattr(demo, "yaw"):
            yaw_deg = demo.yaw
        elif hasattr(demo, "psi"):
            yaw_deg = demo.psi
        elif hasattr(demo, "rotZ"):
            yaw_deg = demo.rotZ
        else:
            yaw_deg = 0.0

        yaw_rad = math.radians(yaw_deg)

        # time integration
        now = time.time()
        dt = now - self.last_pose_time
        self.last_pose_time = now

        # rotate body velocities into world frame
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        world_vx = vx * cos_y - vy * sin_y
        world_vy = vx * sin_y + vy * cos_y

        self.est_x += world_vx * dt
        self.est_y += world_vy * dt

        return (self.est_x, self.est_y, yaw_rad)

    def command_velocity_xy_yaw(self, vx: float, vy: float, yaw_rate: float):
        """
        Command planar velocity in body frame (m/s) and yaw rate (rad/s).
        vx: forward+, vy: right+.
        """
        max_lin = 0.2
        max_yaw = 1.0

        vx_c = float(np.clip(vx, -max_lin, max_lin))
        vy_c = float(np.clip(vy, -max_lin, max_lin))
        yaw_c = float(np.clip(yaw_rate, -max_yaw, max_yaw))

        # map to AR.Drone stick commands
        forward = backward = right = left = ccw = cw = 0.0
        if vx_c > 0:
            forward = vx_c / max_lin
        elif vx_c < 0:
            backward = -vx_c / max_lin
        if vy_c > 0:
            right = vy_c / max_lin
        elif vy_c < 0:
            left = -vy_c / max_lin
        if yaw_c > 0:
            ccw = yaw_c / max_yaw
        elif yaw_c < 0:
            cw = -yaw_c / max_yaw

        self.drone.move(
            forward=forward,
            backward=backward,
            right=right,
            left=left,
            cw=cw,
            ccw=ccw,
            up=0,
            down=0,
        )

    def stop(self):
        self.drone.move(forward=0, backward=0, left=0, right=0, up=0, down=0, cw=0, ccw=0)

    def shutdown(self):
        if self.is_flying:
            self.land()
        print("[INFO] Shutting down...")
        self.drone.close()


# =============================
# Occupancy Grid
# =============================

class OccupancyGrid2D:
    """
    0 = free/unknown, 1 = occupied
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


# =============================
# ArUco helpers
# =============================

def detect_aruco_poses(frame_bgr, marker_size_m, K, dist) -> Dict[int, Tuple[np.ndarray, np.ndarray]]:
    """
    Returns dict: tag_id -> (rvec, tvec) where pose is tag->camera.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()

    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
    poses: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}

    if ids is None:
        return poses

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, marker_size_m, K, dist
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


# =============================
# Obstacle detection (front cam)
# =============================

def detect_obstacles_front(frame_bgr,
                           fx: float,
                           cx: float,
                           assumed_depth: float = 0.7) -> List[Tuple[float, float]]:
    """
    Very rough obstacle detector:
      - Look for strong edges / blobs in a central-lower ROI.
      - Use largest contour as "obstacle".
      - Assume fixed depth (assumed_depth).
      - Use horizontal centroid to infer lateral offset in camera frame.

    Returns a list of obstacle points (x_forward, y_left) in camera frame.
    """
    obstacles: List[Tuple[float, float]] = []
    h, w = frame_bgr.shape[:2]

    # central-lower ROI: things in front & near ground
    roi = frame_bgr[int(h * 0.4):int(h * 0.9), :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return obstacles

    largest = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    MIN_AREA = 800
    if area < MIN_AREA:
        return obstacles

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return obstacles
    # centroid in ROI
    u_roi = M["m10"] / M["m00"]
    u = u_roi  # same columns

    x_forward = assumed_depth
    # y_left: positive to left
    y_left = -assumed_depth * (u - cx) / fx

    obstacles.append((x_forward, y_left))
    return obstacles


# =============================
# A* with turn penalties (4-connected)
# =============================

@dataclass
class Node:
    cx: int
    cy: int
    dir: int      # 0:+x, 1:-x, 2:+y, 3:-y, -1: no heading yet
    g: float
    f: float
    parent: Optional["Node"]


DIR_OFFSETS = {
    0: (1, 0),   # +x
    1: (-1, 0),  # -x
    2: (0, 1),   # +y
    3: (0, -1),  # -y
}


def astar_min_turns(grid: OccupancyGrid2D,
                    start_xy: Tuple[float, float],
                    goal_xy: Tuple[float, float],
                    turn_cost: float = 5.0) -> Optional[List[Tuple[int, int]]]:
    """
    A* on a 4-connected grid that penalizes direction changes.
    State = (cx, cy, dir), where dir ∈ {0,1,2,3} or -1 (no heading yet).

    Returns path as list of (cx, cy) cells, or None if unreachable.
    """

    sx, sy = start_xy
    gx, gy = goal_xy
    scx, scy = grid.world_to_cell(sx, sy)
    gcx, gcy = grid.world_to_cell(gx, gy)

    if not grid.in_bounds(scx, scy) or not grid.in_bounds(gcx, gcy):
        print("[A*] Start or goal out of bounds.")
        return None
    if grid.grid[scy, scx] == 1 or grid.grid[gcy, gcx] == 1:
        print("[A*] Start or goal in occupied cell.")
        return None

    def h(cx: int, cy: int) -> float:
        # Manhattan is admissible/consistent for 4-connected with unit step
        return abs(cx - gcx) + abs(cy - gcy)

    open_set: Dict[Tuple[int, int, int], Node] = {}
    closed: set = set()

    start_node = Node(scx, scy, dir=-1, g=0.0, f=h(scx, scy), parent=None)
    open_set[(scx, scy, -1)] = start_node

    STEP_COST = 1.0

    while open_set:
        current = min(open_set.values(), key=lambda n: n.f)
        key = (current.cx, current.cy, current.dir)

        if current.cx == gcx and current.cy == gcy:
            # reconstruct path in cell coordinates
            cells: List[Tuple[int, int]] = []
            n = current
            while n:
                cells.append((n.cx, n.cy))
                n = n.parent
            cells.reverse()
            return cells

        del open_set[key]
        closed.add(key)

        for dir_next, (dx, dy) in DIR_OFFSETS.items():
            ncx = current.cx + dx
            ncy = current.cy + dy
            nkey = (ncx, ncy, dir_next)

            if not grid.in_bounds(ncx, ncy):
                continue
            if grid.grid[ncy, ncx] == 1:
                continue
            if nkey in closed:
                continue

            # cost to move one step in this direction
            cost_turn = 0.0
            if current.dir != -1 and dir_next != current.dir:
                cost_turn = turn_cost
            ng = current.g + STEP_COST + cost_turn

            if nkey not in open_set or ng < open_set[nkey].g:
                nf = ng + h(ncx, ncy)
                open_set[nkey] = Node(ncx, ncy, dir_next, g=ng, f=nf, parent=current)

    return None


def compress_cells_to_segments(grid: OccupancyGrid2D,
                               path_cells: List[Tuple[int, int]]
                               ) -> List[Tuple[float, float]]:
    """
    Given a path as a list of cells, compress into straight segments:
    return list of world (x, y) waypoints at segment boundaries.
    """
    if not path_cells:
        return []
    if len(path_cells) == 1:
        cx, cy = path_cells[0]
        return [grid.cell_to_world(cx, cy)]

    waypoints_world: List[Tuple[float, float]] = []

    # Always include the start
    prev_cx, prev_cy = path_cells[0]
    waypoints_world.append(grid.cell_to_world(prev_cx, prev_cy))

    last_dx = last_dy = None

    for i in range(1, len(path_cells)):
        cx, cy = path_cells[i]
        dx = cx - prev_cx
        dy = cy - prev_cy
        if last_dx is None:
            last_dx, last_dy = dx, dy
        else:
            if (dx, dy) != (last_dx, last_dy):
                # direction changed at previous cell => segment ends there
                seg_end_cx, seg_end_cy = path_cells[i - 1]
                waypoints_world.append(grid.cell_to_world(seg_end_cx, seg_end_cy))
                last_dx, last_dy = dx, dy
        prev_cx, prev_cy = cx, cy

    # include final cell
    end_cx, end_cy = path_cells[-1]
    waypoints_world.append(grid.cell_to_world(end_cx, end_cy))

    return waypoints_world


# =============================
# Obstacle-aware lawnmower (optional for exploration)
# =============================

def generate_lawnmower_waypoints_with_obstacles(
    grid: OccupancyGrid2D,
    step_rows: int = 1,
    margin_cells: int = 1,
) -> List[Tuple[float, float]]:
    """
    Generate lawnmower-style waypoints that only pass through free cells.
    We operate in grid coordinates; skip occupied cells.

    - step_rows: grid rows between passes
    - margin_cells: border of cells to leave unused at edges
    """
    wps: List[Tuple[float, float]] = []
    direction = 1  # +1: left->right, -1: right->left

    cy = margin_cells
    max_row = grid.height - margin_cells

    while cy < max_row:
        free_cells: List[int] = []
        for cx in range(margin_cells, grid.width - margin_cells):
            if grid.grid[cy, cx] == 0:
                free_cells.append(cx)

        if free_cells:
            xs = free_cells if direction == 1 else list(reversed(free_cells))
            for cx in xs:
                x_w, y_w = grid.cell_to_world(cx, cy)
                wps.append((x_w, y_w))

        direction *= -1
        cy += step_rows

    return wps


# =============================
# Config
# =============================

FT_TO_M = 0.3048
WORLD_SIZE_M = 10 * FT_TO_M        # 10ft x 10ft
GRID_RES_M = 0.10                  # 10cm cells
GRID_W = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))
GRID_H = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))

TAG_ID_START = 0
TAG_ID_GOAL = 1
MARKER_SIZE_M = 0.15

INFLATION_RADIUS_CELLS = 2

EXPLORATION_STEP_TIME = 0.5  # s per motion burst


# =============================
# Main
# =============================

def main():
    # Camera intrinsics (example; replace with your calibrated values)
    # Front camera (for obstacles)
    K_front = np.array([
        [580.28, 0.0,   311.98],
        [0.0,   579.76, 204.62],
        [0.0,     0.0,    1.0],
    ], dtype=np.float64)
    fx_front = K_front[0, 0]
    cx_front = K_front[0, 2]

    # Bottom camera (for tags)
    K_bottom = np.array([
        [223.99, 0.0, 87.30],
        [0.0,   224.06, 72.00],
        [0.0,     0.0,  1.0]
    ], dtype=np.float64)
    dist_bottom = np.zeros((5, 1), dtype=np.float64)

    drone = DroneInterface()
    grid = OccupancyGrid2D(GRID_W, GRID_H, GRID_RES_M)

    try:
        drone.connect()
        drone.takeoff()

        # 1) Find/start on Tag 0 using bottom camera
        T_C0_A = None  # tag 0 in camera frame at start
        start_world_xy = (0.0, 0.0)

        print("[INFO] Searching for start tag #0 with bottom camera...")
        while T_C0_A is None:
            frame_bot = drone.get_bottom_frame()
            if frame_bot is None:
                time.sleep(0.05)
                continue

            poses = detect_aruco_poses(frame_bot, MARKER_SIZE_M, K_bottom, dist_bottom)
            if TAG_ID_START in poses:
                rvecA, tvecA = poses[TAG_ID_START]
                T_C0_A = T_from_rvec_tvec(rvecA, tvecA)  # A -> C0
                print("[INFO] Start tag found. Resetting odom and setting Tag #0 as origin.")
                drone.reset_pose()
            else:
                drone.stop()
                time.sleep(0.05)

        # 2) (Optional) initial inflation before lawnmower
        grid.inflate_obstacles(INFLATION_RADIUS_CELLS)

        # 3) Generate exploration waypoints in (approximate) world frame
        explore_wps = generate_lawnmower_waypoints_with_obstacles(
            grid, step_rows=1, margin_cells=1
        )

        goal_world_xy: Optional[Tuple[float, float]] = None

        print("[INFO] Starting exploration for Tag #1...")
        for wp in explore_wps:
            # simple waypoint follower for exploration
            MAX_TIME = 5.0
            t0 = time.time()
            while time.time() - t0 < MAX_TIME:
                x, y, yaw = drone.get_local_pose_xytheta()
                dx = wp[0] - x
                dy = wp[1] - y
                dist_xy = math.hypot(dx, dy)
                if dist_xy < 0.15:
                    drone.stop()
                    break

                # simple proportional control (still body-frame velocities)
                vx = 0.4 * np.clip(dx, -0.3, 0.3)
                vy = 0.4 * np.clip(dy, -0.3, 0.3)
                drone.command_velocity_xy_yaw(vx, vy, 0.0)
                time.sleep(EXPLORATION_STEP_TIME)

                # (B) obstacle update from front camera
                frame_front = drone.get_front_frame()
                if frame_front is not None:
                    obs_cam = detect_obstacles_front(frame_front, fx_front, cx_front)
                    # transform from camera frame (approx same as body) to world frame using yaw
                    for ox_fwd, oy_left in obs_cam:
                        cos_y = math.cos(yaw)
                        sin_y = math.sin(yaw)
                        # local → world
                        ox_world = x + ox_fwd * cos_y - oy_left * sin_y
                        oy_world = y + ox_fwd * sin_y + oy_left * cos_y
                        grid.mark_occupied_world(ox_world, oy_world)

                # (C) check for Tag #1 with bottom camera
                frame_bot = drone.get_bottom_frame()
                if frame_bot is not None:
                    poses = detect_aruco_poses(frame_bot, MARKER_SIZE_M, K_bottom, dist_bottom)
                    if TAG_ID_GOAL in poses:
                        # Roughly place goal at current drone position (better: use tvec)
                        goal_world_xy = (x, y)
                        print("[INFO] Goal tag #1 detected during exploration!")
                        drone.stop()
                        break

            if goal_world_xy is not None:
                break

        if goal_world_xy is None:
            print("[WARN] Did not find tag #1 during coverage. Landing.")
            drone.land()
            return

        # 4) Inflate obstacles for safety (tall: can't fly over, so treat as walls)
        grid.inflate_obstacles(INFLATION_RADIUS_CELLS)

        # 5) Plan path A -> B with A* that penalizes turns (4-connected only)
        print("[INFO] Planning path from Tag #0 to Tag #1 with A* (min turns)...")
        path_cells = astar_min_turns(grid, start_world_xy, goal_world_xy, turn_cost=5.0)
        if path_cells is None or len(path_cells) < 2:
            print("[WARN] No valid path found. Landing.")
            drone.land()
            return

        # convert to straight segments in world (min direction changes for that path)
        path_world_segments = compress_cells_to_segments(grid, path_cells)
        print(f"[INFO] Planned path has {len(path_world_segments)} segment waypoints.")

        # 6) Follow segment waypoints (approximate; still vx/vy control)
        print("[INFO] Executing path...")
        for wx, wy in path_world_segments:
            MAX_TIME = 8.0
            t0 = time.time()
            while time.time() - t0 < MAX_TIME:
                x, y, yaw = drone.get_local_pose_xytheta()
                dx = wx - x
                dy = wy - y
                d = math.hypot(dx, dy)
                if d < 0.10:
                    drone.stop()
                    break

                # move roughly along world X/Y direction
                vx = 0.5 * np.clip(dx, -0.25, 0.25)
                vy = 0.5 * np.clip(dy, -0.25, 0.25)
                drone.command_velocity_xy_yaw(vx, vy, 0.0)
                time.sleep(0.25)

        drone.stop()
        print("[INFO] Arrived near goal (estimated). Landing.")
        drone.land()

    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt caught. Emergency landing...")
        try:
            drone.land()
        except Exception:
            pass
    except Exception as e:
        print(f"\n[!] Exception occurred: {e}")
    finally:
        try:
            drone.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
