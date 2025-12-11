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
import pyardrone
from pyardrone import at
import cv2.aruco as aruco
import msvcrt  # Windows keyboard input

## -----------------------------
# Drone Connection Functions
# -----------------------------
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    #High-level helpers + navdata, but no internal video connection.
    pass


def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None

# -----------------------------
# Config
# -----------------------------

FT_TO_M = 0.3048
WORLD_SIZE_M = 10 * FT_TO_M  # 10ft x 10ft ~ 3.048m
GRID_RES_M = 0.10            # 10cm cells
GRID_W = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))
GRID_H = int(math.ceil(WORLD_SIZE_M / GRID_RES_M))

TAG_ID_START = 0
TAG_ID_GOAL = 1
MARKER_SIZE_M = 0.175  # <-- set your real tag size

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
    def __init__(self):
        self.drone = ARDroneNoVideo()
        self.is_flying = False

        self.est_x = 0.0
        self.est_y = 0.0
        self.last_pose_time = time.time()

    def connect(self):
        print("[INFO] Connecting to AR.Drone (no internal video)...")
        print("[INFO] Connected.")
        print("[INFO] Waiting for navdata...")
        self.drone.navdata_ready.wait(timeout=10.0)
        if self.drone.navdata_ready.is_set():
            # Enable navdata demo so battery etc. are easy to read
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
            print("TAKEOFF")
            self.drone.takeoff()
            self.is_flying = True
            time.sleep(3)
            print("[OK] Airborne!")

    def land(self):
        if self.is_flying:
            print("\n[*] Landing before exit...")
            self.drone.land()
            time.sleep(3)
            self.is_flying = False

    def _get_frame_from_channel(self, channel: int, label: str) -> Optional[np.ndarray]:
        """
        Internal helper to grab one BGR frame from the given AR.Drone video channel.
        channel: 0 = front, 1 = bottom
        """
        print(f"[INFO] Switching to {label} camera (channel {channel})...")
        self.drone.send(at.CONFIG("video:video_channel", channel))
        time.sleep(0.5) 

        stream_url = "tcp://192.168.1.1:5555"
        print(f"[INFO] Opening {label} video stream: {stream_url}")
        cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print(f"[!] Could not open {label} video stream. Check stream URL / connection.")
            cap.release()
            return None
        
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            print(f"[!] Failed to read frame from {label} stream.")
            time.sleep(0.1)
            return None
        return frame

    def get_front_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from front camera."""
        return self._get_frame_from_channel(channel=0, label="FRONT")

    def get_bottom_frame(self) -> Optional[np.ndarray]:
        """Return BGR image from bottom camera."""
        return self._get_frame_from_channel(channel=1, label="BOTTOM")

    def get_local_pose_xytheta(self) -> Tuple[float, float, float]:
        """
        Return (x, y, yaw) estimate in meters/radians in a LOCAL odom frame.
        This can be VIO, optical flow + IMU fusion, etc.
        If you don't have this, you can still run exploration open-loop,
        but mapping accuracy will suffer.
        """
        demo = getattr(self.drone.navdata, "demo", None)
        if demo is None:
            return (self.est_x, self.est_y, 0.0)

        # --- Extract velocities from navdata ---
        vx = demo.vx / 1000.0    # convert mm/s → m/s (forward)
        vy = demo.vy / 1000.0    # convert mm/s → m/s (rightward)

        # --- Extract yaw angle ---
        yaw_rad = math.radians(demo.yaw)

        # --- Time integration ---
        now = time.time()
        dt = now - self.last_pose_time
        self.last_pose_time = now

        # Rotate body velocities into world frame
        world_vx =  vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)
        world_vy =  vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)

        # Integrate position
        self.est_x += world_vx * dt
        self.est_y += world_vy * dt

        return (self.est_x, self.est_y, yaw_rad)

    def command_velocity_xy_yaw(self, vx: float, vy: float, yaw_rate: float):
        """
        Command planar velocity in m/s and yaw rate in rad/s for a short burst.
        """
        max_lin = 1.0     # max m/s → scaled to full stick deflection
        max_yaw = 1.0     # max rad/s
        pitch =  np.clip(vx / max_lin, -1.0, 1.0)    # forward/back
        roll  =  np.clip(vy / max_lin, -1.0, 1.0)    # right/left
        yaw   =  np.clip(yaw_rate / max_yaw, -1.0, 1.0)
        gaz   =  0.0  # no vertical motion

        # send command
        self.drone.send(
            at.PCMD(
                flag=1,
                roll=roll,
                pitch=pitch,
                gaz=gaz,
                yaw=yaw
            )
        )

    def stop(self):
        self.command_velocity_xy_yaw(0.0, 0.0, 0.0)

    def shutdown(self): #additional Function to ctrl + C code
        if self.is_flying:
            self.land()
        print("[INFO] Shutting down...")
        self.drone.close()


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
    #Edit: This is a version that replaced the original placeholder function

    Methodology:
        - Look for strong edges / blobs in a central lower ROI.
        - Take the largest contour as "an obstacle".
        - Assume a fixed depth (e.g. 0.7 m).
        - Use horizontal image position to estimate lateral offset.
    """
    obstacles: List[Tuple[float, float]] = []
    h, w = frame_bgr.shape[:2]

    # Focus on the central lower part of the image: stuff in front & near ground
    roi = frame_bgr[int(h * 0.4):int(h * 0.9), :]   # rows [0.4h, 0.9h), all cols
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return obstacles

    # Pick the largest contour as our "main obstacle"
    largest = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    MIN_AREA = 800  # tune this experimentally
    if area < MIN_AREA:
        return obstacles

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return obstacles
    # Centroid in ROI coordinates
    u_roi = M["m10"] / M["m00"]
    # Convert to full-frame pixel x (ROI covers all columns, so this is same)
    u = u_roi
    # Camera intrinsics (must match your actual K!). See Main to know what to replace these.
    fx = 580.0
    cx = 311.98
    # Assume a fixed forward distance for all detections
    ASSUMED_DEPTH = 0.7  # meters in front of drone
    # Image x axis: right is positive.
    # We want y_left: positive to the LEFT, so flip sign.
    # y_left ≈ -Z * (u - cx) / fx
    x_forward = ASSUMED_DEPTH
    y_left = -ASSUMED_DEPTH * (u - cx) / fx

    obstacles.append((x_forward, y_left))
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
    """K = np.array([
        [600.0, 0.0, 320.0],
        [0.0, 600.0, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)"""

    #Changed cx and fx to these values in detect_obstacles_front()
    """K_front = np.array([
        [580.28, 0.0, 311.98], 
        [0.0, 579.76, 204.62],
        [0.0, 0.0,   1.0]
    ], dtype=np.float64)"""
    K_bottom = np.array([
        [223.99, 0.0, 87.30],
        [0.0, 224.06, 72.00],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    dist_bottom = np.zeros((5,1))

    drone = DroneInterface()
    try:
        drone.connect()
        drone.takeoff()

        grid = OccupancyGrid2D(GRID_W, GRID_H, GRID_RES_M)

        # 1) Find/start on Tag 0 using bottom camera
        T_C0_A = None
        start_world_xy = (0.0, 0.0)

        print("Searching for start tag #1 with bottom camera...")
        while T_C0_A is None:
            frame_bot = drone.get_bottom_frame()
            if frame_bot is None:
                time.sleep(0.05)
                continue

            poses = detect_aruco_poses(frame_bot, K_bottom, dist_bottom)
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

        # 3) Explore: update obstacles, scan for Tag 1
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

                # (C) check for tag #1 with bottom camera
                frame_bot = drone.get_bottom_frame()
                if frame_bot is not None:
                    poses = detect_aruco_poses(frame_bot, K_bottom, dist_bottom)
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
    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt caught. Emergency landing...")
    except Exception as e:
        print(f"\n[!] Exception occurred: {e}")
    finally:
        try:
            drone.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
