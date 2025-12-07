# DJI Tello Autonomous Navigation System

Complete autonomous navigation system for DJI Tello drone with obstacle avoidance and ArUco-based path planning.

## System Overview

Two integrated systems:

1. **obstacle_avoidance/** - Real-time obstacle detection and avoidance using YOLOv8
2. **path_planning/** - ArUco tag discovery and optimal path planning using A* algorithm

## Installation

```bash
# Install all dependencies
pip install -r requirements.txt
```

**Requirements:**
- Python 3.8+
- DJI Tello drone
- djitellopy, opencv-python, ultralytics, numpy, matplotlib, python-dotenv

---

## 1. Obstacle Avoidance System

### Features
- YOLOv8-based real-time object detection (80 COCO classes)
- Distance estimation using pinhole camera model
- Lateral and vertical dodge maneuvers
- Dead reckoning position tracking with IMU integration
- Waypoint-based autonomous navigation

### How to Run

```bash
cd obstacle_avoidance
python test_navigation.py
```

Choose from 4 test scenarios:
1. Forward path (400cm)
2. L-shaped path (150cm + turn)
3. Square circuit (100cm × 100cm)
4. Short test (100cm)

### Custom Navigation

```python
from obstacle_avoidance import PathExecutor, Config
from tello_server import DroneController, StateManager, VideoStreamHandler

# Initialize
drone = DroneController(StateManager())
drone.connect()
video = VideoStreamHandler()
video.start()

# Create path executor
executor = PathExecutor(drone, StateManager(), video, Config())

# Define waypoints (x, y, z in cm)
waypoints = [
    (100, 0, 100),    # Forward 100cm
    (100, 100, 100),  # Left 100cm
    (0, 100, 100),    # Back 100cm
    (0, 0, 100)       # Right 100cm (square)
]

# Execute with obstacle avoidance
executor.execute_path(waypoints)
```

### Algorithms

**Obstacle Detection:**
- YOLOv8-nano for real-time detection (10-15 FPS)
- Distance formula: `distance = (object_height × focal_length) / pixel_height`
- Threat levels: HIGH (<1.5m), MEDIUM (<2.5m), LOW (<4.0m)

**Avoidance Maneuvers:**
- **Lateral Dodge**: 80cm sideways → 150cm forward → 80cm return (mathematically balanced)
- **Vertical Dodge**: 60cm up → forward → 60cm down

**Position Tracking:**
- Dead reckoning: `x += dist × cos(yaw)`, `y += dist × sin(yaw)`
- IMU yaw updates for improved accuracy
- Position accuracy: ±10-20cm per 3m traveled

### Configuration

Edit `obstacle_avoidance/config.py`:

```python
DETECTION_CONFIDENCE = 0.5        # Min confidence
THREAT_DISTANCE_HIGH = 1.5        # meters
WAYPOINT_TOLERANCE = 20           # cm
MIN_BATTERY_PERCENT = 20          # Auto-land
MAX_FLIGHT_TIME = 600             # seconds
```

---

## 2. Path Planning System

### Features
- Adaptive grid search for ArUco tag discovery
- A* pathfinding with time-based cost optimization
- Complete movement logging and replay
- Dual-panel visualization (exploration vs optimal)
- 360° rotation scanning for improved detection

### Setup

**1. Generate ArUco Tags** (https://chev.me/arucogen/)
- Dictionary: **4x4 (50, 100, 250, 1000)**
- Tag ID 0: 10cm × 10cm (start)
- Tag ID 1: 2.5cm × 2.5cm (finish)
- Print and place in search area

**2. Configure** `path_planning/.env`:

```bash
# Search area (fully configurable - NOT hardcoded)
SEARCH_AREA_WIDTH_M=5.0
SEARCH_AREA_HEIGHT_M=5.0
SEARCH_ALTITUDE_M=1.2

# Grid search
GRID_CELL_SIZE_M=1.5
ENABLE_ROTATION_SCAN=true
ROTATION_SCAN_STEPS=4

# ArUco tags
TAG_0_SIZE_M=0.1
TAG_1_SIZE_M=0.025
MIN_DETECTION_CONFIDENCE=0.8

# Tello camera (320x240)
TELLO_FX=290.0
TELLO_FY=290.0
TELLO_CX=160.0
TELLO_CY=120.0
```

### How to Run

```bash
cd path_planning
python example_usage.py
```

### Mission Phases

**Phase 1: Exploration (2-7 min)**
1. Generates 4×4 grid waypoints (1.5m spacing)
2. Flies lawnmower pattern at 1.2m altitude
3. At each waypoint: hovers 2s + rotates 360° (4× 90°)
4. Records ALL movements, obstacles, tag detections
5. Early termination when both tags found

**Phase 2: Optimization (1-5 sec)**
1. Builds obstacle heatmap from exploration
2. Runs A* pathfinding from Tag 0 → Tag 1
3. Smooths path (removes redundant waypoints)
4. Estimates execution time

**Phase 3: Visualization**
- Dual-panel matplotlib figure
- Left: Exploration path (red) with obstacles
- Right: Optimal path (green) with waypoints
- Statistics comparison table

**Phase 4: Execution (30-90 sec)**
- User confirmation required
- Executes optimal path with obstacle avoidance
- Lands at finish tag

### Algorithms

**Grid Search:**
- Lawnmower pattern for complete coverage
- Coverage: 5×5m in 5-7 min worst-case
- Early termination when both tags detected

**ArUco Detection:**
- OpenCV `estimatePoseSingleMarkers()` for pose estimation
- Coordinate transform: Camera → Drone → World frame
- Confidence: requires multiple detections (threshold 0.8)
- Detection range: 0.3-3m depending on tag size

**A* Path Optimization:**

Cost function (time-based):
```
f(n) = g(n) + h(n)

g(n) = Σ(distance/speed + rotation/rot_speed + obstacle_penalty)
h(n) = straight_line_distance / speed
```

- **Movement cost**: distance / 0.5 m/s + 0.3s overhead
- **Rotation cost**: angle / 180 deg/s
- **Obstacle penalty**: Weighted heatmap from exploration

**Path Smoothing:**
- Line-of-sight check between waypoints
- Removes intermediate points if clear path exists
- Reduces waypoint count by 30-50%

### Output Files

**Movement Log** (`logs/path_planning/mission_*.json`):
```json
{
  "movements": [
    {
      "command": "move_forward",
      "position_before": {"x": 0, "y": 0, "z": 120},
      "position_after": {"x": 100, "y": 0, "z": 120},
      "type": "planned | obstacle_avoidance"
    }
  ],
  "aruco_detections": [...],
  "obstacles_encountered": [...],
  "statistics": {...}
}
```

**Visualization** (`path_planning_result.png`):
- Exploration vs optimal path comparison
- Obstacle zones marked
- Tag positions shown
- Statistics table

---

## Architecture

### Obstacle Avoidance
```
PathExecutor
  ├── PositionEstimator (dead reckoning)
  ├── ObstacleDetector (YOLOv8)
  ├── ObstacleCircumvention (dodge maneuvers)
  └── VideoProcessor (frame access)
```

### Path Planning
```
MissionController
  ├── SearchPlanner (grid waypoints)
  ├── ArucoDetector (tag detection)
  ├── MovementRecorder (logging)
  ├── PathOptimizer (A* pathfinding)
  ├── PathVisualizer (matplotlib)
  └── Uses PathExecutor (from obstacle_avoidance)
```

**Integration:**
- Path planning uses obstacle avoidance for navigation
- Wraps PathExecutor to record movements
- Runs YOLOv8 + ArUco detection in parallel

---

## Performance

### Obstacle Avoidance
- Detection: 10-15 FPS (YOLOv8-nano)
- Position accuracy: ±10-20cm per 3m
- Response time: <100ms

### Path Planning
- Exploration: 2-7 min (5×5m)
- Optimization: 1-5 sec
- Execution: 30-90 sec
- Battery usage: 25-40% total
- Path improvement: 30-50% time/distance savings

---

## Troubleshooting

### Obstacle Avoidance
**"No obstacles detected"**
- Check lighting
- Verify `yolov8n.pt` in obstacle_avoidance/
- Lower confidence: `DETECTION_CONFIDENCE = 0.3`

**"Drone not responding"**
- Check WiFi: TELLO-XXXXXX
- Battery >20%
- Ping 192.168.10.1

### Path Planning
**"Tags not found"**
- Print tags with high contrast
- Check sizes: 10cm, 2.5cm
- Increase hover time: `WAYPOINT_HOVER_TIME_SEC=3.0`
- Lower confidence: `MIN_DETECTION_CONFIDENCE=0.5`

**"Exploration timeout"**
- Reduce area: `SEARCH_AREA_WIDTH_M=3.0`
- Increase spacing: `GRID_CELL_SIZE_M=2.0`

**"Optimization failed"**
- Verify both tags found
- Check tags within search area
- Increase grid: `GRID_CELL_SIZE_CM=30`

---

## File Structure

```
.
├── obstacle_avoidance/
│   ├── path_executor.py          # Main coordinator
│   ├── obstacle_detector.py      # YOLOv8 detection
│   ├── circumvent.py             # Avoidance maneuvers
│   ├── config.py                 # Configuration
│   └── test_navigation.py        # Test scenarios
│
├── path_planning/
│   ├── mission_controller.py     # Main orchestrator
│   ├── aruco_detector.py         # Tag detection
│   ├── search_planner.py         # Grid generation
│   ├── movement_recorder.py      # Logging
│   ├── path_optimizer.py         # A* pathfinding
│   ├── visualizer.py             # Plotting
│   ├── config.py                 # Environment config
│   ├── .env                      # Configuration
│   └── example_usage.py          # Usage example
│
├── tello_server.py               # Drone control server
└── requirements.txt              # Dependencies
```

---

## Safety

- Test in open area (5m × 5m minimum)
- Battery >60% before starting
- Emergency stop: Ctrl+C or `drone.send_command("emergency")`
- Auto-land at 20% battery
- 10-minute flight time limit

---

## License

EECS C106A Final Project
