# Path Planning System for ArUco Tag Discovery

Autonomous exploration and optimal path planning system for DJI Tello drone using ArUco marker detection.

## Overview

This system enables the Tello drone to:
1. **Explore** a configurable search area to find two ArUco tags (start & finish)
2. **Record** all movements, obstacles, and tag detections during exploration
3. **Optimize** the path from start to finish using A* with time-based costs
4. **Visualize** both the exploration path and optimal path side-by-side
5. **Execute** the optimal path after user confirmation

## Features

- **Adaptive Grid Search**: Systematic lawnmower pattern with early termination when both tags found
- **360° Rotation Scanning**: At each waypoint, rotates to scan all directions
- **Obstacle Avoidance**: Integrated with existing YOLOv8-based obstacle detection
- **Dead Reckoning + ArUco**: Position tracking with absolute tag corrections
- **A* Path Optimization**: Time-based cost function considering distance, rotations, and obstacles
- **Dual-Panel Visualization**: Side-by-side comparison with statistics
- **Configurable via .env**: All parameters adjustable without code changes

## Installation

### Dependencies

```bash
pip install opencv-python opencv-contrib-python numpy matplotlib python-dotenv
```

Also requires dependencies from parent `obstacle_avoidance` system:
```bash
pip install djitellopy ultralytics torch torchvision
```

### ArUco Tag Preparation

1. Generate ArUco tags using DICT_4X4_50 dictionary:
   - Tag ID 0: 10cm x 10cm (start marker)
   - Tag ID 1: 2.5cm x 2.5cm (finish marker)

2. Print tags and place in search area

## Configuration

Edit [.env](.env) file to configure:

### Search Area (NOT HARDCODED!)
```bash
SEARCH_AREA_WIDTH_M=5.0          # Width in meters
SEARCH_AREA_HEIGHT_M=5.0         # Height in meters
SEARCH_ALTITUDE_M=1.2            # Flight altitude
```

### Grid Search
```bash
GRID_CELL_SIZE_M=1.5             # Spacing between waypoints
ENABLE_ROTATION_SCAN=true        # 360° scan at each waypoint
ROTATION_SCAN_STEPS=4            # 90° increments
```

### ArUco Detection
```bash
TAG_0_SIZE_M=0.1                 # Start tag size (10cm)
TAG_1_SIZE_M=0.025               # Finish tag size (2.5cm)
MIN_DETECTION_CONFIDENCE=0.8     # Confidence threshold
```

### Camera Calibration (Tello)
```bash
TELLO_FX=290.0                   # Focal length X
TELLO_FY=290.0                   # Focal length Y
TELLO_CX=160.0                   # Principal point X
TELLO_CY=120.0                   # Principal point Y
```

See [.env](.env) for all configuration options.

## Usage

### Basic Usage

```python
from path_planning import MissionController, PathPlanningConfig
from tello_server import DroneController, StateManager, VideoStreamHandler

# Initialize drone components
drone_controller = DroneController(state_manager)
state_manager = StateManager()
video_handler = VideoStreamHandler()

# Connect to drone
drone_controller.connect()

# Create mission controller
config = PathPlanningConfig()  # Loads from .env
mission = MissionController(
    drone_controller,
    state_manager,
    video_handler,
    config
)

# Run complete mission
success = mission.run_mission()
```

### Mission Phases

The mission runs in 4 automatic phases:

1. **Preflight Checks**
   - Battery level verification
   - Connection status check

2. **Exploration Phase**
   - Generates grid waypoints covering search area
   - Flies lawnmower pattern at configured altitude
   - Rotates 360° at each waypoint to scan for tags
   - Records ALL movements and obstacle avoidances
   - Terminates early when both tags found

3. **Optimization Phase**
   - Builds obstacle heatmap from exploration data
   - Runs A* pathfinding with time-based costs
   - Smooths path to remove redundant waypoints
   - Estimates execution time

4. **Visualization Phase**
   - Displays dual-panel matplotlib figure
   - Left: Exploration path with obstacles
   - Right: Optimal path with waypoint numbers
   - Statistics table with improvements

5. **Execution Phase** (after user confirmation)
   - Executes optimal path with obstacle avoidance
   - Lands at finish tag position

### Customization

#### Change Search Area

```bash
# Edit .env file
SEARCH_AREA_WIDTH_M=10.0    # Change to 10m x 10m
SEARCH_AREA_HEIGHT_M=10.0
```

#### Adjust Grid Density

```bash
GRID_CELL_SIZE_M=1.0        # Smaller = more thorough, slower
```

#### Disable User Confirmation

```bash
REQUIRE_USER_CONFIRMATION=false  # Auto-execute optimal path
```

## Architecture

### Module Overview

```
path_planning/
├── config.py                # Configuration with .env support
├── aruco_detector.py        # ArUco tag detection (adapted from LTT2.py)
├── search_planner.py        # Grid waypoint generation
├── movement_recorder.py     # Records all movements & obstacles
├── path_optimizer.py        # A* pathfinding with time costs
├── visualizer.py            # Matplotlib dual-panel plotting
├── mission_controller.py    # Main orchestrator
└── utils.py                 # Helper functions (transforms, distances)
```

### Integration with Existing System

- **Uses** `PathExecutor` from obstacle_avoidance for waypoint navigation
- **Uses** `ObstacleDetector` (YOLOv8) for obstacle avoidance during search
- **Uses** `VideoProcessor` for Tello video stream access
- **Wraps** PathExecutor command execution to record movements
- **Runs** YOLOv8 + ArUco detection in parallel on same frames

### Data Flow

```
User → MissionController
  ↓
SearchPlanner → generates waypoints
  ↓
PathExecutor → executes waypoints (with obstacle avoidance)
  ↓
ArucoDetector + ObstacleDetector → process frames in parallel
  ↓
MovementRecorder → logs everything
  ↓
PathOptimizer → calculates optimal path
  ↓
PathVisualizer → displays results
  ↓
User confirmation → execute optimal path
```

## Output

### Movement Log (JSON)

Saved to `./logs/path_planning/mission_XXXXXXXX_TIMESTAMP.json`

Contains:
- All movement commands with before/after positions
- ArUco tag detections with timestamps
- Obstacle encounters with positions
- Statistics (distance, time, battery, etc.)

### Visualization

Saved to `path_planning_result.png`

Shows:
- Exploration path (red) vs optimal path (green)
- Obstacle encounter zones
- ArUco tag positions
- Statistics comparison table

## Algorithm Details

### Exploration Strategy

**Adaptive Grid Search** with early termination:
- 4x4 grid at 1.5m spacing (configurable)
- Lawnmower pattern (left-to-right, step forward, repeat)
- 360° rotation scan at each waypoint (4x 90°)
- Stops when both tags detected with high confidence

### Path Optimization

**A* Algorithm** with time-based cost function:

**Cost Components:**
1. **Movement cost**: `distance / speed + command_overhead`
2. **Rotation cost**: `angle / rotation_speed`
3. **Obstacle penalty**: Weighted heatmap from exploration

**Heuristic**: Straight-line distance / speed (admissible)

**Path Smoothing**: Removes waypoints using line-of-sight check

### ArUco Detection

- **Dictionary**: DICT_4X4_50
- **Pose Estimation**: `cv2.aruco.estimatePoseSingleMarkers()`
- **Confidence**: Requires multiple detections (configurable threshold)
- **Coordinate Transform**: Camera frame → Drone frame → World frame

## Performance

### Expected Timings

| Phase | Time |
|-------|------|
| Exploration (5x5m) | 2-7 minutes |
| Optimization | 1-5 seconds |
| Visualization | 1-2 seconds |
| Execution | 30-90 seconds |
| **Total** | **3-10 minutes** |

### Battery Usage

- Exploration: ~20-30%
- Execution: ~5-10%
- **Total**: ~25-40% for complete mission

### Accuracy

- Position tracking: ±10-20cm drift over 3m (dead reckoning)
- ArUco detection range: 0.3-3m depending on tag size
- Path optimization: Typically 30-50% distance/time savings vs exploration

## Troubleshooting

### Tags Not Detected

- Check tag size matches configuration (TAG_0_SIZE_M, TAG_1_SIZE_M)
- Ensure tags are printed clearly with high contrast
- Verify tags are within camera field of view
- Increase `WAYPOINT_HOVER_TIME_SEC` for longer scanning
- Lower `MIN_DETECTION_CONFIDENCE` threshold

### Exploration Takes Too Long

- Increase `GRID_CELL_SIZE_M` (covers area faster, less thorough)
- Disable rotation scan: `ENABLE_ROTATION_SCAN=false`
- Reduce search area: `SEARCH_AREA_WIDTH_M` and `SEARCH_AREA_HEIGHT_M`

### Camera Calibration Inaccurate

- Use checkerboard calibration to refine Tello camera parameters
- Adjust `TELLO_FX`, `TELLO_FY`, `TELLO_CX`, `TELLO_CY` in .env
- See OpenCV camera calibration tutorials

### Path Optimization Fails

- Check that both tags were found during exploration
- Verify tag positions are within search area bounds
- Increase `GRID_CELL_SIZE_CM` for coarser A* grid

## Future Improvements

- [ ] Multi-tag support (>2 tags)
- [ ] Checkerboard camera calibration utility
- [ ] Real-time visualization during exploration
- [ ] Machine learning-based tag position prediction
- [ ] 3D path planning (varying altitude)
- [ ] RRT* alternative path algorithm

## License

Part of EECS C106A Final Project

## References

- OpenCV ArUco: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- A* Algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm
- DJI Tello SDK: https://github.com/damiafuentes/DJITelloPy
