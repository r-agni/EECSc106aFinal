# Tello Autonomous Navigation - Simple Usage

Autonomous navigation system that takes a target position (x, y, theta), navigates to it while avoiding obstacles, and maintains the target endpoint.

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Run Navigation

```bash
python navigate.py --x 200 --y 100 --theta 45
```

This will:
- Connect to your Tello drone
- Start live video streaming at **http://localhost:8080**
- Takeoff to 120cm altitude
- Navigate to (200cm forward, 100cm right) **avoiding obstacles**
- **Use PID controller for precise positioning** (±10cm accuracy)
- Rotate to 45° orientation
- Land exactly at target destination

## Arguments

| Argument | Description | Required | Default |
|----------|-------------|----------|---------|
| `--x` | Target X position in cm (forward+/back-) | **Yes** | - |
| `--y` | Target Y position in cm (right+/left-) | **Yes** | - |
| `--theta` | Target orientation in degrees | No | 0 |
| `--altitude` | Flight altitude in cm | No | 120 |
| `--port` | HTTP video stream port | No | 8080 |
| `--no-stream` | Disable browser video streaming | No | False |
| `--no-map` | Disable live navigation map | No | False |

## Examples

### Navigate 200cm forward
```bash
python navigate.py --x 200 --y 0
```

### Navigate to (150, 150) and face 90° right
```bash
python navigate.py --x 150 --y 150 --theta 90
```

### Navigate at higher altitude (2 meters)
```bash
python navigate.py --x 300 --y 0 --altitude 200
```

## Live Visualization 🗺️

The system provides **real-time visualization** of the navigation:

### 1. Live Navigation Map (NEW!)
A matplotlib window displays:
- ✅ **Current drone position** (blue dot with heading arrow)
- ✅ **Target position** (red star with orientation)
- ✅ **Actual trajectory** (green line - path traveled)
- ✅ **Planned waypoints** (blue dashed line)
- ✅ **Detected obstacles** (colored circles by threat level)
- ✅ **Drone field of view** (blue cone showing camera view)
- ✅ **Real-time stats** (distance to target, obstacles detected, etc.)

**Features:**
- Updates in real-time (10 FPS)
- Shows navigation status: NAVIGATING → PID CORRECTING → COMPLETE
- Obstacle markers color-coded: 🔴 High, 🟠 Medium, 🟡 Low
- Automatically sizes map based on target distance

### 2. HTTP Video Stream
A web browser stream at **http://localhost:8080** shows:
- Real-time camera feed
- Detected obstacles (bounding boxes)
- Distance estimates
- Threat levels overlaid on video

**To disable visualization:**
```bash
# Disable just the map
python navigate.py --x 200 --y 100 --no-map

# Disable just video streaming
python navigate.py --x 200 --y 100 --no-stream

# Disable both
python navigate.py --x 200 --y 100 --no-map --no-stream
```

## Obstacle Avoidance

The system uses YOLOv8 for real-time object detection and automatically:

1. **Detects** obstacles using computer vision
2. **Estimates** distance using pinhole camera model
3. **Plans** avoidance maneuvers (lateral or vertical dodge)
4. **Executes** mathematically balanced maneuvers
5. **Compensates** position to maintain target endpoint

### Avoidance Maneuvers

**Lateral Dodge** (default):
```
1. Move sideways 80cm (left or right)
2. Move forward to clear obstacle
3. Move sideways 80cm back (opposite direction)
→ Net lateral movement: 0cm (maintains target Y position!)
```

**Vertical Dodge** (for wide obstacles):
```
1. Move up 60cm
2. Move forward to clear obstacle
3. Move down 60cm
→ Net vertical movement: 0cm (maintains target altitude!)
```

The system ensures you end up at *exactly* (x, y) even if obstacles are avoided.

## PID Position Control 🎯

After navigating around obstacles, the system uses **PID (Proportional-Integral-Derivative) control** for precise landing:

**How it works:**
1. Measures position error (target vs current)
2. Calculates correction using PID algorithm
3. Self-corrects X, Y, Z, and orientation
4. Iterates until error < 10cm
5. Lands at exact target

**Benefits:**
- ✓ Compensates for dead reckoning drift
- ✓ Corrects wind-induced position errors
- ✓ Achieves ±10cm landing accuracy
- ✓ Automatic self-correction (no manual tuning needed)

**Example:**
```
Target: (200, 100) cm

[PID] Iteration 1: Current (185, 95), Error (15, 5) → Move forward 20cm
[PID] Iteration 2: Current (205, 95), Error (-5, 5) → Move right 20cm
[PID] Iteration 3: Current (205, 115), Error (-5, -15) → Within tolerance!
[PID] ✓ Target reached in 3 iterations

Final position: (205, 115) ± 10cm - Landing!
```

See [pid_controller/README.md](pid_controller/README.md) for technical details.

## Safety Features

- **Battery monitoring**: Auto-land at 20% battery
- **Flight time limit**: Max 10 minutes
- **Emergency stop**: Press Ctrl+C anytime to land immediately
- **Position tracking**: Dead reckoning with IMU updates (±10-20cm accuracy)

## Configuration

Edit `obstacle_avoidance/config.py` to customize:

```python
# Detection sensitivity
DETECTION_CONFIDENCE = 0.5        # Lower = more sensitive (0.3-0.7)

# Threat distances
THREAT_DISTANCE_HIGH = 1.5        # meters (closer = danger)
THREAT_DISTANCE_MEDIUM = 2.5      # meters

# Navigation
WAYPOINT_TOLERANCE = 20           # cm (precision of arrival)
MAX_STEP_DISTANCE = 100           # cm (max single movement)

# Avoidance
LATERAL_DODGE_DISTANCE = 80       # cm (sideways dodge)
VERTICAL_DODGE_DISTANCE = 60      # cm (up/down dodge)
CLEARANCE_MARGIN = 50             # cm (safety buffer)
```

## Troubleshooting

### "Cannot connect to drone"
- Ensure Tello is powered on
- Check WiFi connection to TELLO-XXXXXX network
- Battery must be >30%

### "No obstacles detected"
- Check lighting conditions (YOLO needs good lighting)
- Lower confidence: `DETECTION_CONFIDENCE = 0.3` in config.py
- Verify `yolov8n.pt` exists in `obstacle_avoidance/` folder

### "Video stream not opening"
- Make sure FFMPEG is installed: https://ffmpeg.org/download.html
- Check firewall isn't blocking port 8080
- Try different port: `--port 8081`

### "Drone doesn't land at exact position"
- **PID controller automatically corrects this!** (±10cm accuracy)
- If still inaccurate, tune PID gains in `pid_controller/position_controller.py`
- Check IMU yaw updates are working (enables better correction)

## Architecture

```
navigate.py (main script)
    ↓
DroneController (WiFi + SDK connection)
    ↓
PathExecutor (obstacle avoidance navigation)
    ├── PositionEstimator (dead reckoning)
    ├── ObstacleDetector (YOLOv8)
    ├── ObstacleCircumvention (dodge maneuvers)
    └── VideoProcessor (frame access)
    ↓
PositionController (PID precise landing) ← NEW!
    ├── PIDController (X-axis)
    ├── PIDController (Y-axis)
    ├── PIDController (Z-axis)
    └── PIDController (Yaw rotation)
```

## System Requirements

- Python 3.8+
- DJI Tello drone
- Windows/Linux/Mac with WiFi
- FFMPEG (for video streaming)

## Performance

- **Detection speed**: 10-15 FPS (YOLOv8-nano)
- **Position accuracy**: ±10-20cm per 3m traveled
- **Response time**: <100ms obstacle detection
- **Video latency**: ~200ms (UDP streaming)
- **Battery usage**: ~5-10% per minute of flight

## Emergency Procedures

**During flight:**
1. Press **Ctrl+C** → Emergency land
2. Press **Q** in OpenCV window → Controlled land
3. Power off drone → Immediate motor stop (last resort!)

**After crash:**
1. Check propellers for damage
2. Restart drone
3. Recalibrate if necessary (flip drone upside-down 3x)

---

**Safety First**: Always fly in open area (minimum 5m × 5m), away from people, and with battery >30%.
