# WASD Manual Control Mode

Manual drone control using keyboard with live video streaming and position tracking dashboard.

## Quick Start

```bash
python wasd_control.py
```

## Keyboard Controls

### Flight Controls
- **T** - Takeoff
- **L** - Land
- **SPACE** - Emergency stop
- **ESC** - Quit program

### Movement (only while in flight)
- **W** - Move forward (30cm)
- **S** - Move backward (30cm)
- **A** - Move left (30cm)
- **D** - Move right (30cm)
- **R** - Move up (30cm)
- **F** - Move down (30cm)

### Rotation
- **Q** - Rotate counter-clockwise (30°)
- **E** - Rotate clockwise (30°)

## Features

1. **Live Video Stream**: View drone camera at http://localhost:8080
2. **Interactive Dashboard**: Track position in real-time at http://localhost:8081
3. **Obstacle Detection**: Bounding boxes displayed on video stream
4. **Position Tracking**: Dead reckoning tracks drone position on map
5. **Safety**: Auto-land on exit if drone is still in flight

## Command Line Options

```bash
# Disable video streaming
python wasd_control.py --no-stream

# Disable dashboard map
python wasd_control.py --no-map

# Use custom port for video stream
python wasd_control.py --port 9000
```

## Usage Notes

- **Movement distance**: Each keypress moves 30cm
- **Rotation angle**: Each keypress rotates 30°
- **Safety**: Emergency stop (SPACE) immediately cuts motors
- **Position tracking**: Dashboard shows estimated position based on commands
- **Video overlays**: Detected obstacles shown with bounding boxes

## Comparison to Autonomous Mode

| Feature | WASD Control | Autonomous (`navigate.py`) |
|---------|-------------|---------------------------|
| Control | Manual keyboard | Autonomous waypoint navigation |
| Video Stream | ✓ | ✓ |
| Dashboard | ✓ | ✓ |
| Obstacle Detection | Visual only | Avoidance enabled |
| Position Tracking | Dead reckoning | Dead reckoning + IMU (optional) |
| PID Control | ✗ | ✓ (optional) |

## Tips

1. **Press T to takeoff first** - Movement controls only work while in flight
2. **Watch your battery** - Check battery percentage on startup
3. **Use dashboard** - Monitor position and orientation in real-time
4. **Emergency landing** - Press L to land safely, SPACE for immediate stop
5. **Smooth movements** - Wait for each movement to complete before next keypress

## Troubleshooting

**Controls not working:**
- Make sure you pressed T to takeoff first
- Check that the terminal window has focus
- On Linux, you may need to run with `sudo` for keyboard access

**Drone not responding:**
- Check WiFi connection to Tello network
- Verify battery level is above 30%
- Try emergency stop (SPACE) and re-takeoff (T)

**Dashboard not updating:**
- Verify you didn't use `--no-map` flag
- Check http://localhost:8081 in browser
- Position updates after each movement command
