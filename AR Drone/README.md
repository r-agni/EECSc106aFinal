# AR Drone Complete API

A comprehensive Python API wrapper for controlling the Parrot AR.Drone 2.0 with all available commands and functions.

## Installation

```bash
pip install pyardrone opencv-python
```

## Quick Start

```python
from drone_api import DroneAPI

# Connect to drone
drone = DroneAPI()

# Take off
drone.takeoff()
time.sleep(5)

# Move around
drone.move_forward(speed=0.5)
time.sleep(2)

# Land
drone.land()
```

## Features

### Flight Control Commands

- `takeoff()` - Take off and hover
- `land()` - Land at current position
- `hover()` - Maintain current position
- `emergency()` - Emergency stop (cuts motors)
- `reset_emergency()` - Reset emergency state

### Movement Commands

#### Simple Movement
- `move_forward(speed)` - Move forward (0.0 to 1.0)
- `move_backward(speed)` - Move backward (0.0 to 1.0)
- `move_left(speed)` - Move left (0.0 to 1.0)
- `move_right(speed)` - Move right (0.0 to 1.0)
- `move_up(speed)` - Ascend (0.0 to 1.0)
- `move_down(speed)` - Descend (0.0 to 1.0)
- `rotate_clockwise(speed)` - Rotate CW (0.0 to 1.0)
- `rotate_counterclockwise(speed)` - Rotate CCW (0.0 to 1.0)

#### Combined Movement
```python
drone.move(forward=0.5, left=0.3, up=0.2)
```

#### Advanced Direct Control
```python
drone.fly_direct(pitch=0.3, roll=0.2, yaw=0.4, gaz=0.1)
```

### Calibration

- `flat_trim()` - Calibrate horizontal plane (do this on flat surface before flight)
- `calibrate_magnetometer(device_num=0)` - Calibrate magnetometer

### Configuration

#### Altitude & Speed Limits
```python
drone.set_max_altitude(2000)  # 2 meters in mm
drone.set_max_euler_angle(20.0)  # Max tilt in degrees
drone.set_max_vertical_speed(700)  # mm/s
drone.set_max_rotation_speed(200)  # deg/s
```

#### Flight Mode
```python
drone.enable_outdoor_mode(True)  # Use GPS/magnetometer
drone.enable_outdoor_hull(False)  # Hull removed for outdoor flight
```

#### Custom Configuration
```python
drone.set_config('control:altitude_max', 3000)
```

### Telemetry & Status

#### Individual Sensors
```python
battery = drone.get_battery_percentage()  # 0-100
altitude = drone.get_altitude()  # mm
rotation = drone.get_rotation()  # {'pitch', 'roll', 'yaw'} in degrees
velocity = drone.get_velocity()  # {'vx', 'vy', 'vz'} in mm/s
```

#### State Checks
```python
is_flying = drone.is_flying()
is_emergency = drone.is_emergency()
state = drone.get_state()
```

#### Complete Telemetry
```python
telemetry = drone.get_full_telemetry()
drone.print_status()  # Print formatted status
```

### Video Streaming

#### Video Access
```python
# Wait for video to be ready
drone.wait_for_video(timeout=5.0)

# Get current frame
frame = drone.get_frame()  # OpenCV BGR image

# Check if video is ready
if drone.is_video_ready():
    frame = drone.get_frame()
```

#### Display Video
```python
# Display in OpenCV window
key = drone.display_video("AR.Drone Video")
if key == 27:  # ESC pressed
    print("Exit")
```

#### Save Frames
```python
drone.save_frame("snapshot.jpg")
```

### Utility Functions

```python
# Send watchdog to maintain connection
drone.send_watchdog()

# Print current status
drone.print_status()

# Close connection
drone.close()
```

### Context Manager

```python
with DroneAPI() as drone:
    drone.takeoff()
    time.sleep(5)
    drone.land()
# Automatically closes connection
```

## Examples

### Example 1: Basic Flight

```python
from drone_api import DroneAPI
import time

with DroneAPI() as drone:
    drone.takeoff()
    time.sleep(5)

    # Hover for 5 seconds
    for _ in range(50):
        drone.hover()
        time.sleep(0.1)

    drone.land()
    time.sleep(3)
```

### Example 2: Square Pattern

```python
with DroneAPI() as drone:
    drone.takeoff()
    time.sleep(5)

    # Fly in a square
    for side in range(4):
        # Move forward for 2 seconds
        end_time = time.time() + 2.0
        while time.time() < end_time:
            drone.move_forward(0.4)
            time.sleep(0.05)

        # Rotate 90 degrees
        end_time = time.time() + 1.0
        while time.time() < end_time:
            drone.rotate_clockwise(0.5)
            time.sleep(0.05)

    drone.land()
```

### Example 3: Video Streaming with Movement

```python
drone = DroneAPI()

if drone.wait_for_video(timeout=10.0):
    drone.takeoff()
    time.sleep(5)

    start_time = time.time()
    while time.time() - start_time < 10.0:
        # Move in a pattern
        drone.move_forward(0.3)

        # Display video
        key = drone.display_video()
        if key == 27:  # ESC
            break

        time.sleep(0.05)

    drone.land()
    time.sleep(3)

drone.close()
```

### Example 4: Telemetry Monitoring

```python
with DroneAPI() as drone:
    drone.takeoff()
    time.sleep(5)

    # Monitor for 10 seconds
    for i in range(20):
        telemetry = drone.get_full_telemetry()

        print(f"Battery: {telemetry['battery']}%")
        print(f"Altitude: {telemetry['altitude']} mm")
        print(f"Flying: {telemetry['is_flying']}")

        if telemetry['rotation']:
            print(f"Yaw: {telemetry['rotation']['yaw']:.2f}°")

        drone.hover()
        time.sleep(0.5)

    drone.land()
```

### Example 5: Circular Flight Pattern

```python
with DroneAPI() as drone:
    drone.takeoff()
    time.sleep(5)

    # Fly in circle using direct control
    end_time = time.time() + 5.0
    while time.time() < end_time:
        drone.fly_direct(
            pitch=0.3,  # Forward
            yaw=0.4,    # Rotate
            roll=0.0,
            gaz=0.0
        )
        time.sleep(0.05)

    drone.land()
```

## Running the Examples

The `examples.py` file contains 10 complete example programs:

```bash
python examples.py
```

Select from:
1. Basic Flight
2. Simple Movement
3. Complete Movement Demo
4. Square Pattern Flight
5. Video Streaming
6. Telemetry Monitoring
7. Configuration
8. Programmed Maneuver Sequence
9. Video Recording
10. Advanced Direct Control

## API Reference

### Complete Method List

#### Flight Control
- `takeoff()` - Ascend to hover altitude
- `land()` - Descend and land
- `hover()` - Maintain position
- `emergency()` - Cut motors immediately
- `reset_emergency()` - Reset emergency state

#### Movement (High-Level)
- `move(forward, backward, left, right, up, down, cw, ccw)` - Combined movement
- `move_forward(speed)` - Forward movement
- `move_backward(speed)` - Backward movement
- `move_left(speed)` - Left strafe
- `move_right(speed)` - Right strafe
- `move_up(speed)` - Ascend
- `move_down(speed)` - Descend
- `rotate_clockwise(speed)` - CW rotation
- `rotate_counterclockwise(speed)` - CCW rotation

#### Movement (Low-Level)
- `fly_direct(roll, pitch, yaw, gaz)` - Direct PCMD control

#### Calibration
- `flat_trim()` - Calibrate level
- `calibrate_magnetometer(device_num)` - Calibrate compass

#### Configuration
- `set_config(key, value)` - Set any config parameter
- `set_max_altitude(altitude_mm)` - Max altitude limit
- `set_max_euler_angle(angle_deg)` - Max tilt angle
- `set_max_vertical_speed(speed_mm_s)` - Max vertical speed
- `set_max_rotation_speed(speed_deg_s)` - Max yaw rate
- `enable_outdoor_mode(enabled)` - Outdoor/indoor mode
- `enable_outdoor_hull(enabled)` - Hull configuration

#### Telemetry
- `get_battery_percentage()` - Battery level (0-100)
- `get_altitude()` - Altitude in mm
- `get_state()` - State flags object
- `is_flying()` - Check if airborne
- `is_emergency()` - Check emergency state
- `get_rotation()` - Pitch/roll/yaw angles
- `get_velocity()` - Velocity vector
- `get_full_telemetry()` - All telemetry data
- `print_status()` - Print formatted status

#### Video
- `get_frame()` - Get latest video frame
- `is_video_ready()` - Check video availability
- `wait_for_video(timeout)` - Wait for video stream
- `display_video(window_name)` - Display in OpenCV window
- `save_frame(filepath)` - Save current frame

#### Utility
- `send_watchdog()` - Maintain connection
- `close()` - Close connection and cleanup

## AT Command Reference

The API uses the following AT commands from the pyardrone library:

- `at.REF` - Reference command (takeoff/land/emergency)
- `at.PCMD` - Progressive command (movement)
- `at.PCMD_MAG` - Movement with magnetometer
- `at.FTRIM` - Flat trim calibration
- `at.CALIB` - Device calibration
- `at.CONFIG` - Configuration settings
- `at.CONFIG_IDS` - Config identification
- `at.COMWDG` - Communication watchdog
- `at.CTRL` - Control modes

## Connection Details

Default connection parameters:
- **Host**: 192.168.1.1 (drone's WiFi IP)
- **Navigation Port**: 5554 (telemetry/navdata)
- **Video Port**: 5555 (video stream)
- **AT Command Port**: 5556 (control commands)

## Safety Notes

1. Always perform `flat_trim()` before first flight
2. Ensure adequate space for flying
3. Use `emergency()` only when necessary (drone will fall)
4. Monitor battery level during flight
5. Test movements at low speeds first (0.3-0.5)
6. Indoor mode recommended for testing
7. Keep spare batteries charged

## Troubleshooting

### Video not available
- Check OpenCV installation: `pip install opencv-python`
- Wait longer for video stream: `drone.wait_for_video(timeout=10.0)`

### Drone not responding
- Verify WiFi connection to drone network
- Check battery level
- Reset emergency state if needed: `drone.reset_emergency()`
- Power cycle the drone

### Drift during hover
- Perform flat trim on level surface: `drone.flat_trim()`
- Enable outdoor mode if flying outside
- Check for wind/air currents

### Connection timeout
- Ensure correct IP address (default: 192.168.1.1)
- Check firewall settings
- Verify drone is powered on

## License

This API wrapper is based on the pyardrone library. Check the original library's license for details.

## Credits

Built on top of [pyardrone](https://github.com/afq984/pyardrone) by afq984.

## Further Reading

- [Official pyardrone Documentation](https://afq984.github.io/pyardrone/)
- [AR.Drone Developer Guide](https://www.parrot.com/assets/s3fs-public/2021-04/ARDrone_SDK_2_0_1.pdf)
- [Parrot AR.Drone 2.0](https://www.parrot.com/en/drones/parrot-ardrone-20-elite-edition)
