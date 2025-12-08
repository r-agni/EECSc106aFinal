"""
Comprehensive AR Drone API Wrapper
Based on pyardrone library documentation and AT command protocol

This module provides a complete API for controlling the Parrot AR.Drone 2.0,
including all flight commands, configuration, telemetry, and video streaming.
"""

import time
import cv2
from pyardrone import ARDrone, at
from typing import Optional, Dict, Any, Callable


class DroneAPI:
    """
    Complete AR Drone control API with all available commands and functions.
    """

    def __init__(self, host: str = '192.168.1.1',
                 nav_port: int = 5554,
                 video_port: int = 5555,
                 at_port: int = 5556):
        """
        Initialize connection to AR Drone.

        Args:
            host: Drone IP address (default: 192.168.1.1)
            nav_port: Navigation data port (default: 5554)
            video_port: Video stream port (default: 5555)
            at_port: AT command port (default: 5556)
        """
        print(f"[API] Connecting to AR.Drone at {host}...")
        self.drone = ARDrone(host=host)
        self.host = host
        self.nav_port = nav_port
        self.video_port = video_port
        self.at_port = at_port

        # Wait for navdata to be ready
        print("[API] Waiting for navigation data...")
        self.drone.navdata_ready.wait(timeout=10.0)

        if self.drone.navdata_ready.is_set():
            print("[API] Drone connected and ready!")
        else:
            print("[WARN] NavData not ready - some features may not work")

    # ==================== FLIGHT CONTROL COMMANDS ====================

    def takeoff(self) -> None:
        """
        Command the drone to take off.
        The drone will ascend to hover position automatically.
        """
        print("[FLIGHT] Taking off...")
        self.drone.takeoff()

    def land(self) -> None:
        """
        Command the drone to land.
        The drone will descend and land at current position.
        """
        print("[FLIGHT] Landing...")
        self.drone.land()

    def hover(self) -> None:
        """
        Command the drone to hover in place.
        Stops all movement and maintains current position.
        """
        self.drone.hover()

    def emergency(self) -> None:
        """
        Emergency stop - immediately cuts motors.
        WARNING: Drone will fall from current height!
        """
        print("[EMERGENCY] Emergency stop activated!")
        self.drone.emergency()

    def reset_emergency(self) -> None:
        """
        Reset emergency state to resume normal operation.
        """
        print("[RESET] Resetting emergency state...")
        # Send emergency toggle to reset
        self.drone.emergency()
        time.sleep(0.1)

    # ==================== MOVEMENT COMMANDS ====================

    def move(self, forward: float = 0.0, backward: float = 0.0,
             left: float = 0.0, right: float = 0.0,
             up: float = 0.0, down: float = 0.0,
             cw: float = 0.0, ccw: float = 0.0) -> None:
        """
        Send movement command to drone.

        Args:
            forward: Move forward (0.0 to 1.0)
            backward: Move backward (0.0 to 1.0)
            left: Move left (0.0 to 1.0)
            right: Move right (0.0 to 1.0)
            up: Ascend (0.0 to 1.0)
            down: Descend (0.0 to 1.0)
            cw: Rotate clockwise (0.0 to 1.0)
            ccw: Rotate counter-clockwise (0.0 to 1.0)
        """
        self.drone.move(
            forward=forward, backward=backward,
            left=left, right=right,
            up=up, down=down,
            cw=cw, ccw=ccw
        )

    def move_forward(self, speed: float = 0.5) -> None:
        """Move forward at specified speed (0.0 to 1.0)"""
        self.drone.move(forward=speed)

    def move_backward(self, speed: float = 0.5) -> None:
        """Move backward at specified speed (0.0 to 1.0)"""
        self.drone.move(backward=speed)

    def move_left(self, speed: float = 0.5) -> None:
        """Move left at specified speed (0.0 to 1.0)"""
        self.drone.move(left=speed)

    def move_right(self, speed: float = 0.5) -> None:
        """Move right at specified speed (0.0 to 1.0)"""
        self.drone.move(right=speed)

    def move_up(self, speed: float = 0.5) -> None:
        """Ascend at specified speed (0.0 to 1.0)"""
        self.drone.move(up=speed)

    def move_down(self, speed: float = 0.5) -> None:
        """Descend at specified speed (0.0 to 1.0)"""
        self.drone.move(down=speed)

    def rotate_clockwise(self, speed: float = 0.5) -> None:
        """Rotate clockwise at specified speed (0.0 to 1.0)"""
        self.drone.move(cw=speed)

    def rotate_counterclockwise(self, speed: float = 0.5) -> None:
        """Rotate counter-clockwise at specified speed (0.0 to 1.0)"""
        self.drone.move(ccw=speed)

    # ==================== ADVANCED MOVEMENT ====================

    def fly_direct(self, roll: float = 0.0, pitch: float = 0.0,
                   yaw: float = 0.0, gaz: float = 0.0) -> None:
        """
        Direct low-level flight control using AT PCMD command.

        Args:
            roll: Roll angle (-1.0 to 1.0, negative=left, positive=right)
            pitch: Pitch angle (-1.0 to 1.0, negative=backward, positive=forward)
            yaw: Yaw rate (-1.0 to 1.0, negative=ccw, positive=cw)
            gaz: Vertical speed (-1.0 to 1.0, negative=down, positive=up)
        """
        # Use AT command directly for precise control
        self.drone.send(at.PCMD(
            flag=1,  # 1 = progressive mode
            roll=roll,
            pitch=pitch,
            gaz=gaz,
            yaw=yaw
        ))

    # ==================== CALIBRATION COMMANDS ====================

    def flat_trim(self) -> None:
        """
        Calibrate the drone's horizontal plane (flat trim).
        Should be called when drone is on a flat surface.
        """
        print("[CALIBRATION] Performing flat trim...")
        self.drone.send(at.FTRIM())

    def calibrate_magnetometer(self, device_num: int = 0) -> None:
        """
        Calibrate the magnetometer.

        Args:
            device_num: Magnetometer device number (usually 0)
        """
        print("[CALIBRATION] Calibrating magnetometer...")
        self.drone.send(at.CALIB(device_num=device_num))

    # ==================== CONFIGURATION COMMANDS ====================

    def set_config(self, key: str, value: Any) -> None:
        """
        Set a configuration parameter on the drone.

        Args:
            key: Configuration key (e.g., 'control:altitude_max')
            value: Configuration value
        """
        print(f"[CONFIG] Setting {key} = {value}")
        self.drone.send(at.CONFIG(key=key, value=value))

    def set_max_altitude(self, altitude_mm: int) -> None:
        """
        Set maximum altitude in millimeters.

        Args:
            altitude_mm: Maximum altitude in millimeters (default: 3000)
        """
        self.set_config('control:altitude_max', altitude_mm)

    def set_max_euler_angle(self, angle_deg: float) -> None:
        """
        Set maximum tilt angle in degrees.

        Args:
            angle_deg: Maximum tilt angle (0.0 to 52.0 degrees)
        """
        # Convert to radians * 1000 as expected by drone
        angle_rad = angle_deg * 0.017453292519943295  # deg to rad
        self.set_config('control:euler_angle_max', angle_rad)

    def set_max_vertical_speed(self, speed_mm_s: int) -> None:
        """
        Set maximum vertical speed in mm/s.

        Args:
            speed_mm_s: Maximum vertical speed (default: 700 mm/s)
        """
        self.set_config('control:control_vz_max', speed_mm_s)

    def set_max_rotation_speed(self, speed_deg_s: float) -> None:
        """
        Set maximum yaw rotation speed in degrees/second.

        Args:
            speed_deg_s: Maximum rotation speed (default: 200 deg/s)
        """
        # Convert to radians per second
        speed_rad_s = speed_deg_s * 0.017453292519943295
        self.set_config('control:control_yaw', speed_rad_s)

    def enable_outdoor_mode(self, enabled: bool = True) -> None:
        """
        Enable or disable outdoor flight mode.
        Outdoor mode uses GPS/magnetometer for better stability.

        Args:
            enabled: True for outdoor mode, False for indoor mode
        """
        self.set_config('control:outdoor', 'TRUE' if enabled else 'FALSE')

    def enable_outdoor_hull(self, enabled: bool = False) -> None:
        """
        Configure for outdoor hull (removed for outdoor flight).

        Args:
            enabled: True if outdoor hull is attached
        """
        self.set_config('control:flight_without_shell', 'FALSE' if enabled else 'TRUE')

    # ==================== TELEMETRY & STATUS ====================

    def get_battery_percentage(self) -> Optional[int]:
        """
        Get current battery percentage.

        Returns:
            Battery percentage (0-100) or None if not available
        """
        try:
            if hasattr(self.drone, 'navdata') and hasattr(self.drone.navdata, 'demo'):
                return self.drone.navdata.demo.vbat_flying_percentage
        except (AttributeError, Exception):
            pass
        return None

    def get_altitude(self) -> Optional[int]:
        """
        Get current altitude in millimeters.

        Returns:
            Altitude in mm or None if not available
        """
        try:
            if hasattr(self.drone, 'navdata') and hasattr(self.drone.navdata, 'demo'):
                return self.drone.navdata.demo.altitude
        except (AttributeError, Exception):
            pass
        return None

    def get_state(self) -> Optional[Any]:
        """
        Get current drone state flags.

        Returns:
            State object with flags like fly_mask, emergency_mask, etc.
        """
        try:
            return getattr(self.drone, 'state', None)
        except (AttributeError, Exception):
            return None

    def is_flying(self) -> bool:
        """
        Check if drone is currently flying.

        Returns:
            True if drone is airborne, False otherwise
        """
        try:
            state = self.get_state()
            return state.fly_mask if state and hasattr(state, 'fly_mask') else False
        except (AttributeError, Exception):
            return False

    def is_emergency(self) -> bool:
        """
        Check if drone is in emergency state.

        Returns:
            True if emergency state is active
        """
        state = self.get_state()
        return state.emergency_mask if state and hasattr(state, 'emergency_mask') else False

    def get_rotation(self) -> Optional[Dict[str, float]]:
        """
        Get current rotation angles (pitch, roll, yaw).

        Returns:
            Dictionary with 'pitch', 'roll', 'yaw' in degrees or None
        """
        try:
            if hasattr(self.drone, 'navdata') and hasattr(self.drone.navdata, 'demo'):
                demo = self.drone.navdata.demo
                return {
                    'pitch': demo.theta / 1000.0,  # Convert millidegrees to degrees
                    'roll': demo.phi / 1000.0,
                    'yaw': demo.psi / 1000.0
                }
        except (AttributeError, Exception):
            pass
        return None

    def get_velocity(self) -> Optional[Dict[str, float]]:
        """
        Get current velocity in mm/s.

        Returns:
            Dictionary with 'vx', 'vy', 'vz' or None
        """
        try:
            if hasattr(self.drone, 'navdata') and hasattr(self.drone.navdata, 'demo'):
                demo = self.drone.navdata.demo
                return {
                    'vx': demo.vx,
                    'vy': demo.vy,
                    'vz': demo.vz
                }
        except (AttributeError, Exception):
            pass
        return None

    def get_full_telemetry(self) -> Dict[str, Any]:
        """
        Get all available telemetry data.

        Returns:
            Dictionary containing all telemetry information
        """
        telemetry = {
            'battery': self.get_battery_percentage(),
            'altitude': self.get_altitude(),
            'is_flying': self.is_flying(),
            'is_emergency': self.is_emergency(),
            'rotation': self.get_rotation(),
            'velocity': self.get_velocity(),
            'timestamp': time.time()
        }
        return telemetry

    # ==================== VIDEO STREAMING ====================

    def get_frame(self) -> Optional[Any]:
        """
        Get the latest video frame from drone camera.

        Returns:
            OpenCV BGR image frame or None if not available
        """
        return getattr(self.drone, 'frame', None)

    def is_video_ready(self) -> bool:
        """
        Check if video stream is ready.

        Returns:
            True if video is available
        """
        video_ready = getattr(self.drone, 'video_ready', None)
        return video_ready.is_set() if video_ready else False

    def wait_for_video(self, timeout: float = 5.0) -> bool:
        """
        Wait for video stream to become ready.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if video became ready, False if timeout
        """
        start = time.time()
        while not self.is_video_ready():
            if time.time() - start > timeout:
                return False
            time.sleep(0.05)
        return True

    def display_video(self, window_name: str = "AR.Drone Video") -> int:
        """
        Display current video frame in OpenCV window.

        Args:
            window_name: Name of the display window

        Returns:
            Key code pressed (or -1 if no key), use 27 for ESC
        """
        frame = self.get_frame()
        if frame is not None:
            cv2.imshow(window_name, frame)
            return cv2.waitKey(1) & 0xFF
        return -1

    def save_frame(self, filepath: str) -> bool:
        """
        Save current video frame to file.

        Args:
            filepath: Path to save the image

        Returns:
            True if successful, False otherwise
        """
        frame = self.get_frame()
        if frame is not None:
            return cv2.imwrite(filepath, frame)
        return False

    def switch_camera(self, camera_id: int) -> None:
        """
        Switch between front and bottom cameras.

        Args:
            camera_id: 0 = front camera, 1 = bottom camera,
                      2 = front camera small, 3 = bottom camera small
        """
        print(f"[API] Switching to camera {camera_id}...")
        # ZAP channel for video - configure video codec
        self.set_config('video:video_channel', camera_id)
        time.sleep(0.5)  # Give time for camera switch

    def set_front_camera(self) -> None:
        """Switch to front-facing camera."""
        print("[API] Switching to front camera...")
        self.switch_camera(0)

    def set_bottom_camera(self) -> None:
        """Switch to bottom-facing camera."""
        print("[API] Switching to bottom camera...")
        self.switch_camera(1)

    # ==================== UTILITY FUNCTIONS ====================

    def send_watchdog(self) -> None:
        """
        Send communication watchdog command to maintain connection.
        Should be called periodically to prevent timeout.
        """
        self.drone.send(at.COMWDG())

    def print_status(self) -> None:
        """Print current drone status to console."""
        telemetry = self.get_full_telemetry()
        print("\n" + "="*50)
        print("DRONE STATUS")
        print("="*50)
        print(f"Battery:     {telemetry['battery']}%")
        print(f"Altitude:    {telemetry['altitude']} mm")
        print(f"Flying:      {telemetry['is_flying']}")
        print(f"Emergency:   {telemetry['is_emergency']}")

        if telemetry['rotation']:
            print(f"Rotation:    Pitch={telemetry['rotation']['pitch']:.2f}° "
                  f"Roll={telemetry['rotation']['roll']:.2f}° "
                  f"Yaw={telemetry['rotation']['yaw']:.2f}°")

        if telemetry['velocity']:
            print(f"Velocity:    X={telemetry['velocity']['vx']:.1f} "
                  f"Y={telemetry['velocity']['vy']:.1f} "
                  f"Z={telemetry['velocity']['vz']:.1f} mm/s")
        print("="*50 + "\n")

    def execute_maneuver(self, commands: list, delay: float = 0.5) -> None:
        """
        Execute a sequence of commands with delays.

        Args:
            commands: List of (function, args, kwargs) tuples
            delay: Delay between commands in seconds
        """
        for cmd in commands:
            func = cmd[0]
            args = cmd[1] if len(cmd) > 1 else []
            kwargs = cmd[2] if len(cmd) > 2 else {}

            func(*args, **kwargs)
            time.sleep(delay)

    def close(self) -> None:
        """Close connection and cleanup resources."""
        print("[API] Closing drone connection...")
        cv2.destroyAllWindows()
        # pyardrone handles cleanup automatically
        print("[API] Connection closed.")

    # Context manager support
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
