"""
RC-only command wrapper for Tello drone.
Uses only RC (remote control) commands to avoid IMU dependency.
"""

import time
from djitellopy import Tello


class TelloSafeCommands:
    """RC-only commands for Tello - no IMU required"""

    # RC control speeds (range: -100 to 100)
    RC_SPEED = 20  # Slow speed for accurate movements

    @staticmethod
    def move_forward(tello: Tello, distance: int) -> None:
        """Move forward using RC control."""
        duration = distance / 20.0  # ~20cm/s at speed 20
        tello.send_rc_control(0, TelloSafeCommands.RC_SPEED, 0, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)  # Stop

    @staticmethod
    def move_back(tello: Tello, distance: int) -> None:
        """Move backward using RC control."""
        duration = distance / 20.0
        tello.send_rc_control(0, -TelloSafeCommands.RC_SPEED, 0, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def move_left(tello: Tello, distance: int) -> None:
        """Move left using RC control."""
        duration = distance / 20.0
        tello.send_rc_control(-TelloSafeCommands.RC_SPEED, 0, 0, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def move_right(tello: Tello, distance: int) -> None:
        """Move right using RC control."""
        duration = distance / 20.0
        tello.send_rc_control(TelloSafeCommands.RC_SPEED, 0, 0, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def move_up(tello: Tello, distance: int) -> None:
        """Move up using RC control."""
        duration = distance / 20.0
        tello.send_rc_control(0, 0, TelloSafeCommands.RC_SPEED, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def move_down(tello: Tello, distance: int) -> None:
        """Move down using RC control."""
        duration = distance / 20.0
        tello.send_rc_control(0, 0, -TelloSafeCommands.RC_SPEED, 0)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def rotate_clockwise(tello: Tello, degrees: int) -> None:
        """Rotate clockwise using RC control."""
        duration = degrees / 60.0  # ~60°/s at speed 20
        tello.send_rc_control(0, 0, 0, TelloSafeCommands.RC_SPEED)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)

    @staticmethod
    def rotate_counter_clockwise(tello: Tello, degrees: int) -> None:
        """Rotate counter-clockwise using RC control."""
        duration = degrees / 60.0
        tello.send_rc_control(0, 0, 0, -TelloSafeCommands.RC_SPEED)
        time.sleep(duration)
        tello.send_rc_control(0, 0, 0, 0)
