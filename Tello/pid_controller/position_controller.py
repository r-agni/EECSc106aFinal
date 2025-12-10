"""
3D Position Controller using PID for precise navigation.

Coordinates multiple PID controllers (X, Y, Z, Yaw) to achieve accurate
position control with self-correction.
"""

import time
import math
from typing import Dict, Tuple, Optional
from .pid import PIDController


class PositionController:
    """
    Multi-axis PID controller for precise 3D positioning.

    Uses separate PID controllers for X, Y, Z, and yaw (rotation).
    Automatically corrects position errors to land precisely at target.
    """

    def __init__(self, position_estimator, drone_controller, state_manager):
        """
        Initialize position controller.

        Args:
            position_estimator: PositionEstimator instance
            drone_controller: DroneController instance
            state_manager: StateManager instance
        """
        self.position_est = position_estimator
        self.drone = drone_controller
        self.state = state_manager

        # PID Controllers for each axis
        # Tuned for Tello (adjust gains based on testing)
        self.pid_x = PIDController(
            kp=0.8,  # Strong response to position error
            ki=0.05, # Slow correction for drift
            kd=0.3,  # Damping to prevent overshoot
            output_limits=(-100, 100),  # Max 100cm per step
            integral_limit=30.0
        )

        self.pid_y = PIDController(
            kp=0.8,
            ki=0.05,
            kd=0.3,
            output_limits=(-100, 100),
            integral_limit=30.0
        )

        self.pid_z = PIDController(
            kp=0.6,  # Gentler for altitude (more sensitive)
            ki=0.03,
            kd=0.2,
            output_limits=(-60, 60),  # Smaller range for vertical
            integral_limit=20.0
        )

        self.pid_yaw = PIDController(
            kp=1.2,  # Aggressive rotation correction
            ki=0.0,  # No integral for rotation (yaw drift is minimal)
            kd=0.4,
            output_limits=(-90, 90),  # Max 90° rotation per step
            integral_limit=0.0
        )

        # Control parameters
        self.min_movement = 20  # cm - minimum command to send (Tello limitation)
        self.position_tolerance = 10  # cm - "good enough" for landing
        self.yaw_tolerance = 5  # degrees
        self.max_iterations = 50  # Prevent infinite loops
        self.stabilization_time = 0.5  # seconds between commands

    def set_target(self, x: float, y: float, z: float, yaw: float = 0):
        """
        Set target position for all axes.

        Args:
            x: Target X position in cm
            y: Target Y position in cm
            z: Target Z position in cm
            yaw: Target orientation in degrees
        """
        self.pid_x.set_setpoint(x)
        self.pid_y.set_setpoint(y)
        self.pid_z.set_setpoint(z)
        self.pid_yaw.set_setpoint(yaw)

        print(f"[PID] Target set: ({x:.1f}, {y:.1f}, {z:.1f}) cm, yaw: {yaw:.1f}°")

    def reset(self):
        """Reset all PID controllers (call before new target)."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    def update_yaw_from_imu(self):
        """Update position estimator yaw from drone IMU (more accurate)."""
        state = self.state.get_state()
        imu_yaw = state.get("orientation", {}).get("yaw", 0)
        self.position_est.update_yaw(imu_yaw)

    def get_current_position(self) -> Dict[str, float]:
        """Get current position from estimator."""
        pos = self.position_est.get_position()
        return {
            "x": pos["x"],
            "y": pos["y"],
            "z": pos["z"],
            "yaw": self.position_est.yaw
        }

    def calculate_control_outputs(self) -> Dict[str, float]:
        """
        Calculate PID outputs for all axes.

        Returns:
            Dictionary with control outputs for each axis
        """
        current = self.get_current_position()

        outputs = {
            "x": self.pid_x.update(current["x"]),
            "y": self.pid_y.update(current["y"]),
            "z": self.pid_z.update(current["z"]),
            "yaw": self.pid_yaw.update(current["yaw"])
        }

        return outputs

    def at_target(self) -> bool:
        """
        Check if drone is at target position (within tolerances).

        Returns:
            True if at target, False otherwise
        """
        current = self.get_current_position()

        x_ok = self.pid_x.at_setpoint(current["x"], self.position_tolerance)
        y_ok = self.pid_y.at_setpoint(current["y"], self.position_tolerance)
        z_ok = self.pid_z.at_setpoint(current["z"], self.position_tolerance)
        yaw_ok = self.pid_yaw.at_setpoint(current["yaw"], self.yaw_tolerance)

        return x_ok and y_ok and z_ok and yaw_ok

    def get_position_error(self) -> Dict[str, float]:
        """Get error for each axis."""
        current = self.get_current_position()
        return {
            "x": self.pid_x.get_error(current["x"]),
            "y": self.pid_y.get_error(current["y"]),
            "z": self.pid_z.get_error(current["z"]),
            "yaw": self.pid_yaw.get_error(current["yaw"])
        }

    def execute_control_step(self, outputs: Dict[str, float]) -> bool:
        """
        Convert PID outputs to drone commands and execute.

        Prioritizes axes with largest errors. Handles coordinate transformation
        for yaw-compensated movement.

        Args:
            outputs: Control outputs from calculate_control_outputs()

        Returns:
            True if command executed, False if no significant movement needed
        """
        current = self.get_current_position()

        # Priority 1: Fix yaw first if significantly off
        if abs(outputs["yaw"]) > self.yaw_tolerance:
            rotation = max(min(int(outputs["yaw"]), 90), -90)
            if abs(rotation) >= 15:  # Only rotate if meaningful
                if rotation > 0:
                    self.drone.send_command("rotate_ccw", degrees=abs(rotation))
                    self.position_est.update_from_command("rotate_ccw", {"degrees": abs(rotation)})
                else:
                    self.drone.send_command("rotate_cw", degrees=abs(rotation))
                    self.position_est.update_from_command("rotate_cw", {"degrees": abs(rotation)})
                print(f"[PID] Yaw correction: {rotation:.1f}°")
                time.sleep(self.stabilization_time)
                return True

        # Priority 2: Fix altitude (safer to be at correct height first)
        if abs(outputs["z"]) > self.position_tolerance:
            z_movement = max(min(int(outputs["z"]), 60), -60)
            if abs(z_movement) >= self.min_movement:
                if z_movement > 0:
                    self.drone.send_command("move_up", distance=abs(z_movement))
                    self.position_est.update_from_command("move_up", {"distance": abs(z_movement)})
                else:
                    self.drone.send_command("move_down", distance=abs(z_movement))
                    self.position_est.update_from_command("move_down", {"distance": abs(z_movement)})
                print(f"[PID] Z correction: {z_movement:.1f}cm")
                time.sleep(self.stabilization_time)
                return True

        # Priority 3: Fix horizontal position (X and Y)
        # Transform to drone's local coordinate frame (account for yaw)
        yaw_rad = math.radians(current["yaw"])

        # Convert global X,Y errors to drone's local forward/sideways
        # Global X,Y -> Local forward/right using inverse rotation
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        forward_error = outputs["x"] * cos_yaw + outputs["y"] * sin_yaw
        right_error = -outputs["x"] * sin_yaw + outputs["y"] * cos_yaw

        # Execute largest horizontal correction
        max_horizontal = max(abs(forward_error), abs(right_error))

        if max_horizontal > self.position_tolerance:
            if abs(forward_error) > abs(right_error):
                # Move forward/backward
                movement = max(min(int(forward_error), 100), -100)
                if abs(movement) >= self.min_movement:
                    if movement > 0:
                        self.drone.send_command("move_forward", distance=abs(movement))
                        self.position_est.update_from_command("move_forward", {"distance": abs(movement)})
                    else:
                        self.drone.send_command("move_back", distance=abs(movement))
                        self.position_est.update_from_command("move_back", {"distance": abs(movement)})
                    print(f"[PID] X correction: {movement:.1f}cm (forward)")
                    time.sleep(self.stabilization_time)
                    return True
            else:
                # Move left/right
                movement = max(min(int(right_error), 100), -100)
                if abs(movement) >= self.min_movement:
                    if movement > 0:
                        self.drone.send_command("move_right", distance=abs(movement))
                        self.position_est.update_from_command("move_right", {"distance": abs(movement)})
                    else:
                        self.drone.send_command("move_left", distance=abs(movement))
                        self.position_est.update_from_command("move_left", {"distance": abs(movement)})
                    print(f"[PID] Y correction: {movement:.1f}cm (sideways)")
                    time.sleep(self.stabilization_time)
                    return True

        return False  # No significant movement needed

    def move_to_target(self, x: float, y: float, z: float, yaw: float = 0,
                       verbose: bool = True) -> bool:
        """
        Main control loop - move to target position with PID control.

        Args:
            x: Target X position in cm
            y: Target Y position in cm
            z: Target Z position in cm
            yaw: Target orientation in degrees
            verbose: Print detailed progress

        Returns:
            True if reached target, False if failed
        """
        # Set target and reset controllers
        self.set_target(x, y, z, yaw)
        self.reset()

        iteration = 0
        print(f"\n[PID] Starting position control to ({x:.1f}, {y:.1f}, {z:.1f}), yaw: {yaw:.1f}°")

        while iteration < self.max_iterations:
            iteration += 1

            # Update yaw from IMU (more accurate than dead reckoning)
            self.update_yaw_from_imu()

            # Check if at target
            if self.at_target():
                current = self.get_current_position()
                print(f"\n[PID] ✓ Target reached in {iteration} iterations")
                print(f"      Final position: ({current['x']:.1f}, {current['y']:.1f}, {current['z']:.1f})")
                print(f"      Final yaw: {current['yaw']:.1f}°")
                return True

            # Calculate control outputs
            outputs = self.calculate_control_outputs()
            errors = self.get_position_error()

            if verbose:
                current = self.get_current_position()
                print(f"\n[PID] Iteration {iteration}/{self.max_iterations}")
                print(f"      Current: ({current['x']:.1f}, {current['y']:.1f}, {current['z']:.1f}), yaw: {current['yaw']:.1f}°")
                print(f"      Error: X:{errors['x']:.1f} Y:{errors['y']:.1f} Z:{errors['z']:.1f} Yaw:{errors['yaw']:.1f}°")
                print(f"      Output: X:{outputs['x']:.1f} Y:{outputs['y']:.1f} Z:{outputs['z']:.1f} Yaw:{outputs['yaw']:.1f}°")

            # Execute control step
            moved = self.execute_control_step(outputs)

            if not moved:
                # No significant movement, but not at target - might be stuck
                if iteration > 10:  # Give it a few tries first
                    print(f"\n[PID] ! Position error too small to correct (<{self.min_movement}cm)")
                    print(f"      Settling at current position")
                    return True  # Close enough

            time.sleep(0.1)  # Small delay between iterations

        # Max iterations reached
        current = self.get_current_position()
        errors = self.get_position_error()
        print(f"\n[PID] ✗ Max iterations reached ({self.max_iterations})")
        print(f"      Final position: ({current['x']:.1f}, {current['y']:.1f}, {current['z']:.1f})")
        print(f"      Remaining error: X:{errors['x']:.1f} Y:{errors['y']:.1f} Z:{errors['z']:.1f}")

        # Still consider it success if close enough
        total_error = math.sqrt(errors['x']**2 + errors['y']**2 + errors['z']**2)
        if total_error < self.position_tolerance * 2:  # Within 2x tolerance
            print(f"      Total error {total_error:.1f}cm - acceptable")
            return True

        return False

    def tune_gains(self, axis: str, kp: float, ki: float, kd: float):
        """
        Adjust PID gains for tuning.

        Args:
            axis: 'x', 'y', 'z', or 'yaw'
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        if axis == 'x':
            self.pid_x.set_gains(kp, ki, kd)
        elif axis == 'y':
            self.pid_y.set_gains(kp, ki, kd)
        elif axis == 'z':
            self.pid_z.set_gains(kp, ki, kd)
        elif axis == 'yaw':
            self.pid_yaw.set_gains(kp, ki, kd)
        else:
            raise ValueError(f"Invalid axis: {axis}")

        print(f"[PID] {axis.upper()} gains updated: Kp={kp}, Ki={ki}, Kd={kd}")
