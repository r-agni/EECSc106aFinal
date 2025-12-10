"""
PID Controller for AR Drone Navigation

This module implements a standard PID (Proportional-Integral-Derivative)
controller for smooth and accurate drone navigation towards targets.
"""

import time


class PIDController:
    """
    PID controller for single-axis control.

    The controller calculates control output based on:
    - P (Proportional): Immediate response to current error
    - I (Integral): Correction for accumulated past errors
    - D (Derivative): Damping based on rate of error change
    """

    def __init__(self, kp=0.5, ki=0.0, kd=0.1,
                 output_limits=(-1.0, 1.0),
                 integral_limits=None,
                 deadband=0.0):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) output clamping
            integral_limits: (min, max) integral windup prevention
            deadband: Error threshold below which output is 0
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits if integral_limits else output_limits
        self.deadband = deadband

        # State variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.last_output = 0.0

    def update(self, error, dt=None):
        """
        Calculate PID output based on current error.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step (optional, auto-calculated if None)

        Returns:
            float: Control output (clamped to output_limits)
        """
        current_time = time.time()

        # Calculate dt if not provided
        if dt is None:
            dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.01  # Prevent division by zero

        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.error_sum += error * dt
        # Clamp integral
        if self.integral_limits:
            self.error_sum = max(self.integral_limits[0],
                               min(self.error_sum, self.integral_limits[1]))
        i_term = self.ki * self.error_sum

        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt

        # Calculate total output
        output = p_term + i_term + d_term

        # Clamp output to limits
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # Update state
        self.last_error = error
        self.last_time = current_time
        self.last_output = output

        return output

    def reset(self):
        """Reset PID controller state (clear integral, derivative history)."""
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.last_output = 0.0

    def set_gains(self, kp=None, ki=None, kd=None):
        """
        Update PID gains.

        Args:
            kp: New proportional gain (optional)
            ki: New integral gain (optional)
            kd: New derivative gain (optional)
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def get_state(self):
        """
        Get current PID state.

        Returns:
            dict with 'p_term', 'i_term', 'd_term', 'output', 'error'
        """
        p_term = self.kp * self.last_error
        i_term = self.ki * self.error_sum
        d_term = self.kd * (self.last_error - 0) / 0.01  # Approximate

        return {
            'p_term': p_term,
            'i_term': i_term,
            'd_term': d_term,
            'output': self.last_output,
            'error': self.last_error,
            'error_sum': self.error_sum
        }

    def __repr__(self):
        return f"PIDController(kp={self.kp}, ki={self.ki}, kd={self.kd})"


class MultiAxisPID:
    """
    Multi-axis PID controller for X, Y, Z, Yaw control.

    Manages separate PID controllers for each axis.
    """

    def __init__(self, x_gains=(0.3, 0.0, 0.1),
                      y_gains=(0.3, 0.0, 0.1),
                      z_gains=(0.5, 0.0, 0.2),
                      yaw_gains=(0.4, 0.0, 0.15),
                      output_limits=(-0.5, 0.5)):
        """
        Initialize multi-axis PID controller.

        Args:
            x_gains: (kp, ki, kd) for X axis
            y_gains: (kp, ki, kd) for Y axis
            z_gains: (kp, ki, kd) for Z axis
            yaw_gains: (kp, ki, kd) for yaw rotation
            output_limits: (min, max) for all axes
        """
        self.pid_x = PIDController(*x_gains, output_limits=output_limits)
        self.pid_y = PIDController(*y_gains, output_limits=output_limits)
        self.pid_z = PIDController(*z_gains, output_limits=output_limits)
        self.pid_yaw = PIDController(*yaw_gains, output_limits=output_limits)

    def update(self, error_x=0.0, error_y=0.0, error_z=0.0, error_yaw=0.0):
        """
        Update all PID controllers.

        Args:
            error_x: X-axis error
            error_y: Y-axis error
            error_z: Z-axis error
            error_yaw: Yaw error

        Returns:
            dict with 'x', 'y', 'z', 'yaw' control outputs
        """
        return {
            'x': self.pid_x.update(error_x),
            'y': self.pid_y.update(error_y),
            'z': self.pid_z.update(error_z),
            'yaw': self.pid_yaw.update(error_yaw)
        }

    def reset(self):
        """Reset all PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    def get_state(self):
        """Get state of all PID controllers."""
        return {
            'x': self.pid_x.get_state(),
            'y': self.pid_y.get_state(),
            'z': self.pid_z.get_state(),
            'yaw': self.pid_yaw.get_state()
        }
