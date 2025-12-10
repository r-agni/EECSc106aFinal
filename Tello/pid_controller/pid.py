"""
PID Controller implementation for single-axis control.

Classic PID (Proportional-Integral-Derivative) controller for smooth,
accurate position control with self-correction.
"""

import time
from typing import Optional


class PIDController:
    """
    PID controller for single axis (x, y, or z).

    Formula: output = Kp*error + Ki*integral + Kd*derivative

    - Kp (Proportional): Responds to current error
    - Ki (Integral): Corrects accumulated past errors
    - Kd (Derivative): Dampens oscillations, predicts future error
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limits: tuple = (-100, 100),
                 integral_limit: float = 50.0):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) output limits in cm
            integral_limit: Maximum integral windup (prevents overshoot)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits
        self.integral_limit = integral_limit

        # State variables
        self.setpoint = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains (for tuning)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_setpoint(self, setpoint: float):
        """Set target value."""
        self.setpoint = setpoint

    def reset(self):
        """Reset controller state (call when starting new movement)."""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, current_value: float, dt: Optional[float] = None) -> float:
        """
        Calculate control output based on current position.

        Args:
            current_value: Current position/value
            dt: Time delta in seconds (if None, auto-calculated)

        Returns:
            Control output (movement command in cm)
        """
        # Calculate time delta
        current_time = time.time()
        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # Avoid division by zero
        if dt == 0.0:
            dt = 0.001

        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # Calculate total output
        output = p_term + i_term + d_term

        # Clamp output to limits
        output = max(min(output, self.output_max), self.output_min)

        # Save state for next iteration
        self.last_error = error

        return output

    def at_setpoint(self, current_value: float, tolerance: float = 5.0) -> bool:
        """
        Check if current value is within tolerance of setpoint.

        Args:
            current_value: Current position
            tolerance: Acceptable error margin in same units as setpoint

        Returns:
            True if within tolerance
        """
        error = abs(self.setpoint - current_value)
        return error < tolerance

    def get_error(self, current_value: float) -> float:
        """Get current error (distance from setpoint)."""
        return self.setpoint - current_value

    def __repr__(self):
        return (f"PIDController(Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
                f"setpoint={self.setpoint})")
