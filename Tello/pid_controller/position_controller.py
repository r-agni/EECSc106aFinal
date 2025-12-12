"""
PID Controller for precise drone position control.

Implements a PID (Proportional-Integral-Derivative) controller for accurate
position tracking and landing. Uses feedback from position estimation to
minimize error and compensate for drift.
"""

import time
from typing import Dict, Tuple


class PIDController:
    """
    Single-axis PID controller.
    
    Implements the classic PID control law:
        u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
    
    Where:
        e(t) = error (target - current)
        u(t) = control output
        Kp, Ki, Kd = tuning gains
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 integral_limit: float = 50.0, deadband: float = 0.0):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            integral_limit: Maximum integral term (anti-windup)
            deadband: Error threshold below which output is zero
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.deadband = deadband
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
    
    def compute(self, error: float, current_time: float = None) -> float:
        """
        Compute PID control output.
        
        Args:
            error: Current error (target - current)
            current_time: Current timestamp (uses time.time() if None)
        
        Returns:
            Control output u(t)
        """
        # Get current time if not provided
        if current_time is None:
            current_time = time.time()
        
        # Calculate time delta
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time
        
        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0
            self.integral = 0.0  # Reset integral in deadband
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        if dt > 0:
            self.integral += error * dt
            # Clamp integral to prevent windup
            self.integral = max(-self.integral_limit, 
                              min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # Update state
        self.previous_error = error
        self.previous_time = current_time
        
        # Compute total output
        output = p_term + i_term + d_term
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None


class PositionController:
    """
    Multi-axis position controller for drone navigation.
    
    Uses separate PID controllers for X, Y, Z position and yaw angle.
    Provides precise landing and position tracking capabilities.
    """
    
    def __init__(self):
        """Initialize position controller with tuned PID gains."""
        # Position controllers (X, Y, Z) - tuned for Tello drone
        self.pid_x = PIDController(
            kp=1.0,      # Proportional gain
            ki=0.1,      # Integral gain
            kd=0.3,      # Derivative gain
            integral_limit=50.0,  # 50cm max integral
            deadband=10.0         # 10cm deadband
        )
        
        self.pid_y = PIDController(
            kp=1.0,
            ki=0.1,
            kd=0.3,
            integral_limit=50.0,
            deadband=10.0
        )
        
        self.pid_z = PIDController(
            kp=1.0,
            ki=0.1,
            kd=0.3,
            integral_limit=50.0,
            deadband=10.0
        )
        
        # Yaw controller - lower gains for rotation
        self.pid_yaw = PIDController(
            kp=0.5,      # Lower proportional gain
            ki=0.05,     # Lower integral gain
            kd=0.1,      # Lower derivative gain
            integral_limit=30.0,  # 30° max integral
            deadband=5.0          # 5° deadband
        )
        
        # Command limits (cm and degrees)
        self.min_command = 20   # Minimum movement (cm or degrees)
        self.max_command = 100  # Maximum movement (cm or degrees)
    
    def compute_correction(self, target: Dict[str, float], 
                          current: Dict[str, float]) -> Dict[str, float]:
        """
        Compute position correction commands.
        
        Args:
            target: Target position {'x': cm, 'y': cm, 'z': cm, 'yaw': degrees}
            current: Current position {'x': cm, 'y': cm, 'z': cm, 'yaw': degrees}
        
        Returns:
            Dictionary of corrections {'x': cm, 'y': cm, 'z': cm, 'yaw': degrees}
        """
        current_time = time.time()
        
        # Calculate errors
        error_x = target.get('x', 0) - current.get('x', 0)
        error_y = target.get('y', 0) - current.get('y', 0)
        error_z = target.get('z', 0) - current.get('z', 0)
        error_yaw = target.get('yaw', 0) - current.get('yaw', 0)
        
        # Normalize yaw error to [-180, 180]
        while error_yaw > 180:
            error_yaw -= 360
        while error_yaw < -180:
            error_yaw += 360
        
        # Compute PID outputs
        correction_x = self.pid_x.compute(error_x, current_time)
        correction_y = self.pid_y.compute(error_y, current_time)
        correction_z = self.pid_z.compute(error_z, current_time)
        correction_yaw = self.pid_yaw.compute(error_yaw, current_time)
        
        # Apply command saturation
        correction_x = self._saturate(correction_x)
        correction_y = self._saturate(correction_y)
        correction_z = self._saturate(correction_z)
        correction_yaw = self._saturate(correction_yaw)
        
        return {
            'x': correction_x,
            'y': correction_y,
            'z': correction_z,
            'yaw': correction_yaw
        }
    
    def _saturate(self, value: float) -> float:
        """
        Saturate command to valid range.
        
        Args:
            value: Raw command value
        
        Returns:
            Saturated value in [min_command, max_command] or 0 if below min
        """
        if abs(value) < self.min_command:
            return 0.0
        
        if value > 0:
            return min(value, self.max_command)
        else:
            return max(value, -self.max_command)
    
    def is_at_target(self, target: Dict[str, float], 
                     current: Dict[str, float],
                     position_tolerance: float = 10.0,
                     yaw_tolerance: float = 5.0) -> bool:
        """
        Check if current position is within tolerance of target.
        
        Args:
            target: Target position
            current: Current position
            position_tolerance: Position tolerance in cm (default 10cm)
            yaw_tolerance: Yaw tolerance in degrees (default 5°)
        
        Returns:
            True if within tolerance, False otherwise
        """
        error_x = abs(target.get('x', 0) - current.get('x', 0))
        error_y = abs(target.get('y', 0) - current.get('y', 0))
        error_z = abs(target.get('z', 0) - current.get('z', 0))
        
        error_yaw = abs(target.get('yaw', 0) - current.get('yaw', 0))
        # Normalize yaw error
        while error_yaw > 180:
            error_yaw -= 360
        error_yaw = abs(error_yaw)
        
        position_ok = (error_x <= position_tolerance and 
                      error_y <= position_tolerance and 
                      error_z <= position_tolerance)
        
        yaw_ok = error_yaw <= yaw_tolerance
        
        return position_ok and yaw_ok
    
    def reset(self):
        """Reset all PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()


def tune_pid_gains(kp: float, ki: float, kd: float) -> Tuple[float, float, float]:
    """
    Helper function for PID tuning using Ziegler-Nichols method.
    
    Ziegler-Nichols tuning rules:
    1. Set Ki = 0, Kd = 0
    2. Increase Kp until system oscillates
    3. Record Kp_critical and oscillation period T
    4. Calculate:
        Kp = 0.6 * Kp_critical
        Ki = 2 * Kp / T
        Kd = Kp * T / 8
    
    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
    
    Returns:
        Tuple of (kp, ki, kd) tuned gains
    """
    # This is a placeholder for manual tuning
    # In practice, you would measure system response and adjust
    return (kp, ki, kd)