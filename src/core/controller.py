"""PID controller implementation."""


class PIDController:
    """Generic PID controller for error correction."""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0
    
    def compute(self, error: float, dt: float) -> float:
        """Compute control output from error signal."""
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        
        return output
    
    def reset(self) -> None:
        """Reset controller state for new task."""
        self.integral = 0.0
        self.previous_error = 0.0
