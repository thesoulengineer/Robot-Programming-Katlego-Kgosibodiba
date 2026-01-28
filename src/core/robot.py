"""Robot kinematics and state management."""

from dataclasses import dataclass
import numpy as np


@dataclass
class Pose:
    """Robot pose in 2D space."""
    x: float
    y: float
    theta: float


class Robot:
    """Manages robot state and differential-drive kinematics."""
    
    def __init__(self, initial_pose: Pose, max_velocity: float = 1.0):
        self.pose = initial_pose
        self.velocity = 0.0
        self.max_velocity = max_velocity
    
    def update(self, linear_velocity: float, angular_velocity: float, dt: float) -> None:
        """Update robot pose based on control inputs."""
        self.velocity = np.clip(linear_velocity, 0, self.max_velocity)
        
        self.pose.x += self.velocity * np.cos(self.pose.theta) * dt
        self.pose.y += self.velocity * np.sin(self.pose.theta) * dt
        self.pose.theta += angular_velocity * dt
        
        self.pose.theta = np.arctan2(np.sin(self.pose.theta), 
                                     np.cos(self.pose.theta))
