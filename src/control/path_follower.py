"""Path following using PID control."""

from typing import List, Tuple
import numpy as np
from core.controller import PIDController
from core.robot import Robot
from core.waypoint import Waypoint


class PathFollower:
    """Tracks a planned path using dual PID controllers."""
    
    def __init__(self, heading_gains: Tuple[float, float, float],
                 velocity_gains: Tuple[float, float, float],
                 lookahead_distance: float = 0.3):
        self.heading_controller = PIDController(*heading_gains)
        self.velocity_controller = PIDController(*velocity_gains)
        self.lookahead_distance = lookahead_distance
        
        self.path: List[Waypoint] = []
        self.current_target_index = 0
    
    def set_path(self, path: List[Waypoint]) -> None:
        """Load new path to follow."""
        self.path = path
        self.current_target_index = 0
        self.heading_controller.reset()
        self.velocity_controller.reset()
    
    def compute_control(self, robot: Robot, dt: float) -> Tuple[float, float]:
        """Compute linear and angular velocities for robot."""
        if not self.path or self.current_target_index >= len(self.path):
            return 0.0, 0.0
        
        target = self.path[self.current_target_index]
        
        dx = target.x - robot.pose.x
        dy = target.y - robot.pose.y
        distance = np.sqrt(dx * dx + dy * dy)
        
        if distance < self.lookahead_distance and self.current_target_index < len(self.path) - 1:
            self.current_target_index += 1
            target = self.path[self.current_target_index]
            dx = target.x - robot.pose.x
            dy = target.y - robot.pose.y
            distance = np.sqrt(dx * dx + dy * dy)
        
        desired_heading = np.arctan2(dy, dx)
        heading_error = np.arctan2(np.sin(desired_heading - robot.pose.theta),
                                   np.cos(desired_heading - robot.pose.theta))
        
        angular_velocity = self.heading_controller.compute(heading_error, dt)
        
        desired_velocity = robot.max_velocity
        velocity_error = desired_velocity - robot.velocity
        linear_velocity = self.velocity_controller.compute(velocity_error, dt)
        
        return linear_velocity, angular_velocity
    
    def is_finished(self, robot: Robot, tolerance: float = 0.1) -> bool:
        """Check if robot has reached final waypoint."""
        if not self.path:
            return True
        
        final = self.path[-1]
        dx = final.x - robot.pose.x
        dy = final.y - robot.pose.y
        
        return np.sqrt(dx * dx + dy * dy) < tolerance
