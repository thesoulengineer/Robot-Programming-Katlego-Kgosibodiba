"""Main simulation orchestrator."""

from typing import List
from core.robot import Robot, Pose
from core.waypoint import Waypoint, WaypointQueue
from planning.path_planner import PathPlanner
from control.path_follower import PathFollower


class Simulator:
    """Orchestrates robot navigation simulation."""
    
    def __init__(self, robot: Robot, planner: PathPlanner, 
                 follower: PathFollower, waypoint_queue: WaypointQueue,
                 dt: float = 0.1):
        self.robot = robot
        self.planner = planner
        self.follower = follower
        self.waypoint_queue = waypoint_queue
        self.dt = dt
        self.time = 0.0
        self.trajectory: List[Pose] = []
    
    def step(self) -> None:
        """Execute one simulation step."""
        linear_vel, angular_vel = self.follower.compute_control(self.robot, self.dt)
        self.robot.update(linear_vel, angular_vel, self.dt)
        
        self.trajectory.append(Pose(self.robot.pose.x, 
                                   self.robot.pose.y, 
                                   self.robot.pose.theta))
        
        if self.follower.is_finished(self.robot):
            self.waypoint_queue.advance()
            
            start, goal = self.waypoint_queue.get_next_segment()
            if start and goal:
                path = self.planner.plan(start, goal)
                self.follower.set_path(path)
        
        self.time += self.dt
    
    def run(self, max_time: float = 100.0, max_steps: int = 10000) -> List[Pose]:
        """Run simulation until completion or timeout."""
        steps = 0
        while self.time < max_time and not self.waypoint_queue.is_complete() and steps < max_steps:
            self.step()
            steps += 1
        
        return self.trajectory
    
    def is_running(self) -> bool:
        """Check if simulation should continue."""
        return not self.waypoint_queue.is_complete()
