"""Unit tests for PathFollower."""

import numpy as np
from control.path_follower import PathFollower
from core.robot import Robot, Pose
from core.waypoint import Waypoint


def test_path_follower_initialization():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    assert follower.current_target_index == 0
    assert len(follower.path) == 0


def test_set_path():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    path = [Waypoint(0, 0), Waypoint(1, 1), Waypoint(2, 2)]
    follower.set_path(path)
    
    assert len(follower.path) == 3
    assert follower.current_target_index == 0


def test_compute_control_returns_velocities():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    path = [Waypoint(0, 0), Waypoint(5, 0)]
    follower.set_path(path)
    
    robot = Robot(Pose(0.0, 0.0, 0.0), max_velocity=1.0)
    linear_vel, angular_vel = follower.compute_control(robot, dt=0.1)
    
    assert isinstance(linear_vel, float)
    assert isinstance(angular_vel, float)


def test_is_finished_far_from_goal():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    path = [Waypoint(0, 0), Waypoint(10, 10)]
    follower.set_path(path)
    
    robot = Robot(Pose(0.0, 0.0, 0.0), max_velocity=1.0)
    
    assert not follower.is_finished(robot, tolerance=0.1)


def test_is_finished_at_goal():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    path = [Waypoint(0, 0), Waypoint(1, 0)]
    follower.set_path(path)
    
    robot = Robot(Pose(1.05, 0.0, 0.0), max_velocity=1.0)
    
    assert follower.is_finished(robot, tolerance=0.1)


def test_heading_error_correction():
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0)
    )
    
    path = [Waypoint(0, 0), Waypoint(5, 5)]
    follower.set_path(path)
    
    robot = Robot(Pose(0.0, 0.0, 0.0), max_velocity=1.0)
    linear_vel, angular_vel = follower.compute_control(robot, dt=0.1)
    
    assert angular_vel != 0.0
