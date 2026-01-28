"""Unit tests for Robot kinematics."""

import numpy as np
from core.robot import Robot, Pose


def approx(value, expected, abs_tol=1e-6):
    """Helper for approximate equality."""
    return abs(value - expected) < abs_tol


def test_robot_initialization():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    assert robot.pose.x == 0.0
    assert robot.pose.y == 0.0
    assert robot.pose.theta == 0.0
    assert robot.velocity == 0.0


def test_forward_motion():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    robot.update(linear_velocity=1.0, angular_velocity=0.0, dt=1.0)
    
    assert approx(robot.pose.x, 1.0)
    assert approx(robot.pose.y, 0.0)
    assert approx(robot.pose.theta, 0.0)


def test_rotation():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    robot.update(linear_velocity=0.0, angular_velocity=np.pi/2, dt=1.0)
    
    assert approx(robot.pose.x, 0.0)
    assert approx(robot.pose.y, 0.0)
    assert approx(robot.pose.theta, np.pi/2)


def test_curved_motion():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    robot.update(linear_velocity=1.0, angular_velocity=np.pi/2, dt=1.0)
    
    assert robot.pose.x > 0.0
    assert approx(robot.pose.theta, np.pi/2)


def test_velocity_clipping():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    robot.update(linear_velocity=5.0, angular_velocity=0.0, dt=0.1)
    
    assert robot.velocity == 1.0


def test_heading_normalization():
    pose = Pose(x=0.0, y=0.0, theta=0.0)
    robot = Robot(pose, max_velocity=1.0)
    
    robot.update(linear_velocity=0.0, angular_velocity=3*np.pi, dt=1.0)
    
    assert -np.pi <= robot.pose.theta <= np.pi
