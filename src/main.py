"""Main entry point for PID Waypoint Navigator."""

import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.robot import Robot, Pose
from core.waypoint import Waypoint, WaypointQueue
from planning.path_planner import PathPlanner
from control.path_follower import PathFollower
from simulation.simulator import Simulator
from simulation.visualizer import Visualizer


def main():
    """Run the PID waypoint navigation simulation."""
    
    # Configuration
    grid_size = (10, 10)
    obstacles = [
        (2, 2), (2, 3), (3, 2), (3, 3),
        (6, 5), (6, 6), (7, 5), (7, 6)
    ]
    
    # Define waypoints (square pattern)
    waypoints = [
        Waypoint(0.5, 0.5),
        Waypoint(8.5, 0.5),
        Waypoint(8.5, 8.5),
        Waypoint(0.5, 8.5),
        Waypoint(0.5, 0.5)
    ]
    
    # Initialize components
    robot = Robot(Pose(x=0.5, y=0.5, theta=0.0), max_velocity=1.0)
    planner = PathPlanner(grid_size, obstacles)
    follower = PathFollower(
        heading_gains=(2.0, 0.0, 0.1),
        velocity_gains=(1.0, 0.0, 0.0),
        lookahead_distance=0.3
    )
    waypoint_queue = WaypointQueue(waypoints)
    
    # Plan initial path
    start, goal = waypoint_queue.get_next_segment()
    if start and goal:
        initial_path = planner.plan(start, goal)
        follower.set_path(initial_path)
    
    # Create simulator
    simulator = Simulator(robot, planner, follower, waypoint_queue, dt=0.1)
    
    # Run simulation
    print("Starting simulation...")
    print(f"Grid size: {grid_size}")
    print(f"Obstacles: {len(obstacles)}")
    print(f"Waypoints: {len(waypoints)}")
    print(f"PID Gains - Heading: Kp={follower.heading_controller.kp}, "
          f"Ki={follower.heading_controller.ki}, Kd={follower.heading_controller.kd}")
    print(f"PID Gains - Velocity: Kp={follower.velocity_controller.kp}, "
          f"Ki={follower.velocity_controller.ki}, Kd={follower.velocity_controller.kd}")
    print()
    
    trajectory = simulator.run(max_time=100.0)
    
    print(f"Simulation complete!")
    print(f"Time: {simulator.time:.2f}s")
    print(f"Steps: {len(trajectory)}")
    print(f"Final position: ({robot.pose.x:.2f}, {robot.pose.y:.2f})")
    print(f"All waypoints reached: {waypoint_queue.is_complete()}")
    print()

    # Visualize results
    print("Generating visualization...")
    visualizer = Visualizer(grid_size, obstacles)
    visualizer.draw_waypoints(waypoints, waypoint_queue.current_index)
    visualizer.update_robot(robot, trajectory)
    visualizer.save('simulation_result.png')

    # Create animation video
    print("\nCreating animation video...")
    visualizer_anim = Visualizer(grid_size, obstacles)
    visualizer_anim.create_animation(trajectory, waypoints,
                                     filename='simulation.gif',
                                     fps=10, skip_frames=5)

    # Show interactive plot
    visualizer.show_static(robot, waypoints, trajectory, waypoint_queue.current_index)


if __name__ == "__main__":
    main()
