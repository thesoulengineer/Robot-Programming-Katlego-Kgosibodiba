"""Visualization using matplotlib."""

from typing import List, Tuple
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
from core.robot import Robot, Pose
from core.waypoint import Waypoint


class Visualizer:
    """Real-time visualization of robot navigation."""

    def __init__(self, grid_size: Tuple[int, int], obstacles: List[Tuple[int, int]]):
        self.grid_size = grid_size
        self.obstacles = obstacles

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()

        self.trajectory_line = None
        self.robot_marker = None
        self.heading_arrow = None

    def setup_plot(self) -> None:
        """Initialize plot with grid and obstacles."""
        self.ax.set_xlim(0, self.grid_size[0])
        self.ax.set_ylim(0, self.grid_size[1])
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('PID Waypoint Navigator')

        for obs in self.obstacles:
            rect = patches.Rectangle((obs[0], obs[1]), 1, 1,
                                     linewidth=1, edgecolor='black',
                                     facecolor='red', alpha=0.7)
            self.ax.add_patch(rect)

    def draw_waypoints(self, waypoints: List[Waypoint], current_index: int = 0) -> None:
        """Draw waypoints and planned path."""
        if len(waypoints) < 2:
            return

        for i, wp in enumerate(waypoints):
            if i == current_index:
                self.ax.plot(wp.x, wp.y, 'go', markersize=12, label='Current Target' if i == current_index else '')
            else:
                self.ax.plot(wp.x, wp.y, 'ko', markersize=8)
            self.ax.text(wp.x + 0.2, wp.y + 0.2, str(i), fontsize=10)

        path_x = [wp.x for wp in waypoints]
        path_y = [wp.y for wp in waypoints]
        self.ax.plot(path_x, path_y, 'b--', alpha=0.5, linewidth=2, label='Planned Path')

    def draw_path(self, path: List[Waypoint]) -> None:
        """Draw A* planned path."""
        if len(path) < 2:
            return

        path_x = [wp.x for wp in path]
        path_y = [wp.y for wp in path]
        self.ax.plot(path_x, path_y, 'c-', alpha=0.4, linewidth=1.5)

    def update_robot(self, robot: Robot, trajectory: List[Pose]) -> None:
        """Update robot position and trajectory."""
        if self.trajectory_line:
            self.trajectory_line.remove()
        if self.robot_marker:
            self.robot_marker.remove()
        if self.heading_arrow:
            self.heading_arrow.remove()

        # Draw trajectory
        if len(trajectory) > 1:
            traj_x = [p.x for p in trajectory]
            traj_y = [p.y for p in trajectory]
            self.trajectory_line, = self.ax.plot(traj_x, traj_y, 'purple',
                                                 alpha=0.6, linewidth=2,
                                                 label='Trajectory')

        # Draw robot body as a rectangle with wheels
        robot_width = 0.3
        robot_height = 0.4

        # Robot body (main rectangle)
        rect_x = robot.pose.x - robot_width / 2
        rect_y = robot.pose.y - robot_height / 2

        # Create rotated rectangle for robot body
        rect = patches.Rectangle((rect_x, rect_y), robot_width, robot_height,
                                 linewidth=2, edgecolor='black',
                                 facecolor='darkblue', alpha=0.8)

        # Apply rotation
        transform = plt.matplotlib.transforms.Affine2D().rotate_around(
            robot.pose.x, robot.pose.y, robot.pose.theta) + self.ax.transData
        rect.set_transform(transform)

        self.robot_marker = rect
        self.ax.add_patch(self.robot_marker)

        # Draw wheels (small rectangles on sides)
        wheel_width = 0.08
        wheel_height = 0.15

        # Left wheel
        left_wheel_x = robot.pose.x - robot_width / 2 - wheel_width / 2
        left_wheel_y = robot.pose.y
        left_wheel = patches.Rectangle((left_wheel_x, left_wheel_y - wheel_height / 2),
                                       wheel_width, wheel_height,
                                       linewidth=1, edgecolor='black',
                                       facecolor='gray')
        left_wheel.set_transform(transform)
        self.ax.add_patch(left_wheel)

        # Right wheel
        right_wheel_x = robot.pose.x + robot_width / 2 - wheel_width / 2
        right_wheel_y = robot.pose.y
        right_wheel = patches.Rectangle((right_wheel_x, right_wheel_y - wheel_height / 2),
                                        wheel_width, wheel_height,
                                        linewidth=1, edgecolor='black',
                                        facecolor='gray')
        right_wheel.set_transform(transform)
        self.ax.add_patch(right_wheel)

        # Direction indicator (small circle at front)
        front_x = robot.pose.x + (robot_height / 2) * np.cos(robot.pose.theta)
        front_y = robot.pose.y + (robot_height / 2) * np.sin(robot.pose.theta)
        self.heading_arrow = plt.Circle((front_x, front_y), 0.08,
                                        color='orange', zorder=5)
        self.ax.add_patch(self.heading_arrow)

    def show_static(self, robot: Robot, waypoints: List[Waypoint],
                    trajectory: List[Pose], current_waypoint: int = 0) -> None:
        """Show final static plot."""
        self.draw_waypoints(waypoints, current_waypoint)
        self.update_robot(robot, trajectory)
        self.ax.legend(loc='upper right')
        plt.tight_layout()
        plt.show()

    def create_animation(self, trajectory: List[Pose], waypoints: List[Waypoint],
                         filename: str = 'simulation.gif', fps: int = 10,
                         skip_frames: int = 5) -> None:
        """Create animated video of the simulation.

        Args:
            trajectory: Full trajectory of robot poses
            waypoints: List of waypoints
            filename: Output video filename
            fps: Frames per second for video
            skip_frames: Use every Nth frame (for faster video)
        """
        print(f"Creating animation with {len(trajectory)} frames...")

        # Use subset of frames for reasonable video length
        frames = trajectory[::skip_frames]

        # Store patch objects to remove them later
        robot_patches = []

        def animate(frame_idx):
            # Remove previous robot patches
            for patch in robot_patches:
                patch.remove()
            robot_patches.clear()

            # Redraw obstacles
            for obs in self.obstacles:
                rect = patches.Rectangle((obs[0], obs[1]), 1, 1,
                                         linewidth=1, edgecolor='black',
                                         facecolor='red', alpha=0.7)
                self.ax.add_patch(rect)
                robot_patches.append(rect)

            # Draw waypoints
            for i, wp in enumerate(waypoints):
                point, = self.ax.plot(wp.x, wp.y, 'ko', markersize=8)
                robot_patches.append(point)
                text = self.ax.text(wp.x + 0.2, wp.y + 0.2, str(i), fontsize=10)
                robot_patches.append(text)

            # Draw trajectory up to current frame
            current_trajectory = trajectory[:frame_idx * skip_frames + 1]
            if len(current_trajectory) > 1:
                traj_x = [p.x for p in current_trajectory]
                traj_y = [p.y for p in current_trajectory]
                line, = self.ax.plot(traj_x, traj_y, 'purple', alpha=0.6, linewidth=2)
                robot_patches.append(line)

            # Draw robot at current position
            if frame_idx < len(frames):
                current_pose = frames[frame_idx]

                # Robot body
                robot_width = 0.3
                robot_height = 0.4
                rect_x = current_pose.x - robot_width / 2
                rect_y = current_pose.y - robot_height / 2

                rect = patches.Rectangle((rect_x, rect_y), robot_width, robot_height,
                                         linewidth=2, edgecolor='black',
                                         facecolor='darkblue', alpha=0.8)

                transform = plt.matplotlib.transforms.Affine2D().rotate_around(
                    current_pose.x, current_pose.y, current_pose.theta) + self.ax.transData
                rect.set_transform(transform)
                self.ax.add_patch(rect)
                robot_patches.append(rect)

                # Left wheel
                wheel_width = 0.08
                wheel_height = 0.15
                left_wheel_x = current_pose.x - robot_width / 2 - wheel_width / 2
                left_wheel_y = current_pose.y
                left_wheel = patches.Rectangle((left_wheel_x, left_wheel_y - wheel_height / 2),
                                               wheel_width, wheel_height,
                                               linewidth=1, edgecolor='black',
                                               facecolor='gray')
                left_wheel.set_transform(transform)
                self.ax.add_patch(left_wheel)
                robot_patches.append(left_wheel)

                # Right wheel
                right_wheel_x = current_pose.x + robot_width / 2 - wheel_width / 2
                right_wheel_y = current_pose.y
                right_wheel = patches.Rectangle((right_wheel_x, right_wheel_y - wheel_height / 2),
                                                wheel_width, wheel_height,
                                                linewidth=1, edgecolor='black',
                                                facecolor='gray')
                right_wheel.set_transform(transform)
                self.ax.add_patch(right_wheel)
                robot_patches.append(right_wheel)

                # Direction indicator
                front_x = current_pose.x + (robot_height / 2) * np.cos(current_pose.theta)
                front_y = current_pose.y + (robot_height / 2) * np.sin(current_pose.theta)
                front_circle = plt.Circle((front_x, front_y), 0.08, color='orange', zorder=5)
                self.ax.add_patch(front_circle)
                robot_patches.append(front_circle)

            return robot_patches

        anim = animation.FuncAnimation(self.fig, animate, frames=len(frames),
                                       interval=1000 / fps, blit=True, repeat=False)

        # Save animation
        print(f"Saving animation to {filename}...")
        try:
            Writer = animation.writers['pillow']
            writer = Writer(fps=fps, metadata=dict(artist='PID Navigator'))
            anim.save(filename, writer=writer)
            print(f"Animation saved successfully to {filename}")
        except Exception as e:
            print(f"Error saving animation: {e}")
            print("Trying alternative method...")
            anim.save(filename, fps=fps)
            print(f"Animation saved to {filename}")

    def save(self, filename: str) -> None:
        """Save plot to file."""
        self.ax.legend(loc='upper right')
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {filename}")