"""Unit tests for PathPlanner."""

from planning.path_planner import PathPlanner
from core.waypoint import Waypoint


def test_straight_line_no_obstacles():
    planner = PathPlanner(grid_size=(10, 10), obstacles=[])
    start = Waypoint(0.5, 0.5)
    goal = Waypoint(5.5, 0.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) >= 2
    assert path[0].x == 0.5 and path[0].y == 0.5
    assert path[-1].x == 5.5 and path[-1].y == 0.5


def test_path_around_obstacle():
    obstacles = [(2, 0), (2, 1), (2, 2)]
    planner = PathPlanner(grid_size=(10, 10), obstacles=obstacles)
    
    start = Waypoint(0.5, 1.5)
    goal = Waypoint(4.5, 1.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) >= 2
    for wp in path:
        cell = (int(wp.x), int(wp.y))
        assert cell not in obstacles


def test_same_start_and_goal():
    planner = PathPlanner(grid_size=(10, 10), obstacles=[])
    start = Waypoint(5.5, 5.5)
    goal = Waypoint(5.5, 5.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) == 2


def test_diagonal_path():
    planner = PathPlanner(grid_size=(10, 10), obstacles=[])
    start = Waypoint(0.5, 0.5)
    goal = Waypoint(5.5, 5.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) >= 2
    assert path[0].x == 0.5 and path[0].y == 0.5
    assert path[-1].x == 5.5 and path[-1].y == 5.5


def test_goal_in_obstacle():
    obstacles = [(5, 5)]
    planner = PathPlanner(grid_size=(10, 10), obstacles=obstacles)
    
    start = Waypoint(0.5, 0.5)
    goal = Waypoint(5.5, 5.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) == 2


def test_complex_obstacle_field():
    obstacles = [
        (2, 2), (2, 3), (2, 4),
        (3, 2), (3, 4),
        (4, 2), (4, 3), (4, 4)
    ]
    planner = PathPlanner(grid_size=(10, 10), obstacles=obstacles)
    
    start = Waypoint(1.5, 3.5)
    goal = Waypoint(6.5, 3.5)
    
    path = planner.plan(start, goal)
    
    assert len(path) >= 2
    for wp in path:
        cell = (int(wp.x), int(wp.y))
        assert cell not in obstacles
