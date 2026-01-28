"""Simple test runner for when pytest is unavailable."""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from tests.test_controller import *
from tests.test_robot import *
from tests.test_path_planner import *
from tests.test_path_follower import *

def run_tests():
    test_functions = [
        # Controller tests
        test_proportional_only,
        test_integral_accumulation,
        test_derivative_response,
        test_combined_pid,
        test_reset,
        test_zero_dt,
        # Robot tests
        test_robot_initialization,
        test_forward_motion,
        test_rotation,
        test_curved_motion,
        test_velocity_clipping,
        test_heading_normalization,
        # Path planner tests
        test_straight_line_no_obstacles,
        test_path_around_obstacle,
        test_same_start_and_goal,
        test_diagonal_path,
        test_goal_in_obstacle,
        test_complex_obstacle_field,
        # Path follower tests
        test_path_follower_initialization,
        test_set_path,
        test_compute_control_returns_velocities,
        test_is_finished_far_from_goal,
        test_is_finished_at_goal,
        test_heading_error_correction,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in test_functions:
        try:
            test_func()
            print(f"✓ {test_func.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"✗ {test_func.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"✗ {test_func.__name__}: {type(e).__name__}: {e}")
            failed += 1
    
    print(f"\n{passed} passed, {failed} failed")
    return failed == 0

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
