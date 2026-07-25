import pytest


def test_node_module_imports():
    # Skip cleanly where ROS is not sourced (pure CI); colcon test provides rclpy.
    pytest.importorskip("rclpy")
    from tugbot_maze import wall_follow_solver
    assert hasattr(wall_follow_solver, "WallFollowSolver")
    assert hasattr(wall_follow_solver, "main")
