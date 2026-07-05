import pytest


def test_node_module_imports():
    pytest.importorskip("rclpy")
    from tugbot_maze import flood_fill_solver
    assert hasattr(flood_fill_solver, "FloodFillSolver")
    assert hasattr(flood_fill_solver, "main")
