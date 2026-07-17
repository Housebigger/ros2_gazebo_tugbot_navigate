import pytest


def test_node_module_imports():
    pytest.importorskip("rclpy")
    from tugbot_maze import gait_animator
    assert hasattr(gait_animator, "GaitAnimator")
    assert hasattr(gait_animator, "main")
