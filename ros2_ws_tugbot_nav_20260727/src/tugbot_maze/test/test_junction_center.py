"""Unit tests for compute_junction_center Chebyshev center algorithm."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.maze_perception import OpenDirection, compute_junction_center


def _od(angle_rad: float, distance_m: float) -> OpenDirection:
    """Helper to create an OpenDirection with auto-computed target."""
    return OpenDirection(
        angle_rad=angle_rad,
        target_xy=(math.cos(angle_rad) * distance_m, math.sin(angle_rad) * distance_m),
        distance_m=distance_m,
    )


def test_t_junction_robot_in_arm_shifts_toward_intersection():
    """Robot is 0.5m east of a T-junction center (in one arm of the T).
    Three directions: east (the arm continuing), north, and south.
    The center should shift west (toward the junction center at x=0)."""
    robot_xy = (0.5, 0.0)
    # East (arm), North, South — the T-junction has no west arm from the center
    open_dirs = [
        _od(0.0, 1.5),              # east (arm continuing)
        _od(math.pi / 2, 2.0),      # north
        _od(-math.pi / 2, 2.0),     # south
    ]
    center = compute_junction_center(robot_xy, open_dirs)
    # Center should shift west (toward x=0) from robot at x=0.5
    assert center[0] < robot_xy[0], f"Expected x < {robot_xy[0]}, got {center[0]}"
    # And should be close to the geometric intersection center
    assert abs(center[0]) < 0.3, f"Expected x near 0, got {center[0]}"
    assert abs(center[1]) < 0.2, f"Expected y near 0, got {center[1]}"


def test_symmetric_cross_junction_robot_at_center_unchanged():
    """Robot already at the center of a symmetric cross-junction.
    The computed center should be very close to the robot position."""
    robot_xy = (5.0, 5.0)
    open_dirs = [
        _od(0.0, 1.5),          # right
        _od(math.pi, 1.5),      # left
        _od(math.pi / 2, 1.5),  # up
        _od(-math.pi / 2, 1.5), # down
    ]
    center = compute_junction_center(robot_xy, open_dirs)
    offset = math.hypot(center[0] - robot_xy[0], center[1] - robot_xy[1])
    assert offset < 0.1, f"Expected offset < 0.1m, got {offset:.3f}m"


def test_fewer_than_three_directions_returns_robot_position():
    """With 0, 1, or 2 open directions, should return robot position unchanged."""
    robot_xy = (3.0, 4.0)
    # 0 directions
    assert compute_junction_center(robot_xy, []) == robot_xy
    # 1 direction
    assert compute_junction_center(robot_xy, [_od(0.0, 1.0)]) == robot_xy
    # 2 directions
    assert compute_junction_center(robot_xy, [_od(0.0, 1.0), _od(math.pi, 1.0)]) == robot_xy


def test_asymmetric_junction_shifts_toward_wider_opening():
    """Junction where one direction has much more clearance than others.
    The center should shift toward the wider opening."""
    robot_xy = (0.0, 0.0)
    # Three directions: up (short), right (medium), left (very long)
    open_dirs = [
        _od(math.pi / 2, 0.6),  # up (narrow)
        _od(0.0, 1.0),           # right (medium)
        _od(math.pi, 3.0),       # left (wide)
    ]
    center = compute_junction_center(robot_xy, open_dirs)
    # Center should shift left (toward the wider opening)
    assert center[0] < robot_xy[0], f"Expected x < {robot_xy[0]}, got {center[0]}"


def test_result_within_search_radius():
    """Computed center should always be within search_radius_m of robot."""
    robot_xy = (10.0, 20.0)
    open_dirs = [
        _od(0.0, 2.0),
        _od(2 * math.pi / 3, 2.0),
        _od(4 * math.pi / 3, 2.0),
    ]
    for radius in [0.5, 1.0]:
        center = compute_junction_center(robot_xy, open_dirs, search_radius_m=radius)
        offset = math.hypot(center[0] - robot_xy[0], center[1] - robot_xy[1])
        assert offset <= radius + 0.01, (
            f"Expected offset <= {radius:.2f}m, got {offset:.3f}m"
        )


def test_negative_clearance_candidates_skipped():
    """Candidates that would place us past a wall (negative clearance) are skipped.
    All directions short → center should not shift far from robot."""
    robot_xy = (0.0, 0.0)
    # All directions are short (0.5m), so the center can't shift far
    open_dirs = [
        _od(0.0, 0.5),
        _od(2 * math.pi / 3, 0.5),
        _od(4 * math.pi / 3, 0.5),
    ]
    center = compute_junction_center(robot_xy, open_dirs)
    offset = math.hypot(center[0] - robot_xy[0], center[1] - robot_xy[1])
    # With only 0.5m clearance in each direction, shift should be small
    assert offset < 0.3, f"Expected small offset with short directions, got {offset:.3f}m"
