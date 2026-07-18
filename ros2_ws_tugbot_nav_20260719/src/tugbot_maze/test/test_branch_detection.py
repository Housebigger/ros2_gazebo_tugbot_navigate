import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import (
    DEAD_END,
    JUNCTION,
    CORRIDOR,
    classify_local_topology,
    make_branch_goal,
    make_centered_branch_goal,
    filter_open_directions,
)


def make_view(width, height, data, resolution=0.5, origin_x=0.0, origin_y=0.0):
    return OccupancyGridView(
        info=OccupancyGridInfo(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        ),
        data=list(data),
    )


def empty_grid(width, height):
    return [100] * (width * height)


def set_free(data, width, cells):
    for x, y in cells:
        data[x + y * width] = 0


def test_t_junction_detects_three_open_directions():
    width = height = 9
    data = empty_grid(width, height)
    set_free(data, width, [(4, y) for y in range(1, 8)])
    set_free(data, width, [(x, 4) for x in range(4, 8)])
    view = make_view(width, height, data)
    pose = view.cell_to_world(4, 4) + (math.pi / 2.0,)

    result = classify_local_topology(view, pose, angle_step_deg=90, lookahead_m=1.5, clearance_radius_m=0.0)

    assert result.kind == JUNCTION
    assert len(result.open_directions) == 3


def test_dead_end_detects_single_open_backtrack_direction():
    width = height = 9
    data = empty_grid(width, height)
    set_free(data, width, [(4, y) for y in range(1, 5)])
    view = make_view(width, height, data)
    pose = view.cell_to_world(4, 4) + (math.pi / 2.0,)

    result = classify_local_topology(view, pose, angle_step_deg=90, lookahead_m=1.5, clearance_radius_m=0.0)

    assert result.kind == DEAD_END
    assert len(result.open_directions) == 1


def test_straight_corridor_is_not_a_junction():
    width = height = 9
    data = empty_grid(width, height)
    set_free(data, width, [(4, y) for y in range(1, 8)])
    view = make_view(width, height, data)
    pose = view.cell_to_world(4, 4) + (math.pi / 2.0,)

    result = classify_local_topology(view, pose, angle_step_deg=90, lookahead_m=1.5, clearance_radius_m=0.0)

    assert result.kind == CORRIDOR
    assert len(result.open_directions) == 2


def test_make_branch_goal_uses_farthest_safe_point_up_to_preferred_step():
    width = height = 9
    data = empty_grid(width, height)
    set_free(data, width, [(x, 4) for x in range(1, 7)])
    view = make_view(width, height, data)
    pose_xy = view.cell_to_world(2, 4)

    goal = make_branch_goal(
        view,
        pose_xy=pose_xy,
        direction_rad=0.0,
        preferred_step_m=2.0,
        min_step_m=0.5,
        clearance_radius_m=0.0,
    )

    assert goal is not None
    assert goal[0] > pose_xy[0]
    assert goal[0] <= view.cell_to_world(6, 4)[0]
    assert abs(goal[1] - pose_xy[1]) < 1e-6


def test_centered_branch_goal_moves_goal_to_corridor_centerline():
    width = height = 11
    data = empty_grid(width, height)
    set_free(data, width, [(x, y) for x in range(1, 10) for y in range(3, 8)])
    view = make_view(width, height, data, resolution=0.5)
    pose_xy = view.cell_to_world(2, 6)
    corridor_center_y = view.cell_to_world(2, 5)[1]

    raw_goal = make_branch_goal(
        view,
        pose_xy=pose_xy,
        direction_rad=0.0,
        preferred_step_m=2.0,
        min_step_m=0.5,
        clearance_radius_m=0.0,
    )
    centered_goal = make_centered_branch_goal(
        view,
        pose_xy=pose_xy,
        direction_rad=0.0,
        preferred_step_m=2.0,
        min_step_m=0.5,
        clearance_radius_m=0.0,
        lateral_search_m=2.0,
    )

    assert raw_goal is not None
    assert centered_goal is not None
    assert raw_goal[1] > corridor_center_y
    assert abs(centered_goal[1] - corridor_center_y) < 1e-6
    assert centered_goal[0] >= raw_goal[0]


def test_filter_open_directions_can_suppress_reverse_motion():
    width = height = 9
    data = empty_grid(width, height)
    set_free(data, width, [(x, 4) for x in range(1, 8)])
    view = make_view(width, height, data)
    pose = view.cell_to_world(4, 4) + (0.0,)
    local = classify_local_topology(view, pose, angle_step_deg=90, lookahead_m=1.5, clearance_radius_m=0.0)

    filtered = filter_open_directions(local.open_directions, robot_yaw=0.0, allow_reverse=False, reverse_angle_threshold_deg=135.0)

    assert len(local.open_directions) == 2
    assert len(filtered) == 1
    assert abs(filtered[0].angle_rad) < 1e-6
