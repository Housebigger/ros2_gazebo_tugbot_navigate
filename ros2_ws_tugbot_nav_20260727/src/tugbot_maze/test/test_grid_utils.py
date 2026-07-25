import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView


def make_view(width, height, data, resolution=0.5, origin_x=-1.0, origin_y=-1.0):
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


def test_world_to_cell_and_cell_to_world_are_inverse_at_cell_centers():
    view = make_view(width=4, height=4, data=[0] * 16, resolution=0.5, origin_x=-1.0, origin_y=-1.0)

    cell = view.world_to_cell(-0.25, 0.25)
    xy = view.cell_to_world(*cell)

    assert cell == (1, 2)
    assert xy == (-0.25, 0.25)


def test_cell_value_classifies_free_occupied_unknown_and_out_of_bounds():
    view = make_view(width=3, height=2, data=[0, 100, -1, 20, 80, 0])

    assert view.is_free((0, 0))
    assert view.is_occupied((1, 0))
    assert view.is_unknown((2, 0))
    assert not view.in_bounds((-1, 0))
    assert view.is_occupied((-1, 0))


def test_clearance_rejects_cells_near_obstacles():
    data = [0] * 49
    data[3 + 3 * 7] = 100
    view = make_view(width=7, height=7, data=data, resolution=0.5, origin_x=0.0, origin_y=0.0)

    assert not view.has_clearance((3, 3), radius_m=0.5)
    assert not view.has_clearance((3, 4), radius_m=0.5)
    assert view.has_clearance((1, 1), radius_m=0.5)


def test_clearance_treats_unknown_as_unsafe_by_default():
    data = [0] * 25
    data[2 + 2 * 5] = -1
    view = make_view(width=5, height=5, data=data, resolution=0.5, origin_x=0.0, origin_y=0.0)

    assert not view.has_clearance((1, 2), radius_m=0.5)
    assert view.has_clearance((1, 2), radius_m=0.5, unknown_is_safe=True)
