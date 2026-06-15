from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.cell_walls import sense_cell_walls


def _grid(occupied=(), unknown=(), width=24, height=22, res=1.0, ox=-1.0, oy=-1.0):
    """Fine occupancy grid. res 1.0, origin (-1,-1) -> world (mx,my) maps to fine cell
    (floor(mx+1), floor(my+1)). 0=free, 100=occupied, -1=unknown."""
    data = [0] * (width * height)
    for gx, gy in occupied:
        data[gy * width + gx] = 100
    for gx, gy in unknown:
        data[gy * width + gx] = -1
    return OccupancyGridView(OccupancyGridInfo(width, height, res, ox, oy), data)


def test_open_neighbourhood_reports_all_open():
    # all-free grid -> every center->neighbor connector is clear -> all edges OPEN (False).
    walls = sense_cell_walls(_grid(), (3, 3))
    assert walls == {'N': False, 'S': False, 'E': False, 'W': False}


def test_wall_on_north_connector_detected():
    # occupy the fine cell at the (3,3)->N connector midpoint: world (6,7) -> fine (7,8).
    walls = sense_cell_walls(_grid(occupied=[(7, 8)]), (3, 3))
    assert walls['N'] is True
    assert walls['E'] is False


def test_any_occupied_sample_on_connector_marks_wall():
    # ANY occupied cell on the connector => WALL (a real wall blocks the whole traversal).
    # E connector midpoint of (3,3): world (7,6) -> fine (8,7).
    walls = sense_cell_walls(_grid(occupied=[(8, 7)]), (3, 3))
    assert walls['E'] is True


def test_unmapped_connector_returns_none():
    # the whole (3,3)->N connector is unknown -> None (re-sense later), not a false OPEN.
    walls = sense_cell_walls(_grid(unknown=[(7, 7), (7, 8), (7, 9)]), (3, 3))
    assert walls['N'] is None
