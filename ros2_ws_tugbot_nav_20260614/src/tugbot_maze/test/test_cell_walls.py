from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.cell_walls import sense_cell_walls


def _grid(occupied_cells, width=24, height=22, res=1.0, ox=-1.0, oy=-1.0):
    # build a fine occupancy grid; mark given (gx,gy) fine-cells occupied (=100)
    data = [0] * (width * height)
    for gx, gy in occupied_cells:
        data[gy * width + gx] = 100
    return OccupancyGridView(OccupancyGridInfo(width, height, res, ox, oy), data)


def test_open_neighbourhood_reports_all_open():
    # all-free grid around maze-cell (1,0) center (2,0) -> every edge OPEN (False=not wall)
    view = _grid([])
    walls = sense_cell_walls(view, (1, 0))
    assert walls == {'N': False, 'S': False, 'E': False, 'W': False}


def test_wall_on_north_edge_detected():
    # occupy the fine cells along the north edge of maze-cell (1,0): map y≈1.0, x in [1,3]
    # origin (-1,-1), res 1.0 -> map (mx,my) -> fine ( mx+1, my+1 ); north edge y=1 -> gy=2
    occ = [(gx, 2) for gx in range(2, 5)]    # x≈1..3 -> gx 2..4
    view = _grid(occ)
    walls = sense_cell_walls(view, (1, 0))
    assert walls['N'] is True
    assert walls['E'] is False


def test_unknown_edge_returns_none():
    # mark the north-edge fine cells UNKNOWN (-1); cell_walls leaves that edge None
    view = _grid([])
    for gx in range(2, 5):
        view.data[2 * view.info.width + gx] = -1
    walls = sense_cell_walls(view, (1, 0))
    assert walls['N'] is None
