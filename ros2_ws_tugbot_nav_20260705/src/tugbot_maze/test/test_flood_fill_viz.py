from visualization_msgs.msg import Marker
from tugbot_maze.flood_fill_viz import self_built_wall_markerarray


def test_markerarray_is_line_list_two_points_per_segment():
    segs = [(0.0, 0.0, 2.0, 0.0), (2.0, 0.0, 2.0, 2.0)]
    arr = self_built_wall_markerarray(segs, frame_id='map')
    assert len(arr.markers) == 1
    m = arr.markers[0]
    assert m.type == Marker.LINE_LIST
    assert m.action == Marker.ADD
    assert m.header.frame_id == 'map'
    assert len(m.points) == 4                      # 2 endpoints * 2 segments
    assert (m.points[0].x, m.points[0].y) == (0.0, 0.0)
    assert (m.points[1].x, m.points[1].y) == (2.0, 0.0)
    assert m.scale.x > 0.0 and m.color.a > 0.0
    assert m.pose.orientation.w == 1.0             # valid quaternion (else RViz rejects the marker)


def test_markerarray_empty_segments_is_valid():
    arr = self_built_wall_markerarray([], frame_id='map')
    assert len(arr.markers) == 1
    assert arr.markers[0].points == []


from tugbot_maze.flood_fill_viz import self_built_occupancy_grid


# Synthetic 8x8m world: 4 perimeter centerlines + one sensed cell (1,1)
# (interior [1,3]x[1,3]m) + one interior wall on that cell's north edge (y=3).
_PERIM = [(0.0, 0.0, 8.0, 0.0), (8.0, 0.0, 8.0, 8.0),
          (8.0, 8.0, 0.0, 8.0), (0.0, 8.0, 0.0, 0.0)]
_WALLS = [(1.0, 3.0, 3.0, 3.0)]


def _cell_at(g, x, y):
    """Occupancy value of the grid cell containing map point (x, y)."""
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def test_grid_metadata_from_perimeter_bbox():
    g = self_built_occupancy_grid({(1, 1)}, _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5)
    assert g.header.frame_id == 'map'
    assert g.info.resolution == 0.1
    # bbox [0,8]^2 + 0.5 margin -> origin (-0.5,-0.5), 9x9 m -> 90x90 cells
    assert (g.info.width, g.info.height) == (90, 90)
    assert (g.info.origin.position.x, g.info.origin.position.y) == (-0.5, -0.5)
    assert g.info.origin.orientation.w == 1.0
    assert len(g.data) == 90 * 90


def test_grid_three_value_semantics():
    g = self_built_occupancy_grid({(1, 1)}, _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5)
    assert _cell_at(g, 2.05, 2.05) == 0     # sensed cell interior = free
    assert _cell_at(g, 6.05, 6.05) == -1    # never-sensed area = unknown
    assert _cell_at(g, 4.05, 0.05) == 100   # perimeter wall band = occupied
    assert _cell_at(g, 2.05, 3.05) == 100   # interior wall OVERWRITES free
    assert _cell_at(g, 2.05, 2.55) == 0     # inside cell, off the wall band = free


def test_grid_wall_band_width():
    g = self_built_occupancy_grid(set(), _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5,
                                  wall_half_thickness_m=0.12)
    # 0.35m above the wall centerline is outside the 0.12m band -> unknown
    assert _cell_at(g, 2.05, 3.35) == -1
    assert _cell_at(g, 2.05, 3.05) == 100
