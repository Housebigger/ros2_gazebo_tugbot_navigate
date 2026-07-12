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
