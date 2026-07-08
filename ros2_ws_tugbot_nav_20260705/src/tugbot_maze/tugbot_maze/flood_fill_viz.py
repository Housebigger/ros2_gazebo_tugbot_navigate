"""RViz visualization for the online self-built map: turn confirmed wall segments into a
MarkerArray so RViz shows the map the robot actually built (perimeter + confirmed interior
walls), rather than slam_toolbox's occupancy grid. Pure builder — no ROS node state."""
from __future__ import annotations
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

Segment = Tuple[float, float, float, float]


def self_built_wall_markerarray(segments: Iterable[Segment], frame_id: str = 'map',
                                stamp=None, ns: str = 'self_built_walls') -> MarkerArray:
    """A single green LINE_LIST marker with two points per wall segment, in `frame_id`."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
    m.pose.orientation.w = 1.0
    for x0, y0, x1, y1 in segments:
        m.points.append(Point(x=float(x0), y=float(y0), z=0.0))
        m.points.append(Point(x=float(x1), y=float(y1), z=0.0))
    return MarkerArray(markers=[m])
