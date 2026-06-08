#!/usr/bin/env python3
"""Phase67 Goal 1 timeout RViz MarkerArray overlay.

Visual-only marker publisher for Phase67 Goal 1 Timeout Visual Replay /
Terminal Pose Diagnosis.

Guardrails:
- human visual replay / diagnosis only
- no Nav2/MPPI/controller parameter edits
- no inflation/robot_radius/clearance_radius_m/map threshold tuning
- no branch scoring change
- no target projection integration
- no fallback/terminal acceptance change
- Goal 1 timeout replay is not autonomous exploration success
- Goal 1 timeout replay is not exit success
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

try:  # ROS imports optional for focused tests.
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    Marker = None
    MarkerArray = None
    Point = None
    ColorRGBA = None

RUN_ID = 'phase67_goal1_timeout_visual_replay'
DEFAULT_TOPIC = '/phase67/goal1_timeout_visual_markers'
DEFAULT_PAYLOAD = f'log/{RUN_ID}/{RUN_ID}.json'


def _repo_root_from_here() -> Path:
    return Path(__file__).resolve().parents[3]


def _resolve(path: str | Path) -> Path:
    p = Path(path)
    if p.is_absolute():
        return p
    return _repo_root_from_here() / p


def _line_endpoints(center: list[float], angle_rad: float, length: float) -> tuple[list[float], list[float]]:
    half = length / 2.0
    ux, uy = math.cos(angle_rad), math.sin(angle_rad)
    return [center[0] - ux * half, center[1] - uy * half, center[2]], [center[0] + ux * half, center[1] + uy * half, center[2]]


def load_visual_payload(payload_path: str | Path = DEFAULT_PAYLOAD) -> dict[str, Any]:
    path = _resolve(payload_path)
    data = json.loads(path.read_text(encoding='utf-8', errors='replace'))
    data.setdefault('run_id', RUN_ID)
    data.setdefault('rviz_marker_topic', DEFAULT_TOPIC)
    return data


def _front_wedge_points(pose: list[float], length_m: float, half_angle_rad: float) -> list[list[float]]:
    x, y, yaw = [float(v) for v in pose[:3]]
    left = yaw + half_angle_rad
    right = yaw - half_angle_rad
    return [
        [x, y, 0.08],
        [x + math.cos(left) * length_m, y + math.sin(left) * length_m, 0.08],
        [x + math.cos(yaw) * length_m, y + math.sin(yaw) * length_m, 0.08],
        [x + math.cos(right) * length_m, y + math.sin(right) * length_m, 0.08],
        [x, y, 0.08],
    ]


def build_marker_specs(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Build ROS-independent marker specs for tests and MarkerArray conversion."""
    goal = payload['goal1']
    target = [float(goal['target'][0]), float(goal['target'][1]), 0.10]
    dispatch_pose = [float(v) for v in goal['dispatch_pose'][:3]]
    terminal_pose = [float(v) for v in goal['terminal_pose'][:3]]
    trajectory_points = [[float(p[0]), float(p[1]), 0.07] for p in goal.get('trajectory_points', [])]
    if len(trajectory_points) < 2:
        trajectory_points = [[dispatch_pose[0], dispatch_pose[1], 0.07], [terminal_pose[0], terminal_pose[1], 0.07]]
    robot_radius = float(payload.get('robot_radius_m', 0.35))
    tolerance_radius = float(payload.get('target_tolerance_reference_radius_m', 0.25))
    wedge_length = float(payload.get('front_wedge_length_m', 0.80))
    wedge_half = float(payload.get('front_wedge_half_angle_rad', 0.45))
    risk_pose = goal.get('local_cost', {}).get('risk_marker_pose') or terminal_pose
    risk_pose = [float(risk_pose[0]), float(risk_pose[1]), 0.16]

    timeout_text = (
        'Phase67 Goal1 timeout: '
        f"recoveries={goal.get('nav2_feedback', {}).get('number_of_recoveries_max')}; "
        f"final_dist={goal.get('nav2_feedback', {}).get('final_distance_remaining_m')}; "
        f"front_wedge_max={goal.get('local_cost', {}).get('timeout_front_wedge_cost_max')}; "
        f"footprint_lethal={goal.get('local_cost', {}).get('timeout_footprint_lethal_cell_count')}; "
        'not autonomous exploration success; not exit success.'
    )

    return [
        {
            'name': 'goal1_dispatch_target',
            'type': 'sphere',
            'frame_id': 'map',
            'position': target,
            'scale': [0.23, 0.23, 0.23],
            'color': [1.0, 0.05, 0.05, 0.95],
            'purpose': 'Goal 1 dispatch target from Phase66 timeout run',
        },
        {
            'name': 'goal1_dispatch_pose',
            'type': 'arrow',
            'frame_id': 'map',
            'position': [dispatch_pose[0], dispatch_pose[1], 0.06],
            'yaw': dispatch_pose[2],
            'scale': [0.55, 0.08, 0.08],
            'color': [0.1, 0.8, 1.0, 0.90],
            'purpose': 'robot pose when Goal 1 was dispatched',
        },
        {
            'name': 'goal1_timeout_terminal_pose',
            'type': 'arrow',
            'frame_id': 'map',
            'position': [terminal_pose[0], terminal_pose[1], 0.09],
            'yaw': terminal_pose[2],
            'scale': [0.65, 0.10, 0.10],
            'color': [1.0, 0.8, 0.0, 0.95],
            'purpose': 'robot terminal pose at Goal 1 timeout/cancel',
        },
        {
            'name': 'goal1_trajectory_path',
            'type': 'line',
            'frame_id': 'map',
            'points': trajectory_points,
            'scale': [0.035, 0.0, 0.0],
            'color': [0.2, 1.0, 0.2, 0.95],
            'purpose': 'odom trajectory from Goal 1 dispatch to timeout terminal pose',
        },
        {
            'name': 'terminal_robot_footprint_circle',
            'type': 'circle',
            'frame_id': 'map',
            'position': [terminal_pose[0], terminal_pose[1], 0.055],
            'radius_m': robot_radius,
            'color': [1.0, 0.65, 0.0, 0.95],
            'purpose': 'robot footprint circle at timeout terminal pose',
        },
        {
            'name': 'terminal_front_wedge',
            'type': 'line',
            'frame_id': 'map',
            'points': _front_wedge_points(terminal_pose, wedge_length, wedge_half),
            'scale': [0.035, 0.0, 0.0],
            'color': [1.0, 0.2, 0.0, 0.95],
            'purpose': 'front wedge at timeout terminal pose; compare against high-cost/lethal local costmap cells',
        },
        {
            'name': 'target_tolerance_circle',
            'type': 'circle',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.052],
            'radius_m': tolerance_radius,
            'color': [0.3, 0.3, 1.0, 0.90],
            'purpose': 'reference target tolerance circle around Goal 1 target for visual comparison only',
        },
        {
            'name': 'nearest_wall_or_costmap_risk_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': risk_pose,
            'scale': [0.18, 0.18, 0.18],
            'color': [1.0, 0.0, 1.0, 0.95],
            'purpose': 'nearest observed costmap risk point from timeout front wedge / wall-side clearance evidence',
        },
        {
            'name': 'timeout_diagnostics_text',
            'type': 'text',
            'frame_id': 'map',
            'position': [terminal_pose[0] + 0.35, terminal_pose[1] + 0.35, 0.60],
            'scale': [0.0, 0.0, 0.20],
            'color': [1.0, 1.0, 1.0, 1.0],
            'text': timeout_text,
            'purpose': 'summarize timeout cmd_vel/recovery/local-cost evidence for RViz screenshot',
        },
    ]


def _make_point(x: float, y: float, z: float = 0.0):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _make_color(values: list[float]):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = [float(v) for v in values]
    return c


def _circle_points(cx: float, cy: float, z: float, radius: float, samples: int = 96):
    return [
        _make_point(cx + math.cos(2.0 * math.pi * i / samples) * radius,
                    cy + math.sin(2.0 * math.pi * i / samples) * radius,
                    z)
        for i in range(samples + 1)
    ]


def specs_to_marker_array(specs: list[dict[str, Any]]) -> Any:
    if MarkerArray is None:
        raise RuntimeError('ROS visualization_msgs is unavailable; source ROS before running the overlay node')
    array = MarkerArray()
    for idx, spec in enumerate(specs):
        marker = Marker()
        marker.header.frame_id = spec.get('frame_id', 'map')
        marker.ns = RUN_ID
        marker.id = idx
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color = _make_color(spec.get('color', [1.0, 1.0, 1.0, 1.0]))
        kind = spec['type']
        if kind == 'sphere':
            marker.type = Marker.SPHERE
            marker.pose.position = _make_point(*spec['position'])
            marker.scale.x, marker.scale.y, marker.scale.z = [float(v) for v in spec['scale']]
        elif kind == 'arrow':
            marker.type = Marker.ARROW
            x, y, z = spec['position']
            yaw = float(spec.get('yaw', 0.0))
            length = float(spec['scale'][0])
            marker.points = [_make_point(x, y, z), _make_point(x + math.cos(yaw) * length, y + math.sin(yaw) * length, z)]
            marker.scale.x, marker.scale.y, marker.scale.z = [float(v) for v in spec['scale']]
        elif kind == 'circle':
            marker.type = Marker.LINE_STRIP
            x, y, z = spec['position']
            marker.points = _circle_points(x, y, z, float(spec['radius_m']))
            marker.scale.x = 0.025
        elif kind == 'line':
            marker.type = Marker.LINE_STRIP
            marker.points = [_make_point(*p) for p in spec['points']]
            marker.scale.x = float(spec.get('scale', [0.03])[0])
        elif kind == 'text':
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position = _make_point(*spec['position'])
            marker.scale.z = float(spec['scale'][2])
            marker.text = spec['text']
        else:
            raise ValueError(f'unsupported marker spec type: {kind}')
        array.markers.append(marker)
    return array


class Phase67Goal1TimeoutVisualOverlay(Node):
    def __init__(self) -> None:
        super().__init__('phase67_goal1_timeout_visual_overlay')
        self.declare_parameter('phase67_payload', DEFAULT_PAYLOAD)
        self.declare_parameter('marker_topic', DEFAULT_TOPIC)
        self.declare_parameter('publish_period_sec', 1.0)
        payload_path = self.get_parameter('phase67_payload').get_parameter_value().string_value
        topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value or 1.0
        self.payload = load_visual_payload(payload_path)
        self.specs = build_marker_specs(self.payload)
        self.publisher = self.create_publisher(MarkerArray, topic, 1)
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"{RUN_ID}: publishing Goal 1 timeout visual overlay on {topic}; "
            f"classification={self.payload.get('classification')}; "
            "visual evidence only, not autonomous exploration success and not exit success."
        )

    def _publish(self) -> None:
        markers = specs_to_marker_array(self.specs)
        now = self.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = now
        self.publisher.publish(markers)


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError('rclpy is unavailable; source /opt/ros/jazzy/setup.bash before running this node')
    rclpy.init(args=args)
    node = Phase67Goal1TimeoutVisualOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
