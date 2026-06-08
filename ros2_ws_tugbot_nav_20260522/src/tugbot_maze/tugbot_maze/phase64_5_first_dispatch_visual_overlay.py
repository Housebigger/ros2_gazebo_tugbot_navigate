#!/usr/bin/env python3
"""Phase64.5 first dispatch visual overlay marker publisher.

Visual-only/manual comparison helper for:
Phase64.5 First Dispatch Visual Reality Check / Manual Nav Comparison.

Guardrails:
- static/manual visual evidence only
- no runtime dispatch integration
- no Nav2/MPPI/controller parameter edits
- no clearance_radius_m/map threshold edits
- no branch scoring changes
- no entrance/fallback/terminal acceptance changes
- no target projection integration
- no autonomous exploration success claim
- first dispatch is not exit success

The node reads Phase64 geometry evidence, publishes RViz MarkerArray overlays for
first dispatch target marker, dispatch pose, robot_radius circle, Tugbot mesh body
circle, inflation envelope circle, corridor width line, and nearest wall helper.
It does not send goals and does not modify configuration.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

try:  # ROS imports are optional for unit tests that only inspect helpers.
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
except Exception:  # pragma: no cover - exercised in non-ROS static tests.
    rclpy = None
    Node = object
    Marker = None
    MarkerArray = None
    Point = None
    ColorRGBA = None

RUN_ID = 'phase64_5_first_dispatch_visual_reality_check'
DEFAULT_TOPIC = '/phase64_5/first_dispatch_visual_markers'
DEFAULT_PHASE64_ARTIFACT = (
    'log/phase64_corridor_width_robot_footprint_feasibility_decision/'
    'phase64_corridor_width_robot_footprint_feasibility_decision.json'
)
ALLOWED_CLASSIFICATIONS = [
    'VISUALLY_CONFIRMED_TOO_CLOSE',
    'VISUALLY_MARGINAL_BUT_PASSABLE',
    'MANUAL_GOAL_AVOIDS_RISKY_TARGET',
    'VISUAL_EVIDENCE_INCONCLUSIVE',
]


def _round(value: Any, digits: int = 6) -> Any:
    if isinstance(value, (int, float)):
        return round(float(value), digits)
    return value


def _repo_root_from_here() -> Path:
    return Path(__file__).resolve().parents[3]


def _resolve(path: str | Path) -> Path:
    p = Path(path)
    if p.is_absolute():
        return p
    return _repo_root_from_here() / p


def load_overlay_payload(phase64_artifact: str | Path = DEFAULT_PHASE64_ARTIFACT) -> dict[str, Any]:
    """Load Phase64 artifact and normalize the visual overlay payload."""
    artifact_path = _resolve(phase64_artifact)
    data = json.loads(artifact_path.read_text())
    evidence = data.get('first_dispatch_corridor_evidence') or {}
    widths = data.get('required_widths') or {}
    margins = data.get('feasibility_margins') or {}

    target_map = evidence.get('target_map') or [0.0, 0.0]
    dispatch_pose = evidence.get('dispatch_pose_map') or [0.0, 0.0, 0.0]
    robot_radius = float(widths.get('robot_radius_m', 0.35))
    mesh_radius = float(widths.get('mesh_radius_m', robot_radius))
    inflation_radius = float(widths.get('local_inflation_radius_m', 0.7))
    inflation_full_radius = robot_radius + inflation_radius
    effective_width = float(evidence.get('first_dispatch_effective_width_from_replay_clearance_m', 0.0))
    target_clearance = float(evidence.get('target_clearance_m', effective_width / 2.0 if effective_width else 0.0))
    branch_angle = float(evidence.get('branch_angle_rad', math.pi / 2.0))
    nearest_wall_distance = evidence.get('nearest_sdf_wall_distance_m')

    return {
        'run_id': RUN_ID,
        'phase': 'Phase64.5 First Dispatch Visual Reality Check / Manual Nav Comparison',
        'classification': data.get('classification'),
        'phase64_classification_preserved': data.get('classification'),
        'phase62_classification_preserved': data.get('phase62_classification_preserved'),
        'phase62_secondary_factor_preserved': data.get('phase62_secondary_factor_preserved'),
        'phase63_classification_preserved': data.get('phase63_classification_preserved'),
        'target_map': [float(target_map[0]), float(target_map[1])],
        'dispatch_pose_map': [float(dispatch_pose[0]), float(dispatch_pose[1]), float(dispatch_pose[2])],
        'branch_angle_rad': branch_angle,
        'robot_radius_m': _round(robot_radius),
        'mesh_radius_m': _round(mesh_radius),
        'inflation_radius_m': _round(inflation_radius),
        'inflation_full_radius_m': _round(inflation_full_radius),
        'first_dispatch_effective_width_m': _round(effective_width),
        'target_clearance_m': _round(target_clearance),
        'nearest_wall_distance_m': _round(nearest_wall_distance) if nearest_wall_distance is not None else None,
        'nearest_wall_name': evidence.get('nearest_sdf_wall'),
        'local_radius_max_cost': evidence.get('local_radius_max_cost'),
        'footprint_max_cost': evidence.get('footprint_max_cost'),
        'footprint_lethal_count': evidence.get('footprint_lethal_count'),
        'phase63_safe_projection_found': evidence.get('phase63_safe_projection_found'),
        'margins': margins,
        'guardrails': [
            'visual/manual evidence only',
            'no runtime dispatch integration',
            'no Nav2/MPPI/controller parameter edits',
            'no clearance_radius_m/map threshold edits',
            'no branch scoring change',
            'no entrance/fallback/terminal acceptance change',
            'no target projection integration',
            'no autonomous exploration success claim',
            'first dispatch is not exit success',
        ],
        'allowed_visual_classifications': ALLOWED_CLASSIFICATIONS,
    }


def _line_endpoints(center: list[float], angle_rad: float, length: float) -> tuple[list[float], list[float]]:
    ux = math.cos(angle_rad)
    uy = math.sin(angle_rad)
    half = length / 2.0
    return [center[0] - ux * half, center[1] - uy * half, 0.04], [center[0] + ux * half, center[1] + uy * half, 0.04]


def build_marker_specs(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Build ROS-independent marker specs used by tests and MarkerArray conversion."""
    target = payload['target_map']
    dispatch_pose = payload['dispatch_pose_map']
    branch_angle = float(payload.get('branch_angle_rad', math.pi / 2.0))
    width = float(payload['first_dispatch_effective_width_m'])
    wall_dist = payload.get('nearest_wall_distance_m') or float(payload['target_clearance_m'])

    # Width line is perpendicular to travel/open direction through target.
    width_start, width_end = _line_endpoints(target, branch_angle + math.pi / 2.0, width)
    wall_start, wall_end = _line_endpoints(target, branch_angle + math.pi / 2.0, float(wall_dist))

    return [
        {
            'name': 'first_dispatch_target',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.08],
            'scale': [0.22, 0.22, 0.22],
            'color': [1.0, 0.1, 0.1, 0.95],
            'text': 'Phase64.5 first dispatch target marker',
            'purpose': 'show automatic first dispatch target from Phase62/64 replay',
        },
        {
            'name': 'dispatch_pose',
            'type': 'arrow',
            'frame_id': 'map',
            'position': [dispatch_pose[0], dispatch_pose[1], 0.05],
            'yaw': dispatch_pose[2],
            'scale': [0.45, 0.08, 0.08],
            'color': [0.1, 0.8, 1.0, 0.9],
            'purpose': 'show robot dispatch pose before first automatic target',
        },
        {
            'name': 'robot_radius_circle_at_target',
            'type': 'circle',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.035],
            'radius_m': payload['robot_radius_m'],
            'color': [0.0, 0.9, 0.2, 0.9],
            'purpose': 'show Nav2 robot_radius around target',
        },
        {
            'name': 'mesh_radius_circle_at_target',
            'type': 'circle',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.04],
            'radius_m': payload['mesh_radius_m'],
            'color': [0.9, 0.9, 0.1, 0.9],
            'purpose': 'show Tugbot mesh body radius around target',
        },
        {
            'name': 'inflation_envelope_circle_at_target',
            'type': 'circle',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.045],
            'radius_m': payload['inflation_full_radius_m'],
            'color': [1.0, 0.45, 0.0, 0.85],
            'purpose': 'show robot_radius + local inflation radius envelope',
        },
        {
            'name': 'effective_corridor_width_line',
            'type': 'line',
            'frame_id': 'map',
            'points': [width_start, width_end],
            'width_m': payload['first_dispatch_effective_width_m'],
            'scale': [0.035, 0.0, 0.0],
            'color': [0.7, 0.1, 1.0, 0.95],
            'purpose': 'show replay-derived effective corridor width at first dispatch target',
        },
        {
            'name': 'nearest_wall_distance_line',
            'type': 'line',
            'frame_id': 'map',
            'points': [wall_start, wall_end],
            'width_m': payload['nearest_wall_distance_m'],
            'scale': [0.025, 0.0, 0.0],
            'color': [1.0, 1.0, 1.0, 0.95],
            'purpose': 'show nearest wall / clearance helper from Phase64 replay',
        },
        {
            'name': 'manual_goal_comparison_note',
            'type': 'text',
            'frame_id': 'map',
            'position': [target[0] + 0.25, target[1] + 0.25, 0.45],
            'scale': [0.0, 0.0, 0.22],
            'color': [1.0, 1.0, 1.0, 1.0],
            'text': (
                'Phase64.5: compare manual Nav2 Goal path with red first-dispatch target; '
                'do not infer autonomous/exit success.'
            ),
            'purpose': 'manual Nav2 Goal comparison note',
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
        raise RuntimeError('ROS visualization_msgs is unavailable; source ROS before running the node')
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


class Phase645FirstDispatchVisualOverlay(Node):
    def __init__(self) -> None:
        super().__init__('phase64_5_first_dispatch_visual_overlay')
        self.declare_parameter('phase64_artifact', DEFAULT_PHASE64_ARTIFACT)
        self.declare_parameter('marker_topic', DEFAULT_TOPIC)
        self.declare_parameter('publish_period_sec', 1.0)
        artifact = self.get_parameter('phase64_artifact').get_parameter_value().string_value
        topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value or 1.0
        self.payload = load_overlay_payload(artifact)
        self.specs = build_marker_specs(self.payload)
        self.publisher = self.create_publisher(MarkerArray, topic, 1)
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"{RUN_ID}: publishing visual-only first dispatch overlay on {topic}; "
            f"classification={self.payload['phase64_classification_preserved']}; "
            "manual Nav2 Goal comparison may be observed by the user, but this node sends no goals."
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
    node = Phase645FirstDispatchVisualOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
