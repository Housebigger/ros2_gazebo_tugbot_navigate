#!/usr/bin/env python3
"""Phase26P generic MPPI evidence topic recorder.

Diagnostics-only recorder. It discovers topics matching a regex and records
messages from controller/MPPI evidence topics such as critics_stats,
optimal_trajectory, transformed_global_plan, trajectories, plus any additional
runtime topic discovered by the wrapper. It does not publish anything or modify
Nav2 state.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from collections import Counter
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def point_xyz(point: Any) -> tuple[float, float, float] | None:
    if not isinstance(point, dict):
        return None
    x = number(point.get('x'))
    y = number(point.get('y'))
    z = number(point.get('z')) or 0.0
    if x is None or y is None:
        return None
    return (x, y, z)


def path_length(points: list[tuple[float, float, float]]) -> float:
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += math.dist(first, second)
    return total


def sample_evenly(points: list[tuple[float, float, float]], limit: int) -> list[list[float]]:
    if limit <= 0 or not points:
        return []
    if len(points) <= limit:
        selected = points
    elif limit == 1:
        selected = [points[0]]
    else:
        indexes = sorted({round(i * (len(points) - 1) / (limit - 1)) for i in range(limit)})
        selected = [points[i] for i in indexes]
    return [[round(x, 6), round(y, 6), round(z, 6)] for x, y, z in selected]


def numeric_summary(values: list[float]) -> dict[str, float | None]:
    return {
        'min': min(values) if values else None,
        'max': max(values) if values else None,
        'mean': (sum(values) / len(values)) if values else None,
    }


def summarize_marker_array(data: dict[str, Any], sample_limit: int = 30) -> dict[str, Any]:
    markers = data.get('markers') if isinstance(data, dict) else []
    marker_iterable = markers if isinstance(markers, list) else []
    displacements = []
    lengths = []
    point_count = 0
    trajectory_count = 0
    degenerate_count = 0
    near_zero_count = 0
    representative_path_length = None
    representative_marker_id = None
    type_counts: Counter[str] = Counter()
    action_counts: Counter[str] = Counter()
    ns_counts: Counter[str] = Counter()
    scale_values: dict[str, list[float]] = {'x': [], 'y': [], 'z': []}
    alpha_values: list[float] = []
    sampled_marker_points: list[dict[str, Any]] = []
    for marker in marker_iterable:
        if not isinstance(marker, dict):
            continue
        type_counts[str(marker.get('type'))] += 1
        action_counts[str(marker.get('action'))] += 1
        ns_counts[str(marker.get('ns', ''))] += 1
        scale = marker.get('scale') if isinstance(marker.get('scale'), dict) else {}
        for axis in ('x', 'y', 'z'):
            value = number(scale.get(axis))
            if value is not None:
                scale_values[axis].append(value)
        color = marker.get('color') if isinstance(marker.get('color'), dict) else {}
        alpha = number(color.get('a'))
        if alpha is not None:
            alpha_values.append(alpha)
        raw_points = marker.get('points') if isinstance(marker.get('points'), list) else []
        points = [xyz for xyz in (point_xyz(point) for point in raw_points) if xyz is not None]
        point_count += len(points)
        remaining = max(0, sample_limit - len(sampled_marker_points))
        if remaining:
            for sampled in sample_evenly(points, min(remaining, len(points))):
                sampled_marker_points.append({'marker_id': marker.get('id'), 'ns': marker.get('ns', ''), 'point': sampled})
        if len(points) < 2:
            continue
        trajectory_count += 1
        displacement = math.dist(points[0], points[-1])
        length = path_length(points)
        displacements.append(displacement)
        lengths.append(length)
        if displacement <= 0.03:
            degenerate_count += 1
        if length <= 0.05:
            near_zero_count += 1
        if representative_path_length is None or length > representative_path_length:
            representative_path_length = length
            representative_marker_id = marker.get('id')
    return {
        'summary_kind': 'marker_array_trajectory_summary',
        'marker_count': len(marker_iterable),
        'point_count': point_count,
        'trajectory_count': trajectory_count,
        'trajectory_displacement_min': min(displacements) if displacements else None,
        'trajectory_displacement_max': max(displacements) if displacements else None,
        'trajectory_displacement_mean': (sum(displacements) / len(displacements)) if displacements else None,
        'trajectory_path_length_min': min(lengths) if lengths else None,
        'trajectory_path_length_max': max(lengths) if lengths else None,
        'trajectory_path_length_mean': (sum(lengths) / len(lengths)) if lengths else None,
        'representative_path_length': representative_path_length,
        'representative_marker_id': representative_marker_id,
        'degenerate_trajectory_count': degenerate_count,
        'near_zero_trajectory_count': near_zero_count,
        'marker_type_counts': dict(type_counts),
        'marker_action_counts': dict(action_counts),
        'marker_namespace_counts': dict(ns_counts),
        'scale_summary': {axis: numeric_summary(values) for axis, values in scale_values.items()},
        'color_alpha_summary': numeric_summary(alpha_values),
        'sampled_marker_point_count': len(sampled_marker_points),
        'sampled_marker_points': sampled_marker_points,
    }


def extract_path_points(data: Any) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if isinstance(data, dict):
        pose = data.get('pose') if isinstance(data.get('pose'), dict) else None
        position = pose.get('position') if pose and isinstance(pose.get('position'), dict) else data.get('position')
        xyz = point_xyz(position)
        if xyz is not None:
            points.append(xyz)
        for key in ('poses', 'points', 'trajectory', 'samples'):
            if isinstance(data.get(key), list):
                for item in data[key]:
                    points.extend(extract_path_points(item))
    elif isinstance(data, list):
        for item in data:
            points.extend(extract_path_points(item))
    return points


def summarize_path(data: dict[str, Any], sample_limit: int = 30) -> dict[str, Any]:
    points = extract_path_points(data)
    displacement = math.dist(points[0], points[-1]) if len(points) >= 2 else None
    sampled = sample_evenly(points, sample_limit)
    return {
        'summary_kind': 'path_summary',
        'point_count': len(points),
        'path_displacement': displacement,
        'path_length': path_length(points) if len(points) >= 2 else None,
        'sampled_point_count': len(sampled),
        'sampled_points': sampled,
    }


def summarize_message(topic: str, msg_type: str, data: dict[str, Any], sample_limit: int = 30) -> dict[str, Any]:
    if msg_type == 'visualization_msgs/msg/MarkerArray' or topic == '/trajectories':
        return summarize_marker_array(data, sample_limit=sample_limit)
    if msg_type == 'nav_msgs/msg/Path' or topic in ('/optimal_trajectory', '/transformed_global_plan'):
        return summarize_path(data, sample_limit=sample_limit)
    return {'summary_kind': 'generic_summary', 'top_level_keys': sorted(data.keys()) if isinstance(data, dict) else []}


class Phase26PMppiEvidenceRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase26p_mppi_evidence_recorder')
        self.args = args
        self.output_path = Path(args.output).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = self.output_path.open('w', encoding='utf-8')
        self.started_at = time.time()
        self.done = False
        self.topic_regex = re.compile(args.topic_regex)
        self.raw_topics_regex = re.compile(args.raw_topics_regex) if args.raw_topics_regex else None
        self.subscriptions_by_topic: dict[str, Any] = {}
        self.sample_count = 0
        self.discovery_rows: list[dict[str, Any]] = []
        self.discovery_timer = self.create_timer(args.discovery_period_sec, self._discover_topics)
        self.stop_timer = self.create_timer(0.2, self._check_timeout)
        self.get_logger().info(
            f'Phase26P MPPI evidence recorder output={self.output_path} topic_regex={args.topic_regex}'
        )

    def _write(self, row: dict[str, Any]) -> None:
        row.setdefault('wall_time', time.time())
        row['elapsed_sec'] = row['wall_time'] - self.started_at
        self.fp.write(json.dumps(row, sort_keys=True, default=str) + '\n')
        self.fp.flush()

    def _discover_topics(self) -> None:
        for topic, types in self.get_topic_names_and_types():
            if topic in self.subscriptions_by_topic:
                continue
            type_name = types[0] if types else None
            if not type_name or not self.topic_regex.search(topic):
                continue
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError):
                self._write({'event': 'topic_discovery_error', 'topic': topic, 'msg_type': type_name})
                continue
            callback = self._make_callback(topic, type_name)
            self.subscriptions_by_topic[topic] = self.create_subscription(msg_type, topic, callback, 10)
            row = {'event': 'topic_discovered', 'topic': topic, 'msg_type': type_name}
            self.discovery_rows.append(row)
            self._write(row)

    def _make_callback(self, topic: str, msg_type: str):
        def callback(msg: Any) -> None:
            if self.sample_count >= self.args.max_samples:
                self.done = True
                return
            self.sample_count += 1
            try:
                data = message_to_ordereddict(msg)
                summary = summarize_message(topic, msg_type, data, sample_limit=self.args.sample_points)
            except Exception as exc:  # pragma: no cover - defensive for unknown message types
                summary = {'summary_kind': 'conversion_error', 'conversion_error': repr(exc), 'repr': repr(msg)}
                data = None
            row = {'event': 'message', 'topic': topic, 'msg_type': msg_type, 'data_summary': summary}
            if self.raw_topics_regex is not None and self.raw_topics_regex.search(topic) and data is not None:
                row['data'] = data
            self._write(row)
        return callback

    def _check_timeout(self) -> None:
        if time.time() - self.started_at >= self.args.timeout_sec:
            self.done = True

    def close(self) -> None:
        self._write({
            'event': 'summary',
            'sample_count': self.sample_count,
            'discovered_topics': self.discovery_rows,
            'subscribed_topic_count': len(self.subscriptions_by_topic),
        })
        self.fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--topic-regex',
        default=r'(critics_stats|optimal_trajectory|transformed_global_plan|trajectories|trajectory)',
    )
    parser.add_argument(
        '--raw-topics-regex',
        default='',
        help='Optional regex for topics that should also include full raw data. Default writes summaries only.',
    )
    parser.add_argument('--timeout-sec', type=float, default=420.0)
    parser.add_argument('--max-samples', type=int, default=50000)
    parser.add_argument('--sample-points', type=int, default=30, help='Max sampled path or marker points kept per message summary')
    parser.add_argument('--discovery-period-sec', type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Phase26PMppiEvidenceRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                break
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
