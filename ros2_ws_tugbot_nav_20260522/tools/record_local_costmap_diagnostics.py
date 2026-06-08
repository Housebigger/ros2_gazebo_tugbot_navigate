#!/usr/bin/env python3
"""Record or analyze local costmap diagnostics around maze goals.

Offline mode joins `/maze/goal_events` JSONL with previously recorded local costmap
samples and emits per-goal local-cost diagnostics. Runtime recording mode subscribes
to Nav2 local costmap and writes compact JSONL samples.
"""

from __future__ import annotations
import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


@dataclass
class CostmapSample:
    wall_time: float
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list[int]

    def index_for_world(self, x: float, y: float) -> Optional[int]:
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        if cx < 0 or cy < 0 or cx >= self.width or cy >= self.height:
            return None
        return cy * self.width + cx

    def value_at_world(self, x: float, y: float) -> Optional[int]:
        idx = self.index_for_world(x, y)
        if idx is None:
            return None
        return self.data[idx]

    def in_bounds_world(self, x: float, y: float) -> bool:
        return self.index_for_world(x, y) is not None

    def line_sample_count(self, start: list[float], end: list[float], step_m: Optional[float] = None) -> int:
        if step_m is None:
            step_m = max(self.resolution * 0.5, 0.02)
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        dist = math.hypot(dx, dy)
        return max(1, int(math.ceil(dist / step_m))) + 1

    def values_along_line(self, start: list[float], end: list[float], step_m: Optional[float] = None) -> list[int]:
        if step_m is None:
            step_m = max(self.resolution * 0.5, 0.02)
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        dist = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(dist / step_m)))
        values: list[int] = []
        for i in range(steps + 1):
            t = i / steps
            value = self.value_at_world(float(start[0]) + dx * t, float(start[1]) + dy * t)
            if value is not None:
                values.append(value)
        return values

    def values_in_radius(self, center: list[float], radius_m: float) -> list[int]:
        cx = float(center[0])
        cy = float(center[1])
        cells = max(0, int(math.ceil(radius_m / self.resolution)))
        base_x = int(math.floor((cx - self.origin_x) / self.resolution))
        base_y = int(math.floor((cy - self.origin_y) / self.resolution))
        values: list[int] = []
        for yy in range(base_y - cells, base_y + cells + 1):
            for xx in range(base_x - cells, base_x + cells + 1):
                if xx < 0 or yy < 0 or xx >= self.width or yy >= self.height:
                    continue
                wx = self.origin_x + (xx + 0.5) * self.resolution
                wy = self.origin_y + (yy + 0.5) * self.resolution
                if math.hypot(wx - cx, wy - cy) <= radius_m + 1e-9:
                    values.append(self.data[yy * self.width + xx])
        return values


def _as_origin(info: dict[str, Any]) -> tuple[float, float]:
    origin = info.get('origin', [0.0, 0.0])
    if isinstance(origin, dict):
        position = origin.get('position', origin)
        return float(position.get('x', 0.0)), float(position.get('y', 0.0))
    return float(origin[0]), float(origin[1])


def load_costmap_samples(path: Path) -> list[CostmapSample]:
    samples: list[CostmapSample] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        info = row['info']
        origin_x, origin_y = _as_origin(info)
        samples.append(CostmapSample(
            wall_time=float(row['wall_time']),
            width=int(info['width']),
            height=int(info['height']),
            resolution=float(info['resolution']),
            origin_x=origin_x,
            origin_y=origin_y,
            data=[int(v) for v in row['data']],
        ))
    return sorted(samples, key=lambda sample: sample.wall_time)


def load_goal_intervals(path: Path) -> list[dict[str, Any]]:
    by_seq: dict[int, dict[str, Any]] = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        seq = payload.get('goal_sequence')
        if seq is None:
            continue
        rec = by_seq.setdefault(int(seq), {'goal_sequence': int(seq)})
        event = payload.get('event')
        wall_time = row.get('wall_time')
        if event == 'dispatch':
            rec.update({
                'dispatch_wall_time': float(wall_time) if wall_time is not None else None,
                'dispatch_pose': payload.get('dispatch_pose'),
                'target': payload.get('target'),
                'target_clearance_m': payload.get('target_clearance_m'),
            })
        elif event in ('success', 'timeout', 'failure', 'terminal_cancel') and 'outcome' not in rec:
            rec.update({
                'outcome': event,
                'outcome_wall_time': float(wall_time) if wall_time is not None else None,
                'elapsed_sec': payload.get('elapsed_sec'),
            })
    return [rec for _, rec in sorted(by_seq.items()) if rec.get('dispatch_wall_time') is not None and rec.get('outcome_wall_time') is not None]


def nearest_sample(samples: list[CostmapSample], timestamp: float) -> Optional[CostmapSample]:
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample.wall_time - timestamp))


def _mean(values: list[int]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def _max(values: list[int]) -> Optional[int]:
    return max(values) if values else None


def diagnose_goal(goal: dict[str, Any], samples: list[CostmapSample], target_radius_m: float, robot_cluster_radius_m: float, obstacle_threshold: int, inflation_threshold: int) -> dict[str, Any]:
    dispatch_time = float(goal['dispatch_wall_time'])
    outcome_time = float(goal['outcome_wall_time'])
    dispatch_sample = nearest_sample(samples, dispatch_time)
    timeout_sample = nearest_sample(samples, outcome_time)
    target = goal.get('target') or [None, None]
    pose = goal.get('dispatch_pose') or [None, None, None]
    row = dict(goal)
    row.update({
        'dispatch_sample_wall_time': dispatch_sample.wall_time if dispatch_sample else None,
        'timeout_sample_wall_time': timeout_sample.wall_time if timeout_sample else None,
        'dispatch_sample_age_sec': round(abs(dispatch_sample.wall_time - dispatch_time), 6) if dispatch_sample else None,
        'outcome_sample_age_sec': round(abs(timeout_sample.wall_time - outcome_time), 6) if timeout_sample else None,
        'target_in_local_costmap_bounds': None,
        'robot_pose_in_local_costmap_bounds': None,
        'dispatch_path_sample_count': 0,
        'dispatch_path_in_bounds_sample_count': 0,
        'local_cost_sample_coverage_ratio': 0.0,
        'dispatch_target_local_cost': None,
        'dispatch_target_local_cost_max_radius': None,
        'dispatch_path_local_cost_max': None,
        'dispatch_path_local_cost_mean': None,
        'timeout_robot_local_cost_max': None,
        'timeout_robot_local_cost_mean': None,
        'timeout_robot_obstacle_cluster_count': None,
        'footprint_corridor_inflation_squeezed': False,
    })
    if dispatch_sample and target[0] is not None:
        row['target_in_local_costmap_bounds'] = dispatch_sample.in_bounds_world(float(target[0]), float(target[1]))
        row['dispatch_target_local_cost'] = dispatch_sample.value_at_world(float(target[0]), float(target[1]))
        target_values = dispatch_sample.values_in_radius(target, target_radius_m)
        row['dispatch_target_local_cost_max_radius'] = _max(target_values)
        if pose[0] is not None:
            row['robot_pose_in_local_costmap_bounds'] = dispatch_sample.in_bounds_world(float(pose[0]), float(pose[1]))
            expected_path_samples = dispatch_sample.line_sample_count(pose, target)
            path_values = dispatch_sample.values_along_line(pose, target)
            row['dispatch_path_sample_count'] = expected_path_samples
            row['dispatch_path_in_bounds_sample_count'] = len(path_values)
            row['local_cost_sample_coverage_ratio'] = len(path_values) / expected_path_samples if expected_path_samples else 0.0
            row['dispatch_path_local_cost_max'] = _max(path_values)
            row['dispatch_path_local_cost_mean'] = _mean(path_values)
    if timeout_sample and pose[0] is not None:
        robot_values = timeout_sample.values_in_radius(pose, robot_cluster_radius_m)
        row['timeout_robot_local_cost_max'] = _max(robot_values)
        row['timeout_robot_local_cost_mean'] = _mean(robot_values)
        row['timeout_robot_obstacle_cluster_count'] = sum(1 for value in robot_values if value >= obstacle_threshold)
        row['footprint_corridor_inflation_squeezed'] = any(value >= inflation_threshold for value in robot_values)
    return row


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        'goal_count': len(rows),
        'timeout_count': sum(1 for row in rows if row.get('outcome') == 'timeout'),
        'success_count': sum(1 for row in rows if row.get('outcome') == 'success'),
        'squeezed_count': sum(1 for row in rows if row.get('footprint_corridor_inflation_squeezed')),
        'timeout_squeezed_count': sum(1 for row in rows if row.get('outcome') == 'timeout' and row.get('footprint_corridor_inflation_squeezed')),
    }


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'dispatch_target_local_cost',
        'outcome',
        'dispatch_path_local_cost_max',
        'dispatch_path_local_cost_mean',
        'timeout_robot_local_cost_max',
        'timeout_robot_obstacle_cluster_count',
        'footprint_corridor_inflation_squeezed',
        'dispatch_sample_age_sec',
        'outcome_sample_age_sec',
        'target_in_local_costmap_bounds',
        'robot_pose_in_local_costmap_bounds',
        'dispatch_path_sample_count',
        'local_cost_sample_coverage_ratio',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def analyze_offline(args: argparse.Namespace) -> int:
    samples = load_costmap_samples(args.costmap_samples)
    goals = load_goal_intervals(args.goal_events)
    rows = [diagnose_goal(goal, samples, args.target_radius_m, args.robot_cluster_radius_m, args.obstacle_threshold, args.inflation_threshold) for goal in goals]
    data = {'summary': summarize(rows), 'goals': rows}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(rows)
    return 0


def record_runtime(args: argparse.Namespace) -> int:
    # Import ROS only in runtime mode so offline tests work without spinning ROS.
    import rclpy
    from nav_msgs.msg import OccupancyGrid

    rclpy.init()
    node = rclpy.create_node('local_costmap_sample_recorder')
    output = args.output_jsonl
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0

    def callback(msg: OccupancyGrid) -> None:
        nonlocal count
        row = {
            'wall_time': node.get_clock().now().nanoseconds / 1e9,
            'topic': args.costmap_topic,
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
            },
            'data': list(msg.data),
        }
        with output.open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(row, sort_keys=True) + '\n')
        count += 1
        if count >= args.max_samples:
            rclpy.shutdown()

    node.create_subscription(OccupancyGrid, args.costmap_topic, callback, 10)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout_sec
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds / 1e9 >= deadline:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if count < args.min_samples:
        print(f'only recorded {count} samples, expected at least {args.min_samples}', file=sys.stderr)
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', type=Path)
    parser.add_argument('--costmap-samples', type=Path)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--target-radius-m', type=float, default=0.30)
    parser.add_argument('--robot-cluster-radius-m', type=float, default=0.75)
    parser.add_argument('--obstacle-threshold', type=int, default=65)
    parser.add_argument('--inflation-threshold', type=int, default=50)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--costmap-topic', default='/local_costmap/costmap_raw')
    parser.add_argument('--output-jsonl', type=Path)
    parser.add_argument('--max-samples', type=int, default=120)
    parser.add_argument('--min-samples', type=int, default=1)
    parser.add_argument('--timeout-sec', type=float, default=300.0)
    args = parser.parse_args()
    if args.record:
        if args.output_jsonl is None:
            parser.error('--record requires --output-jsonl')
    else:
        if args.goal_events is None or args.costmap_samples is None:
            parser.error('offline mode requires --goal-events and --costmap-samples')
    return args


def main() -> int:
    args = parse_args()
    if args.record:
        return record_runtime(args)
    return analyze_offline(args)


if __name__ == '__main__':
    raise SystemExit(main())
