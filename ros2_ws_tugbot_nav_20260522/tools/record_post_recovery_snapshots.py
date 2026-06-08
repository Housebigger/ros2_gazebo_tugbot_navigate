#!/usr/bin/env python3
"""Runtime post-recovery recorder for Phase 24C diagnostics.

Subscribes to /local_costmap/costmap, /plan, /odom, /cmd_vel_nav, and
/maze/goal_events. It emits JSONL snapshots around inferred recovery windows and
near-zero command onset without changing navigation behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import String

NEAR_ZERO_CMD_LINEAR = 0.01
NEAR_ZERO_CMD_ANGULAR = 0.05
RECOVERY_WINDOW_SEC = 6.0
HIGH_COST_THRESHOLD = 70
PHASE26L_SPATIAL_CONTRACT_FIELDS = [
    'path_ahead_1_0m_high_cost_points',
    'path_ahead_1_0m_high_cost_count',
    'path_ahead_1_0m_high_cost_centroid',
    'path_ahead_1_0m_nearest_high_cost_point',
    'path_ahead_1_0m_first_high_cost_distance_m',
    'chosen_route_corridor_cost_max',
    'chosen_route_corridor_cost_mean',
    'chosen_route_first_high_cost_distance_m',
    'explored_candidate_corridor_cost_max',
    'explored_candidate_corridor_cost_mean',
    'explored_candidate_first_high_cost_distance_m',
    'selected_explored_candidate_target',
]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


class CostmapView:
    def __init__(self, msg: OccupancyGrid) -> None:
        self.msg = msg
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = list(msg.data)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin_x) / self.resolution)), int(math.floor((y - self.origin_y) / self.resolution)))

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cell_value(self, cell: tuple[int, int]) -> Optional[int]:
        if not self.in_bounds(cell):
            return None
        return int(self.data[cell[0] + cell[1] * self.width])

    def line_values(self, start: tuple[float, float], end: tuple[float, float]) -> list[int]:
        return [sample['value'] for sample in self._line_samples(start, end)]

    def _line_samples(self, start: tuple[float, float], end: tuple[float, float]) -> list[dict[str, Any]]:
        distance = point_distance(start, end)
        step = max(self.resolution * 0.5, 0.05)
        steps = max(1, int(math.ceil(distance / step)))
        samples: list[dict[str, Any]] = []
        for index in range(steps + 1):
            ratio = index / steps
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            value = self.cell_value(self.world_to_cell(x, y))
            if value is not None:
                samples.append({
                    'point': [round(x, 6), round(y, 6)],
                    'value': int(value),
                    'distance_m': round(distance * ratio, 6),
                })
        return samples


def max_or_none(values: list[int]) -> Optional[int]:
    return max(values) if values else None


def mean_or_none(values: list[int]) -> Optional[float]:
    return float(sum(values) / len(values)) if values else None


class PostRecoverySnapshotRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('post_recovery_snapshot_recorder')
        self.args = args
        self.output_path = Path(args.output).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = self.output_path.open('w', encoding='utf-8')
        self.started_at = time.time()
        self.done = False
        self.active_goal_sequence: Optional[int] = None
        self.last_costmap: Optional[CostmapView] = None
        self.last_path_points: list[tuple[float, float]] = []
        self.last_pose: Optional[tuple[float, float, float]] = None
        self.last_cmd: Optional[dict[str, Any]] = None
        self.cmd_near_zero = False
        self.near_zero_onset_emitted: set[int] = set()
        self.last_periodic_snapshot_time = 0.0
        self.recent_snapshots: deque[dict[str, Any]] = deque(maxlen=500)
        self.recovery_by_goal: dict[int, dict[str, Any]] = {}
        self.dispatch_by_goal: dict[int, dict[str, Any]] = {}

        self.costmap_sub = self.create_subscription(OccupancyGrid, args.local_costmap_topic, self._on_costmap, 10)
        self.path_sub = self.create_subscription(NavPath, args.path_topic, self._on_path, 10)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self._on_odom, 20)
        self.cmd_sub = self.create_subscription(Twist, args.cmd_topic, self._on_cmd, 20)
        self.goal_events_sub = self.create_subscription(String, args.goal_events_topic, self._on_goal_event, 20)
        self.timer = self.create_timer(0.2, self._on_timer)
        self.get_logger().info(
            'Phase24C recorder started: local_costmap=%s path=%s odom=%s cmd=%s goal_events=%s output=%s'
            % (args.local_costmap_topic, args.path_topic, args.odom_topic, args.cmd_topic, args.goal_events_topic, self.output_path)
        )

    def _write_row(self, row: dict[str, Any]) -> None:
        row.setdefault('wall_time', time.time())
        row['elapsed_sec'] = row['wall_time'] - self.started_at
        self.fp.write(json.dumps(row, sort_keys=True) + '\n')
        self.fp.flush()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.last_costmap = CostmapView(msg)

    def _on_path(self, msg: NavPath) -> None:
        self.last_path_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        seq = self.active_goal_sequence
        now = time.time()
        if seq is not None:
            recovery = self.recovery_by_goal.setdefault(seq, {'path_update_times': [], 'post_recovery_path_update_count': 0})
            recovery['path_update_times'].append(now)
            recovery['post_recovery_path_update_count'] = int(recovery.get('post_recovery_path_update_count', 0)) + 1
            self._write_row({
                'event': 'path_update',
                'goal_sequence': seq,
                'snapshot_type': 'path_update',
                'path_point_count': len(self.last_path_points),
                'post_recovery_path_update_count': recovery['post_recovery_path_update_count'],
                'wall_time': now,
            })

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.last_pose = (
            float(pos.x),
            float(pos.y),
            yaw_from_quaternion(float(ori.x), float(ori.y), float(ori.z), float(ori.w)),
        )

    def _on_cmd(self, msg: Twist) -> None:
        now = time.time()
        self.last_cmd = {
            'wall_time': now,
            'linear_x': float(msg.linear.x),
            'angular_z': float(msg.angular.z),
        }
        near_zero = abs(float(msg.linear.x)) <= NEAR_ZERO_CMD_LINEAR and abs(float(msg.angular.z)) <= NEAR_ZERO_CMD_ANGULAR
        seq = self.active_goal_sequence
        if near_zero and not self.cmd_near_zero and seq is not None and seq not in self.near_zero_onset_emitted:
            self.near_zero_onset_emitted.add(seq)
            self._emit_snapshot(seq, 'near_zero_onset', now)
        self.cmd_near_zero = near_zero

    def _on_goal_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        seq = payload.get('goal_sequence')
        event = payload.get('event')
        if seq is not None:
            seq = int(seq)
        if event == 'dispatch' and seq is not None:
            self.active_goal_sequence = seq
            self.dispatch_by_goal[seq] = payload
            self.last_periodic_snapshot_time = 0.0
            self.near_zero_onset_emitted.discard(seq)
            self.recovery_by_goal.setdefault(seq, {'path_update_times': [], 'post_recovery_path_update_count': 0})
            self._write_row({'event': 'goal_event', 'snapshot_type': 'dispatch', 'goal_sequence': seq, 'wall_time': time.time()})
        elif event in ('timeout', 'failure', 'success', 'terminal_cancel') and seq is not None:
            self._write_row({'event': 'goal_event', 'snapshot_type': event, 'goal_sequence': seq, 'wall_time': time.time()})
            if self.active_goal_sequence == seq:
                self.active_goal_sequence = None

    def _on_timer(self) -> None:
        now = time.time()
        seq = self.active_goal_sequence
        if seq is not None and now - self.last_periodic_snapshot_time >= self.args.periodic_snapshot_sec:
            self.last_periodic_snapshot_time = now
            self._emit_snapshot(seq, 'periodic_active_goal', now)
        if now - self.started_at >= self.args.timeout_sec:
            self.done = True

    def infer_recovery(self, seq: int, recovery_time: float) -> None:
        # Testable hook: emit pre/post recovery snapshots around an externally known clear-costmap time.
        self.active_goal_sequence = seq
        self._emit_snapshot(seq, 'pre_recovery', recovery_time - 0.1)
        self._emit_snapshot(seq, 'post_recovery', recovery_time + 0.1)

    def _emit_snapshot(self, seq: int, snapshot_type: str, wall_time: float) -> None:
        row = self._make_snapshot(seq, snapshot_type, wall_time)
        if snapshot_type in ('pre_recovery', 'post_recovery', 'near_zero_onset'):
            self.recent_snapshots.append(row)
        self._write_row(row)

    def _make_snapshot(self, seq: int, snapshot_type: str, wall_time: float) -> dict[str, Any]:
        pose = self.last_pose
        costmap = self.last_costmap
        recovery = self.recovery_by_goal.setdefault(seq, {'path_update_times': [], 'post_recovery_path_update_count': 0})
        path_ahead_0_5 = self._path_ahead_values(0.5)
        path_ahead_1_0 = self._path_ahead_values(1.0)
        path_ahead_1_0_samples = self._path_ahead_samples(1.0)
        robot_to_path = self._robot_to_path_distance()
        dispatch = self.dispatch_by_goal.get(seq, {})
        chosen_route = self._route_corridor_summary(dispatch, 'chosen_route', dispatch.get('target'))
        explored_candidate = self._selected_explored_candidate(dispatch)
        explored_route = self._route_corridor_summary(
            dispatch,
            'explored_candidate',
            explored_candidate.get('target') if explored_candidate else None,
        )
        row = {
            'event': 'snapshot',
            'snapshot_type': snapshot_type,
            'goal_sequence': seq,
            'wall_time': wall_time,
            'has_local_costmap': costmap is not None,
            'has_path': bool(self.last_path_points),
            'has_odom': pose is not None,
            'path_point_count': len(self.last_path_points),
            'robot_pose': list(pose) if pose is not None else None,
            'robot_to_path_distance_m': robot_to_path,
            'path_ahead_0_5m_cost_max': max_or_none(path_ahead_0_5),
            'path_ahead_0_5m_cost_mean': mean_or_none(path_ahead_0_5),
            'path_ahead_1_0m_cost_max': max_or_none(path_ahead_1_0),
            'path_ahead_1_0m_cost_mean': mean_or_none(path_ahead_1_0),
            'post_recovery_path_update_count': int(recovery.get('post_recovery_path_update_count', 0)),
            'controller_received_path_but_cmd_near_zero': bool(recovery.get('post_recovery_path_update_count', 0) and self.cmd_near_zero),
            'last_cmd': self.last_cmd,
        }
        row.update(self._high_cost_summary('path_ahead_1_0m', path_ahead_1_0_samples))
        row.update(chosen_route)
        row.update(explored_route)
        row['selected_explored_candidate_target'] = self._target_xy(explored_candidate.get('target')) if explored_candidate else None
        return row

    def _path_ahead_values(self, distance_m: float) -> list[int]:
        if self.last_costmap is None or self.last_pose is None:
            return []
        start = (self.last_pose[0], self.last_pose[1])
        end = (self.last_pose[0] + math.cos(self.last_pose[2]) * distance_m, self.last_pose[1] + math.sin(self.last_pose[2]) * distance_m)
        return self.last_costmap.line_values(start, end)

    def _path_ahead_samples(self, distance_m: float) -> list[dict[str, Any]]:
        if self.last_costmap is None or self.last_pose is None:
            return []
        start = (self.last_pose[0], self.last_pose[1])
        end = (self.last_pose[0] + math.cos(self.last_pose[2]) * distance_m, self.last_pose[1] + math.sin(self.last_pose[2]) * distance_m)
        return self.last_costmap._line_samples(start, end)

    def _target_xy(self, value: Any) -> Optional[list[float]]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return [round(float(value[0]), 6), round(float(value[1]), 6)]
            except (TypeError, ValueError):
                return None
        return None

    def _selected_explored_candidate(self, dispatch: dict[str, Any]) -> Optional[dict[str, Any]]:
        candidates = [
            candidate for candidate in dispatch.get('candidate_branches') or []
            if isinstance(candidate, dict) and candidate.get('rejection_reason') == 'explored'
        ]
        if not candidates:
            return None
        def key(candidate: dict[str, Any]) -> float:
            raw = candidate.get('target_exit_dist')
            if raw is None:
                return float('inf')
            try:
                return float(raw)
            except (TypeError, ValueError):
                return float('inf')
        return min(candidates, key=key)

    def _high_cost_summary(self, prefix: str, samples: list[dict[str, Any]]) -> dict[str, Any]:
        high_cost = [sample for sample in samples if int(sample.get('value', -1)) >= HIGH_COST_THRESHOLD]
        limited = high_cost[:max(0, int(self.args.max_high_cost_points))]
        points = [sample.get('point') for sample in limited]
        centroid = None
        if high_cost:
            centroid = [
                round(sum(float(sample['point'][0]) for sample in high_cost) / len(high_cost), 6),
                round(sum(float(sample['point'][1]) for sample in high_cost) / len(high_cost), 6),
            ]
        nearest = high_cost[0].get('point') if high_cost else None
        first_distance = high_cost[0].get('distance_m') if high_cost else None
        return {
            f'{prefix}_high_cost_count': len(high_cost),
            f'{prefix}_high_cost_points': points,
            f'{prefix}_high_cost_centroid': centroid,
            f'{prefix}_nearest_high_cost_point': nearest,
            f'{prefix}_first_high_cost_distance_m': first_distance,
        }

    def _route_corridor_summary(self, dispatch: dict[str, Any], prefix: str, target: Any) -> dict[str, Any]:
        empty = {
            f'{prefix}_corridor_cost_max': None,
            f'{prefix}_corridor_cost_mean': None,
            f'{prefix}_first_high_cost_distance_m': None,
            f'{prefix}_high_cost_count': 0,
        }
        if self.last_costmap is None:
            return empty
        dispatch_xy = self._target_xy(dispatch.get('dispatch_pose'))
        target_xy = self._target_xy(target)
        if dispatch_xy is None or target_xy is None:
            return empty
        samples = self.last_costmap._line_samples((dispatch_xy[0], dispatch_xy[1]), (target_xy[0], target_xy[1]))
        values = [int(sample['value']) for sample in samples]
        high_cost = [sample for sample in samples if int(sample.get('value', -1)) >= HIGH_COST_THRESHOLD]
        return {
            f'{prefix}_corridor_cost_max': max_or_none(values),
            f'{prefix}_corridor_cost_mean': mean_or_none(values),
            f'{prefix}_first_high_cost_distance_m': high_cost[0].get('distance_m') if high_cost else None,
            f'{prefix}_high_cost_count': len(high_cost),
        }

    def _robot_to_path_distance(self) -> Optional[float]:
        if self.last_pose is None or not self.last_path_points:
            return None
        robot_xy = (self.last_pose[0], self.last_pose[1])
        return min(point_distance(robot_xy, point) for point in self.last_path_points)

    def close(self) -> None:
        self.fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--path-topic', default='/plan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='cmd_vel_nav')
    parser.add_argument('--goal-events-topic', default='/maze/goal_events')
    parser.add_argument('--timeout-sec', type=float, default=420.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    parser.add_argument('--max-high-cost-points', type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PostRecoverySnapshotRecorder(args)
    interrupted_by_external_shutdown = False
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except rclpy.executors.ExternalShutdownException:
                interrupted_by_external_shutdown = True
                break
    finally:
        out = node.output_path
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({'output': str(out), 'external_shutdown': interrupted_by_external_shutdown}, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
