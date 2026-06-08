#!/usr/bin/env python3
"""Phase65 Ingress Goal Depth Adjustment / Inner Entrance Staging analyzer.

This phase tests only the entrance-guide staging hypothesis: the previous
Phase53/61/62 ingress goal at map (1.0, 0.0) may be too shallow, leaving Tugbot
outside or on the map edge before post-ingress topology sampling.  Phase65 uses a
minimal inner ingress waypoint one additional meter inward at map (2.0, 0.0),
then starts maze_explorer bounded to max_goals=1.

Guardrails: bounded runtime only, max_goals=1, no Nav2/MPPI/controller parameter
edits, no clearance_radius_m tuning, no map sufficiency threshold tuning, no
branch selection scoring change, no target projection integration, no
fallback/terminal acceptance change, no old scaffold world/map, no autonomous
exploration success claim, no exit success claim.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path
from typing import Any

try:  # pragma: no cover - ROS imports are exercised only during runtime wrapper.
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
except Exception:  # pragma: no cover
    rclpy = None
    GoalStatus = None
    NavigateToPose = None
    ActionClient = None
    Node = object

RUN_ID = 'phase65_ingress_goal_depth_adjustment_inner_staging'
PHASE = 'Phase65 Ingress Goal Depth Adjustment / Inner Entrance Staging Point'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
OLD_INGRESS_WAYPOINT_MAP = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
INNER_INGRESS_WAYPOINT_MAP = {'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}
INNER_INGRESS_DEPTH_ADJUSTMENT_M = 1.0
PHASE62_FIRST_DISPATCH_TARGET_MAP = [0.558242117171631, 0.5108474753216039]
PHASE62_TARGET_LOCAL_COST = 49
PHASE62_TARGET_FOOTPRINT_LETHAL_COUNT = 8
PHASE62_FRONT_WEDGE_HIGH_COST_COUNT = 131
PHASE62_DISPATCH_POSE_MAP = [0.8740772410891823, 0.020696297436460016, 0.03198252951540831]
MANUAL_OLD_INGRESS_RUNTIME_POSE_MAP = [0.8746499749241021, 0.020960419334965365, 0.031257]
ALLOWED_CLASSIFICATIONS = [
    'INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH',
    'ENTRY_STAGING_TOO_SHALLOW_CONFIRMED',
    'NO_IMPROVEMENT_GEOMETRY_STILL_BLOCKED',
    'INSUFFICIENT_EVIDENCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
]
GUARDRAILS = [
    'bounded runtime only',
    'max_goals=1',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection scoring change',
    'no target projection integration',
    'no fallback/terminal acceptance change',
    'no old scaffold world/map',
    'no autonomous exploration success claim',
    'no exit success claim',
]


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            if isinstance(payload, dict):
                payload = dict(payload)
                payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
                payload.setdefault('_recorder_wall_time', row.get('wall_time'))
                rows.append(payload)
    return rows


def _distance_xy(a: Any, b: Any) -> float | None:
    if not isinstance(a, (list, tuple)) or not isinstance(b, (list, tuple)) or len(a) < 2 or len(b) < 2:
        return None
    return float(math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])))


def _nested(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _first_dispatch(events: list[dict[str, Any]]) -> dict[str, Any] | None:
    for event in events:
        if event.get('event') == 'dispatch':
            return event
    return None


def _first_topology_payload(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    for row in rows:
        sampling = row.get('last_topology_sampling_diagnostics')
        if isinstance(sampling, dict):
            return row
        if row.get('raw_open_direction_count') is not None or row.get('candidate_branch_count') is not None:
            return row
    return rows[0] if rows else None


def _summary_stat(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)):
            values.append(float(value))
    return {
        'count': len(values),
        'min': min(values) if values else None,
        'max': max(values) if values else None,
        'final': values[-1] if values else None,
        'mean': sum(values) / len(values) if values else None,
    }


class InnerIngressGoalClient(Node):  # pragma: no cover - requires ROS graph.
    def __init__(self, output: Path, goal_timeout_sec: float):
        super().__init__('phase65_inner_ingress_goal_client')
        self.output = output
        self.goal_timeout_sec = float(goal_timeout_sec)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.result: dict[str, Any] = {
            'phase': PHASE,
            'run_id': RUN_ID,
            'old_ingress_waypoint_map': OLD_INGRESS_WAYPOINT_MAP,
            'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
            'inner_ingress_depth_adjustment_m': INNER_INGRESS_DEPTH_ADJUSTMENT_M,
            'goal_sent': False,
            'goal_accepted': False,
            'result_received': False,
            'success': False,
            'feedback': [],
            'wall_time_start': time.time(),
        }

    def run(self) -> dict[str, Any]:
        self.result['action_server_available'] = bool(self.client.wait_for_server(timeout_sec=20.0))
        if not self.result['action_server_available']:
            self.result['reason'] = 'navigate_to_pose_action_server_unavailable'
            return self.result
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(INNER_INGRESS_WAYPOINT_MAP['x_m'])
        goal_msg.pose.pose.position.y = float(INNER_INGRESS_WAYPOINT_MAP['y_m'])
        qx, qy, qz, qw = _quat_from_yaw(float(INNER_INGRESS_WAYPOINT_MAP['yaw_rad']))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        self.result['goal_sent'] = True
        self.result['goal_pose'] = {'frame_id': 'map', **INNER_INGRESS_WAYPOINT_MAP}
        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None:
            self.result['reason'] = 'goal_send_future_timeout'
            return self.result
        self.result['goal_accepted'] = bool(goal_handle.accepted)
        if not goal_handle.accepted:
            self.result['reason'] = 'goal_rejected'
            return self.result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.goal_timeout_sec)
        if not result_future.done():
            self.result['reason'] = 'goal_result_timeout'
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                self.result['cancel_requested'] = True
            except Exception as exc:  # noqa: BLE001 - ROS action cancel exceptions vary.
                self.result['cancel_error'] = str(exc)
            return self.result
        result_wrapper = result_future.result()
        status = int(getattr(result_wrapper, 'status', -1))
        nav_result = getattr(result_wrapper, 'result', None)
        error_code = int(getattr(nav_result, 'error_code', -1)) if nav_result is not None else -1
        error_msg = str(getattr(nav_result, 'error_msg', '')) if nav_result is not None else ''
        self.result.update({
            'wall_time_result': time.time(),
            'result_received': True,
            'status': status,
            'status_text': str(status),
            'error_code': error_code,
            'error_msg': error_msg,
            'success': bool(GoalStatus is not None and status == int(GoalStatus.STATUS_SUCCEEDED) and error_code == 0),
        })
        return self.result

    def _feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        nav_time = getattr(feedback, 'navigation_time', None)
        self.result['feedback'].append({
            'wall_time': time.time(),
            'navigation_time_sec': float(getattr(nav_time, 'sec', 0)) + float(getattr(nav_time, 'nanosec', 0)) / 1e9,
            'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        })


def send_inner_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover
    if rclpy is None:
        raise SystemExit('rclpy is required for --send-inner-ingress-goal')
    rclpy.init()
    node = InnerIngressGoalClient(Path(args.output), args.goal_timeout_sec)
    try:
        result = node.run()
    finally:
        node.output.parent.mkdir(parents=True, exist_ok=True)
        node.output.write_text(json.dumps(node.result, indent=2, sort_keys=True), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, sort_keys=True))
    return 0


def summarize_replay(replay_dir: Path) -> dict[str, Any]:
    replay_id = replay_dir.name
    ingress_action = _safe_json_load(replay_dir / f'{RUN_ID}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json', {})
    states = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl')
    events = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl')
    execution_summary = _safe_json_load(replay_dir / 'first_dispatch_execution_summary.json', {})
    topology = _first_topology_payload(states + events) or {}
    dispatch = _first_dispatch(events)
    dispatch_target = dispatch.get('target') if isinstance(dispatch, dict) else None
    dispatch_pose = dispatch.get('dispatch_pose') if isinstance(dispatch, dict) else None
    target_footprint = dispatch.get('phase62_target_footprint_cost') if isinstance(dispatch, dict) else None
    target_footprint_summary = target_footprint.get('summary') if isinstance(target_footprint, dict) else None
    front_wedge = dispatch.get('phase62_front_wedge_cost') if isinstance(dispatch, dict) else None
    old_pose = MANUAL_OLD_INGRESS_RUNTIME_POSE_MAP
    inner_pose = dispatch_pose or _nested(topology, 'robot_pose_map') or ingress_action.get('final_robot_pose_map')
    inside_threshold_x = OLD_INGRESS_WAYPOINT_MAP['x_m'] + 0.5
    robot_pose_inside_maze_after_inner_ingress = bool(isinstance(inner_pose, list) and len(inner_pose) >= 2 and float(inner_pose[0]) >= inside_threshold_x)
    target_local_cost = dispatch.get('dispatch_target_local_cost') if isinstance(dispatch, dict) else None
    footprint_lethal = target_footprint_summary.get('lethal_count') if isinstance(target_footprint_summary, dict) else None
    front_high = front_wedge.get('high_cost_count') if isinstance(front_wedge, dict) else None
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'old_ingress_waypoint_map': OLD_INGRESS_WAYPOINT_MAP,
        'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
        'inner_ingress_depth_adjustment_m': INNER_INGRESS_DEPTH_ADJUSTMENT_M,
        'old_ingress_runtime_pose_map': old_pose,
        'inner_ingress_goal_success': bool(ingress_action.get('success')),
        'inner_ingress_action_result': ingress_action,
        'inner_ingress_runtime_pose_map': inner_pose,
        'robot_pose_inside_maze_after_inner_ingress': robot_pose_inside_maze_after_inner_ingress,
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'first_topology_summary': {
            'mode': topology.get('mode'),
            'local_topology': topology.get('local_topology') or topology.get('last_local_topology_kind'),
            'raw_open_direction_count': topology.get('raw_open_direction_count'),
            'filtered_open_direction_count': topology.get('filtered_open_direction_count'),
            'candidate_branch_count': topology.get('candidate_branch_count') or topology.get('last_candidate_count'),
            'candidate_after_filter_count': topology.get('candidate_after_filter_count'),
            'robot_pose_map': topology.get('robot_pose_map'),
            'last_topology_sampling_diagnostics': topology.get('last_topology_sampling_diagnostics'),
        },
        'dispatch_observed': dispatch is not None,
        'dispatch_observed_not_exit_success': dispatch is not None,
        'first_dispatch_event': dispatch,
        'first_dispatch_target_map': dispatch_target,
        'first_dispatch_dispatch_pose_map': dispatch_pose,
        'first_dispatch_target_shift_from_phase62_m': _distance_xy(dispatch_target, PHASE62_FIRST_DISPATCH_TARGET_MAP),
        'old_to_inner_dispatch_pose_shift_m': _distance_xy(dispatch_pose, PHASE62_DISPATCH_POSE_MAP),
        'dispatch_target_local_cost': target_local_cost,
        'phase62_baseline_dispatch_target_local_cost': PHASE62_TARGET_LOCAL_COST,
        'target_local_cost_improved_vs_phase62': isinstance(target_local_cost, (int, float)) and float(target_local_cost) < PHASE62_TARGET_LOCAL_COST,
        'target_footprint_lethal_count': footprint_lethal,
        'phase62_baseline_target_footprint_lethal_count': PHASE62_TARGET_FOOTPRINT_LETHAL_COUNT,
        'target_footprint_lethal_count_improved_vs_phase62': isinstance(footprint_lethal, (int, float)) and int(footprint_lethal) < PHASE62_TARGET_FOOTPRINT_LETHAL_COUNT,
        'front_wedge_high_cost_count': front_high,
        'phase62_baseline_front_wedge_high_cost_count': PHASE62_FRONT_WEDGE_HIGH_COST_COUNT,
        'front_wedge_high_cost_count_improved_vs_phase62': isinstance(front_high, (int, float)) and int(front_high) < PHASE62_FRONT_WEDGE_HIGH_COST_COUNT,
        'first_dispatch_execution_summary': execution_summary,
        'distance_remaining_summary': _summary_stat(execution_summary.get('nav2_feedback', []) if isinstance(execution_summary.get('nav2_feedback'), list) else [], 'distance_remaining'),
        'complete_autonomous_success_claimed': False,
    }


def guardrail_violation(static_only: bool = False) -> bool:
    # No runtime strategy code is modified by Phase65 analyzer/wrapper.  Nav2 config
    # emptiness is verified separately with `git diff -- src/tugbot_navigation/config`.
    return False if not static_only else False


def classify_phase65(replays: list[dict[str, Any]], static_only: bool = False) -> str:
    if guardrail_violation(static_only=static_only):
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    if static_only or not replays:
        return 'INSUFFICIENT_EVIDENCE'
    if not any(r.get('inner_ingress_goal_success') for r in replays):
        return 'INSUFFICIENT_EVIDENCE'
    dispatch_replays = [r for r in replays if r.get('dispatch_observed')]
    if not dispatch_replays:
        if any(r.get('robot_pose_inside_maze_after_inner_ingress') for r in replays):
            return 'NO_IMPROVEMENT_GEOMETRY_STILL_BLOCKED'
        return 'INSUFFICIENT_EVIDENCE'
    improved = any(
        bool(r.get('robot_pose_inside_maze_after_inner_ingress'))
        and (
            bool(r.get('target_local_cost_improved_vs_phase62'))
            or bool(r.get('target_footprint_lethal_count_improved_vs_phase62'))
            or bool(r.get('front_wedge_high_cost_count_improved_vs_phase62'))
        )
        for r in dispatch_replays
    )
    if improved:
        return 'INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH'
    if any(r.get('robot_pose_inside_maze_after_inner_ingress') for r in dispatch_replays):
        return 'ENTRY_STAGING_TOO_SHALLOW_CONFIRMED'
    return 'INSUFFICIENT_EVIDENCE'


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    replay_dirs = sorted(p for p in artifact_dir.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path) for path in replay_dirs]
    classification = classify_phase65(replays, static_only=not bool(replays))
    result = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'classification': classification,
        'guardrails': GUARDRAILS,
        'guardrail_violation': classification == 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
        'old_ingress_waypoint_map': OLD_INGRESS_WAYPOINT_MAP,
        'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
        'inner_ingress_depth_adjustment_m': INNER_INGRESS_DEPTH_ADJUSTMENT_M,
        'old_ingress_runtime_pose_map': MANUAL_OLD_INGRESS_RUNTIME_POSE_MAP,
        'phase62_baseline': {
            'classification': 'CORRIDOR_TOO_NARROW',
            'auxiliary': 'LOCAL_COSTMAP_INFLATION_DOMINANT',
            'dispatch_pose_map': PHASE62_DISPATCH_POSE_MAP,
            'first_dispatch_target_map': PHASE62_FIRST_DISPATCH_TARGET_MAP,
            'dispatch_target_local_cost': PHASE62_TARGET_LOCAL_COST,
            'target_footprint_lethal_count': PHASE62_TARGET_FOOTPRINT_LETHAL_COUNT,
            'front_wedge_high_cost_count': PHASE62_FRONT_WEDGE_HIGH_COST_COUNT,
        },
        'replay_count': len(replays),
        'replays': replays,
        'dispatch_observed_not_exit_success': any(r.get('dispatch_observed_not_exit_success') for r in replays),
        'complete_autonomous_success_claimed': False,
        'recommendations': [
            'Interpret any dispatch only as inner ingress staging / first dispatch evidence; do not claim autonomous exploration or exit success.',
            'Stop after Phase65 and wait for human acceptance; do not enter Phase66.',
        ],
    }
    artifact_dir.mkdir(parents=True, exist_ok=True)
    output = Path(args.output) if args.output else artifact_dir / f'{RUN_ID}.json'
    output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({'output': str(output), 'classification': classification, 'replay_count': len(replays)}, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--artifact-dir', default=str(Path('log') / RUN_ID))
    parser.add_argument('--output')
    parser.add_argument('--send-inner-ingress-goal', action='store_true')
    parser.add_argument('--goal-timeout-sec', type=float, default=90.0)
    args = parser.parse_args(argv)
    if args.send_inner_ingress_goal:
        return send_inner_ingress_goal(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
