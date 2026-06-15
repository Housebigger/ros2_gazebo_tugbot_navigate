#!/usr/bin/env python3
"""Phase125 first exploration goal execution result smoke.

Scope guardrails:
- Reuse the Phase120 readiness/preflight/explicit inner-ingress goal chain.
- Reuse the Phase122 handoff gate and require handoff_allowed before starting maze_explorer.
- Start maze_explorer with max_goals:=1 only after handoff is ready.
- Allow exactly one current-algorithm first goal_kind=explore dispatch.
- Unlike Phase124, do not stop at action acceptance; wait for the bounded Nav2 result for that first goal.
- Stop immediately after success/abort/cancel/timeout/reject/no-valid-goal; do not allow a second exploration goal.
- Do not hand-specify Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goals.
- Do not tune Nav2/MPPI/controller/config or change exploration strategy.
- This is not autonomous exploration success and not exit success.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any

PHASE = 'Phase125'
MODE = 'first_exploration_goal_execution_result_smoke'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP = 'FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP'
FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL'
FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL'
FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL'
FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL'
NO_VALID_FIRST_GOAL_FAIL_CLOSED = 'NO_VALID_FIRST_GOAL_FAIL_CLOSED'
HANDOFF_NOT_READY_NO_EXPLORATION = 'HANDOFF_NOT_READY_NO_EXPLORATION'
CLASSIFICATIONS = [
    FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP,
    FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL,
    FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL,
    FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL,
    FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL,
    NO_VALID_FIRST_GOAL_FAIL_CLOSED,
    HANDOFF_NOT_READY_NO_EXPLORATION,
]

REQUIRED_RESULT_FIELDS = [
    'maze_explorer_start_allowed',
    'maze_explorer_started',
    'maze_explorer_max_goals',
    'exploration_goal_dispatched',
    'first_goal',
    'goal_event_count',
    'dispatch_event_count',
    'second_goal_dispatched',
    'goal_events',
    'explorer_states',
    'nav2_feedback',
    'action_status_samples',
    'local_costmap_samples',
    'scan',
    'map',
    'odom',
    'tf',
    'costmap_freshness',
    'scan_freshness',
    'tf_freshness',
    'stop_reason',
    'process',
]


def _now_wall() -> float:
    return time.time()


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _load_phase124_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase124_first_exploration_goal_dispatch_smoke.py'
    spec = importlib.util.spec_from_file_location('phase124_runner_for_phase125', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase124 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_p124 = _load_phase124_runner()
_phase122 = _p124._phase122


def _status_label(status: Any) -> str | None:
    return _p124._status_label(status)


def _goal_pose_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p124._goal_pose_from_event(event)


def _candidate_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p124._candidate_from_event(event)


def _frontier_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p124._frontier_evidence_from_event(event)


def _topology_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p124._topology_evidence_from_event(event)


def _goal_event_matches_first(dispatch: dict[str, Any], event: dict[str, Any]) -> bool:
    return _p124._goal_event_matches_first(dispatch, event)


def _stamp_dict(stamp: Any) -> dict[str, int] | None:
    return _p124._stamp_dict(stamp)


def _duration_msg_to_sec(msg: Any) -> float | None:
    return _p124._duration_msg_to_sec(msg)


def _runtime_tf_sample(buffer: Any, node: Any) -> dict[str, Any]:
    return _p124._runtime_tf_sample(buffer, node)


def _freshness_from_sample(sample: dict[str, Any], *, max_age_sec: float, required_seen_key: str | None = None) -> dict[str, Any]:
    now = _now_wall()
    received = _float(sample.get('received_wall_time_sec'))
    age = None if received is None else max(0.0, now - received)
    seen = bool(sample.get(required_seen_key)) if required_seen_key else bool(sample)
    fresh = bool(seen and age is not None and age <= max_age_sec)
    return {'fresh': fresh, 'seen': seen, 'age_sec': age, 'max_allowed_age_sec': max_age_sec}


def _costmap_freshness(local_costmap_samples: list[dict[str, Any]], max_age_sec: float = 5.0) -> dict[str, Any]:
    latest = local_costmap_samples[-1] if local_costmap_samples else {}
    received = _float(latest.get('received_wall_time_sec'))
    age = None if received is None else max(0.0, _now_wall() - received)
    return {
        'fresh': bool(latest and age is not None and age <= max_age_sec),
        'local_costmap_available': bool(latest),
        'local_costmap_age_sec': age,
        'max_allowed_age_sec': max_age_sec,
    }


def _robot_pose_from_tf(tf: dict[str, Any]) -> dict[str, Any]:
    trans = _dict(tf.get('translation'))
    x = _float(trans.get('x'))
    y = _float(trans.get('y'))
    return {
        'pose_available': x is not None and y is not None,
        'frame_id': tf.get('frame_id', 'map'),
        'x': x,
        'y': y,
        'yaw': None,
        'sample_wall_time_sec': tf.get('received_wall_time_sec'),
        'source': 'tf_map_base_link',
    }


def _distance_to_goal(robot_pose: dict[str, Any], goal_pose: dict[str, Any]) -> dict[str, Any]:
    rx = _float(robot_pose.get('x'))
    ry = _float(robot_pose.get('y'))
    gx = _float(goal_pose.get('x'))
    gy = _float(goal_pose.get('y'))
    if rx is None or ry is None or gx is None or gy is None:
        return {'available': False, 'meters': None}
    return {'available': True, 'meters': math.hypot(rx - gx, ry - gy)}


def initial_first_goal_result_record(*, max_goals: int = 1) -> dict[str, Any]:
    return {
        'maze_explorer_start_allowed': False,
        'maze_explorer_started': False,
        'maze_explorer_max_goals': int(max_goals),
        'exploration_goal_dispatched': False,
        'first_goal': {
            'candidate': {},
            'pose': {},
            'frame_id': None,
            'goal_kind': None,
            'selection_reason': None,
            'frontier_evidence': {},
            'topology_evidence': {},
            'dispatch_wall_time_sec': None,
            'accepted': False,
            'rejected': False,
            'nav2_feedback_timeline': [],
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
            'robot_pose_at_result': {'pose_available': False},
            'distance_to_first_goal': {'available': False, 'meters': None},
        },
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'second_goal_dispatched': False,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'local_costmap_samples': [],
        'scan': {},
        'map': {},
        'odom': {},
        'tf': {},
        'costmap_freshness': {'fresh': False},
        'scan_freshness': {'fresh': False},
        'tf_freshness': {'fresh': False},
        'observation_timed_out': False,
        'stop_reason': None,
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
        },
    }


def _result_status_from_event(event: dict[str, Any]) -> str | None:
    status_label = _status_label(event.get('result_status'))
    if status_label in (None, 'UNKNOWN'):
        raw_event = str(event.get('event', '')).lower()
        if raw_event == 'success':
            return 'SUCCEEDED'
        if raw_event == 'timeout':
            return 'TIMEOUT'
    return status_label


def _latest_result_event(dispatch: dict[str, Any], goal_events: list[dict[str, Any]]) -> dict[str, Any] | None:
    terminal_events = [
        event for event in goal_events
        if str(event.get('event', '')).lower() in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result'}
        and _goal_event_matches_first(dispatch, event)
    ]
    return terminal_events[-1] if terminal_events else None


def first_goal_result_from_observation(*, goal_events: list[dict[str, Any]], explorer_states: list[dict[str, Any]], action_status_samples: list[dict[str, Any]], evidence_samples: dict[str, Any], process_record: dict[str, Any], dispatch_started_wall_time_sec: float | None, observation_timed_out: bool) -> dict[str, Any]:
    record = initial_first_goal_result_record(max_goals=1)
    record['maze_explorer_start_allowed'] = True
    record['maze_explorer_started'] = True
    record['process'].update(process_record)
    record['goal_events'] = goal_events[-80:]
    record['explorer_states'] = explorer_states[-80:]
    record['goal_event_count'] = len(goal_events)
    dispatch_events = [event for event in goal_events if str(event.get('event', '')).lower() == 'dispatch']
    record['dispatch_event_count'] = len(dispatch_events)
    record['second_goal_dispatched'] = len(dispatch_events) > 1
    record['exploration_goal_dispatched'] = bool(dispatch_events)
    record['observation_timed_out'] = bool(observation_timed_out)

    local_costmap_samples = _list(evidence_samples.get('local_costmap_samples'))
    scan = _dict(evidence_samples.get('scan'))
    map_msg = _dict(evidence_samples.get('map'))
    odom = _dict(evidence_samples.get('odom'))
    tf = _dict(evidence_samples.get('tf'))
    nav2_feedback = _list(evidence_samples.get('nav2_feedback'))
    evidence_action_status = _list(evidence_samples.get('action_status_samples'))
    if evidence_action_status and not action_status_samples:
        action_status_samples = evidence_action_status
    record['local_costmap_samples'] = local_costmap_samples[-20:]
    record['scan'] = scan
    record['map'] = map_msg
    record['odom'] = odom
    record['tf'] = tf
    record['nav2_feedback'] = nav2_feedback[-80:]
    record['action_status_samples'] = action_status_samples[-80:]
    record['costmap_freshness'] = _costmap_freshness(local_costmap_samples)
    record['scan_freshness'] = _freshness_from_sample(scan, max_age_sec=2.0, required_seen_key='seen')
    record['tf_freshness'] = {'fresh': _bool(tf.get('fresh')), 'received_wall_time_sec': tf.get('received_wall_time_sec')}

    if not dispatch_events:
        record['stop_reason'] = 'no_valid_first_goal_fail_closed'
        return record

    dispatch = dispatch_events[0]
    goal_kind = dispatch.get('goal_kind')
    pose = _goal_pose_from_event(dispatch)
    dispatch_wall = dispatch.get('received_wall_time_sec', dispatch_started_wall_time_sec)
    result_event = _latest_result_event(dispatch, goal_events)
    non_explore = goal_kind != 'explore'
    accepted = False
    rejected = False
    timeout = False
    status_label: str | None = None
    abort_text: str | None = None
    stop_reason: str | None = None

    if non_explore:
        rejected = True
        status_label = 'REJECTED_NON_EXPLORE_GOAL_KIND'
        abort_text = f'first dispatch goal_kind={goal_kind!r} is not explore'
        stop_reason = 'first_dispatch_not_explore_diagnostic_stop'
    elif record['second_goal_dispatched']:
        rejected = True
        status_label = 'REJECTED_SECOND_GOAL_DISPATCHED'
        abort_text = 'second exploration goal dispatch observed inside Phase125 bounded first-result smoke'
        stop_reason = 'second_goal_dispatched_diagnostic_stop'
    elif result_event is not None:
        raw_event = str(result_event.get('event', '')).lower()
        status_label = _result_status_from_event(result_event)
        abort_text = result_event.get('result_reason')
        if raw_event == 'success' or status_label == 'SUCCEEDED':
            accepted = True
            status_label = 'SUCCEEDED'
            stop_reason = 'first_explore_goal_result_succeeded_stop'
        elif raw_event == 'timeout' or status_label == 'TIMEOUT':
            accepted = True
            timeout = True
            status_label = 'TIMEOUT'
            abort_text = abort_text or 'bounded_observation_timeout_after_first_dispatch'
            stop_reason = 'first_explore_goal_result_timeout_stop'
        elif status_label == 'CANCELED':
            accepted = True
            rejected = True
            stop_reason = 'first_explore_goal_result_canceled_stop'
        elif status_label == 'ABORTED':
            accepted = True
            rejected = True
            stop_reason = 'first_explore_goal_result_aborted_stop'
        else:
            accepted = True
            rejected = True
            status_label = status_label or 'REJECTED_OR_FAILED'
            abort_text = abort_text or 'first explore goal failed or was rejected'
            stop_reason = 'first_explore_goal_rejected_or_failed_stop'
    elif observation_timed_out:
        accepted = True
        timeout = True
        status_label = 'TIMEOUT'
        abort_text = 'bounded_observation_timeout_waiting_for_first_goal_result'
        stop_reason = 'first_explore_goal_result_timeout_stop'
    else:
        rejected = True
        status_label = 'REJECTED_OR_NOT_CONFIRMED'
        abort_text = 'first explore dispatch observed but no Nav2 result was confirmed inside bounded window'
        stop_reason = 'first_explore_goal_result_not_confirmed_stop'

    robot_pose = _robot_pose_from_tf(tf)
    distance = _distance_to_goal(robot_pose, pose)
    record['first_goal'] = {
        'candidate': _candidate_from_event(dispatch),
        'pose': pose,
        'frame_id': pose.get('frame_id'),
        'goal_kind': goal_kind,
        'selection_reason': dispatch.get('selected_due_to_context') or dispatch.get('centerline_refinement_reason') or dispatch.get('local_topology') or 'maze_explorer_current_algorithm_first_dispatch',
        'frontier_evidence': _frontier_evidence_from_event(dispatch),
        'topology_evidence': _topology_evidence_from_event(dispatch),
        'dispatch_wall_time_sec': dispatch_wall,
        'accepted': bool(accepted),
        'rejected': bool(rejected),
        'nav2_feedback_timeline': nav2_feedback[-80:],
        'result_status_label': status_label,
        'abort_text': abort_text,
        'timeout': bool(timeout),
        'robot_pose_at_result': robot_pose,
        'distance_to_first_goal': distance,
    }
    record['stop_reason'] = stop_reason
    return record


def handoff_ready(handoff_artifact: dict[str, Any]) -> bool:
    return _phase122.handoff_gate_allowed(handoff_artifact)


def classify_phase125(handoff_artifact: dict[str, Any], first_goal_record: dict[str, Any]) -> str:
    if not handoff_ready(handoff_artifact):
        return HANDOFF_NOT_READY_NO_EXPLORATION
    first = _dict(first_goal_record.get('first_goal'))
    dispatch_count = _int(first_goal_record.get('dispatch_event_count'))
    max_goals = _int(first_goal_record.get('maze_explorer_max_goals'), -1)
    if dispatch_count == 0:
        return NO_VALID_FIRST_GOAL_FAIL_CLOSED
    if max_goals != 1 or dispatch_count > 1 or _bool(first_goal_record.get('second_goal_dispatched')):
        return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL
    if first.get('goal_kind') != 'explore':
        return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL
    if _bool(first.get('timeout')) or first.get('result_status_label') == 'TIMEOUT':
        return FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
    status = first.get('result_status_label')
    if status == 'SUCCEEDED' and _bool(first.get('accepted')) and not _bool(first.get('rejected')):
        return FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP
    if status == 'ABORTED':
        return FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL
    if status == 'CANCELED':
        return FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL
    return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL


def build_phase125_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], first_goal_result_record: dict[str, Any], classification: str) -> dict[str, Any]:
    dispatch_count = _int(first_goal_result_record.get('dispatch_event_count'))
    first = _dict(first_goal_result_record.get('first_goal'))
    return {
        'phase': PHASE,
        'run_id': run_id,
        'mode': MODE,
        'created_wall_time_sec': _now_wall(),
        'phase120_source_artifact': phase120_artifact.get('artifact_path') or phase120_artifact.get('run_id'),
        'phase120_ingress_artifact': phase120_artifact,
        'handoff_artifact': handoff_artifact,
        'handoff_allowed': bool(handoff_ready(handoff_artifact)),
        'first_goal_result_artifact': first_goal_result_record,
        'classification': classification,
        'maze_explorer_start_allowed': _bool(first_goal_result_record.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(first_goal_result_record.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int(first_goal_result_record.get('maze_explorer_max_goals'), -1),
        'exploration_goal_dispatched': _bool(first_goal_result_record.get('exploration_goal_dispatched')),
        'goal_event_count': _int(first_goal_result_record.get('goal_event_count')),
        'dispatch_event_count': dispatch_count,
        'second_goal_dispatched': _bool(first_goal_result_record.get('second_goal_dispatched')),
        'first_goal': first,
        'stop_reason': first_goal_result_record.get('stop_reason'),
        'guardrails': {
            'handoff_ready_required': True,
            'same_run_phase120_ingress_required': True,
            'only_explicit_inner_ingress_goal_sent': _phase122.phase120_ingress_success(phase120_artifact) if phase120_artifact else True,
            'max_goals_one': _int(first_goal_result_record.get('maze_explorer_max_goals'), -1) == 1,
            'first_exploration_goal_only': dispatch_count <= 1,
            'second_goal_dispatched_false': not _bool(first_goal_result_record.get('second_goal_dispatched')),
            'manual_goal1_forbidden': True,
            'only_current_algorithm_goal_source': True,
            'fallback_terminal_exit_goal_forbidden_as_success': True,
            'no_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_manual_goal': True,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
            'nav2_config_changed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
        },
        'notes': [
            'Phase125 validates only the first current-algorithm exploration goal Nav2 result boundary.',
            'A succeeded first-goal result is not autonomous exploration success and not exit success.',
            'No second exploration goal or manual Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal is allowed.',
        ],
    }


def run_maze_explorer_first_goal_result(*, duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    import tf2_ros
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    try:
        from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    except Exception:
        NavigateToPose_FeedbackMessage = None

    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1', '-p', 'exploration_rate_hz:=1.0'
    ]
    record = initial_first_goal_result_record(max_goals=1)
    record['maze_explorer_start_allowed'] = True
    record['process']['command'] = command
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stderr_path.parent.mkdir(parents=True, exist_ok=True)
    events: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    action_status_samples: list[dict[str, Any]] = []
    nav2_feedback: list[dict[str, Any]] = []
    local_costmap_samples: list[dict[str, Any]] = []
    evidence: dict[str, Any] = {'scan': {}, 'map': {}, 'odom': {}, 'tf': {}}
    proc: subprocess.Popen[str] | None = None
    first_dispatch_wall: float | None = None
    terminal_seen = False
    second_dispatch_seen = False

    def _event_cb(msg: String) -> None:
        nonlocal first_dispatch_wall, terminal_seen, second_dispatch_seen
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        events.append(payload)
        if str(payload.get('event', '')).lower() == 'dispatch':
            if first_dispatch_wall is None:
                first_dispatch_wall = payload['received_wall_time_sec']
            else:
                second_dispatch_seen = True
        if first_dispatch_wall is not None and str(payload.get('event', '')).lower() in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result'}:
            terminal_seen = True

    def _state_cb(msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        states.append(payload)

    def _scan_cb(msg: LaserScan) -> None:
        evidence['scan'] = {
            'seen': True,
            'frame_id': msg.header.frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'finite_count': sum(1 for value in msg.ranges if math.isfinite(value)),
        }

    def _map_cb(msg: OccupancyGrid) -> None:
        evidence['map'] = {'seen': True, 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'width': int(msg.info.width), 'height': int(msg.info.height)}

    def _local_cb(msg: OccupancyGrid) -> None:
        local_costmap_samples.append({'seen': True, 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'width': int(msg.info.width), 'height': int(msg.info.height)})
        del local_costmap_samples[:-40]

    def _odom_cb(msg: Odometry) -> None:
        evidence['odom'] = {'seen': True, 'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'pose': {'x': float(msg.pose.pose.position.x), 'y': float(msg.pose.pose.position.y), 'z': float(msg.pose.pose.position.z)}}

    def _status_cb(msg: GoalStatusArray) -> None:
        statuses = [int(status.status) for status in msg.status_list]
        sample = {
            'received_wall_time_sec': _now_wall(),
            'statuses': statuses,
            'active_goal_count': sum(1 for status in statuses if status in (1, 2, 3)),
            'pending_goal_count': sum(1 for status in statuses if status == 1),
            'executing_goal_count': sum(1 for status in statuses if status == 2),
            'canceling_goal_count': sum(1 for status in statuses if status == 3),
            'succeeded_goal_count': sum(1 for status in statuses if status == 4),
            'canceled_goal_count': sum(1 for status in statuses if status == 5),
            'aborted_goal_count': sum(1 for status in statuses if status == 6),
        }
        action_status_samples.append(sample)
        del action_status_samples[:-100]

    def _feedback_cb(msg: Any) -> None:
        feedback = getattr(msg, 'feedback', None)
        if feedback is None:
            return
        nav2_feedback.append({
            'received_wall_time_sec': _now_wall(),
            'navigation_time_sec': _duration_msg_to_sec(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_sec': _duration_msg_to_sec(getattr(feedback, 'estimated_time_remaining', None)),
            'distance_remaining': _float(getattr(feedback, 'distance_remaining', None)),
            'number_of_recoveries': _int(getattr(feedback, 'number_of_recoveries', 0)),
        })
        del nav2_feedback[:-100]

    rclpy.init(args=None)
    node = rclpy.create_node('phase125_first_exploration_goal_result_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    costmap_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
    subs: list[Any] = [
        node.create_subscription(String, GOAL_EVENTS_TOPIC, _event_cb, 10),
        node.create_subscription(String, EXPLORER_STATE_TOPIC, _state_cb, 10),
        node.create_subscription(GoalStatusArray, f'{ACTION_NAME}/_action/status', _status_cb, costmap_qos),
        node.create_subscription(OccupancyGrid, '/map', _map_cb, costmap_qos),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', _local_cb, costmap_qos),
        node.create_subscription(LaserScan, '/scan', _scan_cb, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', _odom_cb, qos_profile_sensor_data),
    ]
    if NavigateToPose_FeedbackMessage is not None:
        subs.append(node.create_subscription(NavigateToPose_FeedbackMessage, f'{ACTION_NAME}/_action/feedback', _feedback_cb, 10))

    start = _now_wall()
    observation_timed_out = False
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy(), start_new_session=True)
        record['maze_explorer_started'] = True
        record['process']['pid'] = proc.pid
        deadline = start + float(duration_sec)
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            evidence['tf'] = _runtime_tf_sample(tf_buffer, node)
            dispatch_events = [event for event in events if str(event.get('event', '')).lower() == 'dispatch']
            first_event = dispatch_events[0] if dispatch_events else None
            if second_dispatch_seen or len(dispatch_events) > 1:
                break
            if first_event is not None and first_event.get('goal_kind') != 'explore':
                break
            if terminal_seen:
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    evidence['tf'] = _runtime_tf_sample(tf_buffer, node)
                break
            if proc.poll() is not None:
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.1)
                break
        else:
            observation_timed_out = True
    except Exception as exc:
        record['process']['exception_text'] = str(exc)
        record['process']['traceback_tail'] = traceback.format_exc()[-2000:]
    finally:
        try:
            for sub in subs:
                node.destroy_subscription(sub)
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stdout = ''
        stderr = ''
        if proc is not None:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except Exception:
                    proc.terminate()
                try:
                    stdout, stderr = proc.communicate(timeout=5.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except Exception:
                        proc.kill()
                    stdout, stderr = proc.communicate(timeout=5.0)
            else:
                stdout, stderr = proc.communicate(timeout=5.0)
            record['process']['returncode'] = proc.returncode
        stdout_path.write_text(stdout or '', encoding='utf-8')
        stderr_path.write_text(stderr or '', encoding='utf-8')
        record['process']['stdout_path'] = str(stdout_path)
        record['process']['stderr_path'] = str(stderr_path)

    evidence['local_costmap_samples'] = local_costmap_samples
    evidence['nav2_feedback'] = nav2_feedback
    evidence['action_status_samples'] = action_status_samples
    return first_goal_result_from_observation(
        goal_events=events,
        explorer_states=states,
        action_status_samples=action_status_samples,
        evidence_samples=evidence,
        process_record=_dict(record.get('process')),
        dispatch_started_wall_time_sec=first_dispatch_wall,
        observation_timed_out=observation_timed_out,
    )


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = _p124.run_phase120_ingress_chain(args)
    if args.phase120_output:
        phase120_artifact['artifact_path'] = str(args.phase120_output)
    return phase120_artifact


def run_phase125(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = run_phase120_ingress_chain(args)
    handoff = _phase122.collect_handoff_runtime_evidence(
        phase120_artifact,
        collection_sec=args.handoff_collection_sec,
        pose_tolerance_m=args.pose_tolerance_m,
        yaw_tolerance_rad=args.yaw_tolerance_rad,
        max_costmap_age_sec=args.max_costmap_age_sec,
        max_scan_age_sec=args.max_scan_age_sec,
        max_tf_age_sec=args.max_tf_age_sec,
    )
    result_record = initial_first_goal_result_record(max_goals=1)
    if handoff_ready(handoff):
        result_record = run_maze_explorer_first_goal_result(
            duration_sec=args.first_goal_result_observation_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    classification = classify_phase125(handoff, result_record)
    artifact = build_phase125_artifact(
        run_id=args.run_id,
        phase120_artifact=phase120_artifact,
        handoff_artifact=handoff,
        first_goal_result_record=result_record,
        classification=classification,
    )
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase120-output', type=Path, required=True)
    parser.add_argument('--preflight-output', type=Path, required=True)
    parser.add_argument('--phase120-artifact', type=Path, help='test/debug only: reuse existing Phase120 artifact instead of dispatching')
    parser.add_argument('--run-id', default='phase125_first_exploration_goal_execution_result_smoke')
    parser.add_argument('--launch-log-path')
    parser.add_argument('--preflight-timeout-sec', type=float, default=35.0)
    parser.add_argument('--tf-stability-window-sec', type=float, default=2.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--startup-grace-sec', type=float, default=2.0)
    parser.add_argument('--action-server-wait-sec', type=float, default=10.0)
    parser.add_argument('--bounded-goal-result-wait-sec', type=float, default=45.0)
    parser.add_argument('--readiness-wait-timeout-sec', type=float, default=60.0)
    parser.add_argument('--readiness-wait-sample-period-sec', type=float, default=1.0)
    parser.add_argument('--handoff-collection-sec', type=float, default=4.0)
    parser.add_argument('--pose-tolerance-m', type=float, default=0.35)
    parser.add_argument('--yaw-tolerance-rad', type=float, default=0.35)
    parser.add_argument('--max-costmap-age-sec', type=float, default=5.0)
    parser.add_argument('--max-scan-age-sec', type=float, default=2.0)
    parser.add_argument('--max-tf-age-sec', type=float, default=1.5)
    parser.add_argument('--first-goal-result-observation-sec', type=float, default=70.0)
    args = parser.parse_args()
    artifact = run_phase125(args)
    first = _dict(artifact.get('first_goal'))
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'goal_kind': first.get('goal_kind'),
        'accepted': first.get('accepted'),
        'rejected': first.get('rejected'),
        'timeout': first.get('timeout'),
        'result_status_label': first.get('result_status_label'),
        'dispatch_event_count': artifact.get('dispatch_event_count'),
        'second_goal_dispatched': artifact.get('second_goal_dispatched'),
        'stop_reason': artifact.get('stop_reason'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
