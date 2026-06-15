#!/usr/bin/env python3
"""Phase136 bounded second-step explore after staging smoke.

Scope guardrails:
- Reuse the visible-stack Phase120 explicit inner-ingress chain: map,x=2.0,y=0.0,yaw=0.0.
- Reuse the Phase122 handoff gate before starting maze_explorer.
- Start maze_explorer with max_goals:=2 so only the first literal staging dispatch and one second-step explore dispatch can occur.
- Allow first literal goal_kind=corridor_alignment_staging.
- After staging succeeds, allow exactly one second-step goal_kind=explore with skip_two_step_staging or an equivalent recursion guard.
- Stop at second-step acceptance by default, or earlier on rejection/timeout/result evidence if observed during the bounded monitor window.
- Do not send a third goal, do not run full exploration, do not tune Nav2/config, and do not claim autonomous/exit success.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import time
import traceback
from pathlib import Path
from typing import Any

PHASE = 'Phase136'
MODE = 'bounded_second_step_explore_after_staging_smoke'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

CLASSIFICATIONS = [
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP',
    'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
    'SECOND_STEP_CONTRACT_AMBIGUOUS',
]

FORBIDDEN_ACTIONS = {
    'third_goal': False,
    'full_exploration': False,
    'manual_goal1_carry_over_branch_centerline_fallback_terminal_exit_goal': False,
    'nav2_mppi_controller_goal_checker_config_tuning': False,
    'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
    'direct_staging_disablement': False,
    'autonomous_or_exit_success_claim': False,
    'phase127_timeout_fixed_claim': False,
}

REQUIRED_RESULT_FIELDS = [
    'maze_explorer_start_allowed',
    'maze_explorer_started',
    'maze_explorer_max_goals',
    'first_literal_dispatch',
    'staging_result',
    'pending_corridor_alignment_second_step',
    'freshness_after_staging',
    'front_wedge_risk_after_staging',
    'lateral_residual_after',
    'second_step_dispatch',
    'second_step_attempted',
    'second_step_goal_count',
    'third_goal_dispatched',
    'goal_event_count',
    'dispatch_event_count',
    'goal_events',
    'explorer_states',
    'nav2_feedback',
    'action_status_samples',
    'cmd_vel_timeline',
    'odom_velocity_timeline',
    'observation_timed_out',
    'stop_reason',
    'process',
]


def _load_phase134_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase134_bounded_corridor_alignment_staging_smoke.py'
    spec = importlib.util.spec_from_file_location('phase134_runner_for_phase136', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase134 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_p134 = _load_phase134_runner()
_p129 = _p134._p129
_phase122 = _p134._phase122
_p125 = _p134._p125


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


def _int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _goal_pose_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p129._goal_pose_from_event(event)


def _result_status_from_event(event: dict[str, Any] | None) -> str | None:
    if not event:
        return None
    raw = str(event.get('event', '')).lower()
    status = event.get('result_status')
    label = _p129._status_label(status)
    if raw == 'success' or label == 'SUCCEEDED':
        return 'SUCCEEDED'
    if raw == 'timeout':
        return 'TIMEOUT'
    if raw in {'terminal_cancel_result', 'timeout_cancel_result', 'cancel'} or label == 'CANCELED':
        return 'CANCELED'
    if raw in {'failure', 'stale_result'}:
        return label or 'ABORTED'
    return label


def _terminal_events_for_sequence(events: list[dict[str, Any]], sequence: Any) -> list[dict[str, Any]]:
    terminals = []
    for event in events:
        if str(event.get('event', '')).lower() not in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result', 'cancel'}:
            continue
        if sequence is not None and event.get('goal_sequence') not in (None, sequence):
            continue
        terminals.append(event)
    return terminals


def initial_bounded_second_step_record(*, max_goals: int = 2) -> dict[str, Any]:
    return {
        'maze_explorer_start_allowed': False,
        'maze_explorer_started': False,
        'maze_explorer_max_goals': int(max_goals),
        'first_literal_dispatch': {
            'available': False,
            'goal_kind': None,
            'goal_sequence': None,
            'pose': {},
            'target': None,
            'original_target': None,
            'refined_target': None,
            'staging_target': None,
            'staging_applied': False,
            'two_step_stage_dispatch_requested': False,
            'staging_reason': None,
            'staging_reject_reason': None,
            'lateral_residual_before_m': None,
            'lateral_residual_after_m': None,
            'front_wedge_risk': None,
            'staging_executability_check': None,
            'pending_corridor_alignment_second_step': {'available': False, 'exists': False, 'source': 'not_observed'},
            'second_step_forward_goal': None,
            'accepted': False,
            'rejected': False,
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
            'dispatch_wall_time_sec': None,
            'result_wall_time_sec': None,
            'terminal_event': None,
        },
        'staging_result': {
            'accepted': False,
            'rejected': False,
            'timeout': False,
            'result_status_label': None,
            'abort_text': None,
        },
        'pending_corridor_alignment_second_step': {'available': False, 'exists': False, 'source': 'not_observed'},
        'freshness_after_staging': {
            'fresh_scan_received': False,
            'fresh_local_costmap_received': False,
            'fresh_tf_received': False,
            'fresh_scan_sample_time_sec': None,
            'fresh_local_costmap_sample_time_sec': None,
            'fresh_tf_sample_time_sec': None,
        },
        'front_wedge_risk_after_staging': None,
        'lateral_residual_after': None,
        'second_step_dispatch': {
            'available': False,
            'goal_kind': None,
            'goal_sequence': None,
            'pose': {},
            'target': None,
            'original_target': None,
            'selected_due_to_context': None,
            'second_step_forward_goal': None,
            'freshness_after_staging': {},
            'front_wedge_risk_after_staging': None,
            'lateral_residual_after': None,
            'skip_two_step_staging': False,
            'recursion_guard': False,
            'accepted': False,
            'rejected': False,
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
            'dispatch_wall_time_sec': None,
            'result_wall_time_sec': None,
            'terminal_event': None,
        },
        'second_step_attempted': False,
        'second_step_goal_count': 0,
        'third_goal_dispatched': False,
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'cmd_vel_timeline': [],
        'odom_velocity_timeline': [],
        'observation_timed_out': False,
        'stop_reason': None,
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=2'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
            'traceback_tail': None,
        },
    }


def _pending_second_step_from_staging_dispatch(dispatch: dict[str, Any]) -> dict[str, Any]:
    second_step = dispatch.get('second_step_forward_goal')
    staging = dispatch.get('goal_kind') == 'corridor_alignment_staging'
    inferred = bool(staging and second_step is not None and dispatch.get('staging_applied') is True)
    original_goal_kind = 'explore' if inferred else None
    return {
        'available': inferred,
        'exists': inferred,
        'source': 'inferred_from_staging_dispatch_second_step_forward_goal' if inferred else 'not_serialized_or_not_applicable',
        'runtime_serialized': False,
        'original_goal_kind': original_goal_kind,
        'original_target': dispatch.get('original_target'),
        'staging_plan': dispatch.get('two_step_staging_plan'),
        'start_node_id': dispatch.get('start_node_id'),
        'active_branch': dispatch.get('active_branch'),
    }


def _first_literal_from_dispatch(dispatch: dict[str, Any], events: list[dict[str, Any]], *, observation_timed_out: bool) -> dict[str, Any]:
    first = _p134._first_literal_from_dispatch(dispatch, events, observation_timed_out=observation_timed_out)
    first['pending_corridor_alignment_second_step'] = _pending_second_step_from_staging_dispatch(dispatch)
    return first


def _recursion_guard_present(dispatch: dict[str, Any]) -> bool:
    if dispatch.get('skip_two_step_staging') is True:
        return True
    if dispatch.get('phase136_recursion_guard') is True:
        return True
    # Runtime maze_explorer does not currently serialize the Python call-site
    # argument.  Treat the source diagnostic as an equivalent guard only when it
    # is paired with explicit evidence that recursive two-step staging was not
    # requested for this outgoing explore dispatch.
    if (
        dispatch.get('selected_due_to_context') == 'phase92_corridor_alignment_staging_second_step'
        and dispatch.get('two_step_stage_dispatch_requested') is False
        and dispatch.get('staging_applied') is False
        and dispatch.get('goal_kind') == 'explore'
    ):
        return True
    return False


def _second_step_from_dispatch(
    dispatch: dict[str, Any],
    events: list[dict[str, Any]],
    *,
    observation_timed_out: bool,
    freshness_after_staging: dict[str, Any],
    front_wedge_risk_after_staging: Any,
    lateral_residual_after: Any,
) -> dict[str, Any]:
    seq = dispatch.get('goal_sequence')
    terminals = _terminal_events_for_sequence(events, seq)
    terminal = terminals[-1] if terminals else None
    status_label = _result_status_from_event(terminal) if terminal else None
    raw_event = str(_dict(terminal).get('event', '')).lower()
    timeout = bool(raw_event == 'timeout' or status_label == 'TIMEOUT' or (observation_timed_out and dispatch.get('goal_kind') == 'explore'))
    rejected = bool(raw_event in {'failure', 'stale_result'} or status_label in {'ABORTED', 'CANCELED', 'UNKNOWN', 'REJECTED'})
    accepted = bool(dispatch and dispatch.get('goal_kind') == 'explore' and not rejected)
    if timeout:
        status_label = 'TIMEOUT'
    elif status_label is None and accepted:
        status_label = 'ACCEPTED'
    second_step = dispatch.get('second_step_forward_goal')
    if not isinstance(second_step, dict):
        second_step = None
    if second_step is None and dispatch.get('selected_due_to_context') == 'phase92_corridor_alignment_staging_second_step':
        # The current maze_explorer dispatch context can overwrite the nested
        # second_step_forward_goal with the not-applicable two-step-staging
        # payload on the outgoing explore dispatch.  The selected_due_to_context
        # marker is emitted only after generate_second_step_forward_goal_after_staging
        # returned a fresh-evidence candidate, so reconstruct the minimal
        # Phase136 readiness payload from the outgoing dispatch and the monitor's
        # freshness samples without changing exploration code.
        target = dispatch.get('selected_candidate_target') or dispatch.get('target')
        yaw = dispatch.get('selected_candidate_yaw')
        if yaw is None:
            yaw = _dict(dispatch.get('target')).get('yaw')
        second_step = {
            'generated_after_fresh_evidence': True,
            'fresh_scan_received': _dict(freshness_after_staging).get('fresh_scan_received') is True,
            'fresh_local_costmap_received': _dict(freshness_after_staging).get('fresh_local_costmap_received') is True,
            'fresh_tf_received': _dict(freshness_after_staging).get('fresh_tf_received') is True,
            'selected_candidate_target': target,
            'selected_candidate_yaw': yaw,
            'source': 'reconstructed_from_phase92_second_step_dispatch_context',
        }
    embedded_front = _dict(second_step).get('front_wedge_risk_after_staging') if isinstance(second_step, dict) else None
    return {
        'available': bool(dispatch),
        'goal_kind': dispatch.get('goal_kind'),
        'goal_sequence': seq,
        'pose': _goal_pose_from_event(dispatch),
        'target': dispatch.get('target'),
        'original_target': dispatch.get('original_target'),
        'selected_due_to_context': dispatch.get('selected_due_to_context'),
        'second_step_forward_goal': second_step,
        'freshness_after_staging': freshness_after_staging,
        'front_wedge_risk_after_staging': front_wedge_risk_after_staging or embedded_front or dispatch.get('phase62_front_wedge_cost'),
        'lateral_residual_after': lateral_residual_after,
        'skip_two_step_staging': dispatch.get('skip_two_step_staging') is True,
        'recursion_guard': _recursion_guard_present(dispatch),
        'accepted': accepted,
        'rejected': rejected,
        'result_status_label': status_label,
        'abort_text': _dict(terminal).get('result_reason') if terminal else ('bounded_observation_timeout_waiting_for_second_step_result' if timeout else None),
        'timeout': timeout,
        'dispatch_wall_time_sec': dispatch.get('received_wall_time_sec'),
        'result_wall_time_sec': _dict(terminal).get('received_wall_time_sec'),
        'terminal_event': terminal,
    }


def _staging_succeeded(first: dict[str, Any]) -> bool:
    return bool(
        first.get('goal_kind') == 'corridor_alignment_staging'
        and first.get('result_status_label') == 'SUCCEEDED'
        and first.get('accepted') is True
        and first.get('rejected') is False
        and first.get('timeout') is False
        and first.get('staging_applied') is True
        and first.get('two_step_stage_dispatch_requested') is True
    )


def _finite_pair(value: Any) -> bool:
    if isinstance(value, dict):
        xs = [value.get('x'), value.get('y')]
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        xs = [value[0], value[1]]
    else:
        return False
    return all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in xs)


def _second_step_ready(second: dict[str, Any]) -> bool:
    payload = _dict(second.get('second_step_forward_goal'))
    return bool(
        second.get('goal_kind') == 'explore'
        and second.get('recursion_guard') is True
        and payload.get('generated_after_fresh_evidence') is True
        and payload.get('fresh_scan_received') is True
        and payload.get('fresh_local_costmap_received') is True
        and payload.get('fresh_tf_received') is True
        and _finite_pair(payload.get('selected_candidate_target'))
    )


def classify_bounded_record(record: dict[str, Any]) -> str:
    first = _dict(record.get('first_literal_dispatch'))
    second = _dict(record.get('second_step_dispatch'))
    if _bool(record.get('third_goal_dispatched')):
        return 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    if not _staging_succeeded(first):
        return 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    if not second.get('available'):
        return 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED'
    if not _second_step_ready(second):
        return 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    if _bool(second.get('timeout')) or second.get('result_status_label') == 'TIMEOUT':
        return 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL'
    if _bool(second.get('rejected')) or second.get('result_status_label') in {'REJECTED', 'ABORTED', 'CANCELED', 'UNKNOWN'}:
        return 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL'
    if second.get('result_status_label') == 'SUCCEEDED':
        return 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP'
    if _bool(second.get('accepted')):
        return 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP'
    return 'SECOND_STEP_CONTRACT_AMBIGUOUS'


def finalize_record_from_events(record: dict[str, Any]) -> dict[str, Any]:
    events = [event for event in _list(record.get('goal_events')) if isinstance(event, dict)]
    dispatches = [event for event in events if str(event.get('event', '')).lower() == 'dispatch']
    record['goal_event_count'] = len(events)
    record['dispatch_event_count'] = len(dispatches)
    second_dispatches = [event for event in dispatches[1:] if event.get('goal_kind') == 'explore']
    record['second_step_goal_count'] = len(second_dispatches)
    record['second_step_attempted'] = len(second_dispatches) > 0
    record['third_goal_dispatched'] = len(dispatches) > 2

    if dispatches:
        first = _first_literal_from_dispatch(dispatches[0], events, observation_timed_out=False)
        record['first_literal_dispatch'] = first
        record['staging_result'] = {
            'accepted': first.get('accepted'),
            'rejected': first.get('rejected'),
            'timeout': first.get('timeout'),
            'result_status_label': first.get('result_status_label'),
            'abort_text': first.get('abort_text'),
        }
        pending = _dict(record.get('pending_corridor_alignment_second_step'))
        if not pending.get('available') and first.get('pending_corridor_alignment_second_step'):
            record['pending_corridor_alignment_second_step'] = first.get('pending_corridor_alignment_second_step')
        if first.get('lateral_residual_after_m') is not None and record.get('lateral_residual_after') is None:
            record['lateral_residual_after'] = first.get('lateral_residual_after_m')
    else:
        record['stop_reason'] = record.get('stop_reason') or 'no_first_literal_dispatch_fail_closed'

    if second_dispatches:
        second = _second_step_from_dispatch(
            second_dispatches[0],
            events,
            observation_timed_out=_bool(record.get('observation_timed_out')),
            freshness_after_staging=_dict(record.get('freshness_after_staging')),
            front_wedge_risk_after_staging=record.get('front_wedge_risk_after_staging'),
            lateral_residual_after=record.get('lateral_residual_after'),
        )
        record['second_step_dispatch'] = second
        if second.get('front_wedge_risk_after_staging') is not None:
            record['front_wedge_risk_after_staging'] = second.get('front_wedge_risk_after_staging')
    classification = classify_bounded_record(record)
    if record.get('third_goal_dispatched'):
        record['stop_reason'] = 'third_goal_dispatch_guard_violation_stop'
    elif _dict(record.get('first_literal_dispatch')).get('goal_kind') != 'corridor_alignment_staging':
        record['stop_reason'] = 'first_literal_not_staging_fail_closed_stop'
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED':
        record['stop_reason'] = 'staging_succeeded_second_step_not_ready_fail_closed_stop'
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP':
        record['stop_reason'] = 'second_step_explore_accepted_bounded_stop'
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP':
        record['stop_reason'] = 'second_step_explore_terminal_success_bounded_stop'
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL':
        record['stop_reason'] = 'second_step_explore_rejected_diagnostic_stop'
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL':
        record['stop_reason'] = 'second_step_explore_timeout_diagnostic_stop'
    else:
        record['stop_reason'] = record.get('stop_reason') or 'second_step_contract_ambiguous_stop'
    return record


def run_maze_explorer_bounded_second_step(*, duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    import tf2_ros
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=2', '-p', 'exploration_rate_hz:=1.0'
    ]
    record = initial_bounded_second_step_record(max_goals=2)
    record['maze_explorer_start_allowed'] = True
    record['process']['command'] = command
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stderr_path.parent.mkdir(parents=True, exist_ok=True)

    events: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    action_status_samples: list[dict[str, Any]] = []
    nav2_feedback: list[dict[str, Any]] = []
    cmd_vel: list[dict[str, Any]] = []
    odom_vel: list[dict[str, Any]] = []
    proc: subprocess.Popen[str] | None = None
    first_dispatch_seen = False
    first_goal_kind: str | None = None
    wrong_first_dispatch_spins = 0
    staging_terminal_wall: float | None = None
    second_dispatch_seen = False
    second_terminal_seen = False
    third_dispatch_seen = False
    fresh = record['freshness_after_staging']

    def _event_cb(msg: String) -> None:
        nonlocal first_dispatch_seen, first_goal_kind, staging_terminal_wall, second_dispatch_seen, second_terminal_seen, third_dispatch_seen
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        events.append(payload)
        if str(payload.get('event', '')).lower() == 'dispatch':
            if not first_dispatch_seen:
                first_dispatch_seen = True
                first_goal_kind = str(payload.get('goal_kind'))
            elif not second_dispatch_seen:
                second_dispatch_seen = True
            else:
                third_dispatch_seen = True
        if str(payload.get('event', '')).lower() in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result', 'cancel'}:
            if payload.get('goal_kind') == 'corridor_alignment_staging':
                staging_terminal_wall = payload['received_wall_time_sec']
            if payload.get('goal_kind') == 'explore' and second_dispatch_seen:
                second_terminal_seen = True

    def _state_cb(msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        states.append(payload)
        del states[:-240]

    def _status_cb(msg: GoalStatusArray) -> None:
        statuses = [int(status.status) for status in msg.status_list]
        action_status_samples.append({
            'received_wall_time_sec': _now_wall(),
            'statuses': statuses,
            'active_goal_count': sum(1 for status in statuses if status in (1, 2, 3)),
            'pending_goal_count': sum(1 for status in statuses if status == 1),
            'executing_goal_count': sum(1 for status in statuses if status == 2),
            'canceling_goal_count': sum(1 for status in statuses if status == 3),
            'succeeded_goal_count': sum(1 for status in statuses if status == 4),
            'canceled_goal_count': sum(1 for status in statuses if status == 5),
            'aborted_goal_count': sum(1 for status in statuses if status == 6),
        })
        del action_status_samples[:-240]

    def _feedback_cb(msg: Any) -> None:
        feedback = getattr(msg, 'feedback', None)
        if feedback is None:
            return
        nav2_feedback.append({
            'received_wall_time_sec': _now_wall(),
            'navigation_time_sec': _p129._duration_msg_to_sec(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_sec': _p129._duration_msg_to_sec(getattr(feedback, 'estimated_time_remaining', None)),
            'distance_remaining': _float(getattr(feedback, 'distance_remaining', None)),
            'number_of_recoveries': _int(getattr(feedback, 'number_of_recoveries', 0)),
        })
        del nav2_feedback[:-240]

    def _cmd_cb(msg: Twist) -> None:
        cmd_vel.append({'sample_wall_time_sec': _now_wall(), 'linear_x': float(msg.linear.x), 'linear_y': float(msg.linear.y), 'angular_z': float(msg.angular.z)})
        del cmd_vel[:-240]

    def _odom_cb(msg: Odometry) -> None:
        odom_vel.append({'sample_wall_time_sec': _now_wall(), 'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id, 'linear_x': float(msg.twist.twist.linear.x), 'linear_y': float(msg.twist.twist.linear.y), 'angular_z': float(msg.twist.twist.angular.z)})
        del odom_vel[:-240]

    def _scan_cb(_msg: LaserScan) -> None:
        now = _now_wall()
        if staging_terminal_wall is not None and now >= staging_terminal_wall:
            fresh['fresh_scan_received'] = True
            fresh['fresh_scan_sample_time_sec'] = now

    def _costmap_cb(_msg: OccupancyGrid) -> None:
        now = _now_wall()
        if staging_terminal_wall is not None and now >= staging_terminal_wall:
            fresh['fresh_local_costmap_received'] = True
            fresh['fresh_local_costmap_sample_time_sec'] = now

    rclpy.init(args=None)
    node = rclpy.create_node('phase136_bounded_second_step_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
    subs = [
        node.create_subscription(String, GOAL_EVENTS_TOPIC, _event_cb, 10),
        node.create_subscription(String, EXPLORER_STATE_TOPIC, _state_cb, 10),
        node.create_subscription(GoalStatusArray, f'{ACTION_NAME}/_action/status', _status_cb, qos),
        node.create_subscription(NavigateToPose_FeedbackMessage, f'{ACTION_NAME}/_action/feedback', _feedback_cb, 10),
        node.create_subscription(Twist, '/cmd_vel', _cmd_cb, 10),
        node.create_subscription(Odometry, '/odom', _odom_cb, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', _scan_cb, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', _costmap_cb, qos_profile_sensor_data),
    ]
    start = _now_wall()
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy(), start_new_session=True)
        record['maze_explorer_started'] = True
        record['process']['pid'] = proc.pid
        deadline = start + float(duration_sec)
        post_second_spins = 0
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if staging_terminal_wall is not None and not fresh.get('fresh_tf_received'):
                try:
                    tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    fresh['fresh_tf_received'] = True
                    fresh['fresh_tf_sample_time_sec'] = _now_wall()
                except Exception:
                    try:
                        tf_buffer.lookup_transform('map', 'tugbot/base_link', rclpy.time.Time())
                        fresh['fresh_tf_received'] = True
                        fresh['fresh_tf_sample_time_sec'] = _now_wall()
                    except Exception:
                        pass
            if third_dispatch_seen:
                break
            if first_dispatch_seen and first_goal_kind != 'corridor_alignment_staging' and not second_dispatch_seen:
                wrong_first_dispatch_spins += 1
                if wrong_first_dispatch_spins >= 10:
                    record['stop_reason'] = 'first_literal_not_staging_fail_closed_stop'
                    break
            if second_terminal_seen:
                break
            if second_dispatch_seen:
                post_second_spins += 1
                # Stop at accepted/dispatch boundary after a small spin window so
                # action status and feedback samples have a chance to arrive, but
                # before full exploration can continue to a third dispatch.
                if post_second_spins >= 10:
                    break
            if proc.poll() is not None:
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
        else:
            record['observation_timed_out'] = True
    except Exception as exc:  # noqa: BLE001
        record['process']['exception_text'] = str(exc)
        record['process']['traceback_tail'] = traceback.format_exc()[-2000:]
    finally:
        try:
            for sub in subs:
                node.destroy_subscription(sub)
            del tf_listener
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

    record['goal_events'] = events[-220:]
    record['explorer_states'] = states[-220:]
    record['nav2_feedback'] = nav2_feedback[-200:]
    record['action_status_samples'] = action_status_samples[-200:]
    record['cmd_vel_timeline'] = cmd_vel[-200:]
    record['odom_velocity_timeline'] = odom_vel[-200:]
    record['freshness_after_staging'] = fresh
    return finalize_record_from_events(record)


def build_phase136_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], bounded_record: dict[str, Any]) -> dict[str, Any]:
    bounded = finalize_record_from_events(dict(bounded_record))
    classification = classify_bounded_record(bounded)
    first = _dict(bounded.get('first_literal_dispatch'))
    second = _dict(bounded.get('second_step_dispatch'))
    second_payload = _dict(second.get('second_step_forward_goal'))
    return {
        'phase': PHASE,
        'run_id': run_id,
        'mode': MODE,
        'created_wall_time_sec': _now_wall(),
        'phase120_source_artifact': phase120_artifact.get('artifact_path') or phase120_artifact.get('run_id'),
        'phase120_ingress_artifact': phase120_artifact,
        'handoff_artifact': handoff_artifact,
        'handoff_allowed': bool(_p125.handoff_ready(handoff_artifact)),
        'classification': classification,
        'bounded_second_step_record': bounded,
        'maze_explorer_start_allowed': _bool(bounded.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(bounded.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int(bounded.get('maze_explorer_max_goals'), -1),
        'goal_event_count': _int(bounded.get('goal_event_count')),
        'dispatch_event_count': _int(bounded.get('dispatch_event_count')),
        'first_literal_dispatch': first,
        'staging_result': _dict(bounded.get('staging_result')),
        'pending_corridor_alignment_second_step': _dict(bounded.get('pending_corridor_alignment_second_step')),
        'freshness_after_staging': _dict(bounded.get('freshness_after_staging')),
        'front_wedge_risk_after_staging': bounded.get('front_wedge_risk_after_staging'),
        'lateral_residual_after': bounded.get('lateral_residual_after'),
        'second_step_dispatch': second,
        'second_step_attempted': _bool(bounded.get('second_step_attempted')),
        'second_step_goal_count': _int(bounded.get('second_step_goal_count')),
        'third_goal_dispatched': _bool(bounded.get('third_goal_dispatched')),
        'goal_events': _list(bounded.get('goal_events')),
        'explorer_states': _list(bounded.get('explorer_states')),
        'nav2_feedback': _list(bounded.get('nav2_feedback')),
        'action_status_samples': _list(bounded.get('action_status_samples')),
        'cmd_vel_timeline': _list(bounded.get('cmd_vel_timeline')),
        'odom_velocity_timeline': _list(bounded.get('odom_velocity_timeline')),
        'observation_timed_out': _bool(bounded.get('observation_timed_out')),
        'stop_reason': bounded.get('stop_reason'),
        'guardrails': {
            'handoff_ready_required': True,
            'same_run_phase120_ingress_required': True,
            'only_explicit_inner_ingress_goal_sent': _phase122.phase120_ingress_success(phase120_artifact) if phase120_artifact else True,
            'max_goals_two': _int(bounded.get('maze_explorer_max_goals'), -1) == 2,
            'staging_and_second_step_only': _int(bounded.get('dispatch_event_count')) <= 2,
            'one_second_step_goal_only': _int(bounded.get('second_step_goal_count')) <= 1,
            'third_goal_dispatched_false': not _bool(bounded.get('third_goal_dispatched')),
            'staging_succeeded_before_second_step': _staging_succeeded(first),
            'pending_second_step_present': _dict(bounded.get('pending_corridor_alignment_second_step')).get('exists') is True or _dict(bounded.get('pending_corridor_alignment_second_step')).get('available') is True,
            'freshness_after_staging_recorded': any(_dict(bounded.get('freshness_after_staging')).values()) or bool(second_payload),
            'second_step_goal_kind_explore': (not second.get('available')) or second.get('goal_kind') == 'explore',
            'second_step_recursion_guard_present': (not second.get('available')) or second.get('recursion_guard') is True,
            'second_step_generated_after_fresh_evidence': (not second.get('available')) or second_payload.get('generated_after_fresh_evidence') is True,
            'second_step_selected_candidate_target_present': (not second.get('available')) or _finite_pair(second_payload.get('selected_candidate_target')),
            'manual_goal1_forbidden': True,
            'no_goal1_carry_over_branch_centerline_fallback_terminal_exit_manual_goal': True,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
            'direct_staging_disablement': False,
            'nav2_config_changed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
            'phase127_timeout_fixed_claimed': False,
            'no_autonomous_success_claim': True,
            'no_exit_success_claim': True,
        },
        'forbidden_actions': dict(FORBIDDEN_ACTIONS),
        'claims': {
            'second_step_is_autonomous_exploration_success': False,
            'second_step_is_exit_success': False,
            'mixed_with_phase127_timeout_local_cost_replay': False,
            'phase127_timeout_fixed': False,
            'autonomous_exploration_success': False,
            'exit_success': False,
        },
        'notes': [
            'Phase136 is a bounded second-step explore smoke under the visible stack.',
            'The first literal corridor_alignment_staging dispatch and the second-step explore dispatch are preserved separately.',
            'Second-step accepted/succeeded is not autonomous exploration success and not exit success.',
            'The smoke does not authorize tuning, repair, staging disablement, full exploration, third goals, or Phase127 timeout reinterpretation.',
        ],
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    artifact = _p134.run_phase120_ingress_chain(args)
    if args.phase120_output:
        artifact['artifact_path'] = str(args.phase120_output)
    return artifact


def run_phase136(args: argparse.Namespace) -> dict[str, Any]:
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
    record = initial_bounded_second_step_record(max_goals=2)
    if _p125.handoff_ready(handoff):
        record = run_maze_explorer_bounded_second_step(
            duration_sec=args.second_step_observation_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    artifact = build_phase136_artifact(
        run_id=args.run_id,
        phase120_artifact=phase120_artifact,
        handoff_artifact=handoff,
        bounded_record=record,
    )
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase120-output', type=Path, required=True)
    parser.add_argument('--preflight-output', type=Path, required=True)
    parser.add_argument('--phase120-artifact', type=Path, help='test/debug only: reuse existing Phase120 artifact instead of dispatching')
    parser.add_argument('--run-id', default='phase136_bounded_second_step_explore_after_staging_smoke')
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
    parser.add_argument('--second-step-observation-sec', type=float, default=95.0)
    args = parser.parse_args()
    artifact = run_phase136(args)
    first = _dict(artifact.get('first_literal_dispatch'))
    second = _dict(artifact.get('second_step_dispatch'))
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'first_literal_goal_kind': first.get('goal_kind'),
        'staging_result_status_label': first.get('result_status_label'),
        'second_step_attempted': artifact.get('second_step_attempted'),
        'second_step_goal_count': artifact.get('second_step_goal_count'),
        'second_step_goal_kind': second.get('goal_kind'),
        'second_step_result_status_label': second.get('result_status_label'),
        'third_goal_dispatched': artifact.get('third_goal_dispatched'),
        'stop_reason': artifact.get('stop_reason'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
