#!/usr/bin/env python3
"""Phase134 bounded corridor-alignment staging-contract smoke.

Scope guardrails:
- Reuse the Phase120 explicit inner-ingress goal chain: map,x=2.0,y=0.0,yaw=0.0.
- Reuse the Phase122 handoff gate before starting maze_explorer.
- Start maze_explorer with max_goals:=1 and observe only the first literal dispatch.
- If the first literal dispatch is goal_kind=corridor_alignment_staging, wait for a bounded staging result/timeout and stop.
- Do not attempt a second-step goal_kind=explore or any second exploration goal.
- Do not hand-specify Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goals.
- Do not tune Nav2/MPPI/controller/goal-checker/config or change exploration strategy.
- This smoke never claims autonomous exploration success or exit success.
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

PHASE = 'Phase134'
MODE = 'bounded_corridor_alignment_staging_smoke'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

CLASSIFICATIONS = [
    'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
    'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
    'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
]

FORBIDDEN_ACTIONS = {
    'second_step_goal_kind_explore': False,
    'second_exploration_goal': False,
    'manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': False,
    'nav2_mppi_controller_goal_checker_config_tuning': False,
    'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
    'direct_staging_disablement': False,
    'autonomous_or_exit_success_claim': False,
}

REQUIRED_RESULT_FIELDS = [
    'maze_explorer_start_allowed',
    'maze_explorer_started',
    'maze_explorer_max_goals',
    'first_literal_dispatch',
    'goal_event_count',
    'dispatch_event_count',
    'second_step_attempted',
    'second_goal_dispatched',
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


def _load_phase129_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase129_instrumented_first_goal_timeout_diagnosis.py'
    spec = importlib.util.spec_from_file_location('phase129_runner_for_phase134', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase129 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_p129 = _load_phase129_runner()
_phase122 = _p129._phase122
_p125 = _p129._p125


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


def _result_status_from_event(event: dict[str, Any]) -> str | None:
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


def initial_bounded_staging_record(*, max_goals: int = 1) -> dict[str, Any]:
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
            'is_first_exploration_goal': False,
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
            'dispatch_wall_time_sec': None,
            'result_wall_time_sec': None,
            'terminal_event': None,
            'second_step_attempted': False,
            'second_goal_dispatched': False,
        },
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'second_step_attempted': False,
        'second_goal_dispatched': False,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'cmd_vel_timeline': [],
        'odom_velocity_timeline': [],
        'observation_timed_out': False,
        'stop_reason': None,
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
            'traceback_tail': None,
        },
    }


def _pending_second_step_from_dispatch(dispatch: dict[str, Any]) -> dict[str, Any]:
    second_step = dispatch.get('second_step_forward_goal')
    staging = dispatch.get('goal_kind') == 'corridor_alignment_staging'
    inferred = bool(staging and second_step is not None and dispatch.get('staging_applied') is True)
    return {
        'available': inferred,
        'exists': inferred,
        'source': 'inferred_from_staging_dispatch_second_step_forward_goal' if inferred else 'not_serialized_or_not_applicable',
        'runtime_serialized': False,
    }


def _first_literal_from_dispatch(dispatch: dict[str, Any], events: list[dict[str, Any]], *, observation_timed_out: bool) -> dict[str, Any]:
    goal_kind = dispatch.get('goal_kind')
    seq = dispatch.get('goal_sequence')
    terminals = _terminal_events_for_sequence(events, seq)
    terminal = terminals[-1] if terminals else None
    status_label = _result_status_from_event(terminal) if terminal else None
    raw_event = str(_dict(terminal).get('event', '')).lower()
    timeout = bool(raw_event == 'timeout' or status_label == 'TIMEOUT' or (observation_timed_out and goal_kind == 'corridor_alignment_staging'))
    rejected = bool(raw_event in {'failure', 'stale_result'} or status_label in {'ABORTED', 'CANCELED', 'UNKNOWN', 'REJECTED'})
    accepted = bool(dispatch and (terminal is not None or goal_kind in {'explore', 'corridor_alignment_staging'}))
    if timeout:
        status_label = 'TIMEOUT'
    elif goal_kind == 'explore' and status_label is None:
        status_label = 'ACCEPTED'
    elif goal_kind == 'corridor_alignment_staging' and status_label is None and not observation_timed_out:
        status_label = 'ACCEPTED' if accepted else None

    staging_pose = _dict(dispatch.get('staging_goal_pose'))
    first = initial_bounded_staging_record()['first_literal_dispatch']
    first.update({
        'available': bool(dispatch),
        'goal_kind': goal_kind,
        'goal_sequence': seq,
        'pose': _goal_pose_from_event(dispatch),
        'target': dispatch.get('target'),
        'original_target': dispatch.get('original_target'),
        'refined_target': dispatch.get('refined_target'),
        'staging_target': dispatch.get('staging_goal_pose') or dispatch.get('target'),
        'staging_applied': bool(dispatch.get('staging_applied', False)),
        'two_step_stage_dispatch_requested': bool(dispatch.get('two_step_stage_dispatch_requested', False)),
        'staging_reason': dispatch.get('staging_reason'),
        'staging_reject_reason': dispatch.get('staging_reject_reason'),
        'lateral_residual_before_m': staging_pose.get('lateral_residual_before_m'),
        'lateral_residual_after_m': staging_pose.get('lateral_residual_after_m'),
        'front_wedge_risk': dispatch.get('phase62_front_wedge_cost') or dispatch.get('front_wedge_risk'),
        'staging_executability_check': dispatch.get('staging_executability_check'),
        'pending_corridor_alignment_second_step': _pending_second_step_from_dispatch(dispatch),
        'second_step_forward_goal': dispatch.get('second_step_forward_goal'),
        'accepted': accepted and not rejected,
        'rejected': rejected,
        'is_first_exploration_goal': goal_kind == 'explore',
        'result_status_label': status_label,
        'abort_text': _dict(terminal).get('result_reason') if terminal else ('bounded_observation_timeout_waiting_for_staging_result' if timeout else None),
        'timeout': timeout,
        'dispatch_wall_time_sec': dispatch.get('received_wall_time_sec'),
        'result_wall_time_sec': _dict(terminal).get('received_wall_time_sec'),
        'terminal_event': terminal,
    })
    return first


def _staging_fields_are_coherent(first: dict[str, Any]) -> bool:
    return bool(
        first.get('goal_kind') == 'corridor_alignment_staging'
        and first.get('staging_applied') is True
        and first.get('two_step_stage_dispatch_requested') is True
        and first.get('original_target') is not None
        and first.get('staging_target') is not None
        and first.get('staging_reason')
        and first.get('staging_executability_check') is not None
        and first.get('lateral_residual_before_m') is not None
        and first.get('lateral_residual_after_m') is not None
    )


def classify_bounded_record(record: dict[str, Any]) -> str:
    first = _dict(record.get('first_literal_dispatch'))
    if not first.get('available'):
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
    if _bool(record.get('second_goal_dispatched')) or _bool(record.get('second_step_attempted')):
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
    goal_kind = first.get('goal_kind')
    if goal_kind == 'explore':
        return 'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP'
    if goal_kind != 'corridor_alignment_staging':
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
    if _bool(first.get('timeout')) or first.get('result_status_label') == 'TIMEOUT':
        return 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL'
    if _bool(first.get('rejected')) or first.get('result_status_label') in {'REJECTED', 'ABORTED', 'CANCELED', 'UNKNOWN'}:
        return 'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL'
    if not _staging_fields_are_coherent(first):
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
    if first.get('result_status_label') == 'SUCCEEDED' or _bool(first.get('accepted')):
        return 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP'
    return 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP'


def finalize_record_from_events(record: dict[str, Any]) -> dict[str, Any]:
    events = [event for event in _list(record.get('goal_events')) if isinstance(event, dict)]
    dispatches = [event for event in events if str(event.get('event', '')).lower() == 'dispatch']
    record['goal_event_count'] = len(events)
    record['dispatch_event_count'] = len(dispatches)
    record['second_goal_dispatched'] = len(dispatches) > 1
    record['second_step_attempted'] = any(event.get('goal_kind') == 'explore' for event in dispatches[1:])
    if dispatches:
        previous_first = _dict(record.get('first_literal_dispatch'))
        first = _first_literal_from_dispatch(dispatches[0], events, observation_timed_out=_bool(record.get('observation_timed_out')))
        # Preserve explicit diagnostic overrides injected by focused tests or a
        # future monitor path when no terminal event carried the same field.
        if first.get('terminal_event') is None:
            for key in ('accepted', 'rejected', 'result_status_label', 'abort_text', 'timeout'):
                if previous_first.get(key) not in (None, False):
                    first[key] = previous_first.get(key)
        first['second_step_attempted'] = bool(record['second_step_attempted'])
        first['second_goal_dispatched'] = bool(record['second_goal_dispatched'])
        record['first_literal_dispatch'] = first
        if record['second_goal_dispatched']:
            record['stop_reason'] = 'second_literal_dispatch_guard_violation_stop'
        elif record['second_step_attempted']:
            record['stop_reason'] = 'second_step_explore_guard_violation_stop'
        elif first.get('goal_kind') == 'explore':
            record['stop_reason'] = 'first_literal_explore_dispatch_observed_stop'
        elif first.get('goal_kind') == 'corridor_alignment_staging' and first.get('timeout'):
            record['stop_reason'] = 'first_literal_staging_timeout_bounded_stop'
        elif first.get('goal_kind') == 'corridor_alignment_staging' and first.get('terminal_event') is not None:
            record['stop_reason'] = 'first_literal_staging_terminal_result_bounded_stop'
        elif first.get('goal_kind') == 'corridor_alignment_staging':
            record['stop_reason'] = 'first_literal_staging_observed_no_second_step_stop'
        else:
            record['stop_reason'] = 'first_literal_dispatch_kind_ambiguous_stop'
    else:
        record['stop_reason'] = record.get('stop_reason') or 'no_first_literal_dispatch_fail_closed'
    return record


def run_maze_explorer_bounded_staging(*, duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String

    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1', '-p', 'exploration_rate_hz:=1.0'
    ]
    record = initial_bounded_staging_record(max_goals=1)
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
    terminal_seen = False
    second_dispatch_seen = False

    def _event_cb(msg: String) -> None:
        nonlocal first_dispatch_seen, first_goal_kind, terminal_seen, second_dispatch_seen
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
            else:
                second_dispatch_seen = True
        if first_dispatch_seen and str(payload.get('event', '')).lower() in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result', 'cancel'}:
            terminal_seen = True

    def _state_cb(msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        states.append(payload)
        del states[:-200]

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
        del action_status_samples[:-200]

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

    rclpy.init(args=None)
    node = rclpy.create_node('phase134_bounded_staging_contract_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
    subs = [
        node.create_subscription(String, GOAL_EVENTS_TOPIC, _event_cb, 10),
        node.create_subscription(String, EXPLORER_STATE_TOPIC, _state_cb, 10),
        node.create_subscription(GoalStatusArray, f'{ACTION_NAME}/_action/status', _status_cb, qos),
        node.create_subscription(NavigateToPose_FeedbackMessage, f'{ACTION_NAME}/_action/feedback', _feedback_cb, 10),
        node.create_subscription(Twist, '/cmd_vel', _cmd_cb, 10),
        node.create_subscription(Odometry, '/odom', _odom_cb, qos_profile_sensor_data),
    ]
    start = _now_wall()
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy(), start_new_session=True)
        record['maze_explorer_started'] = True
        record['process']['pid'] = proc.pid
        deadline = start + float(duration_sec)
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if second_dispatch_seen:
                break
            if first_dispatch_seen and first_goal_kind == 'explore':
                # Phase134 only observes/classifies the first literal explore dispatch.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
            if first_dispatch_seen and first_goal_kind == 'corridor_alignment_staging' and terminal_seen:
                # Stop before the node can enter any post-staging second-step path.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
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

    record['goal_events'] = events[-160:]
    record['explorer_states'] = states[-160:]
    record['nav2_feedback'] = nav2_feedback[-160:]
    record['action_status_samples'] = action_status_samples[-160:]
    record['cmd_vel_timeline'] = cmd_vel[-160:]
    record['odom_velocity_timeline'] = odom_vel[-160:]
    return finalize_record_from_events(record)


def build_phase134_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], bounded_record: dict[str, Any]) -> dict[str, Any]:
    bounded = finalize_record_from_events(dict(bounded_record))
    classification = classify_bounded_record(bounded)
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
        'bounded_staging_record': bounded,
        'maze_explorer_start_allowed': _bool(bounded.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(bounded.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int(bounded.get('maze_explorer_max_goals'), -1),
        'goal_event_count': _int(bounded.get('goal_event_count')),
        'dispatch_event_count': _int(bounded.get('dispatch_event_count')),
        'first_literal_dispatch': _dict(bounded.get('first_literal_dispatch')),
        'second_step_attempted': _bool(bounded.get('second_step_attempted')),
        'second_goal_dispatched': _bool(bounded.get('second_goal_dispatched')),
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
            'max_goals_one': _int(bounded.get('maze_explorer_max_goals'), -1) == 1,
            'first_literal_dispatch_only': _int(bounded.get('dispatch_event_count')) <= 1,
            'second_step_attempted_false': not _bool(bounded.get('second_step_attempted')),
            'second_goal_dispatched_false': not _bool(bounded.get('second_goal_dispatched')),
            'manual_goal1_forbidden': True,
            'no_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_manual_goal': True,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
            'direct_staging_disablement': False,
            'nav2_config_changed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
            'no_autonomous_success_claim': True,
            'no_exit_success_claim': True,
        },
        'forbidden_actions': dict(FORBIDDEN_ACTIONS),
        'claims': {
            'staging_is_exploration_success': False,
            'staging_is_exit_success': False,
            'mixed_with_phase127_timeout_local_cost_replay': False,
            'autonomous_exploration_success': False,
            'exit_success': False,
        },
        'notes': [
            'Phase134 is a bounded staging-contract smoke under the visible stack.',
            'If first literal dispatch is corridor_alignment_staging, the run stops after bounded staging outcome/timeout.',
            'A staging accepted/succeeded result is not autonomous exploration success and not exit success.',
            'The smoke does not authorize tuning, repair, staging disablement, or Phase127 timeout reinterpretation.',
        ],
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    artifact = _p129.run_phase120_ingress_chain(args)
    if args.phase120_output:
        artifact['artifact_path'] = str(args.phase120_output)
    return artifact


def run_phase134(args: argparse.Namespace) -> dict[str, Any]:
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
    record = initial_bounded_staging_record(max_goals=1)
    if _p125.handoff_ready(handoff):
        record = run_maze_explorer_bounded_staging(
            duration_sec=args.staging_result_observation_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    artifact = build_phase134_artifact(
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
    parser.add_argument('--run-id', default='phase134_bounded_corridor_alignment_staging_smoke')
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
    parser.add_argument('--staging-result-observation-sec', type=float, default=75.0)
    args = parser.parse_args()
    artifact = run_phase134(args)
    first = _dict(artifact.get('first_literal_dispatch'))
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'first_literal_goal_kind': first.get('goal_kind'),
        'staging_applied': first.get('staging_applied'),
        'two_step_stage_dispatch_requested': first.get('two_step_stage_dispatch_requested'),
        'accepted': first.get('accepted'),
        'rejected': first.get('rejected'),
        'timeout': first.get('timeout'),
        'result_status_label': first.get('result_status_label'),
        'second_step_attempted': artifact.get('second_step_attempted'),
        'second_goal_dispatched': artifact.get('second_goal_dispatched'),
        'stop_reason': artifact.get('stop_reason'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
