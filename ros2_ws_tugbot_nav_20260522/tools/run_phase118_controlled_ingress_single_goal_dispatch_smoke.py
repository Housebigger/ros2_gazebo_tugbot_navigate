#!/usr/bin/env python3
"""Phase118 controlled ingress single-goal dispatch smoke.

single_goal_dispatch_smoke_only

This runner performs a strict preflight-first smoke test and may dispatch only the
locked explicit inner-ingress NavigateToPose goal. It never starts maze_explorer,
never tunes Nav2/config, and never claims autonomous or exit success.

Required state rule: ingress_goal_sent=false before send; ingress_goal_sent=true only after accepted.
timeout/abort/cancel are diagnostic fail.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any

PHASE = 'Phase118'
MODE = 'single_goal_dispatch_smoke_only'
PHASE120_MODE = 'controlled_ingress_dispatch_with_managed_active_readiness_wait'
ACTION_NAME = '/navigate_to_pose'
MANAGED_ACTIVE_MARKER = 'Managed nodes are active'
REQUIRED_ACTIVE_NODES = ['controller_server', 'bt_navigator', 'planner_server', 'behavior_server']

READY_WAIT_TIMEOUT_NO_DISPATCH = 'READY_WAIT_TIMEOUT_NO_DISPATCH'
PREFLIGHT_FAILED_NO_DISPATCH = 'PREFLIGHT_FAILED_NO_DISPATCH'
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL = 'INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER = 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL'
INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE = 'INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE'

LOCKED_GOAL = {
    'frame_id': 'map',
    'x': 2.0,
    'y': 0.0,
    'yaw': 0.0,
}

REQUIRED_DISPATCH_FIELDS = [
    'dispatch_attempted',
    'preflight_pass_required',
    'preflight_failed_gates_required_empty',
    'preflight_reject_reason_required_absent',
    'goal_pose',
    'frame_id',
    'stamp',
    'x',
    'y',
    'yaw',
    'action_name',
    'action_server_ready',
    'action_server_ready_checked_at_wall_time',
    'send_time',
    'send_wall_time_sec',
    'send_ros_time_sec',
    'goal_response_wall_time_sec',
    'accepted',
    'rejected',
    'goal_handle_available',
    'result_wait_started_wall_time_sec',
    'bounded_goal_result_wait_sec',
    'result_received',
    'result_status',
    'result_status_label',
    'abort_text',
    'cancel_requested',
    'cancel_result',
    'exception_text',
    'traceback_tail',
    'ingress_goal_sent',
    'maze_explorer_started',
]
REQUIRED_READINESS_WAIT_FIELDS = [
    'enabled',
    'readiness_wait_start_wall_time_sec',
    'readiness_wait_end_wall_time_sec',
    'readiness_wait_elapsed_sec',
    'readiness_wait_timeout_sec',
    'marker_found',
    'marker_found_wall_time_sec',
    'multi_source_ready',
    'timed_out',
    'lifecycle_states',
    'action_server_ready',
    'samples',
]


def _now_wall() -> float:
    return time.time()


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _load_preflight_module() -> Any:
    path = Path(__file__).resolve().parent / 'phase105_inner_ingress_tf_controller_preflight.py'
    spec = importlib.util.spec_from_file_location('phase105_preflight_for_phase118', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import preflight module from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def locked_goal_pose(*, stamp: Any | None = None) -> dict[str, Any]:
    goal = dict(LOCKED_GOAL)
    goal['stamp'] = stamp
    return goal


def evaluate_dispatch_preconditions(preflight: dict[str, Any], *, action_server_ready: bool) -> dict[str, Any]:
    reasons: list[str] = []
    if not _bool(preflight.get('evaluated')):
        reasons.append('preflight_not_evaluated')
    if not _bool(preflight.get('passed')):
        reasons.append('preflight_not_passed')
    if _list(preflight.get('failed_gates')):
        reasons.append('preflight_failed_gates_non_empty')
    if preflight.get('ingress_preflight_reject_reason') not in (None, ''):
        reasons.append('preflight_reject_reason_present')
    if not bool(action_server_ready):
        reasons.append('action_server_not_ready')
    allowed = not reasons
    return {
        'allowed': allowed,
        'reasons': reasons,
        'classification': None if allowed else PREFLIGHT_FAILED_NO_DISPATCH,
        'dispatch_precondition_failed': not allowed,
    }


def initial_dispatch_record(*, bounded_goal_result_wait_sec: float) -> dict[str, Any]:
    # Keep required schema present even if preflight rejects and no dispatch happens.
    goal = locked_goal_pose(stamp=None)
    return {
        'dispatch_attempted': False,
        'preflight_pass_required': True,
        'preflight_failed_gates_required_empty': True,
        'preflight_reject_reason_required_absent': True,
        'goal_pose': goal,
        'frame_id': goal['frame_id'],
        'stamp': goal['stamp'],
        'x': goal['x'],
        'y': goal['y'],
        'yaw': goal['yaw'],
        'action_name': ACTION_NAME,
        'action_server_ready': False,
        'action_server_ready_checked_at_wall_time': None,
        'send_time': None,
        'send_wall_time_sec': None,
        'send_ros_time_sec': None,
        'goal_response_wall_time_sec': None,
        'accepted': False,
        'rejected': False,
        'goal_handle_available': False,
        'result_wait_started_wall_time_sec': None,
        'bounded_goal_result_wait_sec': float(bounded_goal_result_wait_sec),
        'result_received': False,
        'result_status': None,
        'result_status_label': None,
        'abort_text': None,
        'cancel_requested': False,
        'cancel_result': None,
        'exception_text': None,
        'traceback_tail': None,
        'ingress_goal_sent': False,
        'maze_explorer_started': False,
    }


def status_label(status: int | None) -> str | None:
    mapping = {
        0: 'UNKNOWN',
        1: 'ACCEPTED',
        2: 'EXECUTING',
        3: 'CANCELING',
        4: 'SUCCEEDED',
        5: 'CANCELED',
        6: 'ABORTED',
    }
    return mapping.get(status, str(status) if status is not None else None)


def initial_readiness_wait_record(*, timeout_sec: float) -> dict[str, Any]:
    return {
        'enabled': False,
        'readiness_wait_start_wall_time_sec': None,
        'readiness_wait_end_wall_time_sec': None,
        'readiness_wait_elapsed_sec': None,
        'readiness_wait_timeout_sec': float(timeout_sec),
        'marker_found': False,
        'marker_found_wall_time_sec': None,
        'multi_source_ready': False,
        'timed_out': False,
        'lifecycle_states': {},
        'action_server_ready': False,
        'samples': [],
        'required_active_nodes': list(REQUIRED_ACTIVE_NODES),
        'marker_token': MANAGED_ACTIVE_MARKER,
    }


def readiness_wait_passed(readiness_wait: dict[str, Any]) -> bool:
    if not _bool(readiness_wait.get('enabled')):
        return True
    if _bool(readiness_wait.get('timed_out')):
        return False
    return _bool(readiness_wait.get('marker_found')) or _bool(readiness_wait.get('multi_source_ready'))


def evaluate_readiness_wait_decision(readiness_wait: dict[str, Any]) -> dict[str, Any]:
    allowed = readiness_wait_passed(readiness_wait)
    reasons: list[str] = []
    if _bool(readiness_wait.get('enabled')) and not allowed:
        reasons.append('managed_active_readiness_wait_timeout')
    return {
        'allowed': allowed,
        'reasons': reasons,
        'classification': None if allowed else READY_WAIT_TIMEOUT_NO_DISPATCH,
    }


def _read_launch_log_contains_marker(path: str | None) -> bool:
    if not path:
        return False
    log_path = Path(path)
    if not log_path.exists():
        return False
    try:
        return MANAGED_ACTIVE_MARKER in log_path.read_text(encoding='utf-8', errors='replace')
    except Exception:
        return False


def _ros_command(args: list[str], timeout_sec: float = 1.5) -> tuple[bool, str]:
    try:
        proc = subprocess.run(args, text=True, capture_output=True, check=False, timeout=timeout_sec)
        return proc.returncode == 0, (proc.stdout or proc.stderr or '').strip()
    except subprocess.TimeoutExpired:
        return False, 'query_timeout'
    except Exception as exc:
        return False, f'query_exception:{exc}'


def _query_lifecycle_states() -> dict[str, str]:  # pragma: no cover - ROS runtime
    states: dict[str, str] = {}
    for node in REQUIRED_ACTIVE_NODES:
        ok, out = _ros_command(['ros2', 'lifecycle', 'get', f'/{node}'], timeout_sec=1.2)
        states[node] = out.splitlines()[0].strip() if ok and out else out or 'query_failed'
    return states


def _query_action_server_ready() -> bool:  # pragma: no cover - ROS runtime
    ok, out = _ros_command(['ros2', 'action', 'list'], timeout_sec=1.5)
    return ok and ACTION_NAME in out


def _states_all_active(states: dict[str, Any]) -> bool:
    return all(str(states.get(node, '')).startswith('active') for node in REQUIRED_ACTIVE_NODES)


def wait_for_managed_active_readiness(*, timeout_sec: float, sample_period_sec: float, launch_log_path: str | None) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    record = initial_readiness_wait_record(timeout_sec=timeout_sec)
    record['enabled'] = True
    start = _now_wall()
    record['readiness_wait_start_wall_time_sec'] = start
    deadline = start + float(timeout_sec)
    while _now_wall() <= deadline:
        marker = _read_launch_log_contains_marker(launch_log_path)
        states = _query_lifecycle_states()
        action_ready = _query_action_server_ready()
        multi_source_ready = bool(action_ready and _states_all_active(states))
        sample = {
            'wall_time_sec': _now_wall(),
            'marker_found': marker,
            'lifecycle_states': states,
            'action_server_ready': action_ready,
            'multi_source_ready': multi_source_ready,
        }
        record['samples'].append(sample)
        record['marker_found'] = bool(record['marker_found'] or marker)
        if marker and record['marker_found_wall_time_sec'] is None:
            record['marker_found_wall_time_sec'] = sample['wall_time_sec']
        record['lifecycle_states'] = states
        record['action_server_ready'] = action_ready
        record['multi_source_ready'] = bool(record['multi_source_ready'] or multi_source_ready)
        if record['marker_found'] or record['multi_source_ready']:
            break
        time.sleep(float(sample_period_sec))
    end = _now_wall()
    record['readiness_wait_end_wall_time_sec'] = end
    record['readiness_wait_elapsed_sec'] = end - start
    record['timed_out'] = not readiness_wait_passed(record)
    return record


def classify_dispatch_record(dispatch: dict[str, Any]) -> str:
    if not _bool(dispatch.get('dispatch_attempted')):
        return PREFLIGHT_FAILED_NO_DISPATCH
    if _bool(dispatch.get('rejected')) or (_bool(dispatch.get('dispatch_attempted')) and not _bool(dispatch.get('accepted')) and dispatch.get('goal_response_wall_time_sec') is not None):
        return INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
    if not _bool(dispatch.get('accepted')):
        return INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE
    if not _bool(dispatch.get('result_received')):
        return INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
    label = dispatch.get('result_status_label') or status_label(dispatch.get('result_status'))
    if label == 'SUCCEEDED':
        return INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
    if label == 'ABORTED':
        return INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
    if label == 'CANCELED':
        return INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
    return INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE


def build_artifact(*, run_id: str, preflight: dict[str, Any], dispatch: dict[str, Any], classification: str, phase: str = PHASE, mode: str = MODE, readiness_wait: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        'phase': phase,
        'run_id': run_id,
        'mode': mode,
        'readiness_wait': readiness_wait if readiness_wait is not None else initial_readiness_wait_record(timeout_sec=0.0),
        'preflight': preflight,
        'dispatch': dispatch,
        'classification': classification,
        'guardrails': {
            'ingress_goal_sent': _bool(dispatch.get('ingress_goal_sent')),
            'maze_explorer_started': False,
            'no_maze_explorer_auto_start_guard_valid': not _bool(dispatch.get('maze_explorer_started')),
            'nav2_config_tuned': False,
            'exploration_strategy_changed': False,
            'preflight_removed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
            'only_locked_explicit_inner_ingress_goal_allowed': True,
        },
        'notes': [
            'timeout/abort/cancel are diagnostic fail',
            'ingress_goal_sent=true only after accepted',
            'single-goal result is not autonomous exploration success or exit success',
        ],
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_preflight_subprocess(*, output: Path, timeout_sec: float, tf_stability_window_sec: float, sample_period_sec: float, startup_grace_sec: float, launch_log_path: str | None, stdout_path: Path | None = None, stderr_path: Path | None = None) -> tuple[int, dict[str, Any]]:
    script = Path(__file__).resolve().parent / 'phase105_inner_ingress_tf_controller_preflight.py'
    cmd = [
        sys.executable,
        str(script),
        '--output', str(output),
        '--timeout-sec', str(timeout_sec),
        '--tf-stability-window-sec', str(tf_stability_window_sec),
        '--sample-period-sec', str(sample_period_sec),
        '--startup-grace-sec', str(startup_grace_sec),
        '--use-sim-time',
    ]
    if launch_log_path:
        cmd += ['--launch-log-path', launch_log_path]
    proc = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if stdout_path:
        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stdout_path.write_text(proc.stdout or '', encoding='utf-8')
    if stderr_path:
        stderr_path.parent.mkdir(parents=True, exist_ok=True)
        stderr_path.write_text(proc.stderr or '', encoding='utf-8')
    data = json.loads(output.read_text(encoding='utf-8')) if output.exists() else {}
    return proc.returncode, _dict(data.get('ingress_preflight'))


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def dispatch_locked_goal_runtime(*, bounded_goal_result_wait_sec: float, action_server_wait_sec: float) -> dict[str, Any]:  # pragma: no cover - requires ROS runtime
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter

    dispatch = initial_dispatch_record(bounded_goal_result_wait_sec=bounded_goal_result_wait_sec)
    rclpy.init(args=None)
    node = rclpy.create_node('phase118_controlled_ingress_single_goal_dispatch_smoke')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    client = ActionClient(node, NavigateToPose, ACTION_NAME)
    try:
        dispatch['action_server_ready_checked_at_wall_time'] = _now_wall()
        ready = bool(client.wait_for_server(timeout_sec=float(action_server_wait_sec)))
        dispatch['action_server_ready'] = ready
        if not ready:
            dispatch['exception_text'] = 'action server not ready before dispatch'
            return dispatch

        now = node.get_clock().now()
        stamp_msg = now.to_msg()
        stamp = {'sec': int(stamp_msg.sec), 'nanosec': int(stamp_msg.nanosec)}
        goal_pose = locked_goal_pose(stamp=stamp)
        dispatch['goal_pose'] = goal_pose
        dispatch['frame_id'] = goal_pose['frame_id']
        dispatch['stamp'] = stamp
        dispatch['x'] = goal_pose['x']
        dispatch['y'] = goal_pose['y']
        dispatch['yaw'] = goal_pose['yaw']

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = stamp_msg
        goal_msg.pose.pose.position.x = float(goal_pose['x'])
        goal_msg.pose.pose.position.y = float(goal_pose['y'])
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _quaternion_from_yaw(float(goal_pose['yaw']))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        dispatch['dispatch_attempted'] = True
        dispatch['send_wall_time_sec'] = _now_wall()
        dispatch['send_ros_time_sec'] = float(now.nanoseconds) / 1_000_000_000.0
        dispatch['send_time'] = {'wall_time_sec': dispatch['send_wall_time_sec'], 'ros_time_sec': dispatch['send_ros_time_sec']}
        goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
        if not goal_future.done():
            dispatch['exception_text'] = 'goal response timeout before accepted'
            dispatch['accepted'] = False
            dispatch['rejected'] = False
            dispatch['ingress_goal_sent'] = False
            return dispatch
        dispatch['goal_response_wall_time_sec'] = _now_wall()
        goal_handle = goal_future.result()
        dispatch['goal_handle_available'] = goal_handle is not None
        if goal_handle is None or not bool(goal_handle.accepted):
            dispatch['accepted'] = False
            dispatch['rejected'] = True
            dispatch['ingress_goal_sent'] = False
            return dispatch

        dispatch['accepted'] = True
        dispatch['rejected'] = False
        dispatch['ingress_goal_sent'] = True
        dispatch['result_wait_started_wall_time_sec'] = _now_wall()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=float(bounded_goal_result_wait_sec))
        if not result_future.done():
            dispatch['result_received'] = False
            dispatch['cancel_requested'] = True
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
            dispatch['cancel_result'] = str(cancel_future.result()) if cancel_future.done() else 'cancel_timeout'
            return dispatch

        result_response = result_future.result()
        dispatch['result_received'] = True
        status = int(getattr(result_response, 'status', GoalStatus.STATUS_UNKNOWN))
        dispatch['result_status'] = status
        dispatch['result_status_label'] = status_label(status)
        result_obj = getattr(result_response, 'result', None)
        dispatch['abort_text'] = '' if dispatch['result_status_label'] == 'SUCCEEDED' else str(result_obj)
        return dispatch
    except Exception as exc:  # keep diagnostic artifact instead of crashing silently
        dispatch['exception_text'] = str(exc)
        dispatch['traceback_tail'] = traceback.format_exc()[-2000:]
        return dispatch
    finally:
        try:
            client.destroy()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def run_smoke(args: argparse.Namespace) -> dict[str, Any]:
    preflight_output = args.preflight_output or args.output.with_name(args.output.stem + '_preflight.json')
    preflight_stdout = args.output.with_name(args.output.stem + '_preflight_stdout.json')
    preflight_stderr = args.output.with_name(args.output.stem + '_preflight_stderr.log')
    phase = 'Phase120' if args.enable_readiness_wait else PHASE
    mode = PHASE120_MODE if args.enable_readiness_wait else MODE
    readiness_wait = initial_readiness_wait_record(timeout_sec=args.readiness_wait_timeout_sec)
    if args.enable_readiness_wait:
        readiness_wait = wait_for_managed_active_readiness(
            timeout_sec=args.readiness_wait_timeout_sec,
            sample_period_sec=args.readiness_wait_sample_period_sec,
            launch_log_path=args.launch_log_path,
        )
        readiness_decision = evaluate_readiness_wait_decision(readiness_wait)
        if not readiness_decision['allowed']:
            dispatch = initial_dispatch_record(bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec)
            dispatch['precondition_decision'] = readiness_decision
            preflight = {
                'evaluated': False,
                'passed': False,
                'failed_gates': [],
                'ingress_preflight_reject_reason': None,
            }
            artifact = build_artifact(
                run_id=args.run_id,
                preflight=preflight,
                dispatch=dispatch,
                classification=READY_WAIT_TIMEOUT_NO_DISPATCH,
                phase=phase,
                mode=mode,
                readiness_wait=readiness_wait,
            )
            artifact['preflight_exit_code'] = None
            artifact['preflight_artifact'] = str(preflight_output)
            write_json(args.output, artifact)
            return artifact

    preflight_rc, preflight = run_preflight_subprocess(
        output=preflight_output,
        timeout_sec=args.preflight_timeout_sec,
        tf_stability_window_sec=args.tf_stability_window_sec,
        sample_period_sec=args.sample_period_sec,
        startup_grace_sec=args.startup_grace_sec,
        launch_log_path=args.launch_log_path,
        stdout_path=preflight_stdout,
        stderr_path=preflight_stderr,
    )

    if args.dry_run_dispatch:
        dispatch = initial_dispatch_record(bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec)
        dispatch['action_server_ready'] = True
        dispatch['action_server_ready_checked_at_wall_time'] = _now_wall()
    else:
        # Check action readiness inside the dispatch function, but if preflight failed
        # we do not dispatch and keep the state false.
        dispatch = initial_dispatch_record(bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec)
        if _bool(preflight.get('passed')) and not _list(preflight.get('failed_gates')) and preflight.get('ingress_preflight_reject_reason') in (None, ''):
            dispatch = dispatch_locked_goal_runtime(
                bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec,
                action_server_wait_sec=args.action_server_wait_sec,
            )

    decision = evaluate_dispatch_preconditions(preflight, action_server_ready=_bool(dispatch.get('action_server_ready')))
    if not decision['allowed']:
        # If dispatch ran only because action readiness was checked inside it, protect
        # against impossible contradiction by preserving the actual diagnostic record
        # but classifying failed/no-dispatch when dispatch_attempted is false.
        if not _bool(dispatch.get('dispatch_attempted')):
            classification = PREFLIGHT_FAILED_NO_DISPATCH
            dispatch['precondition_decision'] = decision
            dispatch['ingress_goal_sent'] = False
        else:
            classification = classify_dispatch_record(dispatch)
    else:
        classification = classify_dispatch_record(dispatch)

    # Keep preflight wrapper artifact synchronized with accepted-goal state.
    try:
        preflight_module = _load_preflight_module()
        preflight_module.mark_wrapper_state(
            preflight_output,
            ingress_goal_sent=_bool(dispatch.get('ingress_goal_sent')),
            maze_explorer_started=False,
        )
        preflight = _dict(json.loads(preflight_output.read_text(encoding='utf-8')).get('ingress_preflight'))
    except Exception:
        pass

    artifact = build_artifact(
        run_id=args.run_id,
        preflight=preflight,
        dispatch=dispatch,
        classification=classification,
        phase=phase,
        mode=mode,
        readiness_wait=readiness_wait,
    )
    artifact['preflight_exit_code'] = preflight_rc
    artifact['preflight_artifact'] = str(preflight_output)
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preflight-output', type=Path)
    parser.add_argument('--run-id', default='phase118_controlled_ingress_single_goal_dispatch_smoke')
    parser.add_argument('--launch-log-path')
    parser.add_argument('--preflight-timeout-sec', type=float, default=25.0)
    parser.add_argument('--tf-stability-window-sec', type=float, default=2.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--startup-grace-sec', type=float, default=2.0)
    parser.add_argument('--action-server-wait-sec', type=float, default=10.0)
    parser.add_argument('--bounded-goal-result-wait-sec', type=float, default=45.0)
    parser.add_argument('--dry-run-dispatch', action='store_true', help='test-only: do not use ROS action client')
    parser.add_argument('--enable-readiness-wait', action='store_true', help='Phase120: wait for managed-active readiness before preflight')
    parser.add_argument('--readiness-wait-timeout-sec', type=float, default=45.0)
    parser.add_argument('--readiness-wait-sample-period-sec', type=float, default=1.0)
    args = parser.parse_args()
    artifact = run_smoke(args)
    print(json.dumps({
        'classification': artifact.get('classification'),
        'preflight_passed': _dict(artifact.get('preflight')).get('passed'),
        'accepted': _dict(artifact.get('dispatch')).get('accepted'),
        'result_status_label': _dict(artifact.get('dispatch')).get('result_status_label'),
        'ingress_goal_sent': _dict(artifact.get('dispatch')).get('ingress_goal_sent'),
        'maze_explorer_started': _dict(artifact.get('dispatch')).get('maze_explorer_started'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
