#!/usr/bin/env python3
"""Phase124 first exploration goal dispatch smoke.

Scope guardrails:
- Reuse the Phase120 readiness/preflight/explicit inner-ingress goal chain.
- Reuse the Phase122 handoff gate and require handoff_allowed before dispatch-capable startup.
- Start maze_explorer with max_goals:=1 only after handoff is ready.
- Allow exactly one current-algorithm first goal_kind=explore dispatch, then stop on accepted/rejected/timeout/no-valid-goal.
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

PHASE = 'Phase124'
MODE = 'first_exploration_goal_dispatch_smoke'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP = 'FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP'
FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL'
FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL = 'FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL'
NO_VALID_FIRST_GOAL_FAIL_CLOSED = 'NO_VALID_FIRST_GOAL_FAIL_CLOSED'
HANDOFF_NOT_READY_NO_EXPLORATION = 'HANDOFF_NOT_READY_NO_EXPLORATION'
CLASSIFICATIONS = [
    FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP,
    FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL,
    FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL,
    NO_VALID_FIRST_GOAL_FAIL_CLOSED,
    HANDOFF_NOT_READY_NO_EXPLORATION,
]

REQUIRED_FIRST_GOAL_FIELDS = [
    'maze_explorer_start_allowed',
    'maze_explorer_started',
    'maze_explorer_max_goals',
    'exploration_goal_dispatched',
    'first_goal',
    'goal_event_count',
    'dispatch_event_count',
    'goal_events',
    'explorer_states',
    'nav2_feedback',
    'local_costmap_samples',
    'scan',
    'map',
    'odom',
    'tf',
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


def _load_phase122_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase122_post_ingress_handoff_dry_start.py'
    spec = importlib.util.spec_from_file_location('phase122_runner_for_phase124', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase122 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_phase122 = _load_phase122_runner()


def _status_label(status: Any) -> str | None:
    if status is None:
        return None
    try:
        value = int(status)
    except (TypeError, ValueError):
        return str(status)
    labels = {
        0: 'UNKNOWN',
        1: 'ACCEPTED',
        2: 'EXECUTING',
        3: 'CANCELING',
        4: 'SUCCEEDED',
        5: 'CANCELED',
        6: 'ABORTED',
    }
    return labels.get(value, str(value))


def _point_xy(value: Any) -> tuple[float | None, float | None]:
    if isinstance(value, dict):
        return _float(value.get('x')), _float(value.get('y'))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return _float(value[0]), _float(value[1])
    return None, None


def _goal_pose_from_event(event: dict[str, Any]) -> dict[str, Any]:
    target = event.get('target') or event.get('refined_target') or event.get('original_target')
    x, y = _point_xy(target)
    pose = {
        'x': x,
        'y': y,
        'yaw': _float(event.get('branch_angle'), 0.0),
        'frame_id': 'map',
        'stamp': event.get('received_wall_time_sec'),
    }
    return pose


def _candidate_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return {
        'candidate_id': event.get('goal_sequence'),
        'candidate_family': event.get('candidate_family') or event.get('local_topology'),
        'candidate_source': 'maze_explorer_current_algorithm',
        'candidate_rank': event.get('chosen_branch_rank'),
        'candidate_count': event.get('candidate_count', event.get('last_candidate_count')),
        'selected_candidate_index': event.get('selected_candidate_index'),
        'raw_target': event.get('target'),
        'refined_target': event.get('refined_target'),
        'original_target': event.get('original_target'),
        'branch_angle': event.get('branch_angle'),
        'goal_kind': event.get('goal_kind'),
        'near_exit': event.get('near_exit'),
        'start_node_id': event.get('start_node_id'),
        'current_node_id': event.get('current_node_id'),
    }


def _frontier_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return {
        'last_open_direction_count': event.get('last_open_direction_count'),
        'last_candidate_count': event.get('last_candidate_count'),
        'candidate_branch_count': event.get('candidate_branch_count'),
        'candidate_branches': event.get('candidate_branches'),
        'chosen_branch_rank': event.get('chosen_branch_rank'),
        'chosen_branch_score_components': event.get('chosen_branch_score_components'),
        'dispatch_readiness_gate': event.get('dispatch_readiness_gate'),
        'local_topology': event.get('local_topology'),
        'selection_priority_trace': event.get('selection_priority_trace'),
    }


def _topology_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    keys = [
        'topology_consistency_guard',
        'topology_consistency_guard_status',
        'post_ingress_single_open_exception',
        'single_open_exception_applied',
        'blocked_branch_count',
        'blacklisted_goal_count',
        'start_node_id',
        'current_node_id',
        'mode',
    ]
    return {key: event.get(key) for key in keys}


def initial_first_goal_record(*, max_goals: int = 1) -> dict[str, Any]:
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
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
        },
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'local_costmap_samples': [],
        'scan': {},
        'map': {},
        'odom': {},
        'tf': {},
        'stop_reason': None,
        'observation_timed_out': False,
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
        },
    }


def _goal_event_matches_first(dispatch: dict[str, Any], event: dict[str, Any]) -> bool:
    seq = dispatch.get('goal_sequence')
    if seq is None:
        return True
    return event.get('goal_sequence') == seq


def _accepted_from_status(action_status_samples: list[dict[str, Any]], dispatch_wall_time_sec: float | None) -> bool:
    for sample in action_status_samples:
        sample_wall = _float(sample.get('received_wall_time_sec'))
        if dispatch_wall_time_sec is not None and sample_wall is not None and sample_wall < dispatch_wall_time_sec:
            continue
        if _int(sample.get('active_goal_count')) > 0 or _int(sample.get('executing_goal_count')) > 0 or _int(sample.get('pending_goal_count')) > 0:
            return True
    return False


def _latest_evidence(samples: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    local_costmap_samples = _list(samples.get('local_costmap_samples'))
    scan = _dict(samples.get('scan'))
    map_msg = _dict(samples.get('map'))
    odom = _dict(samples.get('odom'))
    tf = _dict(samples.get('tf'))
    nav2_feedback = _list(samples.get('nav2_feedback'))
    action_status_samples = _list(samples.get('action_status_samples'))
    return local_costmap_samples, scan, map_msg, odom, tf, nav2_feedback, action_status_samples


def first_goal_from_observation(*, goal_events: list[dict[str, Any]], explorer_states: list[dict[str, Any]], action_status_samples: list[dict[str, Any]], evidence_samples: dict[str, Any], process_record: dict[str, Any], dispatch_started_wall_time_sec: float | None, observation_timed_out: bool) -> dict[str, Any]:
    record = initial_first_goal_record(max_goals=1)
    record['maze_explorer_start_allowed'] = True
    record['maze_explorer_started'] = True
    record['process'].update(process_record)
    record['goal_events'] = goal_events[-50:]
    record['explorer_states'] = explorer_states[-50:]
    record['goal_event_count'] = len(goal_events)
    dispatch_events = [event for event in goal_events if str(event.get('event', '')).lower() == 'dispatch']
    record['dispatch_event_count'] = len(dispatch_events)
    record['exploration_goal_dispatched'] = bool(dispatch_events)
    record['observation_timed_out'] = bool(observation_timed_out)
    local_costmap_samples, scan, map_msg, odom, tf, nav2_feedback, evidence_action_status = _latest_evidence(evidence_samples)
    if evidence_action_status and not action_status_samples:
        action_status_samples = evidence_action_status
    record['local_costmap_samples'] = local_costmap_samples[-10:]
    record['scan'] = scan
    record['map'] = map_msg
    record['odom'] = odom
    record['tf'] = tf
    record['nav2_feedback'] = nav2_feedback[-20:]
    record['action_status_samples'] = action_status_samples[-20:]

    if not dispatch_events:
        record['stop_reason'] = 'no_valid_first_goal_fail_closed'
        return record

    dispatch = dispatch_events[0]
    goal_kind = dispatch.get('goal_kind')
    pose = _goal_pose_from_event(dispatch)
    dispatch_wall = dispatch.get('received_wall_time_sec', dispatch_started_wall_time_sec)
    result_events = [event for event in goal_events if str(event.get('event', '')).lower() in {'success', 'failure', 'timeout'} and _goal_event_matches_first(dispatch, event)]
    timeout_events = [event for event in result_events if str(event.get('event', '')).lower() == 'timeout']
    failure_events = [event for event in result_events if str(event.get('event', '')).lower() == 'failure']
    success_events = [event for event in result_events if str(event.get('event', '')).lower() == 'success']
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
    elif success_events:
        accepted = True
        status_label = 'SUCCEEDED'
        stop_reason = 'first_explore_goal_result_succeeded_stop'
    elif timeout_events or (observation_timed_out and dispatch_events):
        timeout = True
        accepted = True if _accepted_from_status(action_status_samples, _float(dispatch_wall)) else bool(observation_timed_out)
        status_label = 'TIMEOUT'
        abort_text = _dict(timeout_events[0]).get('result_reason') if timeout_events else 'bounded_observation_timeout_after_first_dispatch'
        stop_reason = 'first_explore_goal_timeout_stop'
    elif failure_events:
        rejected = True
        status_label = _status_label(failure_events[0].get('result_status')) or 'REJECTED'
        abort_text = failure_events[0].get('result_reason') or 'first_goal_failure'
        stop_reason = 'first_explore_goal_rejected_or_failed_stop'
    elif _accepted_from_status(action_status_samples, _float(dispatch_wall)):
        accepted = True
        status_label = 'ACCEPTED'
        stop_reason = 'first_explore_goal_accepted_stop'
    else:
        rejected = True
        status_label = 'REJECTED_OR_NOT_CONFIRMED'
        abort_text = 'first explore dispatch observed but Nav2 acceptance was not confirmed inside bounded window'
        stop_reason = 'first_explore_goal_acceptance_not_confirmed_stop'

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
        'result_status_label': status_label,
        'abort_text': abort_text,
        'timeout': bool(timeout),
    }
    record['stop_reason'] = stop_reason
    return record


def handoff_ready(handoff_artifact: dict[str, Any]) -> bool:
    return _phase122.handoff_gate_allowed(handoff_artifact)


def classify_phase124(handoff_artifact: dict[str, Any], first_goal_record: dict[str, Any]) -> str:
    if not handoff_ready(handoff_artifact):
        return HANDOFF_NOT_READY_NO_EXPLORATION
    first = _dict(first_goal_record.get('first_goal'))
    dispatch_count = _int(first_goal_record.get('dispatch_event_count'))
    max_goals = _int(first_goal_record.get('maze_explorer_max_goals'), -1)
    if dispatch_count == 0:
        return NO_VALID_FIRST_GOAL_FAIL_CLOSED
    if max_goals != 1 or dispatch_count > 1:
        return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL
    if first.get('goal_kind') != 'explore':
        return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL
    if _bool(first.get('timeout')):
        return FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL
    if _bool(first.get('accepted')) and not _bool(first.get('rejected')):
        return FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP
    return FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL


def build_phase124_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], first_goal_record: dict[str, Any], classification: str) -> dict[str, Any]:
    handoff_allowed = handoff_ready(handoff_artifact)
    dispatch_count = _int(first_goal_record.get('dispatch_event_count'))
    first = _dict(first_goal_record.get('first_goal'))
    return {
        'phase': PHASE,
        'run_id': run_id,
        'mode': MODE,
        'created_wall_time_sec': _now_wall(),
        'phase120_source_artifact': phase120_artifact.get('artifact_path') or phase120_artifact.get('run_id'),
        'phase120_artifact': phase120_artifact,
        'handoff_artifact': handoff_artifact,
        'handoff_allowed': bool(handoff_allowed),
        'first_goal_artifact': first_goal_record,
        'classification': classification,
        'maze_explorer_start_allowed': _bool(first_goal_record.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(first_goal_record.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int(first_goal_record.get('maze_explorer_max_goals'), -1),
        'exploration_goal_dispatched': _bool(first_goal_record.get('exploration_goal_dispatched')),
        'goal_event_count': _int(first_goal_record.get('goal_event_count')),
        'dispatch_event_count': dispatch_count,
        'first_goal': first,
        'stop_reason': first_goal_record.get('stop_reason'),
        'guardrails': {
            'handoff_ready_required': True,
            'same_run_phase120_ingress_required': True,
            'only_explicit_inner_ingress_goal_sent': _phase122.phase120_ingress_success(phase120_artifact) if phase120_artifact else True,
            'max_goals_one': _int(first_goal_record.get('maze_explorer_max_goals'), -1) == 1,
            'first_exploration_goal_only': dispatch_count <= 1,
            'exactly_one_dispatch_event_when_success': classification != FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP or dispatch_count == 1,
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
            'Phase124 validates only the first current-algorithm exploration goal dispatch boundary.',
            'Accepted first-goal smoke is not autonomous exploration success and not exit success.',
            'No manual Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal is sent.',
        ],
    }


def _stamp_dict(stamp: Any) -> dict[str, int] | None:
    if stamp is None:
        return None
    return {'sec': int(getattr(stamp, 'sec', 0)), 'nanosec': int(getattr(stamp, 'nanosec', 0))}


def _duration_msg_to_sec(msg: Any) -> float | None:
    if msg is None:
        return None
    return float(getattr(msg, 'sec', 0)) + float(getattr(msg, 'nanosec', 0)) / 1_000_000_000.0


def _runtime_tf_sample(buffer: Any, node: Any) -> dict[str, Any]:
    try:
        import rclpy
        from rclpy.time import Time
        trans = buffer.lookup_transform('map', 'base_link', Time(), timeout=rclpy.duration.Duration(seconds=0.05))
        return {
            'fresh': True,
            'frame_id': 'map',
            'child_frame_id': 'base_link',
            'stamp': _stamp_dict(trans.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'translation': {
                'x': float(trans.transform.translation.x),
                'y': float(trans.transform.translation.y),
                'z': float(trans.transform.translation.z),
            },
        }
    except Exception as exc:
        return {'fresh': False, 'exception_text': str(exc), 'received_wall_time_sec': _now_wall()}


def run_maze_explorer_first_goal(*, duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
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
    except Exception:  # noqa: BLE001 - generated action module naming can vary.
        NavigateToPose_FeedbackMessage = None

    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1', '-p', 'exploration_rate_hz:=1.0'
    ]
    record = initial_first_goal_record(max_goals=1)
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
    accepted_seen = False
    terminal_seen = False

    def _event_cb(msg: String) -> None:
        nonlocal first_dispatch_wall, terminal_seen
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        events.append(payload)
        if str(payload.get('event', '')).lower() == 'dispatch' and first_dispatch_wall is None:
            first_dispatch_wall = payload['received_wall_time_sec']
        if first_dispatch_wall is not None and str(payload.get('event', '')).lower() in {'success', 'failure', 'timeout'}:
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
        evidence['map'] = {
            'seen': True,
            'frame_id': msg.header.frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
        }

    def _local_cb(msg: OccupancyGrid) -> None:
        local_costmap_samples.append({
            'seen': True,
            'frame_id': msg.header.frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
        })
        del local_costmap_samples[:-20]

    def _odom_cb(msg: Odometry) -> None:
        evidence['odom'] = {
            'seen': True,
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'pose': {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'z': float(msg.pose.pose.position.z),
            },
        }

    def _status_cb(msg: GoalStatusArray) -> None:
        nonlocal accepted_seen
        statuses = [int(status.status) for status in msg.status_list]
        sample = {
            'received_wall_time_sec': _now_wall(),
            'statuses': statuses,
            'active_goal_count': sum(1 for status in statuses if status in (1, 2, 3)),
            'pending_goal_count': sum(1 for status in statuses if status == 1),
            'executing_goal_count': sum(1 for status in statuses if status == 2),
            'canceling_goal_count': sum(1 for status in statuses if status == 3),
        }
        action_status_samples.append(sample)
        del action_status_samples[:-50]
        if first_dispatch_wall is not None and sample['active_goal_count'] > 0:
            accepted_seen = True

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
        del nav2_feedback[:-50]

    rclpy.init(args=None)
    node = rclpy.create_node('phase124_first_exploration_goal_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    costmap_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
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
            dispatch_count = len([event for event in events if str(event.get('event', '')).lower() == 'dispatch'])
            first_event = next((event for event in events if str(event.get('event', '')).lower() == 'dispatch'), None)
            if dispatch_count > 1:
                break
            if first_event is not None and first_event.get('goal_kind') != 'explore':
                break
            if accepted_seen or terminal_seen:
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
    process_record = _dict(record.get('process'))
    return first_goal_from_observation(
        goal_events=events,
        explorer_states=states,
        action_status_samples=action_status_samples,
        evidence_samples=evidence,
        process_record=process_record,
        dispatch_started_wall_time_sec=first_dispatch_wall,
        observation_timed_out=observation_timed_out,
    )


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = _phase122.run_phase120_ingress_chain(args)
    if args.phase120_output:
        phase120_artifact['artifact_path'] = str(args.phase120_output)
    return phase120_artifact


def run_phase124(args: argparse.Namespace) -> dict[str, Any]:
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
    first_goal_record = initial_first_goal_record(max_goals=1)
    if handoff_ready(handoff):
        first_goal_record = run_maze_explorer_first_goal(
            duration_sec=args.first_goal_observation_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    classification = classify_phase124(handoff, first_goal_record)
    artifact = build_phase124_artifact(
        run_id=args.run_id,
        phase120_artifact=phase120_artifact,
        handoff_artifact=handoff,
        first_goal_record=first_goal_record,
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
    parser.add_argument('--run-id', default='phase124_first_exploration_goal_dispatch_smoke')
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
    parser.add_argument('--first-goal-observation-sec', type=float, default=20.0)
    args = parser.parse_args()
    artifact = run_phase124(args)
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'goal_kind': _dict(artifact.get('first_goal')).get('goal_kind'),
        'accepted': _dict(artifact.get('first_goal')).get('accepted'),
        'timeout': _dict(artifact.get('first_goal')).get('timeout'),
        'dispatch_event_count': artifact.get('dispatch_event_count'),
        'stop_reason': artifact.get('stop_reason'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
