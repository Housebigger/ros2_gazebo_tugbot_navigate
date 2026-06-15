#!/usr/bin/env python3
"""Phase105 fail-closed inner-ingress TF/controller preflight.

This tool is intentionally wrapper-local preflight tooling for the explicit
inner-ingress Nav2 goal handoff. It does not change maze_explorer, Phase88/92/101
logic, branch scoring, exploration order, fallback/terminal acceptance, or Nav2
parameters.

The module exposes pure evaluation helpers for focused tests and a bounded ROS
runtime sampler for validation wrappers. If the bounded preflight rejects, the
wrapper must not send the ingress goal and must not start maze_explorer.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

RUN_ID = 'phase105_inner_ingress_tf_controller_preflight_hardening'

INNER_INGRESS_GOAL_POSE: dict[str, Any] = {
    'frame_id': 'map',
    'x_m': 2.0,
    'y_m': 0.0,
    'yaw_rad': 0.0,
}

REJECT_TOKENS = {
    'ingress_tf_unstable',
    'ingress_map_base_tf_missing',
    'ingress_map_odom_tf_stale',
    'ingress_odom_base_tf_stale',
    'ingress_scan_transform_unstable',
    'ingress_controller_robot_pose_unavailable',
    'ingress_goal_pose_transform_unavailable',
    'ingress_lifecycle_ambiguous',
    'ingress_first_scan_timeout',
    'ingress_tf_stable_window_not_met',
    'ingress_raw_snapshot_cross_check_failed',
    'ingress_preflight_timeout',
}

SAMPLE_ONLY_TOKENS = {
    'waiting_for_first_scan',
}

REJECT_PRIORITY = [
    'ingress_raw_snapshot_cross_check_failed',
    'ingress_lifecycle_ambiguous',
    'ingress_map_base_tf_missing',
    'ingress_tf_unstable',
    'ingress_tf_stable_window_not_met',
    'ingress_map_odom_tf_stale',
    'ingress_odom_base_tf_stale',
    'ingress_first_scan_timeout',
    'ingress_scan_transform_unstable',
    'ingress_controller_robot_pose_unavailable',
    'ingress_goal_pose_transform_unavailable',
    'ingress_preflight_timeout',
]

PASSED_GATES = [
    'map_base_tf_continuous_stability',
    'map_odom_tf_fresh',
    'odom_base_tf_fresh',
    'scan_transform_stable',
    'controller_robot_pose_available',
    'goal_pose_transform_available',
    'tf_detector_clear',
]


def _now_wall() -> float:
    return time.time()


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _first_priority(tokens: list[str]) -> str | None:
    for token in REJECT_PRIORITY:
        if token in tokens:
            return token
    return tokens[0] if tokens else None


def derive_lifecycle_confirmation(
    *,
    lifecycle_query: dict[str, Any] | None,
    node_graph_present: bool,
    action_available: bool,
    lifecycle_service_present: bool = False,
    launch_active_marker_present: bool = False,
    launch_active_marker_token: str | None = None,
) -> dict[str, Any]:
    """Derive a fail-closed lifecycle state from multiple evidence sources.

    Active is confirmed only when explicit lifecycle query evidence is active,
    or when a lifecycle query timeout is outweighed by a strict four-source
    runtime threshold: node graph present, action available, lifecycle service
    present, and a launch-level active marker. Missing any source remains
    ambiguous/fail-closed.
    """
    query = dict(lifecycle_query or {})
    returncode = query.get('returncode')
    state = str(query.get('state') or '').strip().lower()
    query_error_text = query.get('query_error') or query.get('stderr') or None
    query_ok = returncode == 0 and bool(state)
    query_timed_out = _bool(query.get('timed_out'))
    graph_present = bool(node_graph_present)
    action_ready = bool(action_available)
    service_present = bool(lifecycle_service_present)
    launch_marker = bool(launch_active_marker_present)

    active_from_query = query_ok and state == 'active'
    inactive_from_query = query_ok and state in {'inactive', 'unconfigured', 'finalized', 'unknown', 'error'}
    timeout_override = bool(query_timed_out and graph_present and action_ready and service_present and launch_marker)

    active_confirmed_by = None
    ambiguous_detail = None
    if active_from_query and graph_present and action_ready:
        derived = 'active_confirmed'
        active_confirmed_by = 'lifecycle_query_active'
    elif timeout_override:
        derived = 'active_confirmed'
        active_confirmed_by = 'multi_source_query_timeout_override'
        ambiguous_detail = 'lifecycle subprocess timed out but graph/action/service/launch-active marker agree'
    elif inactive_from_query and not graph_present and not action_ready:
        derived = 'inactive_confirmed'
    elif not query_ok and not graph_present and not action_ready:
        derived = 'query_error'
    elif active_from_query or inactive_from_query or graph_present or action_ready or service_present or launch_marker:
        derived = 'ambiguous'
        if query_timed_out:
            missing = []
            if not graph_present:
                missing.append('node_graph')
            if not action_ready:
                missing.append('action')
            if not service_present:
                missing.append('lifecycle_service')
            if not launch_marker:
                missing.append('launch_active_marker')
            ambiguous_detail = 'lifecycle subprocess timed out; missing strict active confirmation sources: ' + ','.join(missing)
    else:
        derived = 'query_error'

    return {
        'state': derived,
        'active_confirmed': derived == 'active_confirmed',
        'inactive_confirmed': derived == 'inactive_confirmed',
        'ambiguous': derived == 'ambiguous',
        'query_error': (not query_ok) or bool(query_error_text),
        'active_confirmed_by': active_confirmed_by,
        'ambiguous_detail': ambiguous_detail,
        'lifecycle_query': {
            'command': query.get('command'),
            'timeout_sec': query.get('timeout_sec'),
            'duration_sec': query.get('duration_sec'),
            'returncode': returncode,
            'state': state or None,
            'query_error': query_error_text,
            'timed_out': query_timed_out,
            'stdout': query.get('stdout'),
            'stderr': query.get('stderr'),
        },
        'node_graph': {'present': graph_present},
        'action_availability': {'available': action_ready},
        'lifecycle_service': {'present': service_present},
        'launch_active_marker': {'present': launch_marker, 'token': launch_active_marker_token},
    }


def _normalize_lifecycle_confirmation(value: Any, *, active_fallback: bool | None = None, action_ready: bool = False) -> dict[str, Any]:
    data = _dict(value)
    if data.get('state') in {'active_confirmed', 'inactive_confirmed', 'ambiguous', 'query_error'}:
        return data
    if active_fallback is None:
        active_fallback = _bool(data.get('active_confirmed')) or _bool(data.get('active'))
    return derive_lifecycle_confirmation(
        lifecycle_query={'returncode': 0 if active_fallback else 1, 'state': 'active' if active_fallback else None},
        node_graph_present=bool(active_fallback),
        action_available=bool(action_ready or active_fallback),
    )


def _lifecycle_sources_from_controller(controller: dict[str, Any]) -> dict[str, Any]:
    action_ready = _bool(controller.get('navigate_to_pose_action_ready'))
    existing = _dict(controller.get('lifecycle_sources'))
    controller_state = _normalize_lifecycle_confirmation(
        existing.get('controller_server') or controller.get('controller_server_lifecycle'),
        active_fallback=_bool(controller.get('controller_server_active')),
        action_ready=action_ready,
    )
    bt_state = _normalize_lifecycle_confirmation(
        existing.get('bt_navigator') or controller.get('bt_navigator_lifecycle'),
        active_fallback=_bool(controller.get('bt_navigator_active')),
        action_ready=action_ready,
    )
    return {'controller_server': controller_state, 'bt_navigator': bt_state}


def _stable_sort_tokens(tokens: list[str]) -> list[str]:
    return sorted(set(tokens), key=lambda token: REJECT_PRIORITY.index(token) if token in REJECT_PRIORITY else 999)


class PreflightConfig:
    def __init__(
        self,
        *,
        tf_stability_window_sec: float = 2.0,
        timeout_sec: float = 20.0,
        sample_period_sec: float = 0.5,
        tf_max_age_sec: float = 1.5,
        scan_max_age_sec: float = 1.5,
        startup_grace_sec: float = 2.0,
        use_sim_time: bool = True,
        spin_warmup_sec: float = 0.5,
        lifecycle_query_timeout_sec: float = 2.0,
        launch_log_path: str | None = None,
    ) -> None:
        self.tf_stability_window_sec = float(tf_stability_window_sec)
        self.timeout_sec = float(timeout_sec)
        self.sample_period_sec = float(sample_period_sec)
        self.tf_max_age_sec = float(tf_max_age_sec)
        self.scan_max_age_sec = float(scan_max_age_sec)
        self.startup_grace_sec = float(startup_grace_sec)
        self.use_sim_time = bool(use_sim_time)
        self.spin_warmup_sec = float(spin_warmup_sec)
        self.lifecycle_query_timeout_sec = float(lifecycle_query_timeout_sec)
        self.launch_log_path = launch_log_path


class PreflightResult:
    def __init__(
        self,
        *,
        passed: bool,
        reject_reason: str | None,
        failed_gates: list[str],
        passed_gates: list[str],
        samples: list[dict[str, Any]],
        config: PreflightConfig,
        bounded_wait_elapsed_sec: float | None = None,
        start_time_sec: float | None = None,
        end_time_sec: float | None = None,
        last_specific_reject_reason: str | None = None,
    ) -> None:
        self.passed = bool(passed)
        self.reject_reason = reject_reason
        self.failed_gates = sorted(set(failed_gates), key=lambda token: REJECT_PRIORITY.index(token) if token in REJECT_PRIORITY else 999)
        self.passed_gates = list(passed_gates)
        self.samples = list(samples)
        self.config = config
        self.bounded_wait_elapsed_sec = bounded_wait_elapsed_sec
        self.start_time_sec = start_time_sec
        self.end_time_sec = end_time_sec
        self.last_specific_reject_reason = last_specific_reject_reason
        self.ingress_goal_sent = False
        self.maze_explorer_started = False

    def to_artifact(self) -> dict[str, Any]:
        samples = _complete_sample_history_contract(self.samples, self.config)
        latest = samples[-1] if samples else make_synthetic_sample()
        map_base = _dict(latest.get('map_base_tf_check'))
        scan = _dict(latest.get('scan_transform_check'))
        controller = _dict(latest.get('controller_pose_check'))
        goal = _dict(latest.get('goal_pose_transform_check'))
        detector = _dict(latest.get('tf_detector'))
        lifecycle_sources = _lifecycle_sources_from_controller(controller)
        tf_jump_count = _int(latest.get('tf_jump_count')) + _int(detector.get('tf_jump_count'))
        cache_drop_count = _int(detector.get('cache_drop_count')) + (1 if _bool(scan.get('cache_drop_detected')) else 0)
        robot_pose_unavailable_count = _int(detector.get('robot_pose_unavailable_count')) + _int(controller.get('robot_pose_unavailable_log_count'))
        goal_pose_transform_failure_count = _int(detector.get('goal_pose_transform_failure_count')) + _int(goal.get('exception_count'))
        first_scan_seen = any(_bool(_dict(sample.get('scan_transform_check')).get('scan_available')) for sample in samples)
        first_scan_wait_elapsed = None
        if samples:
            if first_scan_seen:
                for sample in samples:
                    if _bool(_dict(sample.get('scan_transform_check')).get('scan_available')):
                        first_scan_wait_elapsed = _float(sample.get('elapsed_sec'))
                        break
            else:
                first_scan_wait_elapsed = _float(latest.get('elapsed_sec'))
        raw_snapshot = _dict(latest.get('raw_style_snapshot_cross_check')) or build_raw_style_snapshot_cross_check(latest, self.failed_gates)
        failed_gates = list(self.failed_gates)
        if raw_snapshot.get('ambiguous') and 'ingress_raw_snapshot_cross_check_failed' not in failed_gates:
            failed_gates.append('ingress_raw_snapshot_cross_check_failed')
        failed_gates = _stable_sort_tokens(failed_gates)
        return {
            'ingress_preflight': {
                'evaluated': True,
                'passed': self.passed,
                'start_time_sec': self.start_time_sec,
                'end_time_sec': self.end_time_sec,
                'bounded_wait_elapsed_sec': self.bounded_wait_elapsed_sec,
                'startup_grace_sec': self.config.startup_grace_sec,
                'use_sim_time': self.config.use_sim_time,
                'spin_warmup_sec': self.config.spin_warmup_sec,
                'lifecycle_query_timeout_sec': self.config.lifecycle_query_timeout_sec,
                'launch_log_path': self.config.launch_log_path,
                'stable_window_sec': self.config.tf_stability_window_sec,
                'sample_period_sec': self.config.sample_period_sec,
                'ingress_preflight_timeout_sec': self.config.timeout_sec,
                'ingress_preflight_reject_reason': self.reject_reason,
                'last_specific_reject_reason': self.last_specific_reject_reason,
                'failed_gates': failed_gates,
                'passed_gates': self.passed_gates if self.passed else [],
                'ingress_goal_sent': self.ingress_goal_sent,
                'maze_explorer_started': self.maze_explorer_started,
                'inner_ingress_goal_pose': dict(INNER_INGRESS_GOAL_POSE),
                'tf_stability_window_sec': self.config.tf_stability_window_sec,
                'tf_jump_count': tf_jump_count,
                'first_scan_seen': first_scan_seen,
                'first_scan_wait_elapsed_sec': first_scan_wait_elapsed,
                'first_scan_deadline_sec': self.config.startup_grace_sec,
                'raw_style_snapshot_cross_check': raw_snapshot,
                'map_base_tf_check': {
                    'available': _bool(map_base.get('available')),
                    'stable': _bool(map_base.get('stable')),
                    'sample_count': _int(map_base.get('sample_count')),
                    'latest_age_sec': map_base.get('latest_age_sec'),
                },
                'map_base_tf_age_sec': latest.get('map_base_tf_age_sec', map_base.get('latest_age_sec')),
                'map_odom_tf_age_sec': latest.get('map_odom_tf_age_sec'),
                'odom_base_tf_age_sec': latest.get('odom_base_tf_age_sec'),
                'scan_transform_check': {
                    'scan_available': _bool(scan.get('scan_available')),
                    'scan_frame_id': scan.get('scan_frame_id'),
                    'target_frame': scan.get('target_frame'),
                    'transform_available': _bool(scan.get('transform_available')),
                    'stable': _bool(scan.get('stable')),
                    'cache_drop_detected': _bool(scan.get('cache_drop_detected')),
                    'exception_count': _int(scan.get('exception_count')),
                    'waiting_for_first_scan': bool(not first_scan_seen and self.bounded_wait_elapsed_sec is not None and self.bounded_wait_elapsed_sec <= self.config.startup_grace_sec),
                },
                'controller_pose_check': {
                    'controller_server_active': lifecycle_sources['controller_server']['active_confirmed'],
                    'controller_server_inactive_confirmed': lifecycle_sources['controller_server']['inactive_confirmed'],
                    'controller_server_ambiguous': lifecycle_sources['controller_server']['ambiguous'],
                    'bt_navigator_active': lifecycle_sources['bt_navigator']['active_confirmed'],
                    'bt_navigator_inactive_confirmed': lifecycle_sources['bt_navigator']['inactive_confirmed'],
                    'bt_navigator_ambiguous': lifecycle_sources['bt_navigator']['ambiguous'],
                    'navigate_to_pose_action_ready': _bool(controller.get('navigate_to_pose_action_ready')),
                    'robot_pose_available': _bool(controller.get('robot_pose_available')),
                    'robot_pose_unavailable_log_count': _int(controller.get('robot_pose_unavailable_log_count')),
                    'lifecycle_sources': lifecycle_sources,
                    'lifecycle_ambiguous': lifecycle_sources['controller_server']['ambiguous'] or lifecycle_sources['bt_navigator']['ambiguous'],
                },
                'goal_pose_transform_check': {
                    'goal_frame_id': goal.get('goal_frame_id', INNER_INGRESS_GOAL_POSE['frame_id']),
                    'global_costmap_frame': goal.get('global_costmap_frame'),
                    'controller_frame': goal.get('controller_frame'),
                    'transform_to_global_costmap_available': _bool(goal.get('transform_to_global_costmap_available')),
                    'transform_to_controller_frame_available': _bool(goal.get('transform_to_controller_frame_available')),
                    'exception_count': _int(goal.get('exception_count')),
                },
                'tf_detector': {
                    'tf_jump_count': tf_jump_count,
                    'cache_drop_count': cache_drop_count,
                    'robot_pose_unavailable_count': robot_pose_unavailable_count,
                    'goal_pose_transform_failure_count': goal_pose_transform_failure_count,
                },
                'sample_count': len(samples),
                'sample_history': samples,
                'sample_tail': samples[-5:],
            }
        }


def make_synthetic_sample(**overrides: Any) -> dict[str, Any]:
    sample: dict[str, Any] = {
        'elapsed_sec': 0.0,
        'map_base_tf_check': {'available': True, 'stable': True, 'sample_count': 1, 'latest_age_sec': 0.05},
        'map_base_tf_age_sec': 0.05,
        'map_odom_tf_age_sec': 0.05,
        'odom_base_tf_age_sec': 0.05,
        'tf_jump_detected': False,
        'tf_jump_count': 0,
        'scan_transform_check': {
            'scan_available': True,
            'scan_frame_id': 'lidar_link',
            'target_frame': 'map',
            'transform_available': True,
            'stable': True,
            'cache_drop_detected': False,
            'exception_count': 0,
        },
        'controller_pose_check': {
            'controller_server_active': True,
            'bt_navigator_active': True,
            'navigate_to_pose_action_ready': True,
            'robot_pose_available': True,
            'robot_pose_unavailable_log_count': 0,
        },
        'goal_pose_transform_check': {
            'goal_frame_id': 'map',
            'global_costmap_frame': 'map',
            'controller_frame': 'odom',
            'transform_to_global_costmap_available': True,
            'transform_to_controller_frame_available': True,
            'exception_count': 0,
        },
        'tf_detector': {
            'tf_jump_count': 0,
            'cache_drop_count': 0,
            'robot_pose_unavailable_count': 0,
            'goal_pose_transform_failure_count': 0,
        },
    }
    sample.update(overrides)
    return sample


def _sample_has_received_scan_evidence(sample: dict[str, Any]) -> bool:
    scan = _dict(sample.get('scan_transform_check'))
    if _bool(scan.get('scan_available')) or bool(scan.get('scan_frame_id')):
        return True
    diag = _dict(sample.get('internal_sampling_diagnostics'))
    sub = _dict(diag.get('scan_subscription'))
    return _int(sub.get('callback_count_total')) > 0 or bool(sub.get('latest_frame_id')) or _float(sub.get('latest_stamp_sec')) is not None


def derive_controller_pose_unavailable_reason(
    *,
    map_base_diag: dict[str, Any],
    map_base_age_sec: float | None,
    robot_pose_available: bool,
    controller_active: bool,
    bt_active: bool,
    action_ready: bool,
    tf_max_age_sec: float,
) -> str | None:
    """Return the direct Phase116 reason for robot pose/controller unavailability.

    This is diagnostic-only: it explains which prerequisite made the controller
    pose gate fail without changing the gate decision itself.
    """
    if not action_ready or not controller_active or not bt_active:
        return 'controller_action_or_state_insufficient'
    if robot_pose_available:
        return None
    diag = _dict(map_base_diag)
    if not _bool(diag.get('lookup_success')):
        text = ' '.join(str(diag.get(key) or '') for key in ['lookup_exception', 'can_transform_exception']).lower()
        if 'extrapolation' in text or 'cache' in text or 'earlier than all' in text:
            return 'map_base_tf_lookup_exception_or_cache'
        if 'does not exist' in text or 'target_frame' in text or 'source_frame' in text:
            return 'map_base_tf_missing'
        return 'map_base_tf_lookup_exception'
    age = _float(map_base_age_sec)
    if age is not None and age > float(tf_max_age_sec):
        return 'map_base_tf_stale'
    if not _bool(diag.get('finite')):
        return 'map_base_tf_non_finite'
    return 'robot_pose_unavailable_unknown'


def failure_reasons_for_sample(sample: dict[str, Any], config: PreflightConfig) -> dict[str, list[str]]:
    reasons: dict[str, list[str]] = {}

    def add(gate: str, reason: str) -> None:
        reasons.setdefault(gate, []).append(reason)

    if _bool(sample.get('force_timeout')):
        add('preflight', 'preflight_timeout')
        return reasons

    elapsed = _float(sample.get('elapsed_sec')) or 0.0
    map_base = _dict(sample.get('map_base_tf_check'))
    if not _bool(map_base.get('available')):
        add('tf', 'map_base_tf_missing')
    elif not _bool(map_base.get('stable')):
        add('tf', 'map_base_tf_unstable')
    age = _float(sample.get('map_base_tf_age_sec', map_base.get('latest_age_sec')))
    if age is not None and age > config.tf_max_age_sec:
        add('tf', 'map_base_tf_stale')
    if _bool(sample.get('tf_jump_detected')) or _int(sample.get('tf_jump_count')) > 0:
        add('tf', 'tf_jump_detected')
    detector = _dict(sample.get('tf_detector'))
    if _int(detector.get('tf_jump_count')) > 0:
        add('tf', 'tf_detector_jump_count')

    map_odom_age = _float(sample.get('map_odom_tf_age_sec'))
    if map_odom_age is None or map_odom_age > config.tf_max_age_sec:
        add('tf', 'map_odom_tf_stale')
    odom_base_age = _float(sample.get('odom_base_tf_age_sec'))
    if odom_base_age is None or odom_base_age > config.tf_max_age_sec:
        add('tf', 'odom_base_tf_stale')

    scan = _dict(sample.get('scan_transform_check'))
    scan_available = _bool(scan.get('scan_available'))
    if not scan_available:
        if _sample_has_received_scan_evidence(sample):
            add('scan', 'scan_transform_unstable')
        elif elapsed <= config.startup_grace_sec:
            add('scan', 'waiting_for_first_scan')
        else:
            add('scan', 'first_scan_timeout')
    elif (not _bool(scan.get('transform_available')) or not _bool(scan.get('stable'))
          or _bool(scan.get('cache_drop_detected')) or _int(scan.get('exception_count')) > 0
          or _int(detector.get('cache_drop_count')) > 0):
        add('scan', 'scan_transform_unstable')

    controller = _dict(sample.get('controller_pose_check'))
    lifecycle_sources = _lifecycle_sources_from_controller(controller)
    if lifecycle_sources['controller_server']['ambiguous'] or lifecycle_sources['bt_navigator']['ambiguous']:
        add('lifecycle', 'lifecycle_ambiguous')
    elif (not lifecycle_sources['controller_server']['active_confirmed']
          or not lifecycle_sources['bt_navigator']['active_confirmed']
          or not _bool(controller.get('navigate_to_pose_action_ready'))):
        add('controller', 'controller_or_bt_not_active_confirmed')
    if (not _bool(controller.get('robot_pose_available'))
            or _int(controller.get('robot_pose_unavailable_log_count')) > 0
            or _int(detector.get('robot_pose_unavailable_count')) > 0):
        add('controller', 'robot_pose_unavailable')

    goal = _dict(sample.get('goal_pose_transform_check'))
    if (not _bool(goal.get('transform_to_global_costmap_available'))
            or not _bool(goal.get('transform_to_controller_frame_available'))
            or _int(goal.get('exception_count')) > 0
            or _int(detector.get('goal_pose_transform_failure_count')) > 0):
        add('goal_transform', 'goal_pose_transform_unavailable')

    return reasons


def tokens_from_failure_reasons(reasons: dict[str, list[str]]) -> list[str]:
    tokens: list[str] = []
    flat = {reason for values in reasons.values() for reason in values}
    if 'preflight_timeout' in flat:
        tokens.append('ingress_preflight_timeout')
    if 'lifecycle_ambiguous' in flat:
        tokens.append('ingress_lifecycle_ambiguous')
    if 'map_base_tf_missing' in flat:
        tokens.append('ingress_map_base_tf_missing')
    if 'map_base_tf_unstable' in flat or 'map_base_tf_stale' in flat or 'tf_jump_detected' in flat or 'tf_detector_jump_count' in flat:
        tokens.append('ingress_tf_unstable')
    if 'map_odom_tf_stale' in flat:
        tokens.append('ingress_map_odom_tf_stale')
    if 'odom_base_tf_stale' in flat:
        tokens.append('ingress_odom_base_tf_stale')
    if 'waiting_for_first_scan' in flat:
        tokens.append('waiting_for_first_scan')
    if 'first_scan_timeout' in flat:
        tokens.append('ingress_first_scan_timeout')
    if 'scan_transform_unstable' in flat:
        tokens.append('ingress_scan_transform_unstable')
    if 'controller_or_bt_not_active_confirmed' in flat or 'robot_pose_unavailable' in flat:
        tokens.append('ingress_controller_robot_pose_unavailable')
    if 'goal_pose_transform_unavailable' in flat:
        tokens.append('ingress_goal_pose_transform_unavailable')
    return _stable_sort_tokens(tokens)


def failed_tokens_for_sample(sample: dict[str, Any], config: PreflightConfig) -> list[str]:
    return tokens_from_failure_reasons(failure_reasons_for_sample(sample, config))


def _sample_phase(elapsed_sec: float, config: PreflightConfig) -> str:
    return 'startup_grace' if elapsed_sec <= config.startup_grace_sec else 'stable_window'


def build_raw_style_snapshot_cross_check(sample: dict[str, Any], failed_gates: list[str]) -> dict[str, Any]:
    override = _dict(sample.get('raw_style_snapshot_override'))
    map_base = _dict(sample.get('map_base_tf_check'))
    scan = _dict(sample.get('scan_transform_check'))
    controller = _dict(sample.get('controller_pose_check'))
    snapshot: dict[str, Any] = {
        'present': bool(override) or bool(sample),
        'capture_wall_time_sec': sample.get('wall_time_sec'),
        'scan_available': _bool(scan.get('scan_available')),
        'scan_topic': override.get('scan_topic', '/scan'),
        'scan_frame_id': scan.get('scan_frame_id'),
        'map_available': bool(override.get('map_available', False)),
        'map_frame_id': override.get('map_frame_id', 'map' if override.get('map_available') else None),
        'local_costmap_available': bool(override.get('local_costmap_available', False)),
        'local_costmap_frame_id': override.get('local_costmap_frame_id'),
        'odom_available': bool(override.get('odom_available', False)),
        'odom_frame_id': override.get('odom_frame_id'),
        'odom_child_frame_id': override.get('odom_child_frame_id'),
        'tf_pairs_available': dict(override.get('tf_pairs_available') or {'map->base_link': _bool(map_base.get('available'))}),
        'lifecycle_node_graph_present': dict(override.get('lifecycle_node_graph_present') or {}),
        'navigate_to_pose_action_ready': bool(override.get('navigate_to_pose_action_ready', _bool(controller.get('navigate_to_pose_action_ready')))),
        'cross_check_contradictions': [],
        'ambiguous': False,
    }
    contradictions: list[str] = []
    if 'ingress_map_base_tf_missing' in failed_gates and snapshot['tf_pairs_available'].get('map->base_link'):
        contradictions.append('map_base_tf_missing_but_raw_snapshot_has_map_base_tf')
    if 'ingress_scan_transform_unstable' in failed_gates and not _bool(scan.get('scan_available')) and snapshot.get('scan_available'):
        contradictions.append('scan_missing_but_raw_snapshot_has_scan')
    if ('ingress_controller_robot_pose_unavailable' in failed_gates
            and bool(override)
            and snapshot.get('navigate_to_pose_action_ready')
            and (not _bool(controller.get('controller_server_active'))
                 or not _bool(controller.get('bt_navigator_active'))
                 or not _bool(controller.get('navigate_to_pose_action_ready')))):
        contradictions.append('controller_readiness_failure_but_raw_snapshot_has_action_ready')
    if 'ingress_lifecycle_ambiguous' in failed_gates:
        contradictions.append('lifecycle_sources_ambiguous_before_reject')
    snapshot['cross_check_contradictions'] = contradictions
    snapshot['ambiguous'] = bool(contradictions)
    return snapshot


def _annotate_sample(sample: dict[str, Any], config: PreflightConfig, pass_window_start: float | None) -> list[str]:
    elapsed = _float(sample.get('elapsed_sec')) or 0.0
    reasons = failure_reasons_for_sample(sample, config)
    tokens = tokens_from_failure_reasons(reasons)
    sample['phase'] = _sample_phase(elapsed, config)
    sample['failed_gates'] = tokens
    sample['failure_reasons_by_gate'] = reasons
    sample['lifecycle_sources'] = _lifecycle_sources_from_controller(_dict(sample.get('controller_pose_check')))
    sample['lifecycle_ambiguous'] = bool(
        sample['lifecycle_sources']['controller_server']['ambiguous'] or sample['lifecycle_sources']['bt_navigator']['ambiguous']
    )
    sample['stable_window_elapsed_sec'] = 0.0 if pass_window_start is None else max(0.0, elapsed - pass_window_start)
    return tokens


def _complete_sample_history_contract(samples: list[dict[str, Any]], config: PreflightConfig) -> list[dict[str, Any]]:
    """Return samples with Phase109/112 diagnostic fields on every path.

    Runtime evaluation may return early on first-scan timeout or force-timeout,
    leaving later already-collected samples unannotated. Artifact generation must
    be contract-stable, so this completion pass annotates every sample without
    changing the fail-closed decision captured in PreflightResult.
    """
    completed = [dict(sample) for sample in samples]
    pass_window_start: float | None = None
    last_elapsed = 0.0
    for sample in completed:
        elapsed = _float(sample.get('elapsed_sec'))
        if elapsed is None:
            elapsed = last_elapsed
            sample['elapsed_sec'] = elapsed
        last_elapsed = max(last_elapsed, elapsed)
        tokens = _annotate_sample(sample, config, pass_window_start)
        reject_tokens = [token for token in tokens if token not in SAMPLE_ONLY_TOKENS]
        if reject_tokens or tokens:
            pass_window_start = None
            continue
        if pass_window_start is None:
            pass_window_start = elapsed
            sample['stable_window_elapsed_sec'] = 0.0
        else:
            sample['stable_window_elapsed_sec'] = max(0.0, elapsed - pass_window_start)
    return completed


def evaluate_preflight_samples(samples: list[dict[str, Any]], config: PreflightConfig | None = None) -> PreflightResult:
    config = config or PreflightConfig()
    normalized = [dict(sample) for sample in samples]
    start_time = _float(normalized[0].get('wall_time_sec')) if normalized else None
    end_time = _float(normalized[-1].get('wall_time_sec')) if normalized else None
    all_failed: list[str] = []
    pass_window_start: float | None = None
    last_elapsed = 0.0
    last_specific: str | None = None

    for sample in normalized:
        elapsed = _float(sample.get('elapsed_sec'))
        if elapsed is None:
            elapsed = last_elapsed
            sample['elapsed_sec'] = elapsed
        last_elapsed = max(last_elapsed, elapsed)
        tokens = _annotate_sample(sample, config, pass_window_start)
        reject_tokens = [t for t in tokens if t not in SAMPLE_ONLY_TOKENS]
        if reject_tokens:
            all_failed.extend(reject_tokens)
            last_specific = _first_priority([t for t in reject_tokens if t != 'ingress_preflight_timeout']) or last_specific
            pass_window_start = None
            if 'ingress_preflight_timeout' in reject_tokens or 'ingress_first_scan_timeout' in reject_tokens:
                reason = 'ingress_first_scan_timeout' if 'ingress_first_scan_timeout' in reject_tokens else 'ingress_preflight_timeout'
                return PreflightResult(
                    passed=False,
                    reject_reason=reason,
                    failed_gates=_stable_sort_tokens(all_failed),
                    passed_gates=[],
                    samples=normalized,
                    config=config,
                    bounded_wait_elapsed_sec=last_elapsed,
                    start_time_sec=start_time,
                    end_time_sec=end_time,
                    last_specific_reject_reason=last_specific or reason,
                )
            continue
        if tokens:  # waiting_for_first_scan only: not passable and not final failure during grace.
            pass_window_start = None
            continue
        if pass_window_start is None:
            pass_window_start = elapsed
            sample['stable_window_elapsed_sec'] = 0.0
        else:
            sample['stable_window_elapsed_sec'] = max(0.0, elapsed - pass_window_start)
        if elapsed - pass_window_start >= config.tf_stability_window_sec:
            sample['raw_style_snapshot_cross_check'] = build_raw_style_snapshot_cross_check(sample, [])
            return PreflightResult(
                passed=True,
                reject_reason=None,
                failed_gates=[],
                passed_gates=PASSED_GATES,
                samples=normalized,
                config=config,
                bounded_wait_elapsed_sec=last_elapsed,
                start_time_sec=start_time,
                end_time_sec=end_time,
                last_specific_reject_reason=None,
            )

    latest_tokens = []
    if normalized:
        latest_tokens = [token for token in normalized[-1].get('failed_gates', []) if token not in SAMPLE_ONLY_TOKENS]
    if pass_window_start is not None and not latest_tokens:
        # A prior transient miss recovered, but the later good samples did not
        # cover the requested stable window. Do not report the early miss as the
        # final blocker.
        reason = 'ingress_tf_stable_window_not_met'
        all_failed = [reason]
        last_specific = reason
    elif all_failed:
        reason = _first_priority(all_failed)
    else:
        reason = 'ingress_tf_stable_window_not_met'
        all_failed.append(reason)
    if normalized:
        normalized[-1]['raw_style_snapshot_cross_check'] = build_raw_style_snapshot_cross_check(normalized[-1], _stable_sort_tokens(all_failed))
        if normalized[-1]['raw_style_snapshot_cross_check'].get('ambiguous'):
            all_failed.append('ingress_raw_snapshot_cross_check_failed')
            reason = _first_priority(all_failed)
    return PreflightResult(
        passed=False,
        reject_reason=reason,
        failed_gates=_stable_sort_tokens(all_failed),
        passed_gates=[],
        samples=normalized,
        config=config,
        bounded_wait_elapsed_sec=last_elapsed,
        start_time_sec=start_time,
        end_time_sec=end_time,
        last_specific_reject_reason=last_specific or reason,
    )


def mark_wrapper_state(path: Path, *, ingress_goal_sent: bool, maze_explorer_started: bool) -> None:
    data = _read_json(path)
    preflight = _dict(data.get('ingress_preflight'))
    preflight['ingress_goal_sent'] = bool(ingress_goal_sent)
    preflight['maze_explorer_started'] = bool(maze_explorer_started)
    data['ingress_preflight'] = preflight
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def write_preflight_artifact(result: PreflightResult, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result.to_artifact(), indent=2, sort_keys=True) + '\n')


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists() or not path.stat().st_size:
        return {}
    try:
        value = json.loads(path.read_text(encoding='utf-8', errors='replace'))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _stamp_age_sec(node: Any, stamp: Any) -> float | None:
    try:
        sec = int(getattr(stamp, 'sec', 0)) + int(getattr(stamp, 'nanosec', 0)) / 1_000_000_000.0
        if sec <= 0:
            return None
        now_msg = node.get_clock().now().to_msg()
        now = int(now_msg.sec) + int(now_msg.nanosec) / 1_000_000_000.0
        return max(0.0, now - sec)
    except Exception:
        return None


def _finite_transform(transform: Any) -> bool:
    try:
        t = transform.transform.translation
        r = transform.transform.rotation
        vals = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
        return all(math.isfinite(float(v)) for v in vals)
    except Exception:
        return False


def _ros_graph_names() -> set[str]:
    try:
        proc = subprocess.run(['ros2', 'node', 'list'], text=True, capture_output=True, timeout=2.0, check=False)
        return {line.strip().lstrip('/') for line in proc.stdout.splitlines() if line.strip()}
    except Exception:
        return set()


def _ros_action_ready(action_name: str = '/navigate_to_pose') -> bool:
    try:
        proc = subprocess.run(['ros2', 'action', 'list'], text=True, capture_output=True, timeout=2.0, check=False)
        return action_name in {line.strip() for line in proc.stdout.splitlines() if line.strip()}
    except Exception:
        return False


def _ros_services() -> set[str]:
    try:
        proc = subprocess.run(['ros2', 'service', 'list'], text=True, capture_output=True, timeout=2.0, check=False)
        return {line.strip() for line in proc.stdout.splitlines() if line.strip()}
    except Exception:
        return set()


def _lifecycle_service_present(node_name: str, services: set[str]) -> bool:
    normalized = node_name if node_name.startswith('/') else f'/{node_name}'
    candidates = {
        f'{normalized}/get_state',
        f'{normalized}/change_state',
    }
    return any(candidate in services for candidate in candidates)


def _launch_active_marker(path_value: str | None) -> dict[str, Any]:
    if not path_value:
        return {'present': False, 'path': None, 'token': None}
    path = Path(path_value)
    try:
        text = path.read_text(encoding='utf-8', errors='replace')[-20000:]
    except Exception as exc:
        return {'present': False, 'path': str(path), 'token': None, 'error': str(exc)}
    tokens = ['Managed nodes are active', 'all lifecycle nodes reached active state']
    for token in tokens:
        if token in text:
            return {'present': True, 'path': str(path), 'token': token}
    return {'present': False, 'path': str(path), 'token': None}


def _ros_lifecycle_query(node_name: str, timeout_sec: float = 2.0) -> dict[str, Any]:
    """Return raw lifecycle subprocess evidence for multi-source confirmation."""
    normalized = node_name if node_name.startswith('/') else f'/{node_name}'
    command = ['ros2', 'lifecycle', 'get', normalized]
    started = time.monotonic()
    try:
        proc = subprocess.run(command, text=True, capture_output=True, timeout=float(timeout_sec), check=False)
        duration = time.monotonic() - started
        text = f'{proc.stdout}\n{proc.stderr}'.lower()
        state = None
        for candidate in ['active', 'inactive', 'unconfigured', 'finalized']:
            if candidate in text:
                state = candidate
                break
        return {
            'command': command,
            'timeout_sec': float(timeout_sec),
            'duration_sec': duration,
            'returncode': proc.returncode,
            'state': state,
            'stdout': proc.stdout,
            'stderr': proc.stderr,
            'timed_out': False,
            'query_error': None if proc.returncode == 0 else (proc.stderr or proc.stdout or f'returncode={proc.returncode}'),
        }
    except subprocess.TimeoutExpired as exc:
        duration = time.monotonic() - started
        stdout = exc.output if isinstance(exc.output, str) else (exc.output.decode(errors='replace') if exc.output else '')
        stderr = exc.stderr if isinstance(exc.stderr, str) else (exc.stderr.decode(errors='replace') if exc.stderr else '')
        return {
            'command': command,
            'timeout_sec': float(timeout_sec),
            'duration_sec': duration,
            'returncode': None,
            'state': None,
            'stdout': stdout,
            'stderr': stderr,
            'timed_out': True,
            'query_error': f'timeout after {float(timeout_sec)} sec',
        }
    except Exception as exc:
        duration = time.monotonic() - started
        return {
            'command': command,
            'timeout_sec': float(timeout_sec),
            'duration_sec': duration,
            'returncode': 1,
            'state': None,
            'stdout': '',
            'stderr': str(exc),
            'timed_out': False,
            'query_error': str(exc),
        }


def _ros_lifecycle_active(node_name: str) -> bool:
    query = _ros_lifecycle_query(node_name)
    return query.get('returncode') == 0 and query.get('state') == 'active'


def _ros_param_value(node_name: str, param_name: str, default: str) -> str:
    try:
        proc = subprocess.run(['ros2', 'param', 'get', node_name, param_name], text=True, capture_output=True, timeout=2.0, check=False)
        if proc.returncode != 0:
            return default
        # Typical output: "String value is: map".
        text = proc.stdout.strip()
        if ':' in text:
            value = text.rsplit(':', 1)[-1].strip().strip('"\'')
            return value or default
        return text or default
    except Exception:
        return default


class _RuntimeSampler:  # pragma: no cover - exercised only in ROS dry/smoke runtime.
    def __init__(self, config: PreflightConfig):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        from rclpy.parameter import Parameter
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        class NodeImpl(Node):
            pass

        self.rclpy = rclpy
        self.ActionClient = ActionClient
        self.NavigateToPose = NavigateToPose
        self.Duration = Duration
        self.node = NodeImpl('phase105_inner_ingress_preflight', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, bool(config.use_sim_time))])
        self.config = config
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.scan_msg = None
        self.scan_callback_count = 0
        self.scan_last_received_wall_time_sec = None
        self.spin_once_count = 0
        self.scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.node.create_subscription(LaserScan, '/scan', self._on_scan, self.scan_qos)
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._executor_stop = threading.Event()
        self.background_executor_thread = threading.Thread(target=self._background_executor_loop, name='phase114_preflight_background_executor_thread', daemon=True)
        self.executor_thread = self.background_executor_thread
        self.background_executor_thread.start()
        self._spin_callbacks(config.spin_warmup_sec)

    def _on_scan(self, msg: Any) -> None:
        self.scan_msg = msg
        self.scan_callback_count += 1
        self.scan_last_received_wall_time_sec = _now_wall()

    def _spin_callbacks(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        ran_once = False
        while time.monotonic() < deadline or not ran_once:
            time.sleep(0.05)
            self.spin_once_count += 1
            ran_once = True
            if self.scan_callback_count > 0 and self.node.get_clock().now().nanoseconds > 0:
                # Once sim time and at least one sensor callback have arrived,
                # further bounded sampling can proceed without burning the whole warmup.
                break

    def _background_executor_loop(self) -> None:
        while not self._executor_stop.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.05)
                self.spin_once_count += 1
            except Exception:
                if self._executor_stop.is_set():
                    break
                time.sleep(0.05)

    def _lookup(self, parent: str, child: str) -> tuple[bool, float | None, str | None, bool]:
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, self.rclpy.time.Time())
            return True, _stamp_age_sec(self.node, transform.header.stamp), None, _finite_transform(transform)
        except Exception as exc:
            return False, None, str(exc), False

    def _tf_buffer_frames(self) -> str:
        try:
            return str(self.tf_buffer.all_frames_as_string())
        except Exception as exc:
            return f'<frame_list_unavailable: {exc}>'

    def _tf_diag(self, parent: str, child: str) -> dict[str, Any]:
        wall_time = _now_wall()
        ros_time = None
        try:
            now_msg = self.node.get_clock().now().to_msg()
            ros_time = int(now_msg.sec) + int(now_msg.nanosec) / 1_000_000_000.0
        except Exception:
            pass
        diag: dict[str, Any] = {
            'parent': parent,
            'child': child,
            'can_transform': False,
            'lookup_success': False,
            'can_transform_exception': None,
            'lookup_exception': None,
            'age_sec': None,
            'finite': False,
            'sample_wall_time_sec': wall_time,
            'sample_ros_time_sec': ros_time,
            'can_transform_duration_sec': None,
            'lookup_duration_sec': None,
            'failure_reason': None,
            'buffer_frames': None,
        }
        started = time.monotonic()
        try:
            diag['can_transform'] = bool(self.tf_buffer.can_transform(parent, child, self.rclpy.time.Time(), timeout=self.Duration(seconds=0.05)))
        except Exception as exc:
            diag['can_transform_exception'] = str(exc)
        finally:
            diag['can_transform_duration_sec'] = time.monotonic() - started
        started = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, self.rclpy.time.Time())
            diag['lookup_success'] = True
            diag['age_sec'] = _stamp_age_sec(self.node, transform.header.stamp)
            diag['finite'] = _finite_transform(transform)
        except Exception as exc:
            diag['lookup_exception'] = str(exc)
        finally:
            diag['lookup_duration_sec'] = time.monotonic() - started
        if not diag['lookup_success'] or not diag['can_transform']:
            text = ' '.join(str(diag.get(key) or '') for key in ['lookup_exception', 'can_transform_exception']).lower()
            if 'does not exist' in text or 'target_frame' in text or 'source_frame' in text:
                diag['failure_reason'] = 'frame_missing_or_mismatch'
            elif 'extrapolation' in text or 'earlier than all' in text or 'cache' in text:
                diag['failure_reason'] = 'tf_cache_or_extrapolation'
            else:
                diag['failure_reason'] = 'can_transform_or_lookup_failed'
            diag['buffer_frames'] = self._tf_buffer_frames()
        return diag

    def _can_transform(self, parent: str, child: str) -> bool:
        return bool(self._tf_diag(parent, child).get('can_transform'))

    def _stamp_sec(self, stamp: Any) -> float | None:
        try:
            sec = int(getattr(stamp, 'sec', 0)) + int(getattr(stamp, 'nanosec', 0)) / 1_000_000_000.0
            return sec if sec > 0 else None
        except Exception:
            return None

    def _clock_diag(self) -> dict[str, Any]:
        try:
            ros_msg = self.node.get_clock().now().to_msg()
            ros_time = int(ros_msg.sec) + int(ros_msg.nanosec) / 1_000_000_000.0
        except Exception:
            ros_time = None
        use_sim_time = None
        try:
            use_sim_time = bool(self.node.get_parameter('use_sim_time').value)
        except Exception:
            pass
        return {
            'use_sim_time': use_sim_time,
            'ros_time_sec': ros_time,
            'wall_time_sec': _now_wall(),
        }

    def sample(self, elapsed_sec: float) -> dict[str, Any]:
        callback_count_before = self.scan_callback_count
        self._spin_callbacks(0.1)
        map_base_ok, map_base_age, map_base_exc, finite = self._lookup('map', 'base_link')
        map_odom_ok, map_odom_age, map_odom_exc, _ = self._lookup('map', 'odom')
        odom_base_ok, odom_base_age, odom_base_exc, _ = self._lookup('odom', 'base_link')
        map_base_diag = self._tf_diag('map', 'base_link')
        map_odom_diag = self._tf_diag('map', 'odom')
        odom_base_diag = self._tf_diag('odom', 'base_link')
        scan_frame = None
        scan_age = None
        scan_available = self.scan_msg is not None
        scan_transform_ok = False
        scan_exception = None
        scan_tf_diag = {'parent': 'map', 'child': None, 'can_transform': False, 'lookup_success': False, 'lookup_exception': 'no_scan_frame'}
        if self.scan_msg is not None:
            scan_frame = getattr(self.scan_msg.header, 'frame_id', None)
            scan_age = _stamp_age_sec(self.node, self.scan_msg.header.stamp)
            if scan_frame:
                scan_tf_diag = self._tf_diag('map', scan_frame)
                scan_transform_ok = bool(scan_tf_diag.get('can_transform'))
                scan_exception = scan_tf_diag.get('lookup_exception') or scan_tf_diag.get('can_transform_exception')
        # Lifecycle is checked from several sources and kept ambiguous on conflict.
        names = _ros_graph_names()
        action_ready = _ros_action_ready('/navigate_to_pose')
        services = _ros_services()
        launch_marker = _launch_active_marker(self.config.launch_log_path)
        controller_lifecycle = derive_lifecycle_confirmation(
            lifecycle_query=_ros_lifecycle_query('/controller_server', timeout_sec=self.config.lifecycle_query_timeout_sec),
            node_graph_present='controller_server' in names,
            action_available=action_ready,
            lifecycle_service_present=_lifecycle_service_present('/controller_server', services),
            launch_active_marker_present=_bool(launch_marker.get('present')),
            launch_active_marker_token=launch_marker.get('token'),
        )
        bt_lifecycle = derive_lifecycle_confirmation(
            lifecycle_query=_ros_lifecycle_query('/bt_navigator', timeout_sec=self.config.lifecycle_query_timeout_sec),
            node_graph_present='bt_navigator' in names,
            action_available=action_ready,
            lifecycle_service_present=_lifecycle_service_present('/bt_navigator', services),
            launch_active_marker_present=_bool(launch_marker.get('present')),
            launch_active_marker_token=launch_marker.get('token'),
        )
        controller_active = controller_lifecycle['active_confirmed']
        bt_active = bt_lifecycle['active_confirmed']
        robot_pose_available = bool(map_base_ok and finite)
        robot_pose_unavailable_reason = derive_controller_pose_unavailable_reason(
            map_base_diag=map_base_diag,
            map_base_age_sec=map_base_age,
            robot_pose_available=robot_pose_available,
            controller_active=controller_active,
            bt_active=bt_active,
            action_ready=action_ready,
            tf_max_age_sec=self.config.tf_max_age_sec,
        )
        global_costmap_frame = _ros_param_value('/global_costmap/global_costmap', 'global_frame', 'map')
        controller_frame = _ros_param_value('/local_costmap/local_costmap', 'global_frame', 'odom')
        goal_to_global_ok = INNER_INGRESS_GOAL_POSE['frame_id'] == global_costmap_frame or self._can_transform(global_costmap_frame, INNER_INGRESS_GOAL_POSE['frame_id'])
        goal_to_controller_ok = INNER_INGRESS_GOAL_POSE['frame_id'] == controller_frame or self._can_transform(controller_frame, INNER_INGRESS_GOAL_POSE['frame_id'])
        tf_exception_text = ' '.join(str(x or '') for x in [map_base_exc, map_odom_exc, odom_base_exc, scan_exception]).lower()
        cache_drop = 'cache' in tf_exception_text and ('drop' in tf_exception_text or 'earlier than all' in tf_exception_text)
        jump = 'jump back' in tf_exception_text or 'time reset' in tf_exception_text
        return {
            'elapsed_sec': elapsed_sec,
            'wall_time_sec': _now_wall(),
            'map_base_tf_check': {'available': map_base_ok, 'stable': bool(map_base_ok and finite), 'sample_count': 1, 'latest_age_sec': map_base_age},
            'map_base_tf_age_sec': map_base_age,
            'map_odom_tf_age_sec': map_odom_age if map_odom_ok else None,
            'odom_base_tf_age_sec': odom_base_age if odom_base_ok else None,
            'tf_jump_detected': jump,
            'tf_jump_count': 1 if jump else 0,
            'scan_transform_check': {
                'scan_available': scan_available and (scan_age is None or scan_age <= self.config.scan_max_age_sec),
                'scan_frame_id': scan_frame,
                'target_frame': 'map',
                'transform_available': scan_transform_ok,
                'stable': bool(scan_transform_ok and not cache_drop),
                'cache_drop_detected': cache_drop,
                'exception_count': 1 if scan_exception else 0,
                'age_sec': scan_age,
                'transform_age_sec': scan_tf_diag.get('age_sec'),
                'failure_reason': None if scan_transform_ok else (scan_tf_diag.get('failure_reason') or scan_exception or 'scan_transform_unavailable'),
                'scan_transform_failure_reason': None if scan_transform_ok else (scan_tf_diag.get('failure_reason') or scan_exception or 'scan_transform_unavailable'),
            },
            'controller_pose_check': {
                'controller_server_active': controller_active,
                'bt_navigator_active': bt_active,
                'navigate_to_pose_action_ready': action_ready,
                'robot_pose_available': robot_pose_available,
                'robot_pose_unavailable_log_count': 0 if robot_pose_available else 1,
                'robot_pose_unavailable_reason': robot_pose_unavailable_reason,
                'lifecycle_sources': {
                    'controller_server': controller_lifecycle,
                    'bt_navigator': bt_lifecycle,
                },
            },
            'goal_pose_transform_check': {
                'goal_frame_id': INNER_INGRESS_GOAL_POSE['frame_id'],
                'global_costmap_frame': global_costmap_frame,
                'controller_frame': controller_frame,
                'transform_to_global_costmap_available': goal_to_global_ok,
                'transform_to_controller_frame_available': goal_to_controller_ok,
                'exception_count': 0 if (goal_to_global_ok and goal_to_controller_ok) else 1,
            },
            'tf_detector': {
                'tf_jump_count': 1 if jump else 0,
                'cache_drop_count': 1 if cache_drop else 0,
                'robot_pose_unavailable_count': 0 if robot_pose_available else 1,
                'goal_pose_transform_failure_count': 0 if (goal_to_global_ok and goal_to_controller_ok) else 1,
            },
            'internal_sampling_diagnostics': {
                'sample_index': self.spin_once_count - 1,
                'executor': {
                    'spin_once_count_total': self.spin_once_count,
                    'last_spin_timeout_sec': 0.05,
                },
                'clock': self._clock_diag(),
                'scan_subscription': {
                    'topic': '/scan',
                    'publisher_count': self.node.count_publishers('/scan'),
                    'subscriber_count': self.node.count_subscribers('/scan'),
                    'callback_count_total': self.scan_callback_count,
                    'callbacks_since_last_sample': max(0, self.scan_callback_count - callback_count_before),
                    'latest_frame_id': scan_frame,
                    'latest_stamp_sec': self._stamp_sec(self.scan_msg.header.stamp) if self.scan_msg is not None else None,
                    'latest_age_sec': scan_age,
                    'latest_received_wall_time_sec': self.scan_last_received_wall_time_sec,
                },
                'tf_buffer': {
                    'map->base_link': map_base_diag,
                    'map->odom': map_odom_diag,
                    'odom->base_link': odom_base_diag,
                    f'map->{scan_frame}' if scan_frame else 'map-><no_scan_frame>': scan_tf_diag,
                },
                'tf_buffer_frame_list': self._tf_buffer_frames(),
                'controller_direct_reason': robot_pose_unavailable_reason,
                'scan_transform_failure_reason': None if scan_transform_ok else (scan_tf_diag.get('failure_reason') or scan_exception or 'scan_transform_unavailable'),
                'lifecycle': {
                    'controller_server': controller_lifecycle,
                    'bt_navigator': bt_lifecycle,
                    'node_graph_names': sorted(names),
                    'service_names': sorted(services),
                    'launch_active_marker': launch_marker,
                },
            },
        }

    def close(self) -> None:
        try:
            self._executor_stop.set()
        except Exception:
            pass
        try:
            self.executor.wake()
        except Exception:
            pass
        try:
            self.executor_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


def run_bounded_runtime(config: PreflightConfig) -> PreflightResult:  # pragma: no cover
    import rclpy
    rclpy.init(args=None)
    sampler = _RuntimeSampler(config)
    start = time.monotonic()
    samples: list[dict[str, Any]] = []
    try:
        while time.monotonic() - start < config.timeout_sec:
            elapsed = time.monotonic() - start
            samples.append(sampler.sample(elapsed))
            partial = evaluate_preflight_samples(samples, config)
            if partial.passed:
                return partial
            time.sleep(max(0.05, config.sample_period_sec))
        elapsed = time.monotonic() - start
        if samples:
            samples[-1]['force_timeout'] = True
            samples[-1]['elapsed_sec'] = elapsed
        else:
            samples.append(make_synthetic_sample(force_timeout=True, elapsed_sec=elapsed))
        return evaluate_preflight_samples(samples, config)
    finally:
        sampler.close()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    parser.add_argument('--tf-stability-window-sec', type=float, default=2.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--tf-max-age-sec', type=float, default=1.5)
    parser.add_argument('--scan-max-age-sec', type=float, default=1.5)
    parser.add_argument('--startup-grace-sec', type=float, default=2.0)
    parser.add_argument('--spin-warmup-sec', type=float, default=0.5)
    parser.add_argument('--lifecycle-query-timeout-sec', type=float, default=2.0)
    parser.add_argument('--launch-log-path', help='optional launch log used only as one strict lifecycle-active confirmation source')
    sim_time_group = parser.add_mutually_exclusive_group()
    sim_time_group.add_argument('--use-sim-time', dest='use_sim_time', action='store_true', default=True, help='run runtime preflight node with use_sim_time=true (default for Gazebo/Nav2 simulation)')
    sim_time_group.add_argument('--no-use-sim-time', dest='use_sim_time', action='store_false', help='diagnostic override: run runtime preflight node with wall time')
    parser.add_argument('--synthetic-pass', action='store_true', help='unit/dry-run mode: emit a passing artifact without ROS')
    parser.add_argument('--synthetic-reject-token', choices=sorted(REJECT_TOKENS), help='unit/dry-run mode: emit a rejecting artifact without ROS')
    parser.add_argument('--mark-wrapper-state', action='store_true')
    parser.add_argument('--ingress-goal-sent', choices=['true', 'false'])
    parser.add_argument('--maze-explorer-started', choices=['true', 'false'])
    args = parser.parse_args()

    if args.mark_wrapper_state:
        mark_wrapper_state(
            args.output,
            ingress_goal_sent=args.ingress_goal_sent == 'true',
            maze_explorer_started=args.maze_explorer_started == 'true',
        )
        return 0

    config = PreflightConfig(
        tf_stability_window_sec=args.tf_stability_window_sec,
        timeout_sec=args.timeout_sec,
        sample_period_sec=args.sample_period_sec,
        tf_max_age_sec=args.tf_max_age_sec,
        scan_max_age_sec=args.scan_max_age_sec,
        startup_grace_sec=args.startup_grace_sec,
        use_sim_time=args.use_sim_time,
        spin_warmup_sec=args.spin_warmup_sec,
        lifecycle_query_timeout_sec=args.lifecycle_query_timeout_sec,
        launch_log_path=args.launch_log_path,
    )
    if args.synthetic_pass:
        samples = [make_synthetic_sample(elapsed_sec=0.0), make_synthetic_sample(elapsed_sec=config.tf_stability_window_sec + 0.1)]
        result = evaluate_preflight_samples(samples, config)
    elif args.synthetic_reject_token:
        token = args.synthetic_reject_token
        sample = make_synthetic_sample(elapsed_sec=0.0)
        if token == 'ingress_map_base_tf_missing':
            sample['map_base_tf_check'] = {'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}
        elif token == 'ingress_tf_unstable':
            sample['tf_jump_detected'] = True
            sample['tf_jump_count'] = 1
        elif token == 'ingress_map_odom_tf_stale':
            sample['map_odom_tf_age_sec'] = config.tf_max_age_sec + 1.0
        elif token == 'ingress_odom_base_tf_stale':
            sample['odom_base_tf_age_sec'] = config.tf_max_age_sec + 1.0
        elif token == 'ingress_scan_transform_unstable':
            sample['scan_transform_check'] = {'scan_available': True, 'scan_frame_id': 'lidar', 'target_frame': 'map', 'transform_available': False, 'stable': False, 'cache_drop_detected': True, 'exception_count': 1}
        elif token == 'ingress_controller_robot_pose_unavailable':
            sample['controller_pose_check'] = {'controller_server_active': True, 'bt_navigator_active': True, 'navigate_to_pose_action_ready': True, 'robot_pose_available': False, 'robot_pose_unavailable_log_count': 1}
        elif token == 'ingress_goal_pose_transform_unavailable':
            sample['goal_pose_transform_check'] = {'goal_frame_id': 'map', 'global_costmap_frame': 'map', 'controller_frame': 'odom', 'transform_to_global_costmap_available': True, 'transform_to_controller_frame_available': False, 'exception_count': 1}
        elif token == 'ingress_preflight_timeout':
            sample['force_timeout'] = True
        result = evaluate_preflight_samples([sample], config)
    else:
        result = run_bounded_runtime(config)

    write_preflight_artifact(result, args.output)
    print(json.dumps({'passed': result.passed, 'reject_reason': result.reject_reason, 'failed_gates': result.failed_gates}, sort_keys=True))
    return 0 if result.passed else 3


if __name__ == '__main__':
    raise SystemExit(main())
