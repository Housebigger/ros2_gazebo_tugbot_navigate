#!/usr/bin/env python3
"""Phase127 first-goal timeout artifact replay diagnosis.

Diagnosis-only offline analyzer. It reads Phase125 artifacts/logs and does not launch
ROS/Gazebo/Nav2, send goals, start maze_explorer, tune parameters, or change any
navigation/exploration logic.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase127'
MODE = 'first_goal_timeout_artifact_replay'

CLASSIFICATIONS = [
    'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
    'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL',
    'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE',
    'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY',
    'INSUFFICIENT_TIMEOUT_EVIDENCE',
]

FORBIDDEN_RUNTIME_ACTIONS = {
    'launch_runtime': False,
    'send_navigate_to_pose': False,
    'start_maze_explorer': False,
    'send_second_exploration_goal': False,
    'tune_nav2_mppi_controller_goal_checker_config': False,
    'change_exploration_strategy': False,
    'claim_autonomous_or_exit_success': False,
}

PHASE125_TIMEOUT_CLASSIFICATION = 'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL'

LOCAL_COST_RE = re.compile(
    r'phase23 timeout local cost seq=(?P<seq>\d+)\s+'
    r'footprint_max=(?P<footprint_max>-?\d+)\s+'
    r'front_max=(?P<front_max>-?\d+)\s+'
    r'path_0_5m_max=(?P<path_0_5m_max>-?\d+)\s+'
    r'path_1_0m_max=(?P<path_1_0m_max>-?\d+)'
)


def _safe_get(obj: Any, path: list[Any], default: Any = None) -> Any:
    cur = obj
    for key in path:
        if isinstance(cur, dict):
            cur = cur.get(key, default)
        elif isinstance(cur, list) and isinstance(key, int) and 0 <= key < len(cur):
            cur = cur[key]
        else:
            return default
        if cur is default:
            return default
    return cur


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        f = float(value)
        if math.isnan(f):
            return default
        return f
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int | None = None) -> int | None:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def parse_timeout_local_cost_line(text: str) -> dict[str, int] | None:
    """Parse the Phase23 timeout local-cost diagnostic line from maze_explorer stderr."""
    if not text:
        return None
    match = LOCAL_COST_RE.search(text)
    if not match:
        return None
    return {key: int(value) for key, value in match.groupdict().items()}


def _feedback_evidence(first_goal: dict[str, Any], artifact: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    gaps: list[str] = []
    feedback = first_goal.get('nav2_feedback_timeline') or _safe_get(artifact, ['first_goal_result_artifact', 'nav2_feedback'], []) or []
    distances = [_as_float(item.get('distance_remaining')) for item in feedback if isinstance(item, dict)]
    distances = [d for d in distances if d is not None]
    recoveries = [_as_int(item.get('number_of_recoveries')) for item in feedback if isinstance(item, dict)]
    recoveries = [r for r in recoveries if r is not None]
    nav_times = [_as_float(item.get('navigation_time_sec')) for item in feedback if isinstance(item, dict)]
    nav_times = [t for t in nav_times if t is not None]
    if not feedback:
        gaps.append('Nav2 feedback timeline missing')
    distance_delta = None
    plateau = False
    if len(distances) >= 2:
        distance_delta = distances[-1] - distances[0]
        plateau = abs(distance_delta) <= 0.05
    elif distances:
        gaps.append('Nav2 feedback timeline too short for distance-to-goal curve')
    return {
        'count': len(feedback),
        'first_distance_remaining': distances[0] if distances else None,
        'last_distance_remaining': distances[-1] if distances else None,
        'distance_delta': distance_delta,
        'distance_plateau': plateau,
        'max_recoveries': max(recoveries) if recoveries else 0,
        'first_navigation_time_sec': nav_times[0] if nav_times else None,
        'last_navigation_time_sec': nav_times[-1] if nav_times else None,
        'navigation_time_span_sec': (nav_times[-1] - nav_times[0]) if len(nav_times) >= 2 else None,
    }, gaps


def _local_cost_evidence(artifact: dict[str, Any], stderr_text: str) -> tuple[dict[str, Any], list[str]]:
    gaps: list[str] = []
    parsed = parse_timeout_local_cost_line(stderr_text or '')
    if not parsed:
        gaps.append('timeout local cost line missing')
    local_samples = _safe_get(artifact, ['first_goal_result_artifact', 'local_costmap_samples'], []) or []
    costmap_freshness = _safe_get(artifact, ['first_goal_result_artifact', 'costmap_freshness'], {}) or {}
    if not local_samples:
        gaps.append('local costmap samples missing')
    if not costmap_freshness.get('fresh'):
        gaps.append('costmap freshness missing or stale')
    supports = False
    severity = 'none'
    if parsed:
        footprint = parsed['footprint_max']
        front = parsed['front_max']
        path05 = parsed['path_0_5m_max']
        path10 = parsed['path_1_0m_max']
        supports = (footprint >= 90 or front >= 90) and (path05 >= 70 or path10 >= 70)
        if supports:
            severity = 'high_front_or_footprint_and_path_cost'
        elif max(footprint, front, path05, path10) >= 90:
            severity = 'high_cost_without_path_confirmation'
    # Phase125 artifact has costmap sample metadata but not full windows/cell arrays.
    gaps.append('local/global costmap windows unavailable in Phase125 artifact')
    return {
        'timeout_local_cost': parsed,
        'local_costmap_sample_count': len(local_samples),
        'costmap_fresh': bool(costmap_freshness.get('fresh')),
        'supports_local_cost_blocked': supports,
        'severity': severity,
    }, gaps


def _selected_branch(first_goal: dict[str, Any]) -> dict[str, Any] | None:
    frontier = first_goal.get('frontier_evidence') or {}
    branches = frontier.get('candidate_branches') or []
    chosen_rank = frontier.get('chosen_branch_rank') or _safe_get(first_goal, ['candidate', 'candidate_rank'])
    for branch in branches:
        if branch.get('rank') == chosen_rank or branch.get('rejection_reason') is None:
            return branch
    return branches[0] if branches else None


def _candidate_risk_evidence(first_goal: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    gaps: list[str] = []
    frontier = first_goal.get('frontier_evidence') or {}
    branches = frontier.get('candidate_branches') or []
    selected = _selected_branch(first_goal)
    if not branches:
        gaps.append('candidate branch evidence missing')
    if not selected:
        gaps.append('selected candidate evidence missing')
        return {
            'supports_candidate_too_risky': False,
            'selected_branch': None,
            'selected_rank': None,
            'safer_lower_rank_exists': False,
            'risk_reasons': [],
        }, gaps
    selected_rank = selected.get('rank')
    target_local_cost = _as_float(selected.get('target_local_cost'), 0.0) or 0.0
    target_max_radius = _as_float(selected.get('target_local_cost_max_radius'), 0.0) or 0.0
    corridor_clearance = _as_float(selected.get('path_corridor_min_clearance_m'))
    target_clearance = _as_float(selected.get('target_clearance_m'))
    path_cost = _as_float(selected.get('dispatch_path_local_cost_max'), 0.0) or 0.0
    risk_reasons: list[str] = []
    if target_local_cost >= 90:
        risk_reasons.append('selected target_local_cost high')
    if target_max_radius >= 90:
        risk_reasons.append('selected target_local_cost_max_radius high')
    if path_cost >= 90:
        risk_reasons.append('selected dispatch_path_local_cost_max high')
    if corridor_clearance is not None and corridor_clearance < 0.35:
        risk_reasons.append('selected path_corridor_min_clearance_m low')
    if target_clearance is not None and target_clearance < 0.35:
        risk_reasons.append('selected target_clearance_m low')
    safer_lower_rank_exists = False
    for branch in branches:
        if branch is selected:
            continue
        other_rank = branch.get('rank')
        # "lower-ranked" means a numerically worse rank than the selected one.
        if isinstance(other_rank, int) and isinstance(selected_rank, int) and other_rank <= selected_rank:
            continue
        other_cost = _as_float(branch.get('target_local_cost'), 999.0) or 999.0
        other_max = _as_float(branch.get('target_local_cost_max_radius'), 999.0) or 999.0
        other_path = _as_float(branch.get('dispatch_path_local_cost_max'), 999.0) or 999.0
        other_clearance = _as_float(branch.get('path_corridor_min_clearance_m'), 0.0) or 0.0
        if other_cost <= 20 and other_max <= 54 and other_path <= 20 and other_clearance > max(corridor_clearance or 0.0, 0.55):
            safer_lower_rank_exists = True
            break
    if safer_lower_rank_exists:
        risk_reasons.append('safer lower-ranked branch exists')
    supports = bool(risk_reasons) and (
        target_local_cost >= 90
        or target_max_radius >= 90
        or path_cost >= 90
        or (corridor_clearance is not None and corridor_clearance < 0.35)
        or safer_lower_rank_exists
    )
    return {
        'supports_candidate_too_risky': supports,
        'candidate_family': _safe_get(first_goal, ['candidate', 'candidate_family']) or frontier.get('local_topology'),
        'selected_rank': selected_rank,
        'selected_branch': {
            'target_local_cost': target_local_cost,
            'target_local_cost_max_radius': target_max_radius,
            'path_corridor_min_clearance_m': corridor_clearance,
            'target_clearance_m': target_clearance,
            'dispatch_path_local_cost_max': path_cost,
        },
        'candidate_branch_count': len(branches),
        'safer_lower_rank_exists': safer_lower_rank_exists,
        'risk_reasons': risk_reasons,
    }, gaps


def _controller_stall_evidence(artifact: dict[str, Any], feedback_ev: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    gaps: list[str] = []
    ext = artifact.get('diagnostic_replay_extensions') or {}
    cmd_vel = ext.get('cmd_vel_timeline') or []
    odom_velocity = ext.get('odom_velocity_timeline') or []
    if not cmd_vel:
        gaps.append('cmd_vel timeline missing')
    if not odom_velocity:
        gaps.append('odom velocity timeline missing')
    supports = False
    reasons: list[str] = []
    if cmd_vel and odom_velocity:
        cmd_nonzero = any(abs(_as_float(item.get('linear_x'), 0.0) or 0.0) > 0.05 or abs(_as_float(item.get('angular_z'), 0.0) or 0.0) > 0.1 for item in cmd_vel if isinstance(item, dict))
        odom_near_zero = all(abs(_as_float(item.get('linear_x'), 0.0) or 0.0) < 0.03 and abs(_as_float(item.get('angular_z'), 0.0) or 0.0) < 0.05 for item in odom_velocity if isinstance(item, dict))
        signs = [math.copysign(1, _as_float(item.get('angular_z'), 0.0) or 0.0) for item in cmd_vel if abs(_as_float(item.get('angular_z'), 0.0) or 0.0) > 0.1]
        oscillating = len(set(signs)) > 1
        if cmd_nonzero and odom_near_zero:
            reasons.append('nonzero cmd_vel with near-zero odom velocity')
        if oscillating:
            reasons.append('cmd_vel angular sign oscillation')
        supports = bool(reasons) and (feedback_ev.get('distance_plateau') or feedback_ev.get('max_recoveries', 0) >= 1)
    return {
        'supports_controller_stall': supports,
        'cmd_vel_count': len(cmd_vel),
        'odom_velocity_count': len(odom_velocity),
        'reasons': reasons,
    }, gaps


def _goal_tolerance_evidence(artifact: dict[str, Any], first_goal: dict[str, Any], feedback_ev: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    gaps: list[str] = []
    ext = artifact.get('diagnostic_replay_extensions') or {}
    pose_trace = ext.get('robot_pose_trace') or []
    goal_checker = ext.get('goal_checker_state') or {}
    if not pose_trace:
        gaps.append('robot pose trace missing')
    if not goal_checker:
        gaps.append('goal checker state missing')
    dist = _as_float(_safe_get(first_goal, ['distance_to_first_goal', 'meters']))
    xy_tol = _as_float(goal_checker.get('xy_tolerance_m'))
    yaw_err = _as_float(goal_checker.get('yaw_error_rad'))
    yaw_tol = _as_float(goal_checker.get('yaw_tolerance_rad'))
    supports = False
    reasons: list[str] = []
    if pose_trace and goal_checker and dist is not None and xy_tol is not None:
        near_xy_edge = abs(dist - xy_tol) <= 0.08 or (xy_tol < dist <= xy_tol + 0.10)
        yaw_edge = yaw_err is not None and yaw_tol is not None and yaw_err > yaw_tol
        if near_xy_edge:
            reasons.append('distance near xy tolerance edge')
        if yaw_edge:
            reasons.append('yaw error outside tolerance')
        supports = bool(reasons) and feedback_ev.get('distance_plateau', False)
    return {
        'supports_goal_tolerance_edge': supports,
        'pose_trace_count': len(pose_trace),
        'goal_checker_state_available': bool(goal_checker),
        'distance_to_first_goal_m': dist,
        'xy_tolerance_m': xy_tol,
        'yaw_error_rad': yaw_err,
        'yaw_tolerance_rad': yaw_tol,
        'reasons': reasons,
    }, gaps


def _base_guardrails(artifact: dict[str, Any]) -> dict[str, bool]:
    guardrails = artifact.get('guardrails') or {}
    return {
        'no_runtime_replay': True,
        'no_success_claim': True,
        'phase125_timeout_input': artifact.get('classification') == PHASE125_TIMEOUT_CLASSIFICATION,
        'max_goals_one': artifact.get('maze_explorer_max_goals') == 1,
        'second_goal_dispatched_false': artifact.get('second_goal_dispatched') is False,
        'dispatch_event_count_one': artifact.get('dispatch_event_count') == 1,
        'goal_kind_explore': _safe_get(artifact, ['first_goal', 'goal_kind']) == 'explore',
        'accepted_timeout_not_success': _safe_get(artifact, ['first_goal', 'accepted']) is True and _safe_get(artifact, ['first_goal', 'timeout']) is True and _safe_get(artifact, ['first_goal', 'result_status_label']) == 'TIMEOUT',
        'nav2_config_unchanged': not bool(guardrails.get('nav2_config_changed', False)),
        'branch_scoring_unchanged': not bool(guardrails.get('branch_scoring_changed', False)),
        'centerline_gate_unchanged': not bool(guardrails.get('centerline_gate_changed', False)),
        'fallback_unchanged': not bool(guardrails.get('fallback_changed', False)),
        'terminal_acceptance_unchanged': not bool(guardrails.get('terminal_acceptance_changed', False)),
    }


def analyze_phase125_artifact(artifact: dict[str, Any], stderr_text: str = '') -> dict[str, Any]:
    first_goal = artifact.get('first_goal') or {}
    evidence_gaps: list[str] = []

    feedback_ev, gaps = _feedback_evidence(first_goal, artifact)
    evidence_gaps.extend(gaps)
    local_ev, gaps = _local_cost_evidence(artifact, stderr_text)
    evidence_gaps.extend(gaps)
    candidate_ev, gaps = _candidate_risk_evidence(first_goal)
    evidence_gaps.extend(gaps)
    controller_ev, gaps = _controller_stall_evidence(artifact, feedback_ev)
    evidence_gaps.extend(gaps)
    tolerance_ev, gaps = _goal_tolerance_evidence(artifact, first_goal, feedback_ev)
    evidence_gaps.extend(gaps)

    guardrails = _base_guardrails(artifact)

    classification = 'INSUFFICIENT_TIMEOUT_EVIDENCE'
    classification_reason = 'insufficient discriminating evidence'

    if not all([
        guardrails['phase125_timeout_input'],
        guardrails['max_goals_one'],
        guardrails['second_goal_dispatched_false'],
        guardrails['dispatch_event_count_one'],
        guardrails['goal_kind_explore'],
        guardrails['accepted_timeout_not_success'],
    ]):
        classification = 'INSUFFICIENT_TIMEOUT_EVIDENCE'
        classification_reason = 'input artifact does not satisfy Phase125 first-goal timeout boundary'
    elif local_ev['supports_local_cost_blocked']:
        classification = 'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED'
        classification_reason = 'phase23 timeout local-cost line shows high footprint/front/path costs with timeout and recovery evidence'
    elif controller_ev['supports_controller_stall']:
        classification = 'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL'
        classification_reason = 'cmd_vel/odom velocity evidence supports controller stall with distance plateau/recovery evidence'
    elif tolerance_ev['supports_goal_tolerance_edge']:
        classification = 'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE'
        classification_reason = 'pose trace and goal checker evidence place robot near tolerance edge at timeout'
    elif candidate_ev['supports_candidate_too_risky']:
        classification = 'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY'
        classification_reason = 'selected candidate evidence shows high-cost/low-clearance/riskier selected target'

    # Deduplicate while preserving order.
    deduped_gaps: list[str] = []
    for gap in evidence_gaps:
        if gap not in deduped_gaps:
            deduped_gaps.append(gap)

    result = {
        'phase': PHASE,
        'mode': MODE,
        'created_wall_time_sec': time.time(),
        'classification': classification,
        'classification_reason': classification_reason,
        'source_phase125_classification': artifact.get('classification'),
        'guardrails': guardrails,
        'forbidden_runtime_actions': FORBIDDEN_RUNTIME_ACTIONS,
        'evidence': {
            'nav2_feedback': feedback_ev,
            'local_cost': local_ev,
            'candidate_risk': candidate_ev,
            'controller_stall': controller_ev,
            'goal_tolerance': tolerance_ev,
            'first_goal_summary': {
                'goal_kind': first_goal.get('goal_kind'),
                'accepted': first_goal.get('accepted'),
                'timeout': first_goal.get('timeout'),
                'result_status_label': first_goal.get('result_status_label'),
                'abort_text': first_goal.get('abort_text'),
                'pose': first_goal.get('pose'),
                'robot_pose_at_result': first_goal.get('robot_pose_at_result'),
                'distance_to_first_goal': first_goal.get('distance_to_first_goal'),
                'candidate': first_goal.get('candidate'),
            },
        },
        'evidence_gaps': deduped_gaps,
        'interpretation_boundary': {
            'timeout_is_not_success': True,
            'accepted_dispatch_is_not_execution_success': True,
            'no_autonomous_exploration_success_claim': True,
            'no_exit_success_claim': True,
            'repair_not_authorized': True,
        },
    }
    result['valid'] = result['classification'] in CLASSIFICATIONS and all(guardrails.values())
    return result


def _write_summary(analysis: dict[str, Any], output_path: Path) -> None:
    ev = analysis['evidence']
    lines = [
        '# Phase127 first-goal timeout artifact replay summary',
        '',
        f"classification: `{analysis['classification']}`",
        f"reason: {analysis['classification_reason']}",
        '',
        '## Key evidence',
        '',
        f"- source Phase125 classification: `{analysis['source_phase125_classification']}`",
        f"- Nav2 feedback samples: `{ev['nav2_feedback']['count']}`",
        f"- last distance_remaining: `{ev['nav2_feedback']['last_distance_remaining']}`",
        f"- max recoveries: `{ev['nav2_feedback']['max_recoveries']}`",
        f"- timeout local cost: `{ev['local_cost']['timeout_local_cost']}`",
        f"- local-cost blocked support: `{ev['local_cost']['supports_local_cost_blocked']}`",
        f"- candidate-risk support: `{ev['candidate_risk']['supports_candidate_too_risky']}`",
        f"- controller-stall support: `{ev['controller_stall']['supports_controller_stall']}`",
        f"- goal-tolerance support: `{ev['goal_tolerance']['supports_goal_tolerance_edge']}`",
        '',
        '## Evidence gaps',
        '',
    ]
    for gap in analysis.get('evidence_gaps') or []:
        lines.append(f'- {gap}')
    if not analysis.get('evidence_gaps'):
        lines.append('- none')
    lines.extend([
        '',
        '## Guardrails',
        '',
    ])
    for key, value in analysis.get('guardrails', {}).items():
        lines.append(f'- {key}: `{value}`')
    lines.extend([
        '',
        'No autonomous exploration success or exit success is claimed. Phase128 not entered.',
        '',
    ])
    output_path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase125-artifact', required=True, type=Path)
    parser.add_argument('--stderr-log', required=False, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--summary', required=False, type=Path)
    args = parser.parse_args()

    artifact = json.loads(args.phase125_artifact.read_text(encoding='utf-8'))
    stderr_text = args.stderr_log.read_text(encoding='utf-8') if args.stderr_log and args.stderr_log.exists() else ''
    analysis = analyze_phase125_artifact(artifact, stderr_text=stderr_text)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(analysis, indent=2, sort_keys=True), encoding='utf-8')
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        _write_summary(analysis, args.summary)
    print(json.dumps({'classification': analysis['classification'], 'valid': analysis['valid'], 'output': str(args.output)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
