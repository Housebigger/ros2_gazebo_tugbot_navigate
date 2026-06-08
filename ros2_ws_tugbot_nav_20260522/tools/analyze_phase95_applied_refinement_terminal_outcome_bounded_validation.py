#!/usr/bin/env python3
"""Phase95 applied refinement terminal outcome bounded validation analyzer.

Read-only analyzer for a bounded visible reproduction that continues the Phase94
applied Phase88 single-step refinement scene until a terminal outcome is
captured: succeeded / timeout / cancel / failure. It consumes /maze/goal_events,
Nav2 feedback, local-cost samples, raw scan/odom/TF/costmap evidence, and prior
Phase85/89/94 summaries for comparison.

No maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order,
centerline gate, directional readiness, fallback/terminal acceptance, Nav2,
MPPI, controller, inflation, robot radius, clearance radius, or map thresholds
are changed by this analyzer.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase95_applied_refinement_terminal_outcome_bounded_validation'
OLD_GOAL2_TARGET = [2.057855221699651, 1.0261005743935105]
GOAL2_TARGET_MATCH_TOLERANCE_M = 0.45
RECOVERY_DOMINANT_THRESHOLD = 5
LETHAL_COST_THRESHOLD = 1

ALLOWED_CLASSIFICATIONS = {
    'APPLIED_REFINEMENT_TERMINAL_SUCCEEDED',
    'APPLIED_REFINEMENT_TERMINAL_TIMEOUT_RECOVERY_DOMINANT',
    'APPLIED_REFINEMENT_TERMINAL_LOCAL_COST_BLOCKED',
    'APPLIED_REFINEMENT_TERMINAL_CANCEL_OR_FAILURE',
    'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE',
}

REQUIRED_EXTRACTED_FIELDS = {
    'selected_candidate_target',
    'selected_candidate_yaw',
    'hard_safety_pass_candidate_count',
    'selection_priority_trace',
    'terminal_event',
    'terminal_reason',
    'terminal_pose',
    'terminal_local_cost_metrics',
}

PRIORITY_TRACE = [
    'hard_safety_pass',
    'no_footprint_front_wedge_lethal_regression',
    'safety_floor_ok',
    'forward_progress_ok',
    'clearance_better',
    'balance_error_smaller',
]

JSONDict = dict[str, Any]


def _load_json(path: Path | None, default: Any = None) -> Any:
    if default is None:
        default = {}
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_jsonl(path: Path | None) -> list[JSONDict]:
    rows: list[JSONDict] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except Exception:
            continue
        payload = raw.get('state') if isinstance(raw, dict) and isinstance(raw.get('state'), dict) else raw
        if isinstance(payload, dict):
            row = dict(payload)
            if isinstance(raw, dict):
                row.setdefault('_recorder_elapsed_sec', raw.get('elapsed_sec'))
                row.setdefault('_recorder_wall_time', raw.get('wall_time'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path | None, suffix: str, run_id: str | None = None) -> Path | None:
    if artifact_dir is None:
        return None
    artifact_dir = Path(artifact_dir)
    names: list[str] = []
    if run_id:
        names.append(f'{run_id}_{suffix}')
    names.extend([f'phase95_{suffix}', suffix])
    for name in names:
        direct = artifact_dir / name
        if direct.exists():
            return direct
    matches = sorted(artifact_dir.glob(f'*{suffix}'))
    return matches[0] if matches else None


def _as_xy(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        if 'x' in value and 'y' in value:
            try:
                return [float(value['x']), float(value['y'])]
            except (TypeError, ValueError):
                return None
        if isinstance(value.get('position'), dict):
            return _as_xy(value['position'])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
    return None


def _as_pose(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return None
    if isinstance(value, dict):
        if all(k in value for k in ('x', 'y', 'yaw')):
            try:
                return [float(value['x']), float(value['y']), float(value['yaw'])]
            except (TypeError, ValueError):
                return None
        if isinstance(value.get('pose'), dict):
            return _as_pose(value['pose'])
    return None


def _distance(a: Any, b: Any) -> float | None:
    ax = _as_xy(a)
    bx = _as_xy(b)
    if ax is None or bx is None:
        return None
    return math.hypot(ax[0] - bx[0], ax[1] - bx[1])


def _row_goal_sequence(row: JSONDict) -> int | None:
    value = row.get('goal_sequence')
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _nested(row: JSONDict | None, key: str) -> JSONDict:
    if not isinstance(row, dict):
        return {}
    value = row.get(key)
    return value if isinstance(value, dict) else {}


def _extract_refinement(row: JSONDict | None) -> JSONDict:
    nested = _nested(row, 'centerline_target_refinement')
    return nested if nested else (row if isinstance(row, dict) else {})


def _norm_token(value: Any) -> str:
    return str(value).strip().lower().replace('-', '_').replace('/', '_').replace(' ', '_')


def _candidate_values_for_goal2_match(row: JSONDict) -> list[Any]:
    ref = _extract_refinement(row)
    return [
        row.get('original_target'),
        row.get('target'),
        row.get('refined_target'),
        row.get('selected_candidate_target'),
        ref.get('original_target'),
        ref.get('refined_target'),
        ref.get('selected_candidate_target'),
        ref.get('centerline_projected_target'),
    ]


def _matches_goal2(row: JSONDict) -> bool:
    return any((_distance(value, OLD_GOAL2_TARGET) or 999.0) <= GOAL2_TARGET_MATCH_TOLERANCE_M for value in _candidate_values_for_goal2_match(row))


def _has_applied_refinement(row: JSONDict) -> bool:
    ref = _extract_refinement(row)
    return bool(
        ref.get('refinement_applied')
        or ref.get('applied')
        or row.get('centerline_refinement_applied')
    )


def _select_goal2_dispatch(events: list[JSONDict]) -> tuple[JSONDict | None, list[JSONDict], int | None, bool]:
    dispatches = [row for row in events if row.get('event') == 'dispatch']
    # Phase95's target scene is the Phase88 applied-refinement execution. In
    # real bounded runs, a prior legacy Goal2-like dispatch can appear before
    # the applied refinement dispatch; selecting the old-target row would hide
    # the terminal outcome that Phase95 was created to validate.
    applied = [row for row in dispatches if _has_applied_refinement(row)]
    matched_applied = [row for row in applied if _matches_goal2(row)]
    if matched_applied:
        dispatch = matched_applied[0]
        seq = _row_goal_sequence(dispatch)
        related = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return dispatch, related, seq, True
    if applied:
        dispatch = applied[0]
        seq = _row_goal_sequence(dispatch)
        related = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return dispatch, related, seq, False
    matched = [row for row in dispatches if _matches_goal2(row)]
    if matched:
        dispatch = matched[0]
        seq = _row_goal_sequence(dispatch)
        related = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return dispatch, related, seq, True
    with_refinement = [row for row in dispatches if isinstance(row.get('centerline_target_refinement'), dict)]
    if with_refinement:
        dispatch = with_refinement[0]
        seq = _row_goal_sequence(dispatch)
        related = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return dispatch, related, seq, False
    return None, [], None, False


def _selected_refinement(dispatch: JSONDict | None) -> JSONDict:
    ref = _extract_refinement(dispatch)
    selected_refined_target = ref.get('refined_target') or ref.get('selected_candidate_target') or (dispatch or {}).get('target')
    rejected = ref.get('rejected_candidate_summaries')
    return {
        'refinement_applied': bool(ref.get('refinement_applied') or ref.get('applied')),
        'refinement_reject_reason': ref.get('refinement_reject_reason'),
        'original_target': ref.get('original_target') or (dispatch or {}).get('original_target'),
        'nav2_goal_target': (dispatch or {}).get('target'),
        'selected_refined_target': selected_refined_target,
        'selected_candidate_index': ref.get('selected_candidate_index'),
        'selected_candidate_target': ref.get('selected_candidate_target'),
        'selected_candidate_yaw': ref.get('selected_candidate_yaw'),
        'selection_priority_trace': ref.get('selection_priority_trace'),
        'hard_safety_pass_candidate_count': ref.get('hard_safety_pass_candidate_count'),
        'candidate_count': ref.get('candidate_count'),
        'candidate_family': ref.get('candidate_family'),
        'rejected_candidate_summary_count': len(rejected) if isinstance(rejected, list) else 0,
        'branch_scoring_changed': ref.get('branch_scoring_changed') if 'branch_scoring_changed' in ref else (dispatch or {}).get('branch_scoring_changed'),
        'fallback_terminal_acceptance_used': ref.get('fallback_terminal_acceptance_used') if 'fallback_terminal_acceptance_used' in ref else (dispatch or {}).get('fallback_terminal_acceptance_used'),
        'target_shift_from_original_m': _distance(ref.get('original_target') or (dispatch or {}).get('original_target'), selected_refined_target),
    }


def _terminal_event(events: list[JSONDict]) -> JSONDict | None:
    terminal_events = {'success', 'succeeded', 'timeout', 'failure', 'failed', 'cancel', 'canceled', 'timeout_cancel_result'}
    terminal_reasons = {'goal_timeout', 'goal_canceled_after_timeout', 'goal_canceled', 'canceled', 'goal_reached', 'succeeded', 'nav2_failed'}
    for row in events:
        if row.get('event') in terminal_events or row.get('result_reason') in terminal_reasons:
            return row
    return None


def _feedback_summary(feedback: list[JSONDict], goal_sequence: int | None) -> JSONDict:
    selected = [row for row in feedback if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    distances: list[float] = []
    recoveries: list[int] = []
    timeline: list[JSONDict] = []
    prev_recovery: int | None = None
    for row in selected:
        value = row.get('distance_remaining')
        try:
            if value is not None:
                distances.append(float(value))
        except (TypeError, ValueError):
            pass
        recovery = None
        try:
            if row.get('number_of_recoveries') is not None:
                recovery = int(row['number_of_recoveries'])
                recoveries.append(recovery)
        except (TypeError, ValueError):
            pass
        if recovery is not None and prev_recovery is not None and recovery > prev_recovery:
            timeline.append({
                'elapsed_sec': row.get('elapsed_sec') or row.get('_recorder_elapsed_sec'),
                'wall_time': row.get('wall_time') or row.get('_recorder_wall_time'),
                'number_of_recoveries': recovery,
                'distance_remaining': row.get('distance_remaining'),
            })
        if recovery is not None:
            prev_recovery = recovery
    nonzero = [d for d in distances if d > 0.01]
    first = nonzero[0] if nonzero else None
    last = distances[-1] if distances else None
    return {
        'feedback_sample_count': len(selected),
        'distance_remaining_first_nonzero': first,
        'distance_remaining_last': last,
        'distance_remaining_min_nonzero': min(nonzero) if nonzero else None,
        'distance_remaining_progress_delta': (first - last) if isinstance(first, float) and isinstance(last, float) else None,
        'distance_remaining_curve_excerpt': selected[:3] + selected[-3:] if len(selected) > 6 else selected,
        'recoveries_max': max(recoveries) if recoveries else 0,
        'recoveries_last': recoveries[-1] if recoveries else 0,
        'recoveries_timeline': timeline,
        'recovery_dominant': bool((max(recoveries) if recoveries else 0) >= RECOVERY_DOMINANT_THRESHOLD),
    }


def _num_from_cost(row: JSONDict, flat_key: str, nested_key: str, sub_key: str = 'lethal_count') -> float | None:
    value = row.get(flat_key)
    if isinstance(value, (int, float)):
        return float(value)
    nested = row.get(nested_key)
    if isinstance(nested, dict):
        value = nested.get(sub_key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


def _local_cost_metrics(rows: list[JSONDict], goal_sequence: int | None) -> JSONDict:
    selected = [row for row in rows if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    front: list[float] = []
    footprint: list[float] = []
    target: list[float] = []
    front_clearance: list[float] = []
    for row in selected:
        f = _num_from_cost(row, 'front_wedge_lethal_count', 'front_wedge_cost')
        fp = _num_from_cost(row, 'robot_footprint_lethal_count', 'robot_footprint_cost')
        t = _num_from_cost(row, 'target_footprint_lethal_count', 'target_footprint_cost')
        if f is not None:
            front.append(f)
        if fp is not None:
            footprint.append(fp)
        if t is not None:
            target.append(t)
        try:
            if row.get('front_wedge_clearance_m') is not None:
                front_clearance.append(float(row['front_wedge_clearance_m']))
        except (TypeError, ValueError):
            pass
    last_row = selected[-1] if selected else {}
    blocked = bool((front[-1] if front else 0) >= LETHAL_COST_THRESHOLD or (footprint[-1] if footprint else 0) >= LETHAL_COST_THRESHOLD)
    return {
        'sample_count': len(selected),
        'terminal_sample_elapsed_sec': last_row.get('elapsed_sec'),
        'terminal_sample_pose': _as_pose(last_row.get('robot_pose')),
        'front_wedge_lethal_count_max': max(front) if front else None,
        'front_wedge_lethal_count_last': front[-1] if front else None,
        'robot_footprint_lethal_count_max': max(footprint) if footprint else None,
        'robot_footprint_lethal_count_last': footprint[-1] if footprint else None,
        'target_footprint_lethal_count_max': max(target) if target else None,
        'target_footprint_lethal_count_last': target[-1] if target else None,
        'front_wedge_clearance_min_m': min(front_clearance) if front_clearance else None,
        'terminal_local_cost_blocked': blocked,
    }


def _raw_terminal_pose(raw: JSONDict) -> list[float] | None:
    if not isinstance(raw, dict):
        return None
    pose_by_frame = raw.get('robot_pose_by_frame')
    if isinstance(pose_by_frame, dict):
        for key in ('map', 'odom'):
            pose = _as_pose(pose_by_frame.get(key))
            if pose is not None:
                return pose
    odom = raw.get('odom')
    pose = _as_pose(odom)
    if pose is not None:
        return pose
    return None


def _terminal_summary(events: list[JSONDict], raw_capture: JSONDict, local_metrics: JSONDict) -> JSONDict:
    terminal = _terminal_event(events)
    event = terminal.get('event') if isinstance(terminal, dict) else None
    reason = terminal.get('result_reason') if isinstance(terminal, dict) else None
    terminal_pose = _as_pose((terminal or {}).get('robot_pose')) or local_metrics.get('terminal_sample_pose') or _raw_terminal_pose(raw_capture)
    timed_out = bool(event in {'timeout', 'timeout_cancel_result'} or reason in {'goal_timeout', 'goal_canceled_after_timeout'})
    succeeded = bool(event in {'success', 'succeeded'} or reason in {'goal_reached', 'succeeded'})
    canceled = bool(event in {'cancel', 'canceled'} or reason in {'goal_canceled', 'canceled', 'goal_canceled_after_timeout'})
    failed = bool(event in {'failure', 'failed'} or (reason and not timed_out and not succeeded and not canceled))
    return {
        'terminal_observed': terminal is not None,
        'terminal_event': event,
        'terminal_reason': reason,
        'terminal_succeeded': succeeded,
        'terminal_timed_out': timed_out,
        'terminal_canceled': canceled,
        'terminal_failed': failed,
        'terminal_pose': terminal_pose,
        'terminal_event_row': terminal,
    }


def _prior_summary(phase_dir: Path | None) -> JSONDict:
    if phase_dir is None:
        return {'available': False}
    analysis_path = _glob_first(phase_dir, 'analysis.json')
    analysis = _load_json(analysis_path, {})
    if not analysis:
        return {'available': False, 'analysis_path': str(analysis_path) if analysis_path else None}
    # Accept both compact Phase95 synthetic summaries and richer Phase94 analyzer output.
    selected = analysis.get('selected_refinement') if isinstance(analysis.get('selected_refinement'), dict) else {}
    execution = analysis.get('execution_outcome') if isinstance(analysis.get('execution_outcome'), dict) else {}
    feedback = analysis.get('nav2_feedback_summary') if isinstance(analysis.get('nav2_feedback_summary'), dict) else {}
    local = analysis.get('local_cost_risk') if isinstance(analysis.get('local_cost_risk'), dict) else analysis.get('local_costmap_summary', {})
    if not isinstance(local, dict):
        local = {}
    def first(*values: Any) -> Any:
        for value in values:
            if value is not None:
                return value
        return None
    return {
        'available': True,
        'analysis_path': str(analysis_path) if analysis_path else None,
        'classification': analysis.get('classification'),
        'refinement_applied': first(analysis.get('refinement_applied'), selected.get('refinement_applied')),
        'hard_safety_pass_candidate_count': first(analysis.get('hard_safety_pass_candidate_count'), selected.get('hard_safety_pass_candidate_count')),
        'recoveries_max': first(analysis.get('recoveries_max'), execution.get('recoveries_max'), feedback.get('recoveries_max')),
        'distance_remaining_last': first(analysis.get('distance_remaining_last'), execution.get('distance_remaining_last'), feedback.get('distance_remaining_last')),
        'terminal_outcome': first(analysis.get('terminal_outcome'), 'succeeded' if execution.get('terminal_succeeded') else None, 'timeout' if execution.get('terminal_timed_out') else None, 'held_before_terminal' if execution.get('held_before_terminal_outcome') else None),
        'front_wedge_lethal_count_max': first(analysis.get('front_wedge_lethal_count_max'), local.get('front_wedge_lethal_count_max')),
        'robot_footprint_lethal_count_max': first(analysis.get('robot_footprint_lethal_count_max'), local.get('robot_footprint_lethal_count_max')),
    }


def _comparison(current: JSONDict, phase85_dir: Path | None, phase89_dir: Path | None, phase94_dir: Path | None) -> JSONDict:
    phase85 = _prior_summary(phase85_dir)
    phase89 = _prior_summary(phase89_dir)
    phase94 = _prior_summary(phase94_dir)
    selected = current['selected_refinement']
    nav = current['nav2_feedback_summary']
    terminal = current['terminal_outcome']
    local = current['terminal_local_cost_metrics']
    def delta(cur: Any, old: Any) -> float | None:
        if isinstance(cur, (int, float)) and isinstance(old, (int, float)):
            return float(cur) - float(old)
        return None
    def compare(prior: JSONDict) -> JSONDict:
        return {
            'refinement_applied_changed_false_to_true': prior.get('refinement_applied') is False and selected.get('refinement_applied') is True,
            'hard_safety_pass_candidate_count_delta': delta(selected.get('hard_safety_pass_candidate_count'), prior.get('hard_safety_pass_candidate_count')),
            'recoveries_max_delta': delta(nav.get('recoveries_max'), prior.get('recoveries_max')),
            'distance_remaining_last_delta': delta(nav.get('distance_remaining_last'), prior.get('distance_remaining_last')),
            'terminal_outcome_current': 'succeeded' if terminal.get('terminal_succeeded') else 'timeout' if terminal.get('terminal_timed_out') else 'cancel_or_failure' if (terminal.get('terminal_canceled') or terminal.get('terminal_failed')) else None,
            'terminal_outcome_prior': prior.get('terminal_outcome'),
            'front_wedge_lethal_count_max_delta': delta(local.get('front_wedge_lethal_count_max'), prior.get('front_wedge_lethal_count_max')),
            'robot_footprint_lethal_count_max_delta': delta(local.get('robot_footprint_lethal_count_max'), prior.get('robot_footprint_lethal_count_max')),
        }
    phase95_vs94 = compare(phase94)
    phase95_vs94['terminal_outcome_changed_from_held_to_terminal'] = phase94.get('terminal_outcome') == 'held_before_terminal' and terminal.get('terminal_observed') is True
    return {
        'phase85': phase85,
        'phase89': phase89,
        'phase94': phase94,
        'phase95_vs_phase85': compare(phase85),
        'phase95_vs_phase89': compare(phase89),
        'phase95_vs_phase94': phase95_vs94,
    }


def analyze_artifact_dir(
    artifact_dir: Path | str,
    *,
    phase85_dir: Path | str | None = None,
    phase89_dir: Path | str | None = None,
    phase94_dir: Path | str | None = None,
) -> JSONDict:
    artifact_dir = Path(artifact_dir)
    phase85_path = Path(phase85_dir) if phase85_dir is not None else None
    phase89_path = Path(phase89_dir) if phase89_dir is not None else None
    phase94_path = Path(phase94_dir) if phase94_dir is not None else None

    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl', RUN_ID)
    feedback_path = _glob_first(artifact_dir, 'nav2_feedback.jsonl', RUN_ID)
    local_cost_path = _glob_first(artifact_dir, 'local_costmap_samples.jsonl', RUN_ID)
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json', RUN_ID)

    events = _read_jsonl(goal_events_path)
    feedback = _read_jsonl(feedback_path)
    local_rows = _read_jsonl(local_cost_path)
    raw_capture = _load_json(raw_capture_path, {})

    dispatch, validation_events, goal_sequence, used_target_match = _select_goal2_dispatch(events)
    selected = _selected_refinement(dispatch)
    nav = _feedback_summary(feedback, goal_sequence)
    local_metrics = _local_cost_metrics(local_rows, goal_sequence)
    terminal = _terminal_summary(validation_events, raw_capture, local_metrics)

    gaps: list[str] = []
    if not events:
        gaps.append('missing_goal_events_artifact')
    if dispatch is None:
        gaps.append('missing_goal2_equivalent_dispatch_event')
    if selected.get('refinement_applied') is not True:
        gaps.append('applied_refinement_not_observed')
    if selected.get('hard_safety_pass_candidate_count') is None:
        gaps.append('missing_refinement_field:hard_safety_pass_candidate_count')
    if selected.get('selected_candidate_target') is None:
        gaps.append('missing_refinement_field:selected_candidate_target')
    if selected.get('selected_candidate_yaw') is None:
        gaps.append('missing_refinement_field:selected_candidate_yaw')
    trace = selected.get('selection_priority_trace')
    if isinstance(trace, list):
        if [_norm_token(t) for t in trace][: len(PRIORITY_TRACE)] != PRIORITY_TRACE:
            gaps.append('selection_priority_trace_not_safety_first')
    else:
        gaps.append('missing_refinement_field:selection_priority_trace')
    if selected.get('branch_scoring_changed') is not False:
        gaps.append('branch_scoring_changed_not_false')
    if selected.get('fallback_terminal_acceptance_used') is not False:
        gaps.append('fallback_terminal_acceptance_used_not_false')
    if not feedback:
        gaps.append('missing_nav2_feedback_artifact')
    if not local_rows:
        gaps.append('missing_local_costmap_samples_artifact')
    if not raw_capture:
        gaps.append('missing_raw_capture_artifact')
    if not terminal.get('terminal_observed'):
        gaps.append('missing_terminal_outcome_for_applied_refinement')
    if terminal.get('terminal_pose') is None:
        gaps.append('missing_terminal_pose')

    fatal = any(
        gap.startswith('missing_refinement_field:') or gap in {
            'missing_goal_events_artifact',
            'missing_goal2_equivalent_dispatch_event',
            'applied_refinement_not_observed',
            'branch_scoring_changed_not_false',
            'fallback_terminal_acceptance_used_not_false',
            'missing_terminal_outcome_for_applied_refinement',
        }
        for gap in gaps
    )
    if fatal:
        classification = 'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE'
    elif terminal['terminal_succeeded']:
        classification = 'APPLIED_REFINEMENT_TERMINAL_SUCCEEDED'
    elif terminal['terminal_canceled'] or terminal['terminal_failed']:
        classification = 'APPLIED_REFINEMENT_TERMINAL_CANCEL_OR_FAILURE'
    elif terminal['terminal_timed_out'] and nav['recovery_dominant']:
        classification = 'APPLIED_REFINEMENT_TERMINAL_TIMEOUT_RECOVERY_DOMINANT'
    elif terminal['terminal_timed_out'] and local_metrics['terminal_local_cost_blocked']:
        classification = 'APPLIED_REFINEMENT_TERMINAL_LOCAL_COST_BLOCKED'
    elif terminal['terminal_timed_out']:
        classification = 'APPLIED_REFINEMENT_TERMINAL_TIMEOUT_RECOVERY_DOMINANT'
    else:
        classification = 'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE'

    result: JSONDict = {
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': gaps,
        'validation_goal_sequence': goal_sequence,
        'used_target_match_fallback': used_target_match,
        'selected_refinement': selected,
        'nav2_feedback_summary': nav,
        'terminal_outcome': terminal,
        'terminal_local_cost_metrics': local_metrics,
        # Alias for tests and report consumers.
        'terminal_event': terminal.get('terminal_event'),
        'terminal_reason': terminal.get('terminal_reason'),
        'terminal_pose': terminal.get('terminal_pose'),
        'comparison_to_phase85_phase89_phase94': {},
        'source_files': {
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'nav2_feedback': str(feedback_path) if feedback_path else None,
            'local_costmap_samples': str(local_cost_path) if local_cost_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
            'phase85_dir': str(phase85_path) if phase85_path else None,
            'phase89_dir': str(phase89_path) if phase89_path else None,
            'phase94_dir': str(phase94_path) if phase94_path else None,
        },
        'guardrails': {
            'no_maze_explorer_strategy_changed': True,
            'no_phase88_or_phase92_logic_changed': True,
            'no_branch_scoring_changed': selected.get('branch_scoring_changed') is False,
            'no_exploration_order_changed': True,
            'no_centerline_gate_changed': True,
            'no_directional_readiness_changed': True,
            'no_fallback_terminal_acceptance_changed': selected.get('fallback_terminal_acceptance_used') is False,
            'no_nav2_mppi_controller_tuning': True,
            'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
        },
        'success_claim_guardrails': {
            'timeout_treated_as_success': False,
            'autonomous_exploration_success_claimed': False,
            'exit_success_claimed': False,
            'phase96_entered': False,
        },
    }
    result['comparison_to_phase85_phase89_phase94'] = _comparison(result, phase85_path, phase89_path, phase94_path)
    return result


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    selected = result.get('selected_refinement') or {}
    nav = result.get('nav2_feedback_summary') or {}
    terminal = result.get('terminal_outcome') or {}
    local = result.get('terminal_local_cost_metrics') or {}
    comparison = result.get('comparison_to_phase85_phase89_phase94') or {}
    lines = [
        '# Phase95 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"selected_candidate_target: {selected.get('selected_candidate_target')}",
        f"selected_candidate_yaw: {selected.get('selected_candidate_yaw')}",
        f"hard_safety_pass_candidate_count: {selected.get('hard_safety_pass_candidate_count')}",
        f"distance_remaining: first={nav.get('distance_remaining_first_nonzero')} last={nav.get('distance_remaining_last')} delta={nav.get('distance_remaining_progress_delta')}",
        f"recoveries timeline: max={nav.get('recoveries_max')} timeline={nav.get('recoveries_timeline')}",
        f"terminal_event: {terminal.get('terminal_event')}",
        f"terminal_reason: {terminal.get('terminal_reason')}",
        f"terminal_pose: {terminal.get('terminal_pose')}",
        f"front_wedge terminal metrics: last={local.get('front_wedge_lethal_count_last')} max={local.get('front_wedge_lethal_count_max')}",
        f"footprint terminal metrics: robot_last={local.get('robot_footprint_lethal_count_last')} target_last={local.get('target_footprint_lethal_count_last')}",
        f"Phase95 vs Phase94: {(comparison.get('phase95_vs_phase94') or {})}",
        '',
        'No autonomous exploration success claimed. No exit success claimed. Timeout is not success.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log') / RUN_ID)
    parser.add_argument('--phase85-dir', type=Path, default=Path('log/phase85_goal2_corridor_aligned_refinement_bounded_validation'))
    parser.add_argument('--phase89-dir', type=Path, default=Path('log/phase89_safety_first_refinement_bounded_goal2_validation'))
    parser.add_argument('--phase94-dir', type=Path, default=Path('log/phase94_applied_single_step_refinement_execution_outcome_diagnosis'))
    parser.add_argument('--output-json', type=Path, default=None)
    parser.add_argument('--minimal-summary', type=Path, default=None)
    args = parser.parse_args()

    result = analyze_artifact_dir(args.artifact_dir, phase85_dir=args.phase85_dir, phase89_dir=args.phase89_dir, phase94_dir=args.phase94_dir)
    output = args.output_json or args.artifact_dir / f'{RUN_ID}_analysis.json'
    summary = args.minimal_summary or args.artifact_dir / f'{RUN_ID}_minimal_field_summary.md'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_minimal_summary(result, summary)
    print(json.dumps({
        'classification': result['classification'],
        'evidence_gaps': result['evidence_gaps'],
        'output_json': str(output),
        'minimal_summary': str(summary),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
