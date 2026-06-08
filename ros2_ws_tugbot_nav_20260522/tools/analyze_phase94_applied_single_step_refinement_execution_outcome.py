#!/usr/bin/env python3
"""Phase94 applied single-step refinement execution outcome diagnosis.

Read-only analyzer. It consumes Phase93 bounded reproduction artifacts plus
Phase89/Phase85 comparison artifacts and diagnoses the Nav2 execution outcome
after Phase88 single-step safety-first refinement applied. It does not modify
maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order,
centerline gate, directional readiness, fallback/terminal acceptance, Nav2,
MPPI, controller, inflation, robot radius, clearance radius, or map thresholds.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase94_applied_single_step_refinement_execution_outcome_diagnosis'
PHASE93_RUN_ID = 'phase93_two_step_staging_bounded_goal2_validation'
PHASE89_RUN_ID = 'phase89_safety_first_refinement_bounded_goal2_validation'
PHASE85_RUN_ID = 'phase85_goal2_corridor_aligned_refinement_bounded_validation'
OLD_GOAL2_TARGET = [2.057855221699651, 1.0261005743935105]
GOAL2_TARGET_MATCH_TOLERANCE_M = 0.45
RECOVERY_DOMINANT_THRESHOLD = 5
LOCAL_COST_LETHAL_THRESHOLD = 1

ALLOWED_CLASSIFICATIONS = {
    'APPLIED_REFINEMENT_EXECUTION_IMPROVED',
    'APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT',
    'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME',
    'APPLIED_REFINEMENT_LOCAL_COST_RISK_PERSISTS',
    'APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
}

REQUIRED_EXTRACTED_REFINEMENT_FIELDS = [
    'selected_refined_target',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
    'selection_priority_trace',
    'hard_safety_pass_candidate_count',
    'rejected_candidate_summaries',
]

PRIORITY_TRACE = [
    'hard_safety_pass',
    'no_footprint_front_wedge_lethal_regression',
    'safety_floor_ok',
    'forward_progress_ok',
    'clearance_better',
    'balance_error_smaller',
]

JSONDict = dict[str, Any]


def _load_json(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_jsonl(path: Path | None) -> list[JSONDict]:
    rows: list[JSONDict] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        loaded = _loads(line)
        if not isinstance(loaded, dict):
            continue
        payload = loaded.get('state') if isinstance(loaded.get('state'), dict) else loaded
        if isinstance(payload, dict):
            row = dict(payload)
            row.setdefault('_recorder_elapsed_sec', loaded.get('elapsed_sec'))
            row.setdefault('_recorder_wall_time', loaded.get('wall_time'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path | None, suffix: str, run_id: str | None = None) -> Path | None:
    if artifact_dir is None:
        return None
    artifact_dir = Path(artifact_dir)
    names = []
    if run_id:
        names.append(f'{run_id}_{suffix}')
    names.append(suffix)
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
            return _as_xy(value.get('position'))
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        return [float(value[0]), float(value[1])]
    except (TypeError, ValueError):
        return None


def _distance(a: Any, b: Any) -> float | None:
    ax = _as_xy(a)
    bx = _as_xy(b)
    if ax is None or bx is None:
        return None
    return math.hypot(ax[0] - bx[0], ax[1] - bx[1])


def _row_goal_sequence(row: JSONDict) -> int | None:
    try:
        return int(row.get('goal_sequence'))
    except (TypeError, ValueError):
        return None


def _nested(row: JSONDict | None, key: str) -> JSONDict:
    if not isinstance(row, dict):
        return {}
    value = row.get(key)
    return value if isinstance(value, dict) else {}


def _norm_token(value: Any) -> str:
    return str(value).strip().lower().replace('-', '_').replace('/', '_').replace(' ', '_')


def _extract_refinement(row: JSONDict | None) -> JSONDict:
    if not isinstance(row, dict):
        return {}
    nested = _nested(row, 'centerline_target_refinement')
    return nested if nested else row


def _candidate_values_for_goal2_match(row: JSONDict) -> list[Any]:
    ref = _extract_refinement(row)
    return [
        row.get('original_target'),
        row.get('target'),
        row.get('refined_target'),
        row.get('selected_candidate_target'),
        row.get('centerline_projected_target'),
        ref.get('original_target'),
        ref.get('refined_target'),
        ref.get('selected_candidate_target'),
        ref.get('centerline_projected_target'),
    ]


def _matches_goal2_equivalent(row: JSONDict) -> bool:
    return any((_distance(candidate, OLD_GOAL2_TARGET) or 999.0) <= GOAL2_TARGET_MATCH_TOLERANCE_M for candidate in _candidate_values_for_goal2_match(row))


def _select_goal2_dispatch(events: list[JSONDict]) -> tuple[JSONDict | None, list[JSONDict], int | None, bool]:
    dispatches = [row for row in events if row.get('event') == 'dispatch']
    matched = [row for row in dispatches if _matches_goal2_equivalent(row)]
    if matched:
        seq = _row_goal_sequence(matched[0])
        rows = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return matched[0], rows, seq, True
    phase88 = [row for row in dispatches if isinstance(row.get('centerline_target_refinement'), dict)]
    if phase88:
        seq = _row_goal_sequence(phase88[0])
        rows = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return phase88[0], rows, seq, False
    seq1 = [row for row in events if _row_goal_sequence(row) == 1]
    dispatch = next((row for row in seq1 if row.get('event') == 'dispatch'), None)
    return dispatch, seq1, 1 if seq1 else None, False


def _extract_selected_refinement(dispatch: JSONDict | None) -> JSONDict:
    ref = _extract_refinement(dispatch)
    if not ref:
        return {
            'refinement_applied': None,
            'selected_refined_target': None,
            'selected_candidate_index': None,
            'selected_candidate_target': None,
            'selected_candidate_yaw': None,
            'selection_priority_trace': None,
            'hard_safety_pass_candidate_count': None,
            'rejected_candidate_summaries': None,
            'rejected_candidate_summary_count': 0,
        }
    rejected = ref.get('rejected_candidate_summaries')
    selected_refined_target = ref.get('refined_target') or ref.get('selected_candidate_target') or (dispatch or {}).get('refined_target')
    return {
        'refinement_applied': bool(ref.get('refinement_applied') or ref.get('applied')),
        'refinement_reject_reason': ref.get('refinement_reject_reason') or ref.get('reason'),
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
        'rejected_candidate_summaries': rejected if isinstance(rejected, list) else None,
        'rejected_candidate_summary_count': len(rejected) if isinstance(rejected, list) else 0,
        'branch_scoring_changed': ref.get('branch_scoring_changed') if 'branch_scoring_changed' in ref else (dispatch or {}).get('branch_scoring_changed'),
        'fallback_terminal_acceptance_used': ref.get('fallback_terminal_acceptance_used') if 'fallback_terminal_acceptance_used' in ref else (dispatch or {}).get('fallback_terminal_acceptance_used'),
        'target_shift_from_original_m': _distance(ref.get('original_target') or (dispatch or {}).get('original_target'), selected_refined_target),
        'nav2_target_shift_from_original_m': _distance(ref.get('original_target') or (dispatch or {}).get('original_target'), (dispatch or {}).get('target')),
    }


def _outcome_summary(goal_events: list[JSONDict], feedback: list[JSONDict], goal_sequence: int | None) -> JSONDict:
    outcome = next((row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout', 'cancel', 'timeout_cancel_result'}), None)
    event = outcome.get('event') if isinstance(outcome, dict) else None
    reason = outcome.get('result_reason') if isinstance(outcome, dict) else None
    timed_out = bool(event in {'timeout', 'timeout_cancel_result'} or reason in {'goal_timeout', 'goal_canceled_after_timeout'})
    succeeded = bool(event == 'success' or reason in {'goal_reached', 'succeeded'})
    canceled = bool(event == 'cancel' or reason in {'goal_canceled', 'canceled', 'goal_canceled_after_timeout'})
    failed = bool(event == 'failure' or (reason and not timed_out and not succeeded and not canceled))
    selected_feedback = [row for row in feedback if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    distances: list[float] = []
    recoveries: list[int] = []
    recovery_timeline: list[JSONDict] = []
    previous_recovery: int | None = None
    for row in selected_feedback:
        try:
            if row.get('distance_remaining') is not None:
                distances.append(float(row.get('distance_remaining')))
        except (TypeError, ValueError):
            pass
        recovery: int | None = None
        try:
            if row.get('number_of_recoveries') is not None:
                recovery = int(row.get('number_of_recoveries'))
                recoveries.append(recovery)
        except (TypeError, ValueError):
            pass
        if recovery is not None and previous_recovery is not None and recovery > previous_recovery:
            recovery_timeline.append({
                'elapsed_sec': row.get('elapsed_sec') or row.get('_recorder_elapsed_sec'),
                'wall_time': row.get('wall_time') or row.get('_recorder_wall_time'),
                'number_of_recoveries': recovery,
                'distance_remaining': row.get('distance_remaining'),
            })
        if recovery is not None:
            previous_recovery = recovery
    nonzero_distances = [distance for distance in distances if distance > 0.01]
    first_nonzero = nonzero_distances[0] if nonzero_distances else None
    last = distances[-1] if distances else None
    min_nonzero = min(nonzero_distances) if nonzero_distances else None
    progress_delta = None
    if first_nonzero is not None and last is not None:
        progress_delta = first_nonzero - last
    return {
        'terminal_observed': outcome is not None,
        'terminal_event': event,
        'terminal_reason': reason,
        'terminal_succeeded': succeeded,
        'terminal_timed_out': timed_out,
        'terminal_canceled': canceled,
        'terminal_failed': failed,
        'held_before_terminal_outcome': outcome is None and bool(selected_feedback),
        'feedback_sample_count': len(selected_feedback),
        'distance_remaining_first_nonzero': first_nonzero,
        'distance_remaining_min_nonzero': min_nonzero,
        'distance_remaining_last': last,
        'distance_remaining_progress_delta': progress_delta,
        'moved_toward_selected_target': bool(progress_delta is not None and progress_delta > 0.05),
        'recoveries_max': max(recoveries) if recoveries else 0,
        'recoveries_last': recoveries[-1] if recoveries else 0,
        'recovery_timeline': recovery_timeline,
        'recovery_dominant': bool((max(recoveries) if recoveries else 0) >= RECOVERY_DOMINANT_THRESHOLD),
    }


def _numeric_from_nested(row: JSONDict, flat_key: str, nested_key: str, sub_key: str = 'lethal_count') -> float | None:
    value = row.get(flat_key)
    if isinstance(value, (int, float)):
        return float(value)
    nested = row.get(nested_key)
    if isinstance(nested, dict):
        value = nested.get(sub_key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


def _local_cost_risk(rows: list[JSONDict], goal_sequence: int | None) -> JSONDict:
    selected = [row for row in rows if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    front: list[float] = []
    footprint: list[float] = []
    target: list[float] = []
    front_clearance: list[float] = []
    for row in selected:
        for sink, flat, nested in [
            (front, 'front_wedge_lethal_count', 'front_wedge_cost'),
            (footprint, 'robot_footprint_lethal_count', 'robot_footprint_cost'),
            (target, 'target_footprint_lethal_count', 'target_footprint_cost'),
        ]:
            value = _numeric_from_nested(row, flat, nested)
            if value is not None:
                sink.append(value)
        try:
            if row.get('front_wedge_clearance_m') is not None:
                front_clearance.append(float(row.get('front_wedge_clearance_m')))
        except (TypeError, ValueError):
            pass
    max_front = max(front) if front else None
    max_footprint = max(footprint) if footprint else None
    risk = bool((max_front is not None and max_front >= LOCAL_COST_LETHAL_THRESHOLD) or (max_footprint is not None and max_footprint >= LOCAL_COST_LETHAL_THRESHOLD))
    return {
        'sample_count': len(selected),
        'front_wedge_lethal_count_max': max_front,
        'front_wedge_lethal_count_last': front[-1] if front else None,
        'robot_footprint_lethal_count_max': max_footprint,
        'robot_footprint_lethal_count_last': footprint[-1] if footprint else None,
        'target_footprint_lethal_count_max': max(target) if target else None,
        'front_wedge_clearance_min_m': min(front_clearance) if front_clearance else None,
        'local_cost_risk_present': risk,
    }


def _raw_capture_summary(raw: JSONDict | None) -> JSONDict:
    raw = raw if isinstance(raw, dict) else {}
    return {
        'raw_capture_available': bool(raw),
        'scan_available': raw.get('scan') is not None,
        'local_costmap_available': raw.get('local_costmap') is not None,
        'odom_available': raw.get('odom') is not None,
        'tf_available': bool(raw.get('tf')),
        'footprint_available': raw.get('footprint') is not None,
    }


def _summary_from_prior_analysis(phase_dir: Path | None, run_id: str) -> JSONDict:
    analysis_path = _glob_first(phase_dir, 'analysis.json', run_id=run_id)
    analysis = _load_json(analysis_path, {})
    if not analysis and phase_dir is None:
        return {'available': False, 'analysis_path': None}

    ref = analysis.get('goal2_refinement_summary') if isinstance(analysis.get('goal2_refinement_summary'), dict) else {}
    feedback = analysis.get('nav2_feedback_summary') if isinstance(analysis.get('nav2_feedback_summary'), dict) else {}
    local = analysis.get('local_costmap_summary') if isinstance(analysis.get('local_costmap_summary'), dict) else {}
    outcome = analysis.get('goal2_outcome') if isinstance(analysis.get('goal2_outcome'), dict) else {}

    events = _read_jsonl(_glob_first(phase_dir, 'goal_events.jsonl', run_id=run_id))
    dispatch, _rows, validation_goal_sequence, _fallback = _select_goal2_dispatch(events)
    extracted = _extract_selected_refinement(dispatch)
    nav_rows = _read_jsonl(_glob_first(phase_dir, 'nav2_feedback.jsonl', run_id=run_id))
    local_rows = _read_jsonl(_glob_first(phase_dir, 'local_costmap_samples.jsonl', run_id=run_id))
    execution = _outcome_summary(_rows, nav_rows, validation_goal_sequence) if nav_rows else {}
    local_from_rows = _local_cost_risk(local_rows, validation_goal_sequence) if local_rows else {}

    def first_present(*values: Any) -> Any:
        for value in values:
            if value is not None:
                return value
        return None

    available = bool(analysis or events or nav_rows or local_rows)
    return {
        'available': available,
        'analysis_path': str(analysis_path) if analysis_path else None,
        'classification': analysis.get('classification') if isinstance(analysis, dict) else None,
        'refinement_applied': first_present(ref.get('refinement_applied'), extracted.get('refinement_applied')),
        'hard_safety_pass_candidate_count': first_present(ref.get('hard_safety_pass_candidate_count'), extracted.get('hard_safety_pass_candidate_count')),
        'selected_candidate_target': first_present(ref.get('selected_candidate_target'), extracted.get('selected_candidate_target')),
        'nav2_target_shift_m': first_present(ref.get('nav2_target_shift_m'), extracted.get('nav2_target_shift_from_original_m')),
        'recoveries_max': first_present(feedback.get('recoveries_max'), execution.get('recoveries_max')),
        'distance_remaining_last': first_present(feedback.get('distance_remaining_last'), execution.get('distance_remaining_last')),
        'front_wedge_lethal_count_max': first_present(local.get('front_wedge_lethal_count_max'), local_from_rows.get('front_wedge_lethal_count_max')),
        'robot_footprint_lethal_count_max': first_present(local.get('robot_footprint_lethal_count_max'), local_from_rows.get('robot_footprint_lethal_count_max')),
        'terminal_timed_out': first_present(outcome.get('timed_out'), execution.get('terminal_timed_out')),
        'terminal_succeeded': first_present(outcome.get('succeeded'), execution.get('terminal_succeeded')),
        'terminal_observed': first_present(outcome.get('observed'), execution.get('terminal_observed')),
    }


def _comparison_summary(current: JSONDict, local_cost: JSONDict, phase89: JSONDict, phase85: JSONDict) -> JSONDict:
    def delta(cur: Any, old: Any) -> float | None:
        if isinstance(cur, (int, float)) and isinstance(old, (int, float)):
            return float(cur) - float(old)
        return None
    current_ref = current.get('selected_refinement') if isinstance(current.get('selected_refinement'), dict) else {}
    current_exec = current.get('execution_outcome') if isinstance(current.get('execution_outcome'), dict) else {}
    return {
        'phase89': phase89,
        'phase85': phase85,
        'phase93_vs_phase89': {
            'refinement_applied_changed_false_to_true': phase89.get('refinement_applied') is False and current_ref.get('refinement_applied') is True,
            'hard_safety_pass_candidate_count_delta': delta(current_ref.get('hard_safety_pass_candidate_count'), phase89.get('hard_safety_pass_candidate_count')),
            'recoveries_max_delta': delta(current_exec.get('recoveries_max'), phase89.get('recoveries_max')),
            'distance_remaining_last_delta': delta(current_exec.get('distance_remaining_last'), phase89.get('distance_remaining_last')),
            'front_wedge_lethal_count_max_delta': delta(local_cost.get('front_wedge_lethal_count_max'), phase89.get('front_wedge_lethal_count_max')),
            'robot_footprint_lethal_count_max_delta': delta(local_cost.get('robot_footprint_lethal_count_max'), phase89.get('robot_footprint_lethal_count_max')),
        },
        'phase93_vs_phase85': {
            'refinement_applied_changed_false_to_true': phase85.get('refinement_applied') is False and current_ref.get('refinement_applied') is True,
            'hard_safety_pass_candidate_count_delta': delta(current_ref.get('hard_safety_pass_candidate_count'), phase85.get('hard_safety_pass_candidate_count')),
            'recoveries_max_delta': delta(current_exec.get('recoveries_max'), phase85.get('recoveries_max')),
            'distance_remaining_last_delta': delta(current_exec.get('distance_remaining_last'), phase85.get('distance_remaining_last')),
            'front_wedge_lethal_count_max_delta': delta(local_cost.get('front_wedge_lethal_count_max'), phase85.get('front_wedge_lethal_count_max')),
            'robot_footprint_lethal_count_max_delta': delta(local_cost.get('robot_footprint_lethal_count_max'), phase85.get('robot_footprint_lethal_count_max')),
        },
    }


def analyze_artifact_dir(
    artifact_dir: Path | str,
    *,
    phase89_dir: Path | str | None = None,
    phase85_dir: Path | str | None = None,
) -> JSONDict:
    artifact_dir = Path(artifact_dir)
    phase89_dir = Path(phase89_dir) if phase89_dir is not None else None
    phase85_dir = Path(phase85_dir) if phase85_dir is not None else None

    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl', run_id=PHASE93_RUN_ID)
    feedback_path = _glob_first(artifact_dir, 'nav2_feedback.jsonl', run_id=PHASE93_RUN_ID)
    local_cost_path = _glob_first(artifact_dir, 'local_costmap_samples.jsonl', run_id=PHASE93_RUN_ID)
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json', run_id=PHASE93_RUN_ID)
    source_analysis_path = _glob_first(artifact_dir, 'analysis.json', run_id=PHASE93_RUN_ID)

    events = _read_jsonl(goal_events_path)
    feedback = _read_jsonl(feedback_path)
    local_cost_rows = _read_jsonl(local_cost_path)
    raw_capture = _load_json(raw_capture_path, {})
    source_analysis = _load_json(source_analysis_path, {})

    dispatch, validation_events, validation_goal_sequence, used_target_match_fallback = _select_goal2_dispatch(events)
    selected_refinement = _extract_selected_refinement(dispatch)
    execution_outcome = _outcome_summary(validation_events, feedback, validation_goal_sequence)
    local_cost = _local_cost_risk(local_cost_rows, validation_goal_sequence)
    raw_summary = _raw_capture_summary(raw_capture)

    result_stub = {
        'selected_refinement': selected_refinement,
        'execution_outcome': execution_outcome,
    }
    phase89 = _summary_from_prior_analysis(phase89_dir, PHASE89_RUN_ID)
    phase85 = _summary_from_prior_analysis(phase85_dir, PHASE85_RUN_ID)
    comparison = _comparison_summary(result_stub, local_cost, phase89, phase85)

    gaps: list[str] = []
    if not goal_events_path or not events:
        gaps.append('missing_phase93_goal_events_artifact')
    if not feedback_path or not feedback:
        gaps.append('missing_phase93_nav2_feedback_artifact')
    if not local_cost_path or not local_cost_rows:
        gaps.append('missing_phase93_local_costmap_samples_artifact')
    if not raw_capture_path or not raw_capture:
        gaps.append('missing_phase93_raw_capture_artifact')
    if not source_analysis_path or not source_analysis:
        gaps.append('missing_phase93_source_analysis_artifact')
    if dispatch is None:
        gaps.append('missing_goal2_equivalent_dispatch_event')
    if selected_refinement.get('refinement_applied') is not True:
        gaps.append('applied_single_step_refinement_not_observed')
    for field in REQUIRED_EXTRACTED_REFINEMENT_FIELDS:
        if field == 'rejected_candidate_summaries':
            if selected_refinement.get(field) is None:
                gaps.append(f'missing_refinement_field:{field}')
        elif selected_refinement.get(field) is None:
            gaps.append(f'missing_refinement_field:{field}')
    trace = selected_refinement.get('selection_priority_trace')
    if isinstance(trace, list):
        normalized = [_norm_token(item) for item in trace]
        if normalized[: len(PRIORITY_TRACE)] != PRIORITY_TRACE:
            gaps.append('selection_priority_trace_not_safety_first')
    else:
        gaps.append('missing_or_invalid_selection_priority_trace')
    if selected_refinement.get('branch_scoring_changed') is not False:
        gaps.append('branch_scoring_changed_not_false')
    if selected_refinement.get('fallback_terminal_acceptance_used') is not False:
        gaps.append('fallback_terminal_acceptance_used_not_false')
    if not phase89.get('available'):
        gaps.append('missing_phase89_comparison_analysis')
    if not phase85.get('available'):
        gaps.append('missing_phase85_comparison_analysis')
    if not execution_outcome.get('terminal_observed'):
        gaps.append('missing_terminal_outcome_for_applied_refinement')

    fatal_gap = any(
        gap.startswith('missing_refinement_field:')
        or gap in {
            'missing_phase93_goal_events_artifact',
            'missing_goal2_equivalent_dispatch_event',
            'applied_single_step_refinement_not_observed',
            'branch_scoring_changed_not_false',
            'fallback_terminal_acceptance_used_not_false',
        }
        for gap in gaps
    )
    if fatal_gap:
        classification = 'APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE'
    elif execution_outcome['held_before_terminal_outcome']:
        classification = 'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME'
    elif execution_outcome['recovery_dominant']:
        classification = 'APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT'
    elif local_cost['local_cost_risk_present']:
        classification = 'APPLIED_REFINEMENT_LOCAL_COST_RISK_PERSISTS'
    elif execution_outcome['terminal_timed_out'] or execution_outcome['terminal_failed'] or execution_outcome['terminal_canceled']:
        classification = 'APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT'
    elif execution_outcome['terminal_succeeded'] and execution_outcome['recoveries_max'] == 0:
        classification = 'APPLIED_REFINEMENT_EXECUTION_IMPROVED'
    else:
        classification = 'APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE'

    result: JSONDict = {
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': gaps,
        'validation_goal_sequence': validation_goal_sequence,
        'used_target_match_fallback': used_target_match_fallback,
        'selected_refinement': selected_refinement,
        'execution_outcome': execution_outcome,
        'local_cost_risk': local_cost,
        'raw_capture_summary': raw_summary,
        'comparison_to_phase89_phase85': comparison,
        'source_phase93_analysis': {
            'path': str(source_analysis_path) if source_analysis_path else None,
            'classification': source_analysis.get('classification') if isinstance(source_analysis, dict) else None,
        },
        'guardrails': {
            'no_maze_explorer_strategy_changed': True,
            'no_phase88_or_phase92_logic_changed': True,
            'no_branch_scoring_changed': selected_refinement.get('branch_scoring_changed') is False,
            'no_exploration_order_changed': True,
            'no_centerline_gate_changed': True,
            'no_directional_readiness_changed': True,
            'no_fallback_terminal_acceptance_changed': selected_refinement.get('fallback_terminal_acceptance_used') is False,
            'no_nav2_mppi_controller_tuning': True,
            'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
        },
        'success_claim_guardrails': {
            'timeout_treated_as_success': False,
            'autonomous_exploration_success_claimed': False,
            'exit_success_claimed': False,
            'phase95_entered': False,
        },
        'source_files': {
            'phase93_goal_events': str(goal_events_path) if goal_events_path else None,
            'phase93_nav2_feedback': str(feedback_path) if feedback_path else None,
            'phase93_local_costmap_samples': str(local_cost_path) if local_cost_path else None,
            'phase93_raw_capture': str(raw_capture_path) if raw_capture_path else None,
            'phase93_analysis': str(source_analysis_path) if source_analysis_path else None,
            'phase89_dir': str(phase89_dir) if phase89_dir else None,
            'phase85_dir': str(phase85_dir) if phase85_dir else None,
        },
    }
    return result


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    selected = result.get('selected_refinement') or {}
    outcome = result.get('execution_outcome') or {}
    local = result.get('local_cost_risk') or {}
    comparison = result.get('comparison_to_phase89_phase85') or {}
    p89_delta = (comparison.get('phase93_vs_phase89') or {}) if isinstance(comparison, dict) else {}
    lines = [
        '# Phase94 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Validation goal sequence: {result.get('validation_goal_sequence')} target_match_fallback={result.get('used_target_match_fallback')}",
        f"Refinement applied: {selected.get('refinement_applied')}",
        f"Selected refined target: {selected.get('selected_refined_target')}",
        f"Selected candidate index: {selected.get('selected_candidate_index')}",
        f"Selected candidate target: {selected.get('selected_candidate_target')}",
        f"Selected candidate yaw: {selected.get('selected_candidate_yaw')}",
        f"hard_safety_pass_candidate_count={selected.get('hard_safety_pass_candidate_count')}",
        f"rejected_candidate_summary_count={selected.get('rejected_candidate_summary_count')}",
        f"Execution: terminal_observed={outcome.get('terminal_observed')} terminal_event={outcome.get('terminal_event')} terminal_reason={outcome.get('terminal_reason')}",
        f"Distance remaining: first_nonzero={outcome.get('distance_remaining_first_nonzero')} last={outcome.get('distance_remaining_last')} progress_delta={outcome.get('distance_remaining_progress_delta')}",
        f"Recoveries: recoveries_max={outcome.get('recoveries_max')} recoveries_last={outcome.get('recoveries_last')} recovery_dominant={outcome.get('recovery_dominant')}",
        f"Local cost: local_cost_risk_present={local.get('local_cost_risk_present')} front_wedge_lethal_count_max={local.get('front_wedge_lethal_count_max')} robot_footprint_lethal_count_max={local.get('robot_footprint_lethal_count_max')}",
        f"Phase93 vs Phase89: hard_safety_pass_candidate_count_delta={p89_delta.get('hard_safety_pass_candidate_count_delta')} recoveries_max_delta={p89_delta.get('recoveries_max_delta')} distance_remaining_last_delta={p89_delta.get('distance_remaining_last_delta')}",
        '',
        'Conservative interpretation: no terminal success or exit success is claimed unless terminal outcome proves it. Held-before-terminal remains inconclusive even with movement toward target.',
        'Guardrails: No maze_explorer strategy changed; No Phase88/92 logic changed; No branch scoring changed; No exploration order changed; No centerline gate changed; No directional readiness changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; No autonomous exploration success claimed; No exit success claimed; Phase95 not entered.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log') / PHASE93_RUN_ID)
    parser.add_argument('--phase89-dir', type=Path, default=Path('log') / PHASE89_RUN_ID)
    parser.add_argument('--phase85-dir', type=Path, default=Path('log') / PHASE85_RUN_ID)
    parser.add_argument('--output-json', type=Path, default=None)
    parser.add_argument('--minimal-summary', type=Path, default=None)
    args = parser.parse_args()

    result = analyze_artifact_dir(args.artifact_dir, phase89_dir=args.phase89_dir, phase85_dir=args.phase85_dir)
    outdir = Path('log') / RUN_ID
    output = args.output_json or outdir / f'{RUN_ID}_analysis.json'
    summary = args.minimal_summary or outdir / f'{RUN_ID}_minimal_field_summary.md'
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
