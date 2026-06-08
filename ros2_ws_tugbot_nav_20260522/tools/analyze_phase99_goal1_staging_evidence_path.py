#!/usr/bin/env python3
"""Phase99 Goal1 staging evidence path diagnosis analyzer.

Read-only analyzer for Phase97/Phase98 artifacts. It explains why Phase92
staging was triggered for Goal1 but rejected with missing same-corridor /
two-side-wall evidence. It does not run ROS, send goals, mutate maze_explorer,
or tune Nav2/config.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
from typing import Any

PHASE97_RUN_ID = 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'
PHASE99_RUN_ID = 'phase99_goal1_staging_evidence_path_diagnosis'
PHASE98_ANALYSIS_NAME = 'phase98_goal1_recovery_dominant_failure_root_cause_analysis.json'

ALLOWED_CLASSIFICATIONS = [
    'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE',
    'STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88',
    'STAGING_FRAME_OR_POSE_PROJECTION_MISMATCH',
    'ENTRANCE_GOAL1_SINGLE_SIDE_WALL_GEOMETRY',
    'STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN',
    'STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA',
]

GUARDRAILS = {
    'algorithm_changed': False,
    'phase88_92_logic_changed': False,
    'branch_scoring_or_order_changed': False,
    'nav2_config_changed': False,
    'autonomous_exploration_success_claimed': False,
    'exit_success_claimed': False,
    'phase100_entered': False,
}


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        loaded = json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception as exc:  # pragma: no cover - defensive artifact parser
        return {'_parse_error': str(exc), '_path': str(path)}
    return loaded if isinstance(loaded, dict) else {'_unexpected_json_type': type(loaded).__name__}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line_no, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except Exception as exc:  # pragma: no cover - defensive artifact parser
            rows.append({'_parse_error': str(exc), '_line_no': line_no, '_line': line[:300]})
            continue
        if isinstance(row, dict) and isinstance(row.get('state'), dict):
            state = dict(row['state'])
            for key in ('_recorder_elapsed_sec', '_recorder_seq', '_recorder_wall_time'):
                if key in row and key not in state:
                    state[key] = row[key]
            row = state
        if isinstance(row, dict):
            rows.append(row)
    return rows


def _goal_sequence(row: dict[str, Any]) -> int | None:
    value = row.get('goal_sequence')
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return None


def _is_num(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _num(value: Any) -> float | None:
    return float(value) if _is_num(value) else None


def _first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _goal1_dispatch(goal_events: list[dict[str, Any]]) -> dict[str, Any] | None:
    goal1 = [row for row in goal_events if _goal_sequence(row) == 1]
    dispatches = [row for row in goal1 if row.get('event') == 'dispatch']
    return dispatches[0] if dispatches else (goal1[0] if goal1 else None)


def _extract_xy(candidate: dict[str, Any]) -> list[float] | None:
    value = candidate.get('target_xy') or candidate.get('target') or candidate.get('target_pose')
    if isinstance(value, list) and len(value) >= 2 and _is_num(value[0]) and _is_num(value[1]):
        return [float(value[0]), float(value[1])]
    if isinstance(value, dict):
        x = _num(value.get('x'))
        y = _num(value.get('y'))
        if x is not None and y is not None:
            return [x, y]
    return None


def _range(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {'min': min(values), 'max': max(values)}


def _candidate_summary(candidates: list[Any]) -> dict[str, Any]:
    clean = [cand for cand in candidates if isinstance(cand, dict)]
    xs: list[float] = []
    ys: list[float] = []
    distances: list[float] = []
    forward_offsets: list[float] = []
    lateral_offsets: list[float] = []
    reject_counts: collections.Counter[str] = collections.Counter()
    for cand in clean:
        xy = _extract_xy(cand)
        if xy:
            xs.append(xy[0])
            ys.append(xy[1])
        for distance_key in ('staging_distance_m', 'distance_m'):
            val = _num(cand.get(distance_key))
            if val is not None:
                distances.append(val)
                break
        fwd = _num(cand.get('staging_forward_offset_m'))
        lat = _num(cand.get('staging_lateral_offset_m'))
        if fwd is None:
            fwd = _num(cand.get('forward_offset_m'))
        if lat is None:
            lat = _num(cand.get('lateral_offset_m'))
        if fwd is not None:
            forward_offsets.append(fwd)
        if lat is not None:
            lateral_offsets.append(lat)
        for reason in cand.get('candidate_reject_reasons') or []:
            reject_counts[str(reason)] += 1

    def count_bool(key: str, expected: bool) -> int:
        return sum(1 for cand in clean if cand.get(key) is expected)

    sample_keys = [
        'candidate_index', 'target_xy', 'target', 'target_yaw',
        'staging_distance_m', 'staging_forward_offset_m', 'staging_lateral_offset_m',
        'same_corridor', 'two_side_wall_evidence', 'left_wall_hit', 'right_wall_hit',
        'left_wall_clearance_m', 'right_wall_clearance_m', 'min_clearance_m',
        'hard_safety_pass', 'safety_floor_ok', 'bounded_short_distance',
        'forward_progress_ok', 'candidate_reject_reasons',
    ]
    samples = [{key: cand.get(key) for key in sample_keys if key in cand} for cand in clean[:10]]
    return {
        'candidate_count': len(clean),
        'same_corridor_true_count': count_bool('same_corridor', True),
        'same_corridor_false_count': count_bool('same_corridor', False),
        'two_side_wall_true_count': count_bool('two_side_wall_evidence', True),
        'two_side_wall_false_count': count_bool('two_side_wall_evidence', False),
        'hard_safety_pass_true_count': count_bool('hard_safety_pass', True),
        'hard_safety_pass_false_count': count_bool('hard_safety_pass', False),
        'safety_floor_ok_true_count': count_bool('safety_floor_ok', True),
        'bounded_short_distance_true_count': count_bool('bounded_short_distance', True),
        'forward_progress_ok_true_count': count_bool('forward_progress_ok', True),
        'x_range': _range(xs),
        'y_range': _range(ys),
        'x_min': min(xs) if xs else None,
        'x_max': max(xs) if xs else None,
        'y_min': min(ys) if ys else None,
        'y_max': max(ys) if ys else None,
        'staging_distance_min_m': min(distances) if distances else None,
        'staging_distance_max_m': max(distances) if distances else None,
        'staging_forward_offset_range_m': _range(forward_offsets),
        'staging_lateral_offset_range_m': _range(lateral_offsets),
        'reject_reason_counts': dict(sorted(reject_counts.items())),
        'samples': samples,
    }


def _phase88_summary(refinement: dict[str, Any]) -> dict[str, Any]:
    candidates_value = refinement.get('candidates')
    candidates: list[Any] = candidates_value if isinstance(candidates_value, list) else []
    summary = _candidate_summary(candidates)
    original_metrics_value = refinement.get('original_metrics')
    original_metrics: dict[str, Any] = original_metrics_value if isinstance(original_metrics_value, dict) else {}
    forward_check_value = refinement.get('forward_executability_check')
    forward_check: dict[str, Any] = forward_check_value if isinstance(forward_check_value, dict) else {}
    return {
        'candidate_count': _first_present(refinement.get('candidate_count'), summary['candidate_count']),
        'hard_safety_pass_candidate_count': _first_present(refinement.get('hard_safety_pass_candidate_count'), summary['hard_safety_pass_true_count']),
        'two_side_wall_candidate_count': _first_present(refinement.get('two_side_wall_candidate_count'), summary['two_side_wall_true_count']),
        'refinement_applied': refinement.get('refinement_applied'),
        'refinement_reject_reason': refinement.get('refinement_reject_reason'),
        'corridor_heading_yaw': refinement.get('corridor_heading_yaw'),
        'candidate_summary': summary,
        'original_metrics': {
            key: original_metrics.get(key)
            for key in (
                'target', 'same_corridor', 'two_side_wall_evidence', 'left_wall_hit', 'right_wall_hit',
                'left_wall_clearance_m', 'right_wall_clearance_m', 'min_clearance_m',
                'target_has_clearance', 'occupancy_free', 'local_cost_sample_count', 'front_wedge_sample_count',
            )
            if key in original_metrics
        },
        'forward_executability_check': {
            key: forward_check.get(key)
            for key in ('checked', 'reason', 'same_corridor', 'two_side_wall_evidence', 'hard_safety_pass', 'safety_floor_ok', 'target_has_clearance')
            if key in forward_check
        },
    }


def _raw_capture_summary(raw: dict[str, Any] | None) -> dict[str, Any]:
    raw = raw if isinstance(raw, dict) else {}
    out: dict[str, Any] = {}
    for key in ('scan', 'map', 'local_costmap', 'odom', 'tf', 'robot_pose_by_frame', 'footprint'):
        value = raw.get(key)
        if isinstance(value, dict):
            entry: dict[str, Any] = {'available': True}
            if key == 'scan':
                entry.update({k: value.get(k) for k in ('ranges_count', 'ranges_min', 'ranges_max_finite', 'frame_id', 'stamp_sec') if k in value})
            elif key in ('map', 'local_costmap'):
                entry.update({k: value.get(k) for k in ('frame_id', 'stamp_sec', 'summary', 'info') if k in value})
            elif key == 'odom':
                entry.update({k: value.get(k) for k in ('frame_id', 'child_frame_id', 'pose', 'stamp_sec') if k in value})
            elif key == 'tf':
                entry['available_transforms'] = sorted([name for name, item in value.items() if isinstance(item, dict) and item.get('available') is True])
                entry['map_base_link'] = value.get('map->base_link')
            else:
                entry.update(value)
            out[key] = entry
        else:
            out[key] = {'available': False}
    return out


def _load_phase98(phase98_dir: Path) -> dict[str, Any] | None:
    candidate_paths = [
        phase98_dir / PHASE98_ANALYSIS_NAME,
        phase98_dir / 'phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_analysis.json',
    ]
    for path in candidate_paths:
        loaded = _read_json(path)
        if loaded is not None:
            return loaded
    return None


def _staging_vs_phase88_comparison(phase88: dict[str, Any], staging_summary: dict[str, Any]) -> dict[str, Any]:
    p88_value = phase88.get('candidate_summary')
    p88: dict[str, Any] = p88_value if isinstance(p88_value, dict) else {}
    p88_y_value = p88.get('y_range')
    p92_y_value = staging_summary.get('y_range')
    p88_x_value = p88.get('x_range')
    p92_x_value = staging_summary.get('x_range')
    p88_y: dict[str, Any] = p88_y_value if isinstance(p88_y_value, dict) else {}
    p92_y: dict[str, Any] = p92_y_value if isinstance(p92_y_value, dict) else {}
    p88_x: dict[str, Any] = p88_x_value if isinstance(p88_x_value, dict) else {}
    p92_x: dict[str, Any] = p92_x_value if isinstance(p92_x_value, dict) else {}
    p88_y_min = p88_y.get('min')
    p92_y_max = p92_y.get('max')
    y_gap = None
    p88_y_min_num = _num(p88_y_min)
    p92_y_max_num = _num(p92_y_max)
    if p88_y_min_num is not None and p92_y_max_num is not None:
        y_gap = p88_y_min_num - p92_y_max_num
    interpretation = 'Insufficient evidence to compare Phase88 and Phase92 evidence windows.'
    if p88.get('two_side_wall_true_count', 0) > 0 and staging_summary.get('two_side_wall_true_count', 0) == 0:
        if y_gap is not None and y_gap > 0.4:
            interpretation = (
                'Phase88 evidence was available at the forward Goal1 target window, while Phase92 staging candidates were generated in a much earlier/near-dispatch window with no two-side-wall evidence.'
            )
        else:
            interpretation = (
                'Phase88 candidates retained two-side-wall evidence but Phase92 staging candidates did not; the evidence windows differ, but spatial separation is not strongly proven from y-ranges alone.'
            )
    elif p88.get('two_side_wall_true_count', 0) == 0 and staging_summary.get('two_side_wall_true_count', 0) == 0:
        interpretation = 'Both Phase88 and Phase92 lacked two-side-wall candidate evidence; this may indicate entrance/single-side-wall geometry or insufficient map/scan evidence.'
    return {
        'phase88_candidate_count': phase88.get('candidate_count'),
        'phase88_same_corridor_true_count': p88.get('same_corridor_true_count'),
        'phase88_same_corridor_false_count': p88.get('same_corridor_false_count'),
        'phase88_two_side_wall_true_count': p88.get('two_side_wall_true_count'),
        'phase88_two_side_wall_false_count': p88.get('two_side_wall_false_count'),
        'phase88_y_min': p88_y.get('min'),
        'phase88_y_max': p88_y.get('max'),
        'phase88_x_min': p88_x.get('min'),
        'phase88_x_max': p88_x.get('max'),
        'phase92_staging_candidate_count': staging_summary.get('candidate_count'),
        'phase92_same_corridor_true_count': staging_summary.get('same_corridor_true_count'),
        'phase92_same_corridor_false_count': staging_summary.get('same_corridor_false_count'),
        'phase92_two_side_wall_true_count': staging_summary.get('two_side_wall_true_count'),
        'phase92_two_side_wall_false_count': staging_summary.get('two_side_wall_false_count'),
        'phase92_y_min': p92_y.get('min'),
        'phase92_y_max': p92_y.get('max'),
        'phase92_x_min': p92_x.get('min'),
        'phase92_x_max': p92_x.get('max'),
        'evidence_window_delta': {
            'phase88_y_min': p88_y.get('min'),
            'phase88_y_max': p88_y.get('max'),
            'phase92_y_min': p92_y.get('min'),
            'phase92_y_max': p92_y.get('max'),
            'phase88_min_y_minus_phase92_max_y': y_gap,
        },
        'interpretation': interpretation,
    }


def _classify(evidence_gaps: list[str], phase88: dict[str, Any], staging_summary: dict[str, Any], dispatch_pose: Any, comparison: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    if evidence_gaps:
        return 'STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA', {'insufficient_data': True}

    p88 = phase88.get('candidate_summary') if isinstance(phase88.get('candidate_summary'), dict) else {}
    dispatch_y = None
    if isinstance(dispatch_pose, list) and len(dispatch_pose) >= 2 and _is_num(dispatch_pose[1]):
        dispatch_y = float(dispatch_pose[1])
    y_max = staging_summary.get('y_max')
    dist_max = staging_summary.get('staging_distance_max_m')
    p88_two = int(p88.get('two_side_wall_true_count') or 0)
    p92_two = int(staging_summary.get('two_side_wall_true_count') or 0)
    p92_same = int(staging_summary.get('same_corridor_true_count') or 0)
    p92_count = int(staging_summary.get('candidate_count') or 0)
    p88_y_min = comparison.get('phase88_y_min')
    p92_y_max = comparison.get('phase92_y_max')
    y_gap = comparison.get('evidence_window_delta', {}).get('phase88_min_y_minus_phase92_max_y') if isinstance(comparison.get('evidence_window_delta'), dict) else None

    staging_near_dispatch = False
    if _is_num(dispatch_y) and _is_num(y_max):
        staging_near_dispatch = abs(float(y_max) - float(dispatch_y)) <= 0.25
    short_candidates = _is_num(dist_max) and float(dist_max) <= 0.25
    phase88_forward_has_two_wall = p88_two > 0
    phase92_missing_two_wall = p92_count > 0 and p92_two == 0
    phase92_same_corridor_present = p92_same > 0
    window_mismatch = _is_num(y_gap) and float(y_gap) > 0.4
    root = {
        'staging_near_dispatch_pose': staging_near_dispatch,
        'short_staging_candidates': short_candidates,
        'phase88_forward_window_has_two_side_wall': phase88_forward_has_two_wall,
        'phase92_staging_window_missing_two_side_wall': phase92_missing_two_wall,
        'phase92_candidate_same_corridor_present_despite_aggregate_check_false': phase92_same_corridor_present,
        'phase88_phase92_y_window_gap_m': y_gap,
    }
    if phase88_forward_has_two_wall and phase92_missing_two_wall and short_candidates and staging_near_dispatch:
        return 'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE', root
    if phase88_forward_has_two_wall and phase92_missing_two_wall and window_mismatch:
        return 'STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88', root
    if phase88_forward_has_two_wall and phase92_missing_two_wall and phase92_same_corridor_present:
        return 'STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN', root
    if p88_two == 0 and phase92_missing_two_wall:
        return 'ENTRANCE_GOAL1_SINGLE_SIDE_WALL_GEOMETRY', root
    if phase92_missing_two_wall and not phase92_same_corridor_present and not short_candidates:
        return 'STAGING_FRAME_OR_POSE_PROJECTION_MISMATCH', root
    return 'STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA', root


def analyze_phase99(phase97_dir: Path | str, phase98_dir: Path | str) -> dict[str, Any]:
    phase97_dir = Path(phase97_dir)
    phase98_dir = Path(phase98_dir)
    evidence_gaps: list[str] = []

    goal_events = _read_jsonl(phase97_dir / f'{PHASE97_RUN_ID}_goal_events.jsonl')
    local_cost_rows = _read_jsonl(phase97_dir / f'{PHASE97_RUN_ID}_local_costmap_samples.jsonl')
    nav2_rows = _read_jsonl(phase97_dir / f'{PHASE97_RUN_ID}_nav2_feedback.jsonl')
    phase97_analysis = _read_json(phase97_dir / f'{PHASE97_RUN_ID}_analysis.json')
    raw_capture = _read_json(phase97_dir / f'{PHASE97_RUN_ID}_raw_capture.json')
    phase98_analysis = _load_phase98(phase98_dir)

    if not goal_events:
        evidence_gaps.append('phase97_goal_events_missing')
    if phase98_analysis is None:
        evidence_gaps.append('phase98_analysis_missing')
    elif phase98_analysis.get('classification') != 'GOAL1_STAGING_REJECT_ROOT_CAUSE':
        evidence_gaps.append('phase98_goal1_staging_reject_classification_missing')

    dispatch = _goal1_dispatch(goal_events)
    if dispatch is None:
        evidence_gaps.append('goal1_dispatch_missing')
        dispatch = {}

    staging_candidates = dispatch.get('staging_candidates')
    if not isinstance(staging_candidates, list) or not staging_candidates:
        evidence_gaps.append('goal1_staging_candidates_missing')
        staging_candidates = []
    staging_summary = _candidate_summary(staging_candidates)

    refinement = dispatch.get('centerline_target_refinement')
    if not isinstance(refinement, dict):
        evidence_gaps.append('goal1_phase88_centerline_refinement_missing')
        refinement = {}
    phase88 = _phase88_summary(refinement)

    plan = dispatch.get('two_step_staging_plan') if isinstance(dispatch.get('two_step_staging_plan'), dict) else {}
    exec_check = dispatch.get('staging_executability_check') if isinstance(dispatch.get('staging_executability_check'), dict) else {}
    if not exec_check:
        evidence_gaps.append('goal1_staging_executability_check_missing')
    comparison = _staging_vs_phase88_comparison(phase88, staging_summary)
    classification, root_evidence = _classify(evidence_gaps, phase88, staging_summary, dispatch.get('dispatch_pose'), comparison)

    raw_summary = _raw_capture_summary(raw_capture)
    result = {
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'classification': classification,
        'evidence_gaps': evidence_gaps,
        'source_artifacts': {
            'phase97_dir': str(phase97_dir),
            'phase98_dir': str(phase98_dir),
            'goal_events_rows': len(goal_events),
            'local_costmap_samples_rows': len(local_cost_rows),
            'nav2_feedback_rows': len(nav2_rows),
            'phase97_analysis_classification': phase97_analysis.get('classification') if isinstance(phase97_analysis, dict) else None,
            'phase98_analysis_classification': phase98_analysis.get('classification') if isinstance(phase98_analysis, dict) else None,
        },
        'goal1_staging_evidence_path': {
            'dispatch_pose': dispatch.get('dispatch_pose'),
            'robot_pose_from_raw_capture': (raw_capture.get('robot_pose_by_frame') if isinstance(raw_capture, dict) else None),
            'target': dispatch.get('target') or dispatch.get('original_target'),
            'selected_candidate_target': dispatch.get('selected_candidate_target'),
            'selected_candidate_yaw': dispatch.get('selected_candidate_yaw'),
            'corridor_heading_yaw': _first_present(refinement.get('corridor_heading_yaw'), dispatch.get('branch_angle')),
            'staging_plan': plan,
            'staging_goal_pose': dispatch.get('staging_goal_pose'),
            'staging_reject_reason': dispatch.get('staging_reject_reason'),
            'staging_applied': dispatch.get('staging_applied'),
            'staging_executability_check': exec_check,
            'staging_candidate_summary': staging_summary,
            'phase88_single_step_summary': phase88,
        },
        'phase88_vs_phase92_comparison': comparison,
        'root_cause_evidence': root_evidence,
        'raw_capture_summary': raw_summary,
        'future_design_assessment': {
            'needs_phase88_corridor_evidence_carry_over_or_evidence_reuse_for_staging': classification in {
                'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE',
                'STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88',
                'STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN',
            },
            'do_not_relax_safety_directly': True,
            'rationale': (
                'Phase88 forward target/candidate evidence contained same-corridor and two-side-wall wall hits, while Phase92 staging evaluated very short near-dispatch poses that lacked two-side-wall evidence. A future design should consider explicit Phase88 corridor evidence carry-over / evidence reuse for staging eligibility before any safety relaxation.'
            ),
        },
        'answers': {
            'why_same_corridor_two_side_wall_missing': _answer_missing_evidence(classification),
            'why_phase88_had_evidence_but_phase92_did_not': comparison['interpretation'],
            'future_design': 'Evaluate Phase88 corridor evidence carry-over / evidence reuse for staging. Do not directly relax same-corridor/two-side-wall or hard-safety requirements from this diagnosis.',
        },
        'guardrails': dict(GUARDRAILS),
    }
    return result


def _answer_missing_evidence(classification: str) -> str:
    if classification == 'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE':
        return 'Goal1 staging candidates are short near-dispatch/entrance poses; their local evidence window lacks two-side-wall hits even though Phase88 forward Goal1 target evidence has them.'
    if classification == 'STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88':
        return 'Phase92 staging and Phase88 single-step refinement evaluate different spatial evidence windows; Phase92 lacks two-side-wall evidence in its staging window while Phase88 has it forward at the Goal1 target window.'
    if classification == 'STAGING_FRAME_OR_POSE_PROJECTION_MISMATCH':
        return 'The available artifact pattern is consistent with a possible staging frame/projection mismatch, but this requires more bounded diagnostic evidence before changing code.'
    if classification == 'ENTRANCE_GOAL1_SINGLE_SIDE_WALL_GEOMETRY':
        return 'Both Phase88 and Phase92 lack two-side-wall evidence, suggesting entrance/single-side-wall geometry or insufficient local map/scan evidence.'
    if classification == 'STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN':
        return 'Phase92 has some staging same-corridor evidence but no two-side-wall evidence; future design should consider evidence reuse rather than weakening safety checks.'
    return 'Insufficient required staging evidence path artifacts to classify without inventing missing evidence.'


def render_minimal_summary(result: dict[str, Any]) -> str:
    g1 = result.get('goal1_staging_evidence_path', {}) if isinstance(result.get('goal1_staging_evidence_path'), dict) else {}
    staging = g1.get('staging_candidate_summary', {}) if isinstance(g1.get('staging_candidate_summary'), dict) else {}
    comparison = result.get('phase88_vs_phase92_comparison', {}) if isinstance(result.get('phase88_vs_phase92_comparison'), dict) else {}
    future = result.get('future_design_assessment', {}) if isinstance(result.get('future_design_assessment'), dict) else {}
    lines = [
        '# Phase99 Goal1 staging evidence path minimal summary',
        '',
        f"classification: {result.get('classification')}",
        f"evidence_gaps: {json.dumps(result.get('evidence_gaps', []), sort_keys=True)}",
        f"staging_reject_reason: {g1.get('staging_reject_reason')}",
        f"staging_applied: {g1.get('staging_applied')}",
        f"dispatch_pose: {g1.get('dispatch_pose')}",
        f"target: {g1.get('target')}",
        f"corridor_heading_yaw: {g1.get('corridor_heading_yaw')}",
        f"phase92_staging_candidate_count: {staging.get('candidate_count')}",
        f"phase92_same_corridor_true_count: {staging.get('same_corridor_true_count')}",
        f"phase92_two_side_wall_true_count: {staging.get('two_side_wall_true_count')}",
        f"phase92_staging_distance_max_m: {staging.get('staging_distance_max_m')}",
        f"phase92_y_range: {staging.get('y_range')}",
        f"phase88_candidate_count: {comparison.get('phase88_candidate_count')}",
        f"phase88_same_corridor_true_count: {comparison.get('phase88_same_corridor_true_count')}",
        f"phase88_two_side_wall_true_count: {comparison.get('phase88_two_side_wall_true_count')}",
        f"phase88_y_range: {{'min': {comparison.get('phase88_y_min')}, 'max': {comparison.get('phase88_y_max')}}}",
        f"window_delta: {json.dumps(comparison.get('evidence_window_delta'), sort_keys=True)}",
        '',
        'interpretation:',
        f"- {comparison.get('interpretation')}",
        f"- {result.get('answers', {}).get('why_same_corridor_two_side_wall_missing')}",
        '',
        'future design assessment:',
        f"- Phase88 corridor evidence carry-over / evidence reuse for staging: {future.get('needs_phase88_corridor_evidence_carry_over_or_evidence_reuse_for_staging')}",
        f"- do_not_relax_safety_directly: {future.get('do_not_relax_safety_directly')}",
        '',
        'guardrails: no strategy/Phase88/92/Nav2/config change; no autonomous or exit success claim; stop before Phase100.',
    ]
    return '\n'.join(lines) + '\n'


def save_outputs(result: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'phase99_goal1_staging_evidence_path_analysis.json').write_text(
        json.dumps(result, indent=2, sort_keys=True), encoding='utf-8'
    )
    (output_dir / 'phase99_goal1_staging_evidence_path_minimal_summary.md').write_text(
        render_minimal_summary(result), encoding='utf-8'
    )


def main() -> int:
    parser = argparse.ArgumentParser(description='Analyze Phase99 Goal1 staging evidence path from Phase97/98 artifacts.')
    parser.add_argument('--phase97-dir', type=Path, default=Path('log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'))
    parser.add_argument('--phase98-dir', type=Path, default=Path('log/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis'))
    parser.add_argument('--output-dir', type=Path, default=Path('log/phase99_goal1_staging_evidence_path_diagnosis'))
    args = parser.parse_args()
    result = analyze_phase99(args.phase97_dir, args.phase98_dir)
    save_outputs(result, args.output_dir)
    print(json.dumps({'classification': result['classification'], 'evidence_gaps': result['evidence_gaps']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
