#!/usr/bin/env python3
"""Phase86 lethal_cost_regression reject root-cause diagnosis.

Read-only analyzer for the Phase85 Goal2-equivalent bounded validation artifacts.
It diagnoses why Phase84 corridor-aligned refinement rejected all candidates with
`lethal_cost_regression` without changing maze_explorer, branch scoring,
centerline gates, directional readiness, fallback/terminal acceptance, Nav2,
MPPI, controller, inflation, robot radius, clearance radius, or map thresholds.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase86_lethal_cost_regression_reject_root_cause'
PHASE85_RUN_ID = 'phase85_goal2_corridor_aligned_refinement_bounded_validation'

ALLOWED_CLASSIFICATIONS = {
    'REFINED_TARGET_IN_HIGH_COST_BAND',
    'CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE',
    'FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE',
    'BASELINE_ALREADY_RISKY_COMPARISON_AMBIGUOUS',
    'MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED',
    'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE',
}

REJECT_SUBITEMS = [
    'safety_floor_ok',
    'footprint_lethal_not_increased',
    'front_wedge_lethal_not_increased',
    'forward_progress_not_lowered',
    'target_has_clearance',
    'occupancy_free',
    'same_corridor',
    'two_side_wall_evidence',
]

HARD_SAFETY_ITEMS = [
    'safety_floor_ok',
    'footprint_lethal_not_increased',
    'front_wedge_lethal_not_increased',
    'forward_progress_not_lowered',
    'target_has_clearance',
    'occupancy_free',
    'same_corridor',
    'two_side_wall_evidence',
]

GUARDRAILS = {
    'no_strategy_or_config_tuning': True,
    'no_maze_explorer_strategy_changed': True,
    'no_branch_scoring_changed': True,
    'no_centerline_gate_changed': True,
    'no_directional_readiness_override_changed': True,
    'no_fallback_terminal_acceptance_changed': True,
    'no_nav2_mppi_controller_tuning': True,
    'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
}


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        loaded = _safe_json_loads(line)
        if not isinstance(loaded, dict):
            continue
        payload = loaded.get('state') if isinstance(loaded.get('state'), dict) else loaded
        if isinstance(payload, dict):
            row = dict(payload)
            row.setdefault('_recorder_elapsed_sec', loaded.get('elapsed_sec'))
            row.setdefault('_recorder_wall_time', loaded.get('wall_time'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path, suffix: str) -> Path | None:
    direct = artifact_dir / f'{PHASE85_RUN_ID}_{suffix}'
    if direct.exists():
        return direct
    matches = sorted(artifact_dir.glob(f'*{suffix}'))
    return matches[0] if matches else None


def _nested_refinement(context: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(context, dict):
        return {}
    nested = context.get('centerline_target_refinement')
    return nested if isinstance(nested, dict) else {}


def _phase85_context(phase85_analysis: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    context = phase85_analysis.get('goal2_dispatch_context')
    context = context if isinstance(context, dict) else {}
    nested = _nested_refinement(context)
    return context, nested


def _as_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        f = float(value)
        if math.isnan(f):
            return None
        return f
    except (TypeError, ValueError):
        return None


def _bool_counts(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    true_count = sum(1 for row in rows if row.get(key) is True)
    false_count = sum(1 for row in rows if row.get(key) is False)
    missing_count = sum(1 for row in rows if key not in row or row.get(key) is None)
    return {
        'candidate_true_count': true_count,
        'candidate_false_count': false_count,
        'candidate_missing_count': missing_count,
    }


def _reject_subitems(forward_check: dict[str, Any], candidates: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for key in REJECT_SUBITEMS:
        counts = _bool_counts(candidates, key)
        out[key] = {
            'forward_check_value': forward_check.get(key),
            **counts,
        }
    return out


def _hard_safety_pass(candidate: dict[str, Any]) -> bool:
    return all(candidate.get(key) is True for key in HARD_SAFETY_ITEMS)


def _metric_range(candidates: list[dict[str, Any]], key: str) -> dict[str, float | None]:
    vals = [_as_float(c.get(key)) for c in candidates]
    vals = [v for v in vals if v is not None]
    return {
        'min': min(vals) if vals else None,
        'max': max(vals) if vals else None,
    }


def _candidate_aggregate(candidates: list[dict[str, Any]], original_metrics: dict[str, Any]) -> dict[str, Any]:
    candidate_count = len(candidates)
    hard_safe = [c for c in candidates if _hard_safety_pass(c)]
    eligible = [c for c in candidates if c.get('eligible') is True]
    balance_first = [c for c in candidates if c.get('balance_first_eligible') is True]
    best_balance_error: float | None = None
    for c in candidates:
        val = _as_float(c.get('balance_error_m'))
        if val is not None:
            best_balance_error = val if best_balance_error is None else min(best_balance_error, val)
    best_balance_candidates = [c for c in candidates if _as_float(c.get('balance_error_m')) == best_balance_error] if best_balance_error is not None else []

    original_local_max = _as_float(original_metrics.get('local_cost_max_radius'))
    original_front_wedge_lethal = _as_float(original_metrics.get('front_wedge_lethal_count')) or 0.0
    original_footprint_lethal = _as_float(original_metrics.get('footprint_lethal_count')) or 0.0
    original_min_clearance = _as_float(original_metrics.get('min_clearance_m'))

    high_band = []
    lethal_regression = []
    for c in candidates:
        local_max = _as_float(c.get('local_cost_max_radius'))
        front_lethal = _as_float(c.get('front_wedge_lethal_count')) or 0.0
        footprint_lethal = _as_float(c.get('footprint_lethal_count')) or 0.0
        if (local_max is not None and local_max >= 99) or front_lethal > 0 or footprint_lethal > 0:
            high_band.append(c)
        if front_lethal > original_front_wedge_lethal or footprint_lethal > original_footprint_lethal:
            lethal_regression.append(c)

    best_balance_all_clearance_failed = bool(best_balance_candidates) and all(
        c.get('target_has_clearance') is False or c.get('safety_floor_ok') is False for c in best_balance_candidates
    )
    best_balance_all_high_or_lethal = bool(best_balance_candidates) and all(
        (_as_float(c.get('local_cost_max_radius')) or 0.0) >= 70.0
        or (_as_float(c.get('front_wedge_lethal_count')) or 0.0) > 0.0
        or (_as_float(c.get('footprint_lethal_count')) or 0.0) > 0.0
        for c in best_balance_candidates
    )

    return {
        'candidate_count': candidate_count,
        'eligible_candidate_count': len(eligible),
        'balance_first_eligible_candidate_count': len(balance_first),
        'hard_safety_pass_candidate_count': len(hard_safe),
        'high_or_lethal_candidate_count': len(high_band),
        'lethal_regression_candidate_count': len(lethal_regression),
        'best_balance_error_m': best_balance_error,
        'best_balance_candidate_count': len(best_balance_candidates),
        'best_balance_candidates_all_clearance_failed': best_balance_all_clearance_failed,
        'best_balance_candidates_all_high_or_lethal': best_balance_all_high_or_lethal,
        'original_local_cost_max_radius': original_local_max,
        'original_front_wedge_lethal_count': original_front_wedge_lethal,
        'original_footprint_lethal_count': original_footprint_lethal,
        'original_min_clearance_m': original_min_clearance,
        'lateral_offset_range_m': _metric_range(candidates, 'lateral_offset_m'),
        'forward_offset_range_m': _metric_range(candidates, 'forward_offset_m'),
        'local_cost_max_radius_range': _metric_range(candidates, 'local_cost_max_radius'),
        'front_wedge_lethal_count_range': _metric_range(candidates, 'front_wedge_lethal_count'),
        'footprint_lethal_count_range': _metric_range(candidates, 'footprint_lethal_count'),
        'min_clearance_range_m': _metric_range(candidates, 'min_clearance_m'),
    }


def _local_sample_summary(rows: list[dict[str, Any]], goal_sequence: int | None) -> dict[str, Any]:
    if goal_sequence is not None:
        rows = [r for r in rows if r.get('goal_sequence') == goal_sequence]
    def values(path: list[str]) -> list[float]:
        out: list[float] = []
        for row in rows:
            cur: Any = row
            for part in path:
                cur = cur.get(part) if isinstance(cur, dict) else None
            val = _as_float(cur)
            if val is not None:
                out.append(val)
        return out
    summary: dict[str, Any] = {'sample_count': len(rows)}
    for key, path in {
        'robot_footprint_lethal_count': ['robot_footprint_cost', 'lethal_count'],
        'target_footprint_lethal_count': ['target_footprint_cost', 'lethal_count'],
        'front_wedge_lethal_count': ['front_wedge_cost', 'lethal_count'],
        'front_wedge_cost_max': ['front_wedge_cost', 'max'],
        'target_radius_cost_max': ['local_costmap_target_evidence', 'radius_cost_summary', 'max'],
    }.items():
        vals = values(path)
        summary[key] = {
            'min': min(vals) if vals else None,
            'max': max(vals) if vals else None,
            'last': vals[-1] if vals else None,
        }
    return summary


def _raw_capture_summary(raw: dict[str, Any]) -> dict[str, Any]:
    raw = raw if isinstance(raw, dict) else {}
    return {
        'scan_available': isinstance(raw.get('scan'), dict),
        'local_costmap_available': isinstance(raw.get('local_costmap'), dict),
        'footprint_available': isinstance(raw.get('footprint'), dict),
        'odom_available': isinstance(raw.get('odom'), dict),
        'tf_available': isinstance(raw.get('tf'), dict) and bool(raw.get('tf')),
    }


def _classification(
    *,
    evidence_gaps: list[str],
    phase85_classification: str | None,
    reject_reason: str | None,
    aggregate: dict[str, Any],
    forward_check: dict[str, Any],
) -> tuple[str, list[str], list[str]]:
    reasons: list[str] = []
    secondary: list[str] = []
    if evidence_gaps:
        return 'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE', ['missing_or_incomplete_phase85_reject_diagnostics'], secondary
    if phase85_classification != 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED' or reject_reason != 'lethal_cost_regression':
        return 'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE', ['source_phase85_not_lethal_cost_regression_reject'], secondary

    candidate_count = int(aggregate.get('candidate_count') or 0)
    if candidate_count <= 0:
        return 'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE', ['no_candidate_diagnostics_present'], secondary

    original_local_max = _as_float(aggregate.get('original_local_cost_max_radius')) or 0.0
    original_fw_lethal = _as_float(aggregate.get('original_front_wedge_lethal_count')) or 0.0
    original_fp_lethal = _as_float(aggregate.get('original_footprint_lethal_count')) or 0.0
    high_or_lethal_count = int(aggregate.get('high_or_lethal_candidate_count') or 0)
    hard_safe_count = int(aggregate.get('hard_safety_pass_candidate_count') or 0)

    if original_local_max >= 99.0 or original_fw_lethal > 0.0 or original_fp_lethal > 0.0:
        if high_or_lethal_count > 0:
            return 'BASELINE_ALREADY_RISKY_COMPARISON_AMBIGUOUS', ['original_target_already_has_high_or_lethal_cost'], secondary

    if aggregate.get('best_balance_candidates_all_clearance_failed') is True:
        reasons.append('best_balance_centerline_projection_candidates_failed_clearance_or_safety_floor')
        if aggregate.get('best_balance_candidates_all_high_or_lethal') is True:
            reasons.append('best_balance_centerline_projection_candidates_enter_high_or_lethal_band')
        if hard_safe_count > 0:
            secondary.append('MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED')
            secondary.append('FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE')
        return 'CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE', reasons, secondary

    if hard_safe_count > 0 and any(forward_check.get(key) is False for key in ['footprint_lethal_not_increased', 'front_wedge_lethal_not_increased', 'forward_progress_not_lowered', 'safety_floor_ok']):
        return 'FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE', ['hard_safe_candidate_exists_but_reject_reason_lethal_cost_regression'], secondary

    if high_or_lethal_count >= max(1, math.ceil(candidate_count * 0.5)):
        return 'REFINED_TARGET_IN_HIGH_COST_BAND', ['majority_of_refinement_candidates_enter_high_or_lethal_cost_band'], secondary

    if hard_safe_count > 0 and int(aggregate.get('eligible_candidate_count') or 0) == 0:
        return 'MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED', ['safe_candidate_exists_but_current_candidate_selection_has_no_eligible_candidate'], secondary

    return 'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE', ['reject_context_present_but_not_enough_to_distinguish_root_cause'], secondary


def analyze_phase85_artifacts(artifact_dir: Path | str) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    phase85_analysis_path = _glob_first(artifact_dir, 'analysis.json')
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json')
    local_samples_path = _glob_first(artifact_dir, 'local_costmap_samples.jsonl')
    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl')

    phase85_analysis = _safe_json_load(phase85_analysis_path, {})
    raw_capture = _safe_json_load(raw_capture_path, {})
    local_samples = _read_jsonl(local_samples_path)
    goal_events = _read_jsonl(goal_events_path)

    evidence_gaps: list[str] = []
    if not phase85_analysis_path or not phase85_analysis:
        evidence_gaps.append('missing_phase85_analysis_json')
    if not raw_capture_path or not raw_capture:
        evidence_gaps.append('missing_phase85_raw_capture_json')
    if not local_samples_path or not local_samples:
        evidence_gaps.append('missing_phase85_local_costmap_samples_jsonl')
    if not goal_events_path or not goal_events:
        evidence_gaps.append('missing_phase85_goal_events_jsonl')

    context, nested = _phase85_context(phase85_analysis if isinstance(phase85_analysis, dict) else {})
    forward_check = nested.get('forward_executability_check') or context.get('forward_executability_check') or {}
    forward_check = forward_check if isinstance(forward_check, dict) else {}
    raw_candidates = nested.get('candidates')
    candidates = [c for c in raw_candidates if isinstance(c, dict)] if isinstance(raw_candidates, list) else []
    raw_original_metrics = nested.get('original_metrics')
    original_metrics = raw_original_metrics if isinstance(raw_original_metrics, dict) else {}
    phase85_classification = phase85_analysis.get('classification') if isinstance(phase85_analysis, dict) else None
    validation_goal_sequence = phase85_analysis.get('validation_goal_sequence') if isinstance(phase85_analysis, dict) else None
    reject_reason = nested.get('refinement_reject_reason') or context.get('refinement_reject_reason')

    if phase85_analysis and not context:
        evidence_gaps.append('missing_goal2_dispatch_context')
    if context and not nested:
        evidence_gaps.append('missing_centerline_target_refinement_payload')
    if context and reject_reason != 'lethal_cost_regression':
        evidence_gaps.append('source_reject_reason_not_lethal_cost_regression')
    if context and context.get('refinement_applied') is not False:
        evidence_gaps.append('source_refinement_was_not_rejected')
    if nested and not candidates:
        evidence_gaps.append('missing_candidate_diagnostics')
    for key in REJECT_SUBITEMS:
        if key not in forward_check:
            evidence_gaps.append(f'missing_forward_check_subitem:{key}')

    aggregate = _candidate_aggregate(candidates, original_metrics)
    subitems = _reject_subitems(forward_check, candidates)
    local_summary = _local_sample_summary(local_samples, validation_goal_sequence if isinstance(validation_goal_sequence, int) else None)
    raw_summary = _raw_capture_summary(raw_capture if isinstance(raw_capture, dict) else {})

    classification, reasons, secondary = _classification(
        evidence_gaps=evidence_gaps,
        phase85_classification=phase85_classification,
        reject_reason=reject_reason,
        aggregate=aggregate,
        forward_check=forward_check,
    )

    result = {
        'run_id': RUN_ID,
        'source_phase85_artifact_dir': str(artifact_dir),
        'source_phase85_classification': phase85_classification,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'classification_reasons': reasons,
        'secondary_supported_classifications': secondary,
        'evidence_gaps': evidence_gaps,
        'lethal_reject_context': {
            'refinement_applied': context.get('refinement_applied'),
            'refinement_reject_reason': reject_reason,
            'forward_executability_reason': forward_check.get('reason'),
            'validation_goal_sequence': validation_goal_sequence,
            'used_target_match_fallback': phase85_analysis.get('used_target_match_fallback') if isinstance(phase85_analysis, dict) else None,
            'candidate_count': nested.get('candidate_count'),
            'two_side_wall_candidate_count': nested.get('two_side_wall_candidate_count'),
            'strict_eligible_candidate_count': nested.get('strict_eligible_candidate_count'),
            'eligible_candidate_count': nested.get('eligible_candidate_count'),
            'balance_first_eligible_candidate_count': nested.get('balance_first_eligible_candidate_count'),
            'original_target': nested.get('original_target') or context.get('original_target'),
            'centerline_projected_target': nested.get('centerline_projected_target') or context.get('centerline_projected_target'),
            'nav2_goal_target': context.get('target'),
            'corridor_heading_yaw': nested.get('corridor_heading_yaw') or context.get('corridor_heading_yaw'),
        },
        'reject_subitems': subitems,
        'candidate_aggregate': aggregate,
        'forward_executability_check': forward_check,
        'original_metrics': original_metrics,
        'local_costmap_sample_summary': local_summary,
        'raw_capture_summary': raw_summary,
        'guardrails': dict(GUARDRAILS),
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'phase87_entered': False,
        'source_files': {
            'phase85_analysis': str(phase85_analysis_path) if phase85_analysis_path else None,
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'local_costmap_samples': str(local_samples_path) if local_samples_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
        },
    }
    return result


def write_minimal_summary(result: dict[str, Any], path: Path) -> None:
    ctx = result.get('lethal_reject_context') or {}
    agg = result.get('candidate_aggregate') or {}
    lines = [
        '# Phase86 minimal summary',
        '',
        f"Status: {result.get('classification')}",
        f"Source Phase85 classification: {result.get('source_phase85_classification')}",
        f"Reject reason: {ctx.get('refinement_reject_reason')}",
        f"Forward executability reason: {ctx.get('forward_executability_reason')}",
        f"Validation goal sequence: {ctx.get('validation_goal_sequence')}",
        '',
        'Candidate evidence:',
        f"- candidate_count: {agg.get('candidate_count')}",
        f"- hard_safety_pass_candidate_count: {agg.get('hard_safety_pass_candidate_count')}",
        f"- high_or_lethal_candidate_count: {agg.get('high_or_lethal_candidate_count')}",
        f"- best_balance_candidate_count: {agg.get('best_balance_candidate_count')}",
        f"- best_balance_candidates_all_clearance_failed: {agg.get('best_balance_candidates_all_clearance_failed')}",
        f"- original_local_cost_max_radius: {agg.get('original_local_cost_max_radius')}",
        f"- original_front_wedge_lethal_count: {agg.get('original_front_wedge_lethal_count')}",
        f"- original_footprint_lethal_count: {agg.get('original_footprint_lethal_count')}",
        f"- evidence_gaps: {result.get('evidence_gaps')}",
        f"- classification_reasons: {result.get('classification_reasons')}",
        f"- secondary_supported_classifications: {result.get('secondary_supported_classifications')}",
        '',
        'Guardrails: No maze_explorer strategy changed; No branch scoring changed; No centerline gate changed; No directional readiness override changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; No inflation/robot_radius/clearance_radius_m/map threshold tuning; No autonomous exploration success claimed; No exit success claimed; Phase87 not entered.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase85-artifact-dir', type=Path, default=Path('log') / PHASE85_RUN_ID)
    parser.add_argument('--output-json', type=Path, default=Path('log') / RUN_ID / f'{RUN_ID}.json')
    parser.add_argument('--minimal-summary', type=Path, default=Path('log') / RUN_ID / f'{RUN_ID}_minimal_summary.md')
    args = parser.parse_args()

    result = analyze_phase85_artifacts(args.phase85_artifact_dir)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_minimal_summary(result, args.minimal_summary)
    print(json.dumps({
        'classification': result['classification'],
        'evidence_gaps': result['evidence_gaps'],
        'output_json': str(args.output_json),
        'minimal_summary': str(args.minimal_summary),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
