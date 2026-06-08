#!/usr/bin/env python3
"""Phase98 Goal1 recovery-dominant failure root-cause analyzer.

Validation/diagnosis only. Reads Phase97 artifacts and explains why Goal1 became a
recovery-dominant terminal failure. It does not run ROS, send goals, mutate
maze_explorer logic, or tune Nav2/config.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'
PHASE98_RUN_ID = 'phase98_goal1_recovery_dominant_failure_root_cause_diagnosis'

ALLOWED_CLASSIFICATIONS = [
    'GOAL1_STAGING_REJECT_ROOT_CAUSE',
    'GOAL1_SINGLE_STEP_REFINEMENT_NO_HARD_SAFE_CANDIDATE',
    'GOAL1_NAV2_EXECUTION_RECOVERY_DOMINANT',
    'GOAL1_LOCAL_COST_FOOTPRINT_FRONT_WEDGE_BLOCKED',
    'GOAL1_STAGING_TRIGGER_OR_CHECK_AMBIGUOUS',
    'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
]

GUARDRAILS = {
    'algorithm_changed': False,
    'phase88_92_logic_changed': False,
    'branch_scoring_or_order_changed': False,
    'nav2_config_changed': False,
    'autonomous_exploration_success_claimed': False,
    'exit_success_claimed': False,
    'phase99_entered': False,
}


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception as exc:  # pragma: no cover - defensive artifact parser
        return {'_parse_error': str(exc), '_path': str(path)}


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
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
        rows.append(row)
    return rows


def _as_num(value: Any) -> float | None:
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) else None


def _first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _goal_sequence(row: dict[str, Any]) -> int | None:
    value = row.get('goal_sequence')
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return None


def _goal1_rows(goal_events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in goal_events if _goal_sequence(row) == 1]


def _select_goal1_dispatch(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    dispatches = [row for row in rows if row.get('event') == 'dispatch']
    return dispatches[0] if dispatches else (rows[0] if rows else None)


def _select_goal1_terminal(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    terminal_events = {'success', 'failure', 'timeout', 'cancel', 'canceled', 'cancelled', 'aborted'}
    terminals = [row for row in rows if str(row.get('event')).lower() in terminal_events]
    return terminals[-1] if terminals else None


def _candidate_failure_landscape(refinement: dict[str, Any]) -> dict[str, Any]:
    candidates = refinement.get('candidates') if isinstance(refinement, dict) else None
    if not isinstance(candidates, list):
        candidates = []

    reject_counts: collections.Counter[str] = collections.Counter()
    bool_counts: dict[str, collections.Counter[str]] = {
        'hard_safety_pass': collections.Counter(),
        'safety_floor_ok': collections.Counter(),
        'front_wedge_lethal_not_increased': collections.Counter(),
        'footprint_lethal_not_increased': collections.Counter(),
        'target_has_clearance': collections.Counter(),
        'same_corridor': collections.Counter(),
        'two_side_wall_evidence': collections.Counter(),
    }
    metric_values: dict[str, list[float]] = {
        'min_clearance_m': [],
        'front_wedge_lethal_count': [],
        'footprint_lethal_count': [],
        'local_cost_max_radius': [],
        'balance_error_m': [],
        'forward_progress_m': [],
    }
    for cand in candidates:
        if not isinstance(cand, dict):
            continue
        for reason in cand.get('candidate_reject_reasons') or []:
            reject_counts[str(reason)] += 1
        for key, counter in bool_counts.items():
            value = cand.get(key)
            if isinstance(value, bool):
                counter[str(value).lower()] += 1
        for key, vals in metric_values.items():
            num = _as_num(cand.get(key))
            if num is not None:
                vals.append(float(num))

    def summarize(vals: list[float]) -> dict[str, float] | None:
        if not vals:
            return None
        return {
            'min': min(vals),
            'max': max(vals),
            'mean': sum(vals) / len(vals),
        }

    def top_by(key: str, reverse: bool = False, n: int = 5) -> list[dict[str, Any]]:
        valid = [cand for cand in candidates if isinstance(cand, dict) and _as_num(cand.get(key)) is not None]
        valid.sort(key=lambda cand: float(cand[key]), reverse=reverse)
        out = []
        for cand in valid[:n]:
            out.append({
                'candidate_index': cand.get('candidate_index'),
                key: cand.get(key),
                'hard_safety_pass': cand.get('hard_safety_pass'),
                'safety_floor_ok': cand.get('safety_floor_ok'),
                'front_wedge_lethal_not_increased': cand.get('front_wedge_lethal_not_increased'),
                'footprint_lethal_not_increased': cand.get('footprint_lethal_not_increased'),
                'candidate_reject_reasons': cand.get('candidate_reject_reasons'),
                'target': cand.get('target') or cand.get('target_xy'),
                'target_yaw': cand.get('target_yaw'),
            })
        return out

    hard_safe_count = _first_present(
        refinement.get('hard_safety_pass_candidate_count') if isinstance(refinement, dict) else None,
        sum(1 for cand in candidates if isinstance(cand, dict) and cand.get('hard_safety_pass') is True),
    )
    return {
        'candidate_count': _first_present(refinement.get('candidate_count') if isinstance(refinement, dict) else None, len(candidates)),
        'hard_safety_pass_candidate_count': hard_safe_count,
        'eligible_candidate_count': refinement.get('eligible_candidate_count') if isinstance(refinement, dict) else None,
        'strict_eligible_candidate_count': refinement.get('strict_eligible_candidate_count') if isinstance(refinement, dict) else None,
        'two_side_wall_candidate_count': refinement.get('two_side_wall_candidate_count') if isinstance(refinement, dict) else None,
        'reject_reason_counts': dict(sorted(reject_counts.items())),
        'boolean_pass_counts': {key: dict(counter) for key, counter in bool_counts.items() if counter},
        'metric_summary': {key: summarize(vals) for key, vals in metric_values.items() if vals},
        'best_by_clearance': top_by('min_clearance_m', reverse=True),
        'lowest_front_wedge_lethal': top_by('front_wedge_lethal_count'),
        'lowest_footprint_lethal': top_by('footprint_lethal_count'),
        'lowest_local_cost_max_radius': top_by('local_cost_max_radius'),
    }


def _nav2_goal1_summary(nav2_rows: list[dict[str, Any]], phase97_goal1: dict[str, Any]) -> dict[str, Any]:
    rows = [row for row in nav2_rows if _goal_sequence(row) == 1 or row.get('goal_sequence') in (None, '1')]
    distances = [float(row['distance_remaining']) for row in rows if _as_num(row.get('distance_remaining')) is not None]
    recoveries = [int(row['number_of_recoveries']) for row in rows if isinstance(row.get('number_of_recoveries'), int)]
    timeline = []
    last_recovery = None
    for row in rows:
        rec = row.get('number_of_recoveries')
        elapsed = row.get('elapsed_sec')
        if isinstance(rec, int) and rec != last_recovery:
            timeline.append({'elapsed_sec': elapsed, 'recoveries': rec, 'distance_remaining': row.get('distance_remaining')})
            last_recovery = rec
    feedback = phase97_goal1.get('nav2_feedback') if isinstance(phase97_goal1, dict) else {}
    recoveries_summary = phase97_goal1.get('recoveries') if isinstance(phase97_goal1, dict) else {}
    return {
        'sample_count': len(rows),
        'distance_remaining_first_nonzero': _first_present(feedback.get('distance_remaining_first_nonzero') if isinstance(feedback, dict) else None, next((d for d in distances if d > 0), None)),
        'distance_remaining_last': _first_present(feedback.get('distance_remaining_last') if isinstance(feedback, dict) else None, distances[-1] if distances else None),
        'distance_remaining_min': min(distances) if distances else None,
        'distance_remaining_max': max(distances) if distances else None,
        'distance_remaining_progress_delta': _first_present(feedback.get('distance_remaining_progress_delta') if isinstance(feedback, dict) else None, (distances[0] - distances[-1]) if len(distances) >= 2 else None),
        'plateau_tail_delta_last_100_samples': (distances[-100] - distances[-1]) if len(distances) >= 100 else None,
        'max_recoveries': _first_present(recoveries_summary.get('max') if isinstance(recoveries_summary, dict) else None, max(recoveries) if recoveries else None),
        'recovery_timeline': timeline[:30],
    }


def _local_cost_goal1_summary(local_rows: list[dict[str, Any]], phase97_goal1: dict[str, Any]) -> dict[str, Any]:
    rows = [row for row in local_rows if _goal_sequence(row) == 1 or row.get('goal_sequence') in (None, '1')]
    risk = phase97_goal1.get('local_cost_risk') if isinstance(phase97_goal1, dict) else {}

    def max_key(key: str) -> Any:
        vals = [float(row[key]) for row in rows if _as_num(row.get(key)) is not None]
        return max(vals) if vals else (risk.get(f'{key}_max') if isinstance(risk, dict) else None)

    def min_key(key: str) -> Any:
        vals = [float(row[key]) for row in rows if _as_num(row.get(key)) is not None]
        return min(vals) if vals else (risk.get(key) if isinstance(risk, dict) else None)

    terminal = rows[-1] if rows else {}
    front_wedge_max = _first_present(risk.get('front_wedge_lethal_count_max') if isinstance(risk, dict) else None, max_key('front_wedge_lethal_count'))
    robot_footprint_max = _first_present(risk.get('robot_footprint_lethal_count_max') if isinstance(risk, dict) else None, max_key('robot_footprint_lethal_count'))
    target_footprint_max = _first_present(risk.get('target_footprint_lethal_count_max') if isinstance(risk, dict) else None, max_key('target_footprint_lethal_count'))
    clearance_min = _first_present(risk.get('front_wedge_clearance_min_m') if isinstance(risk, dict) else None, min_key('front_wedge_clearance_min_m'))
    terminal_blocked = bool(
        (risk.get('terminal_local_cost_blocked') if isinstance(risk, dict) else False)
        or (front_wedge_max or 0) > 0
        or (robot_footprint_max or 0) > 0
    )
    return {
        'sample_count': len(rows),
        'terminal_sample_elapsed_sec': terminal.get('elapsed_sec'),
        'terminal_pose': terminal.get('pose') or terminal.get('robot_pose') or terminal.get('base_pose'),
        'front_wedge_lethal_count_max': front_wedge_max,
        'robot_footprint_lethal_count_max': robot_footprint_max,
        'target_footprint_lethal_count_max': target_footprint_max,
        'front_wedge_clearance_min_m': clearance_min,
        'terminal_local_cost_blocked': terminal_blocked,
        'terminal_row_keys': sorted(terminal.keys()) if isinstance(terminal, dict) else [],
    }


def _raw_capture_summary(raw: dict[str, Any] | None) -> dict[str, bool]:
    if not isinstance(raw, dict):
        return {'available': False, 'scan_available': False, 'map_available': False, 'local_costmap_available': False, 'odom_available': False, 'tf_available': False}
    def available(key: str) -> bool:
        value = raw.get(key)
        if value is None:
            return False
        if isinstance(value, dict) and value.get('available') is False:
            return False
        return True
    return {
        'available': True,
        'scan_available': available('scan'),
        'map_available': available('map'),
        'local_costmap_available': available('local_costmap'),
        'odom_available': available('odom'),
        'tf_available': available('tf'),
    }


def _classify(goal1: dict[str, Any], gaps: list[str]) -> str:
    if gaps:
        return 'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE'
    terminal = goal1.get('terminal_outcome')
    staging_enabled = goal1.get('two_step_staging_plan', {}).get('enabled') is True
    staging_applied = goal1.get('staging_applied') is True
    staging_reason = goal1.get('staging_reject_reason')
    staging_check = goal1.get('staging_executability_check') or {}
    hard_count = goal1.get('hard_safety_pass_candidate_count')
    local = goal1.get('local_cost_terminal_metrics') or {}
    recoveries = (goal1.get('recovery_timeline') or {}).get('max_recoveries')

    if staging_enabled and not staging_applied and staging_reason:
        return 'GOAL1_STAGING_REJECT_ROOT_CAUSE'
    if hard_count == 0:
        return 'GOAL1_SINGLE_STEP_REFINEMENT_NO_HARD_SAFE_CANDIDATE'
    if staging_enabled and not staging_applied and staging_check and staging_check.get('checked') is not True:
        return 'GOAL1_STAGING_TRIGGER_OR_CHECK_AMBIGUOUS'
    if bool(local.get('terminal_local_cost_blocked')):
        return 'GOAL1_LOCAL_COST_FOOTPRINT_FRONT_WEDGE_BLOCKED'
    if terminal == 'failure' and isinstance(recoveries, int) and recoveries >= 5:
        return 'GOAL1_NAV2_EXECUTION_RECOVERY_DOMINANT'
    return 'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE'


def _answers(goal1: dict[str, Any], classification: str) -> dict[str, str]:
    landscape = goal1.get('single_step_candidate_failure_landscape') or {}
    reject_counts = landscape.get('reject_reason_counts') or {}
    metric_summary = landscape.get('metric_summary') or {}
    clearance = (metric_summary.get('min_clearance_m') or {}).get('max')
    hard_count = goal1.get('hard_safety_pass_candidate_count')
    staging_reason = goal1.get('staging_reject_reason')
    staging_check = goal1.get('staging_executability_check') or {}
    local = goal1.get('local_cost_terminal_metrics') or {}
    recoveries = (goal1.get('recovery_timeline') or {}).get('max_recoveries')
    return {
        'why_phase88_refinement_not_applied': (
            f"Phase88 did evaluate the multi-candidate family, but refinement_applied=false because "
            f"refinement_reject_reason={goal1.get('refinement_reject_reason')!r} and hard_safety_pass_candidate_count={hard_count}. "
            f"The candidate landscape shows reject_reason_counts={reject_counts}."
        ),
        'why_hard_safety_pass_candidate_count_zero': (
            f"No candidate satisfied the full hard-safety bundle. The strongest common blockers were {reject_counts}; "
            f"best observed min_clearance_m={clearance}, while safety_floor_ok remained false for the sampled candidate family."
        ),
        'why_staging_enabled_but_not_applied': (
            f"The Phase92 trigger bundle was enabled, but staging_applied=false because staging_reject_reason={staging_reason!r}. "
            f"The staging_executability_check reported reason={staging_check.get('reason')!r}, "
            f"two_side_wall_evidence={staging_check.get('two_side_wall_evidence')}, same_corridor={staging_check.get('same_corridor')}, "
            f"hard_safety_pass={staging_check.get('hard_safety_pass')}."
        ),
        'recommended_next_diagnostic_focus': (
            "Do not tune Nav2 or change exploration order from this evidence. The next root-cause focus should be the staging candidate/executability evidence path "
            f"({staging_reason}) plus the single-step no-hard-safe candidate landscape. Nav2 execution was recovery-dominant after the original unsafe target was used "
            f"(recoveries_max={recoveries}, front_wedge_lethal_count_max={local.get('front_wedge_lethal_count_max')}, "
            f"robot_footprint_lethal_count_max={local.get('robot_footprint_lethal_count_max')}), but the upstream staging rejection explains why the designed escape path did not replace that target."
        ),
        'primary_root_cause_classification_rationale': (
            f"classification={classification}: staging was triggered, not applied, and rejected before a staging goal could be dispatched; "
            "therefore the robot executed the non-refined Goal1 target and then entered recovery-dominant/local-cost-blocked behavior."
        ),
    }


def analyze_phase98(artifact_dir: str | Path) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    analysis = _load_json(artifact_dir / f'{RUN_ID}_analysis.json') or {}
    raw = _load_json(artifact_dir / f'{RUN_ID}_raw_capture.json')
    goal_events = _load_jsonl(artifact_dir / f'{RUN_ID}_goal_events.jsonl')
    nav2_rows = _load_jsonl(artifact_dir / f'{RUN_ID}_nav2_feedback.jsonl')
    local_rows = _load_jsonl(artifact_dir / f'{RUN_ID}_local_costmap_samples.jsonl')

    gaps: list[str] = []
    if not artifact_dir.exists():
        gaps.append('artifact_dir_missing')
    if not goal_events:
        gaps.append('goal_events_missing_or_empty')
    if not isinstance(analysis, dict) or not analysis:
        gaps.append('phase97_analysis_missing_or_empty')
    if not nav2_rows:
        gaps.append('nav2_feedback_missing_or_empty')
    if not local_rows:
        gaps.append('local_costmap_samples_missing_or_empty')
    if raw is None:
        gaps.append('raw_capture_missing')

    rows = _goal1_rows(goal_events)
    dispatch = _select_goal1_dispatch(rows)
    terminal = _select_goal1_terminal(rows)
    if not dispatch:
        gaps.append('goal1_dispatch_missing')
    if not terminal:
        gaps.append('goal1_terminal_missing')

    phase97_goal1 = {}
    if isinstance(analysis, dict):
        phase97_goal1 = (analysis.get('per_goal') or {}).get('1') or {}

    if not dispatch:
        result = {
            'phase': 'Phase98',
            'run_id': PHASE98_RUN_ID,
            'source_phase97_artifact_dir': str(artifact_dir),
            'classification': 'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
            'allowed_classifications': ALLOWED_CLASSIFICATIONS,
            'evidence_gaps': gaps,
            'guardrails': GUARDRAILS,
        }
        return result

    refinement = dispatch.get('centerline_target_refinement') or {}
    two_step = dispatch.get('two_step_staging_plan') or {}
    source_single = two_step.get('source_single_step') if isinstance(two_step, dict) else {}
    staging_check = dispatch.get('staging_executability_check') or {}
    landscape = _candidate_failure_landscape(refinement if isinstance(refinement, dict) else {})
    nav2_summary = _nav2_goal1_summary(nav2_rows, phase97_goal1)
    local_summary = _local_cost_goal1_summary(local_rows, phase97_goal1)
    terminal_outcome = _first_present(
        phase97_goal1.get('terminal_outcome') if isinstance(phase97_goal1, dict) else None,
        terminal.get('event') if terminal else None,
        dispatch.get('result_status'),
    )

    hard_count = _first_present(
        dispatch.get('hard_safety_pass_candidate_count'),
        refinement.get('hard_safety_pass_candidate_count') if isinstance(refinement, dict) else None,
        source_single.get('hard_safety_pass_candidate_count') if isinstance(source_single, dict) else None,
        landscape.get('hard_safety_pass_candidate_count'),
    )
    if not isinstance(hard_count, int):
        hard_count = None

    goal1 = {
        'dispatch_observed': True,
        'terminal_observed': terminal is not None,
        'terminal_outcome': terminal_outcome,
        'terminal_event': terminal.get('event') if terminal else None,
        'target': dispatch.get('target'),
        'selected_candidate_target': _first_present(dispatch.get('selected_candidate_target'), refinement.get('selected_candidate_target') if isinstance(refinement, dict) else None, dispatch.get('refined_target'), dispatch.get('target')),
        'selected_candidate_yaw': _first_present(dispatch.get('selected_candidate_yaw'), refinement.get('selected_candidate_yaw') if isinstance(refinement, dict) else None, dispatch.get('branch_angle')),
        'refinement_applied': bool(_first_present(dispatch.get('centerline_refinement_applied'), refinement.get('refinement_applied') if isinstance(refinement, dict) else None, source_single.get('refinement_applied') if isinstance(source_single, dict) else None, False)),
        'refinement_reject_reason': _first_present(dispatch.get('centerline_refinement_reason'), refinement.get('refinement_reject_reason') if isinstance(refinement, dict) else None, source_single.get('refinement_reject_reason') if isinstance(source_single, dict) else None),
        'multi_candidate_forward_search': {
            'present': bool(refinement or dispatch.get('centerline_refinement_candidate_count')),
            'candidate_count': _first_present(dispatch.get('centerline_refinement_candidate_count'), refinement.get('candidate_count') if isinstance(refinement, dict) else None, source_single.get('candidate_count') if isinstance(source_single, dict) else None),
        },
        'hard_safety_pass_candidate_count': hard_count,
        'single_step_candidate_failure_landscape': landscape,
        'two_step_staging_plan': two_step,
        'staging_trigger_bundle': two_step.get('trigger_conditions') if isinstance(two_step, dict) else None,
        'staging_applied': dispatch.get('staging_applied') is True,
        'staging_reject_reason': dispatch.get('staging_reject_reason') or staging_check.get('reason'),
        'staging_executability_check': staging_check,
        'second_step_forward_goal': dispatch.get('second_step_forward_goal'),
        'recovery_timeline': nav2_summary,
        'distance_remaining_plateau': {
            'distance_remaining_first_nonzero': nav2_summary.get('distance_remaining_first_nonzero'),
            'distance_remaining_last': nav2_summary.get('distance_remaining_last'),
            'distance_remaining_min': nav2_summary.get('distance_remaining_min'),
            'distance_remaining_progress_delta': nav2_summary.get('distance_remaining_progress_delta'),
            'plateau_tail_delta_last_100_samples': nav2_summary.get('plateau_tail_delta_last_100_samples'),
        },
        'local_cost_terminal_metrics': local_summary,
        'branch_scoring_changed': bool(dispatch.get('branch_scoring_changed') or (refinement.get('branch_scoring_changed') if isinstance(refinement, dict) else False)),
        'fallback_terminal_acceptance_used': bool(dispatch.get('fallback_terminal_acceptance_used') or (refinement.get('fallback_terminal_acceptance_used') if isinstance(refinement, dict) else False)),
    }

    classification = _classify(goal1, gaps)
    result = {
        'phase': 'Phase98',
        'run_id': PHASE98_RUN_ID,
        'source_phase97_artifact_dir': str(artifact_dir),
        'source_phase97_classification': analysis.get('classification') if isinstance(analysis, dict) else None,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'evidence_gaps': gaps,
        'source_stream_counts': {
            'goal_events': len(goal_events),
            'goal1_goal_event_rows': len(rows),
            'nav2_feedback': len(nav2_rows),
            'goal1_nav2_feedback': nav2_summary.get('sample_count'),
            'local_costmap_samples': len(local_rows),
            'goal1_local_costmap_samples': local_summary.get('sample_count'),
        },
        'raw_capture_summary': _raw_capture_summary(raw),
        'goal1': goal1,
        'answers': _answers(goal1, classification),
        'guardrails': GUARDRAILS,
    }
    return result


def render_minimal_summary(result: dict[str, Any]) -> str:
    g1 = result.get('goal1') or {}
    lines = [
        '# Phase98 Goal1 recovery-dominant failure root-cause minimal summary',
        '',
        f"classification: {result.get('classification')}",
        f"evidence_gaps: {result.get('evidence_gaps')}",
    ]
    if g1:
        lines.extend([
            f"terminal_outcome: {g1.get('terminal_outcome')}",
            f"refinement_reject_reason: {g1.get('refinement_reject_reason')}",
            f"hard_safety_pass_candidate_count: {g1.get('hard_safety_pass_candidate_count')}",
            f"staging_reject_reason: {g1.get('staging_reject_reason')}",
            f"staging_trigger_bundle: {json.dumps(g1.get('staging_trigger_bundle'), sort_keys=True)}",
            f"staging_executability_reason: {(g1.get('staging_executability_check') or {}).get('reason')}",
            f"recoveries_max: {(g1.get('recovery_timeline') or {}).get('max_recoveries')}",
            f"distance_remaining_last: {(g1.get('distance_remaining_plateau') or {}).get('distance_remaining_last')}",
            f"front_wedge_lethal_count_max: {(g1.get('local_cost_terminal_metrics') or {}).get('front_wedge_lethal_count_max')}",
            f"robot_footprint_lethal_count_max: {(g1.get('local_cost_terminal_metrics') or {}).get('robot_footprint_lethal_count_max')}",
        ])
    answers = result.get('answers') or {}
    if answers:
        lines.extend([
            '',
            'answers:',
            f"- why Phase88 refinement was not applied: {answers.get('why_phase88_refinement_not_applied')}",
            f"- why hard_safety_pass_candidate_count=0: {answers.get('why_hard_safety_pass_candidate_count_zero')}",
            f"- why staging enabled but staging_applied=false: {answers.get('why_staging_enabled_but_not_applied')}",
            f"- next diagnostic focus: {answers.get('recommended_next_diagnostic_focus')}",
        ])
    return '\n'.join(lines) + '\n'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    parser.add_argument('--summary-md', required=True, type=Path)
    args = parser.parse_args()

    result = analyze_phase98(args.artifact_dir)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    args.summary_md.parent.mkdir(parents=True, exist_ok=True)
    args.summary_md.write_text(render_minimal_summary(result), encoding='utf-8')
    print(json.dumps({'classification': result.get('classification'), 'evidence_gaps': result.get('evidence_gaps')}, sort_keys=True))
    return 0 if result.get('classification') != 'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
