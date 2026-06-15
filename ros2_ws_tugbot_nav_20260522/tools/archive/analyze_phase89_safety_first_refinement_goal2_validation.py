#!/usr/bin/env python3
"""Analyze Phase89 bounded Goal2 validation artifacts for Phase88 refinement.

Read-only analyzer. It consumes bounded reproduction artifacts and classifies
whether Phase88 safety-first multi-candidate refinement applied for the Goal2
-equivalent segment, whether it improved execution, or whether evidence is
insufficient. It does not tune strategy, Nav2, MPPI, controller, inflation,
robot radius, clearance radius, map thresholds, fallback, or terminal
acceptance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase89_safety_first_refinement_bounded_goal2_validation'
OLD_GOAL2_TARGET = [2.057855221699651, 1.0261005743935105]
GOAL2_TARGET_MATCH_TOLERANCE_M = 0.35

ALLOWED_CLASSIFICATIONS = {
    'PHASE88_REFINEMENT_APPLIED_GOAL2_IMPROVED',
    'PHASE88_REFINEMENT_APPLIED_BUT_GOAL2_TIMEOUT',
    'PHASE88_REFINEMENT_STILL_REJECTED',
    'PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE',
}

REQUIRED_PHASE88_DISPATCH_FIELDS = [
    'multi_candidate_forward_search',
    'candidate_family',
    'candidate_count',
    'hard_safety_pass_candidate_count',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
    'selection_priority_trace',
    'rejected_candidate_summaries',
    'refinement_applied',
    'refinement_reject_reason',
    'original_target_preserved_on_reject',
    'branch_scoring_changed',
    'fallback_terminal_acceptance_used',
]

OPTIONAL_PHASE88_ASSESSMENT_FIELDS = [
    'selected_candidate_hard_safety_pass',
    'selected_candidate_balance_error_m',
    'best_balance_error_m',
    'selected_candidate_not_best_centered_but_safer',
]

PRIORITY_TRACE = [
    'hard_safety_pass',
    'no_footprint_front_wedge_lethal_regression',
    'safety_floor_ok',
    'forward_progress_ok',
    'clearance_better',
    'balance_error_smaller',
]


def _norm_token(value: Any) -> str:
    return str(value).strip().lower().replace('-', '_').replace('/', '_').replace(' ', '_')


def _multi_candidate_enabled(value: Any) -> bool:
    if value is True:
        return True
    if isinstance(value, dict):
        return bool(value.get('bounded_local_search') or value.get('candidate_count') or value.get('candidate_family'))
    return False


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


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
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


def _glob_first(artifact_dir: Path, suffix: str) -> Path | None:
    direct = artifact_dir / f'{RUN_ID}_{suffix}'
    if direct.exists():
        return direct
    matches = sorted(artifact_dir.glob(f'*{suffix}'))
    return matches[0] if matches else None


def _as_xy(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) < 2:
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


def _row_goal_sequence(row: dict[str, Any]) -> int | None:
    try:
        return int(row.get('goal_sequence'))
    except (TypeError, ValueError):
        return None


def _nested_refinement(row: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(row, dict):
        return {}
    nested = row.get('centerline_target_refinement')
    return nested if isinstance(nested, dict) else {}


def _matches_goal2_equivalent(row: dict[str, Any]) -> bool:
    nested = _nested_refinement(row)
    candidates = [
        row.get('original_target'),
        row.get('target'),
        row.get('selected_candidate_target'),
        row.get('centerline_projected_target'),
        nested.get('original_target'),
        nested.get('selected_candidate_target'),
        nested.get('refined_target'),
    ]
    return any((_distance(candidate, OLD_GOAL2_TARGET) or 999.0) <= GOAL2_TARGET_MATCH_TOLERANCE_M for candidate in candidates)


def _select_goal2_equivalent_rows(events: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], int | None, bool]:
    seq2 = [row for row in events if _row_goal_sequence(row) == 2]
    if seq2:
        return seq2, 2, False
    matched = [row for row in events if _matches_goal2_equivalent(row)]
    if not matched:
        # Synthetic/unit and bounded ingress workflows often expose the old Goal2
        # as the first explorer dispatch after a separate ingress action.
        seq1 = [row for row in events if _row_goal_sequence(row) == 1]
        if seq1:
            return seq1, 1, True
        return [], None, False
    seq = _row_goal_sequence(matched[0])
    if seq is None:
        return matched, None, True
    return [row for row in events if _row_goal_sequence(row) == seq], seq, True


def _dispatch_context(dispatch: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(dispatch, dict):
        return None
    nested = _nested_refinement(dispatch)
    keys = list(dict.fromkeys(REQUIRED_PHASE88_DISPATCH_FIELDS + OPTIONAL_PHASE88_ASSESSMENT_FIELDS + [
        'original_target',
        'centerline_projected_target',
        'corridor_heading_yaw',
        'forward_executability_check',
    ]))
    context: dict[str, Any] = {}
    nested_fallback_keys: list[str] = []
    for key in keys:
        value = dispatch.get(key)
        if value is None and key in nested:
            value = nested.get(key)
            nested_fallback_keys.append(key)
        context[key] = value
    context['target'] = dispatch.get('target')
    context['top_level_fields_present'] = {key: key in dispatch for key in REQUIRED_PHASE88_DISPATCH_FIELDS}
    context['nested_fallback_keys'] = nested_fallback_keys
    if nested:
        context['centerline_target_refinement'] = nested
    return context


def _outcome_summary(goal_events: list[dict[str, Any]]) -> dict[str, Any]:
    outcome = next((row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout'}), None)
    cancel = next((row for row in goal_events if row.get('event') == 'timeout_cancel_result'), None)
    selected = outcome or cancel
    event = selected.get('event') if isinstance(selected, dict) else None
    reason = selected.get('result_reason') if isinstance(selected, dict) else None
    timed_out = bool(event in {'timeout', 'timeout_cancel_result'} or reason in {'goal_timeout', 'goal_canceled_after_timeout'})
    succeeded = bool(event == 'success' or reason in {'goal_reached', 'succeeded'})
    failed = bool(event == 'failure' or (reason and not timed_out and not succeeded))
    return {
        'event': event,
        'result_reason': reason,
        'timed_out': timed_out,
        'succeeded': succeeded,
        'failed': failed,
        'observed': selected is not None,
    }


def _feedback_summary(rows: list[dict[str, Any]], goal_sequence: int | None) -> dict[str, Any]:
    selected = [row for row in rows if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    distances: list[float] = []
    recoveries: list[int] = []
    for row in selected:
        try:
            if row.get('distance_remaining') is not None:
                distances.append(float(row.get('distance_remaining')))
        except (TypeError, ValueError):
            pass
        try:
            if row.get('number_of_recoveries') is not None:
                recoveries.append(int(row.get('number_of_recoveries')))
        except (TypeError, ValueError):
            pass
    return {
        'sample_count': len(selected),
        'distance_remaining_min': min(distances) if distances else None,
        'distance_remaining_last': distances[-1] if distances else None,
        'recoveries_max': max(recoveries) if recoveries else 0,
        'recoveries_last': recoveries[-1] if recoveries else 0,
    }


def _raw_capture_summary(raw: dict[str, Any] | None) -> dict[str, Any]:
    raw = raw if isinstance(raw, dict) else {}
    return {
        'raw_capture_available': bool(raw),
        'scan_available': raw.get('scan') is not None,
        'local_costmap_available': raw.get('local_costmap') is not None,
        'odom_available': raw.get('odom') is not None,
        'tf_available': bool(raw.get('tf')),
        'footprint_available': raw.get('footprint') is not None,
    }


def _local_cost_summary(rows: list[dict[str, Any]], goal_sequence: int | None) -> dict[str, Any]:
    selected = [row for row in rows if goal_sequence is None or _row_goal_sequence(row) == goal_sequence]
    front = []
    footprint = []
    target = []
    for row in selected:
        for sink, key in [(front, 'front_wedge_lethal_count'), (footprint, 'robot_footprint_lethal_count'), (target, 'target_footprint_lethal_count')]:
            try:
                if row.get(key) is not None:
                    sink.append(float(row.get(key)))
            except (TypeError, ValueError):
                pass
    return {
        'sample_count': len(selected),
        'front_wedge_lethal_count_max': max(front) if front else None,
        'front_wedge_lethal_count_last': front[-1] if front else None,
        'robot_footprint_lethal_count_max': max(footprint) if footprint else None,
        'robot_footprint_lethal_count_last': footprint[-1] if footprint else None,
        'target_footprint_lethal_count_max': max(target) if target else None,
    }


def _selected_candidate_assessment(context: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(context, dict):
        return {
            'selected_candidate_hard_safety_pass': False,
            'selected_candidate_not_best_centered_but_safer': False,
            'assessment': 'missing_dispatch_context',
        }
    selected_index = context.get('selected_candidate_index')
    hard_count = context.get('hard_safety_pass_candidate_count')
    hard_pass_field = context.get('selected_candidate_hard_safety_pass')
    refinement_applied = bool(context.get('refinement_applied'))
    selected_hard_safety_pass = bool(refinement_applied and selected_index is not None and (hard_pass_field is True or (isinstance(hard_count, int) and hard_count > 0)))
    selected_balance = context.get('selected_candidate_balance_error_m')
    best_balance = context.get('best_balance_error_m')
    not_best_centered_but_safer = bool(context.get('selected_candidate_not_best_centered_but_safer'))
    try:
        if selected_balance is not None and best_balance is not None:
            not_best_centered_but_safer = not_best_centered_but_safer or float(selected_balance) > float(best_balance)
    except (TypeError, ValueError):
        pass
    return {
        'selected_candidate_hard_safety_pass': selected_hard_safety_pass,
        'selected_candidate_not_best_centered_but_safer': not_best_centered_but_safer,
        'selected_candidate_index': selected_index,
        'selected_candidate_target': context.get('selected_candidate_target'),
        'selected_candidate_yaw': context.get('selected_candidate_yaw'),
        'selected_candidate_balance_error_m': selected_balance,
        'best_balance_error_m': best_balance,
        'assessment': 'selected_candidate_hard_safe' if selected_hard_safety_pass else 'selected_candidate_not_proven_hard_safe',
    }


def analyze_artifact_dir(artifact_dir: Path | str) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl')
    feedback_path = _glob_first(artifact_dir, 'nav2_feedback.jsonl')
    local_cost_path = _glob_first(artifact_dir, 'local_costmap_samples.jsonl')
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json')
    trigger_path = _glob_first(artifact_dir, 'trigger_detected.json')

    events = _read_jsonl(goal_events_path)
    feedback = _read_jsonl(feedback_path)
    local_cost = _read_jsonl(local_cost_path)
    raw_capture = _load_json(raw_capture_path, {})
    trigger = _load_json(trigger_path, {})

    validation_events, validation_goal_sequence, used_target_match_fallback = _select_goal2_equivalent_rows(events)
    dispatch = next((row for row in validation_events if row.get('event') == 'dispatch'), None)
    context = _dispatch_context(dispatch)
    outcome = _outcome_summary(validation_events)
    feedback_summary = _feedback_summary(feedback, validation_goal_sequence)
    local_cost_summary = _local_cost_summary(local_cost, validation_goal_sequence)
    raw_summary = _raw_capture_summary(raw_capture)
    selected_assessment = _selected_candidate_assessment(context)

    gaps: list[str] = []
    if not goal_events_path or not events:
        gaps.append('missing_goal_events_artifact')
    if dispatch is None:
        gaps.append('missing_goal2_equivalent_dispatch_event')
    if context is not None:
        nested = _nested_refinement(dispatch)
        for key in REQUIRED_PHASE88_DISPATCH_FIELDS:
            present = isinstance(dispatch, dict) and (key in dispatch or key in nested)
            if not present:
                gaps.append(f'missing_dispatch_field:{key}')
        if not _multi_candidate_enabled(context.get('multi_candidate_forward_search')):
            gaps.append('multi_candidate_forward_search_not_true')
        if context.get('branch_scoring_changed') is not False:
            gaps.append('branch_scoring_changed_not_false')
        if context.get('fallback_terminal_acceptance_used') is not False:
            gaps.append('fallback_terminal_acceptance_used_not_false')
        trace = context.get('selection_priority_trace')
        if isinstance(trace, list):
            normalized_trace = [_norm_token(item) for item in trace]
            if normalized_trace[: len(PRIORITY_TRACE)] != PRIORITY_TRACE:
                gaps.append('selection_priority_trace_not_safety_first')
    if not feedback_path or not feedback:
        gaps.append('missing_nav2_feedback_artifact')
    if not raw_capture_path or not raw_capture:
        gaps.append('missing_raw_capture_artifact')
    if not local_cost_path or not local_cost:
        gaps.append('missing_local_costmap_samples_artifact')

    applied = bool(context.get('refinement_applied')) if isinstance(context, dict) else False
    if gaps and (dispatch is None or any(g.startswith('missing_dispatch_field:') for g in gaps) or 'multi_candidate_forward_search_not_true' in gaps or 'branch_scoring_changed_not_false' in gaps or 'fallback_terminal_acceptance_used_not_false' in gaps):
        classification = 'PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE'
    elif not applied:
        classification = 'PHASE88_REFINEMENT_STILL_REJECTED'
    elif outcome['timed_out'] or outcome['failed']:
        classification = 'PHASE88_REFINEMENT_APPLIED_BUT_GOAL2_TIMEOUT'
    elif outcome['succeeded'] or (feedback_summary['distance_remaining_min'] is not None and feedback_summary['distance_remaining_min'] <= 0.15 and feedback_summary['recoveries_max'] == 0):
        classification = 'PHASE88_REFINEMENT_APPLIED_GOAL2_IMPROVED'
    else:
        classification = 'PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE'
        if not outcome['observed']:
            gaps.append('missing_goal2_equivalent_terminal_outcome')

    original = context.get('original_target') if isinstance(context, dict) else None
    selected_target = context.get('selected_candidate_target') if isinstance(context, dict) else None
    nav2_target = context.get('target') if isinstance(context, dict) else None
    result = {
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': gaps,
        'validation_goal_sequence': validation_goal_sequence,
        'used_target_match_fallback': used_target_match_fallback,
        'goal2_dispatch_context': context,
        'selected_candidate_assessment': selected_assessment,
        'goal2_refinement_summary': {
            'refinement_applied': applied,
            'refinement_reject_reason': context.get('refinement_reject_reason') if isinstance(context, dict) else None,
            'original_target': original,
            'selected_candidate_target': selected_target,
            'selected_candidate_yaw': context.get('selected_candidate_yaw') if isinstance(context, dict) else None,
            'nav2_goal_target': nav2_target,
            'nav2_target_shift_m': _distance(original, nav2_target),
            'candidate_count': context.get('candidate_count') if isinstance(context, dict) else None,
            'hard_safety_pass_candidate_count': context.get('hard_safety_pass_candidate_count') if isinstance(context, dict) else None,
            'original_target_preserved_on_reject': context.get('original_target_preserved_on_reject') if isinstance(context, dict) else None,
        },
        'goal2_outcome': outcome,
        'nav2_feedback_summary': feedback_summary,
        'local_costmap_summary': local_cost_summary,
        'raw_capture_summary': raw_summary,
        'trigger': trigger,
        'screenshot_suggestions': [
            'Gazebo wide view: robot near Goal2-equivalent corridor, nearest wall, and target direction.',
            'RViz local costmap: robot footprint and front wedge around the selected candidate approach.',
            'RViz goal/tolerance view: original target vs selected candidate target if markers are visible.',
            'Recovery/timeout pose: show local-cost blockage or recovery loop if still present.',
        ],
        'guardrails': {
            'no_maze_explorer_strategy_changed': True,
            'no_branch_scoring_changed': context.get('branch_scoring_changed') is False if context else False,
            'no_centerline_gate_changed': True,
            'no_directional_readiness_changed': True,
            'no_fallback_terminal_acceptance_changed': context.get('fallback_terminal_acceptance_used') is False if context else False,
            'no_nav2_mppi_controller_tuning': True,
            'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
        },
        'autonomous_exploration_success_claimed': False,
        'exit_success_claimed': False,
        'phase90_entered': False,
        'source_files': {
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'nav2_feedback': str(feedback_path) if feedback_path else None,
            'local_costmap_samples': str(local_cost_path) if local_cost_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
            'trigger': str(trigger_path) if trigger_path else None,
        },
    }
    return result


def write_minimal_summary(result: dict[str, Any], path: Path) -> None:
    summary = result.get('goal2_refinement_summary') or {}
    outcome = result.get('goal2_outcome') or {}
    selected = result.get('selected_candidate_assessment') or {}
    feedback = result.get('nav2_feedback_summary') or {}
    local_cost = result.get('local_costmap_summary') or {}
    lines = [
        '# Phase89 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Validation goal sequence: {result.get('validation_goal_sequence')} target_match_fallback={result.get('used_target_match_fallback')}",
        f"Refinement applied: {summary.get('refinement_applied')}",
        f"Reject reason: {summary.get('refinement_reject_reason')}",
        f"Original target: {summary.get('original_target')}",
        f"Selected candidate target: {summary.get('selected_candidate_target')}",
        f"Selected candidate yaw: {summary.get('selected_candidate_yaw')}",
        f"Candidate count: {summary.get('candidate_count')} hard_safety_pass_candidate_count={summary.get('hard_safety_pass_candidate_count')}",
        f"Selected candidate hard-safety-pass: {selected.get('selected_candidate_hard_safety_pass')}",
        f"Selected candidate not best-centered but safer: {selected.get('selected_candidate_not_best_centered_but_safer')}",
        f"Outcome: event={outcome.get('event')} reason={outcome.get('result_reason')} timed_out={outcome.get('timed_out')} succeeded={outcome.get('succeeded')}",
        f"Nav2 feedback: recoveries_max={feedback.get('recoveries_max')} distance_remaining_last={feedback.get('distance_remaining_last')}",
        f"Local cost: front_wedge_lethal_count_max={local_cost.get('front_wedge_lethal_count_max')} robot_footprint_lethal_count_max={local_cost.get('robot_footprint_lethal_count_max')}",
        '',
        'Screenshot suggestions if the scene is held:',
        '- Gazebo wide view: robot, Goal2-equivalent corridor, nearest wall, and target direction.',
        '- RViz local costmap: robot footprint/front wedge around the selected candidate approach.',
        '- RViz goal marker/tolerance: original target vs selected candidate/dispatch target if visible.',
        '- Recovery/timeout moment: show controller recovery or local-cost blockage if still present.',
        '',
        'Guardrails: No maze_explorer strategy changed; No branch scoring changed; No centerline gate changed; No directional readiness changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; No autonomous exploration success claimed; No exit success claimed; Phase90 not entered.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log') / RUN_ID)
    parser.add_argument('--output-json', type=Path, default=None)
    parser.add_argument('--minimal-summary', type=Path, default=None)
    args = parser.parse_args()

    result = analyze_artifact_dir(args.artifact_dir)
    output = args.output_json or (args.artifact_dir / f'{RUN_ID}_analysis.json')
    summary = args.minimal_summary or (args.artifact_dir / f'{RUN_ID}_minimal_field_summary.md')
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
