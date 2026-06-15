#!/usr/bin/env python3
"""Analyze Phase85 bounded Goal2 reproduction artifacts.

Phase85 validates whether the Phase84 corridor-aligned refinement changed the
old Goal2 near-goal lateral-residual target into a more Nav2-executable goal.
This analyzer is read-only: it consumes artifacts and emits a classification;
it does not tune Nav2, branch scoring, centerline gates, fallback, or terminal
acceptance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase85_goal2_corridor_aligned_refinement_bounded_validation'
ALLOWED_CLASSIFICATIONS = {
    'REFINEMENT_APPLIED_AND_GOAL2_IMPROVED',
    'REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT',
    'REFINEMENT_REJECTED_OR_NOT_TRIGGERED',
    'INSUFFICIENT_VALIDATION_EVIDENCE',
}
REQUIRED_DISPATCH_FIELDS = [
    'original_target',
    'centerline_projected_target',
    'corridor_heading_yaw',
    'refinement_applied',
    'refinement_reject_reason',
    'forward_executability_check',
    'branch_scoring_changed',
    'fallback_terminal_acceptance_used',
]
OLD_GOAL2_TARGET = [2.057855221699651, 1.0261005743935105]
GOAL2_TARGET_MATCH_TOLERANCE_M = 0.35


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


def _first_existing(artifact_dir: Path, names: list[str]) -> Path | None:
    for name in names:
        p = artifact_dir / name
        if p.exists():
            return p
    return None


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


def _goal2_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        seq = row.get('goal_sequence')
        if seq is None:
            continue
        try:
            if int(seq) == 2:
                out.append(row)
        except (TypeError, ValueError):
            continue
    return out


def _feedback_summary(rows: list[dict[str, Any]], goal_sequence: int = 2) -> dict[str, Any]:
    selected = [row for row in rows if _row_goal_sequence(row) == goal_sequence]
    distances = []
    recoveries = []
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


def _matches_old_goal2_segment(row: dict[str, Any]) -> bool:
    nested = _nested_refinement(row)
    candidates = [
        row.get('original_target'),
        row.get('target'),
        nested.get('original_target'),
        nested.get('refined_target'),
    ]
    return any((_distance(candidate, OLD_GOAL2_TARGET) or 999.0) <= GOAL2_TARGET_MATCH_TOLERANCE_M for candidate in candidates)


def _select_validation_goal_rows(events: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], int | None, bool]:
    """Return rows for the bounded Goal2 validation target.

    In Phase85 the ingress goal can be sent before maze_explorer starts.  When
    that happens, the old Goal2 branch is explorer goal_sequence=1 even though
    it is still the Phase79/80/81/82 Goal2 target.  Prefer explicit
    goal_sequence=2, then fall back to the old Goal2 target match.
    """
    explicit = _goal2_rows(events)
    if explicit:
        return explicit, 2, False
    matched = [row for row in events if _matches_old_goal2_segment(row)]
    if not matched:
        return [], None, False
    seq = _row_goal_sequence(matched[0])
    if seq is None:
        return matched, None, True
    return [row for row in events if _row_goal_sequence(row) == seq], seq, True


def _goal2_dispatch_context(dispatch: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(dispatch, dict):
        return None
    context = {key: dispatch.get(key) for key in REQUIRED_DISPATCH_FIELDS}
    nested = _nested_refinement(dispatch)
    nested_fallback_keys: list[str] = []
    for key in REQUIRED_DISPATCH_FIELDS:
        if context.get(key) is None and key in nested:
            context[key] = nested.get(key)
            nested_fallback_keys.append(key)
    context['target'] = dispatch.get('target')
    context['top_level_fields_present'] = {key: key in dispatch for key in REQUIRED_DISPATCH_FIELDS}
    context['nested_fallback_keys'] = nested_fallback_keys
    if nested:
        context['centerline_target_refinement'] = nested
    return context


def _outcome_summary(goal2_events: list[dict[str, Any]]) -> dict[str, Any]:
    outcome = next((row for row in goal2_events if row.get('event') in {'success', 'failure', 'timeout'}), None)
    cancel = next((row for row in goal2_events if row.get('event') == 'timeout_cancel_result'), None)
    selected = outcome or cancel
    event = selected.get('event') if isinstance(selected, dict) else None
    reason = selected.get('result_reason') if isinstance(selected, dict) else None
    timed_out = bool(event == 'timeout' or event == 'timeout_cancel_result' or reason in {'goal_timeout', 'goal_canceled_after_timeout'})
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


def analyze_artifact_dir(artifact_dir: Path | str) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl')
    feedback_path = _glob_first(artifact_dir, 'nav2_feedback.jsonl')
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json')
    trigger_path = _glob_first(artifact_dir, 'trigger_detected.json')

    events = _read_jsonl(goal_events_path)
    feedback = _read_jsonl(feedback_path)
    raw_capture = _safe_json_load(raw_capture_path, {})
    trigger = _safe_json_load(trigger_path, {})

    validation_events, validation_goal_sequence, used_target_match_fallback = _select_validation_goal_rows(events)
    dispatch = next((row for row in validation_events if row.get('event') == 'dispatch'), None)
    context = _goal2_dispatch_context(dispatch)
    outcome = _outcome_summary(validation_events)
    feedback_summary = _feedback_summary(feedback, validation_goal_sequence or 2)
    raw_summary = _raw_capture_summary(raw_capture)

    gaps: list[str] = []
    if not goal_events_path or not events:
        gaps.append('missing_goal_events_artifact')
    if dispatch is None:
        gaps.append('missing_goal2_dispatch_event')
    if context is not None:
        for key in REQUIRED_DISPATCH_FIELDS:
            present = isinstance(dispatch, dict) and (key in dispatch or key in _nested_refinement(dispatch))
            if not present:
                gaps.append(f'missing_dispatch_field:{key}')
        if context.get('branch_scoring_changed') is not False:
            gaps.append('branch_scoring_changed_not_false')
        if context.get('fallback_terminal_acceptance_used') is not False:
            gaps.append('fallback_terminal_acceptance_used_not_false')
    if not feedback_path or not feedback:
        gaps.append('missing_nav2_feedback_artifact')
    if not raw_capture_path or not raw_capture:
        gaps.append('missing_raw_capture_artifact')

    applied = bool(context.get('refinement_applied')) if isinstance(context, dict) else False
    original = context.get('original_target') if isinstance(context, dict) else None
    projected = context.get('centerline_projected_target') if isinstance(context, dict) else None
    target = context.get('target') if isinstance(context, dict) else None
    target_shift = _distance(original, projected) if applied else 0.0
    nav2_target_shift = _distance(original, target) if applied else 0.0
    forward_check = context.get('forward_executability_check') if isinstance(context, dict) else None
    forward_passed = bool(forward_check.get('passed')) if isinstance(forward_check, dict) else False

    if 'missing_goal2_dispatch_event' in gaps or 'missing_goal_events_artifact' in gaps:
        classification = 'INSUFFICIENT_VALIDATION_EVIDENCE'
    elif any(g.startswith('missing_dispatch_field:') for g in gaps):
        classification = 'INSUFFICIENT_VALIDATION_EVIDENCE'
    elif not applied:
        classification = 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED'
    elif outcome['timed_out'] or outcome['failed']:
        classification = 'REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT'
    elif outcome['succeeded'] or (feedback_summary['distance_remaining_min'] is not None and feedback_summary['distance_remaining_min'] <= 0.15 and feedback_summary['recoveries_max'] == 0):
        classification = 'REFINEMENT_APPLIED_AND_GOAL2_IMPROVED'
    else:
        classification = 'INSUFFICIENT_VALIDATION_EVIDENCE'
        gaps.append('missing_goal2_terminal_outcome')

    result = {
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': gaps,
        'goal2_dispatch_context': context,
        'validation_goal_sequence': validation_goal_sequence,
        'used_target_match_fallback': used_target_match_fallback,
        'goal2_refinement_summary': {
            'refinement_applied': applied,
            'refinement_reject_reason': context.get('refinement_reject_reason') if isinstance(context, dict) else None,
            'original_target': original,
            'centerline_projected_target': projected,
            'nav2_goal_target': target,
            'target_shift_m': target_shift,
            'nav2_target_shift_m': nav2_target_shift,
            'corridor_heading_yaw': context.get('corridor_heading_yaw') if isinstance(context, dict) else None,
            'forward_executability_passed': forward_passed,
        },
        'goal2_outcome': outcome,
        'nav2_feedback_summary': feedback_summary,
        'raw_capture_summary': raw_summary,
        'trigger': trigger,
        'guardrails': {
            'no_strategy_or_config_tuning': True,
            'branch_scoring_changed_false': context.get('branch_scoring_changed') is False if context else False,
            'fallback_terminal_acceptance_used_false': context.get('fallback_terminal_acceptance_used') is False if context else False,
            'no_nav2_mppi_controller_tuning': True,
            'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
        },
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'phase86_entered': False,
        'source_files': {
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'nav2_feedback': str(feedback_path) if feedback_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
            'trigger': str(trigger_path) if trigger_path else None,
        },
    }
    return result


def write_minimal_summary(result: dict[str, Any], path: Path) -> None:
    summary = result.get('goal2_refinement_summary') or {}
    outcome = result.get('goal2_outcome') or {}
    feedback = result.get('nav2_feedback_summary') or {}
    lines = [
        '# Phase85 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Refinement applied: {summary.get('refinement_applied')}",
        f"Original target: {summary.get('original_target')}",
        f"Centerline projected target: {summary.get('centerline_projected_target')}",
        f"Nav2 goal target: {summary.get('nav2_goal_target')}",
        f"Corridor heading yaw: {summary.get('corridor_heading_yaw')}",
        f"Forward executability passed: {summary.get('forward_executability_passed')}",
        f"Outcome: event={outcome.get('event')} reason={outcome.get('result_reason')} timed_out={outcome.get('timed_out')} succeeded={outcome.get('succeeded')}",
        f"Nav2 feedback: recoveries_max={feedback.get('recoveries_max')} distance_remaining_min={feedback.get('distance_remaining_min')}",
        '',
        'Screenshot suggestions if the scene is held:',
        '- Gazebo wide view: robot, Goal2 corridor, nearest wall, and target direction.',
        '- RViz local costmap: robot footprint/front wedge around the refined target approach.',
        '- RViz goal marker/tolerance: original vs refined/dispatch target if visible.',
        '- Recovery/timeout moment: show controller recovery or local-cost blockage if still present.',
        '',
        'Guardrails: No maze_explorer strategy changed; No Nav2/MPPI/controller tuning; No autonomous exploration success claimed; No exit success claimed; Phase86 not entered.',
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
