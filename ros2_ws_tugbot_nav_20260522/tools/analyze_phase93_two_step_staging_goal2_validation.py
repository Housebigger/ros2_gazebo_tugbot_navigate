#!/usr/bin/env python3
"""Analyze Phase93 bounded Goal2 validation artifacts for Phase92 two-step staging.

Read-only analyzer. It consumes bounded reproduction artifacts and classifies
whether Phase92 two-step corridor-alignment staging triggered for the
Goal2-equivalent segment, whether the staging goal was hard-safe and dispatched,
whether fresh scan/local-costmap/TF evidence was observed after staging success,
and whether a second-step forward goal was generated/dispatched. It does not
change strategy, branch scoring, exploration order, centerline gate,
directional readiness, fallback/terminal acceptance, Nav2, MPPI, controller,
inflation, robot radius, clearance radius, or map thresholds.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase93_two_step_staging_bounded_goal2_validation'
OLD_GOAL2_TARGET = [2.057855221699651, 1.0261005743935105]
GOAL2_TARGET_MATCH_TOLERANCE_M = 0.40

ALLOWED_CLASSIFICATIONS = {
    'TWO_STEP_STAGING_APPLIED_AND_SECOND_STEP_DISPATCHED',
    'TWO_STEP_STAGING_APPLIED_BUT_SECOND_STEP_FAILED',
    'TWO_STEP_STAGING_REJECTED',
    'TWO_STEP_STAGING_NOT_TRIGGERED',
    'TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE',
}

REQUIRED_PHASE92_DISPATCH_FIELDS = [
    'two_step_staging_plan',
    'staging_goal_pose',
    'staging_reason',
    'staging_executability_check',
    'second_step_forward_goal',
    'staging_applied',
    'staging_reject_reason',
    'branch_scoring_changed',
    'fallback_terminal_acceptance_used',
]

TRIGGER_KEYS = [
    'near_goal_lateral_residual',
    'single_step_forward_search_no_hard_safety_pass',
    'safety_floor_dominant_blocker',
    'execution_time_footprint_front_wedge_risk',
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


def _glob_first(artifact_dir: Path, suffix: str) -> Path | None:
    direct = artifact_dir / f'{RUN_ID}_{suffix}'
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


def _candidate_values_for_goal2_match(row: JSONDict) -> list[Any]:
    staging_plan = _nested(row, 'two_step_staging_plan')
    source_single_step = _nested(staging_plan, 'source_single_step')
    centerline = _nested(row, 'centerline_target_refinement')
    second_step = _nested(row, 'second_step_forward_goal')
    return [
        row.get('original_target'),
        row.get('target'),
        row.get('refined_target'),
        row.get('centerline_projected_target'),
        row.get('selected_candidate_target'),
        row.get('staging_goal_pose'),
        centerline.get('original_target'),
        centerline.get('selected_candidate_target'),
        source_single_step.get('original_target'),
        second_step.get('selected_candidate_target'),
    ]


def _matches_goal2_equivalent(row: JSONDict) -> bool:
    return any((_distance(candidate, OLD_GOAL2_TARGET) or 999.0) <= GOAL2_TARGET_MATCH_TOLERANCE_M for candidate in _candidate_values_for_goal2_match(row))


def _has_phase92_fields(row: JSONDict) -> bool:
    return any(field in row for field in REQUIRED_PHASE92_DISPATCH_FIELDS)


def _select_goal2_dispatch(events: list[JSONDict]) -> tuple[JSONDict | None, list[JSONDict], bool]:
    dispatches = [row for row in events if row.get('event') == 'dispatch']
    matched = [row for row in dispatches if _matches_goal2_equivalent(row)]
    if matched:
        seq = _row_goal_sequence(matched[0])
        rows = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return matched[0], rows, True
    phase92 = [row for row in dispatches if _has_phase92_fields(row)]
    if phase92:
        seq = _row_goal_sequence(phase92[0])
        rows = [row for row in events if seq is None or _row_goal_sequence(row) == seq]
        return phase92[0], rows, False
    seq2 = [row for row in events if _row_goal_sequence(row) == 2]
    if seq2:
        dispatch = next((row for row in seq2 if row.get('event') == 'dispatch'), None)
        return dispatch, seq2, False
    seq1 = [row for row in events if _row_goal_sequence(row) == 1]
    dispatch = next((row for row in seq1 if row.get('event') == 'dispatch'), None)
    return dispatch, seq1, False


def _phase92_dispatch_context(dispatch: JSONDict | None) -> JSONDict | None:
    if not isinstance(dispatch, dict):
        return None
    context: JSONDict = {}
    for key in REQUIRED_PHASE92_DISPATCH_FIELDS:
        context[key] = dispatch.get(key)
    for key in [
        'goal_sequence',
        'goal_kind',
        'target',
        'original_target',
        'refined_target',
        'multi_candidate_forward_search',
        'candidate_count',
        'hard_safety_pass_candidate_count',
        'refinement_applied',
        'refinement_reject_reason',
        'original_target_preserved_on_reject',
        'centerline_target_refinement',
    ]:
        context[key] = dispatch.get(key)
    context['top_level_fields_present'] = {key: key in dispatch for key in REQUIRED_PHASE92_DISPATCH_FIELDS}
    return context


def _trigger_summary(context: JSONDict | None) -> JSONDict:
    plan = _nested(context, 'two_step_staging_plan') if isinstance(context, dict) else {}
    triggers = plan.get('trigger_conditions') if isinstance(plan.get('trigger_conditions'), dict) else {}
    normalized = {key: bool(triggers.get(key, False)) for key in TRIGGER_KEYS}
    present = all(key in triggers for key in TRIGGER_KEYS)
    return {
        'trigger_conditions': normalized,
        'trigger_fields_present': present,
        'all_trigger_conditions_true': present and all(normalized.values()),
        'plan_enabled': bool(plan.get('enabled', False)),
        'source_single_step': plan.get('source_single_step') if isinstance(plan.get('source_single_step'), dict) else {},
    }


def _staging_summary(context: JSONDict | None, events: list[JSONDict]) -> JSONDict:
    check = _nested(context, 'staging_executability_check') if isinstance(context, dict) else {}
    staging_applied = bool(context.get('staging_applied')) if isinstance(context, dict) else False
    nav2_dispatched = bool(
        staging_applied
        and (
            context.get('goal_kind') == 'corridor_alignment_staging'
            or context.get('staging_goal_pose') is not None
            or any(row.get('goal_kind') == 'corridor_alignment_staging' and row.get('event') == 'dispatch' for row in events)
        )
    ) if isinstance(context, dict) else False
    succeeded = any(row.get('goal_kind') == 'corridor_alignment_staging' and row.get('event') == 'success' for row in events)
    return {
        'staging_applied': staging_applied,
        'staging_goal_pose': context.get('staging_goal_pose') if isinstance(context, dict) else None,
        'staging_reason': context.get('staging_reason') if isinstance(context, dict) else None,
        'staging_hard_safety_pass': bool(check.get('hard_safety_pass', False)),
        'staging_executability_check': check,
        'staging_reject_reason': context.get('staging_reject_reason') if isinstance(context, dict) else None,
        'staging_nav2_dispatched': nav2_dispatched,
        'staging_nav2_succeeded': succeeded,
    }


def _fresh_evidence_from_second_step(second_step: JSONDict) -> JSONDict:
    return {
        'fresh_scan_received': bool(second_step.get('fresh_scan_received', False)),
        'fresh_local_costmap_received': bool(second_step.get('fresh_local_costmap_received', False)),
        'fresh_tf_received': bool(second_step.get('fresh_tf_received', False)),
        'generated_after_fresh_evidence': bool(second_step.get('generated_after_fresh_evidence', False)),
    }


def _find_second_step(events: list[JSONDict], initial_sequence: int | None) -> tuple[JSONDict | None, JSONDict]:
    candidates: list[JSONDict] = []
    for row in events:
        if row.get('event') != 'dispatch':
            continue
        second_step = _nested(row, 'second_step_forward_goal')
        seq = _row_goal_sequence(row)
        if second_step.get('generated_after_fresh_evidence') or (initial_sequence is not None and seq is not None and seq > initial_sequence):
            candidates.append(row)
    dispatch = candidates[0] if candidates else None
    if dispatch is not None:
        return dispatch, _nested(dispatch, 'second_step_forward_goal')
    # Some held-scene artifacts update second_step_forward_goal in a success/timeout
    # event rather than the dispatch row; preserve that evidence even if no second
    # dispatch is visible.
    for row in events:
        second_step = _nested(row, 'second_step_forward_goal')
        if second_step.get('generated_after_fresh_evidence'):
            return None, second_step
    return None, {}


def _fresh_evidence_summary(raw_capture: JSONDict, second_step: JSONDict) -> JSONDict:
    direct = _fresh_evidence_from_second_step(second_step)
    fresh_ts = raw_capture.get('fresh_evidence_timestamps') if isinstance(raw_capture.get('fresh_evidence_timestamps'), dict) else {}
    raw_scan = raw_capture.get('scan') is not None
    raw_cost = raw_capture.get('local_costmap') is not None
    raw_tf = bool(raw_capture.get('tf'))
    scan = direct['fresh_scan_received'] or bool(fresh_ts.get('after_staging_success_scan_sec'))
    cost = direct['fresh_local_costmap_received'] or bool(fresh_ts.get('after_staging_success_local_costmap_sec'))
    tf = direct['fresh_tf_received'] or bool(fresh_ts.get('after_staging_success_tf_sec'))
    # Raw timestamps prove the recorder saw scan/local-costmap/TF at the held
    # scene, but they do not prove post-staging fresh evidence unless a
    # second-step object says it was generated after fresh evidence.  This keeps
    # not-triggered/rejected cases from being misread as a successful fresh
    # evidence gate.
    all_fresh = bool(
        direct['generated_after_fresh_evidence']
        and (direct['fresh_scan_received'] or scan)
        and (direct['fresh_local_costmap_received'] or cost)
        and (direct['fresh_tf_received'] or tf)
    )
    return {
        **direct,
        'raw_scan_available': raw_scan,
        'raw_local_costmap_available': raw_cost,
        'raw_tf_available': raw_tf,
        'fresh_evidence_timestamps': fresh_ts,
        'all_fresh_evidence_received': all_fresh,
    }


def _outcome_for_sequences(events: list[JSONDict], sequences: set[int | None]) -> JSONDict:
    selected = []
    for row in events:
        seq = _row_goal_sequence(row)
        if sequences and seq not in sequences:
            continue
        if row.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'}:
            selected.append(row)
    row = selected[-1] if selected else None
    event = row.get('event') if isinstance(row, dict) else None
    reason = row.get('result_reason') if isinstance(row, dict) else None
    timed_out = bool(event in {'timeout', 'timeout_cancel_result'} or reason in {'goal_timeout', 'goal_canceled_after_timeout'})
    succeeded = bool(event == 'success' or reason in {'goal_reached', 'succeeded'})
    failed = bool(event == 'failure' or (reason and not timed_out and not succeeded))
    return {
        'event': event,
        'result_reason': reason,
        'timed_out': timed_out,
        'succeeded': succeeded,
        'failed': failed,
        'observed': row is not None,
    }


def _feedback_summary(rows: list[JSONDict], sequences: set[int | None]) -> JSONDict:
    selected = [row for row in rows if not sequences or _row_goal_sequence(row) in sequences]
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


def _local_cost_summary(rows: list[JSONDict], sequences: set[int | None]) -> JSONDict:
    selected = [row for row in rows if not sequences or _row_goal_sequence(row) in sequences]
    front: list[float] = []
    footprint: list[float] = []
    target: list[float] = []
    ages: list[float] = []
    for row in selected:
        for sink, key in [
            (front, 'front_wedge_lethal_count'),
            (footprint, 'robot_footprint_lethal_count'),
            (target, 'target_footprint_lethal_count'),
            (front, 'timeout_front_wedge_lethal_count'),
            (footprint, 'timeout_footprint_lethal_count'),
        ]:
            try:
                if row.get(key) is not None:
                    sink.append(float(row.get(key)))
            except (TypeError, ValueError):
                pass
        try:
            if row.get('sample_age_sec') is not None:
                ages.append(float(row.get('sample_age_sec')))
        except (TypeError, ValueError):
            pass
    return {
        'sample_count': len(selected),
        'front_wedge_lethal_count_max': max(front) if front else None,
        'front_wedge_lethal_count_last': front[-1] if front else None,
        'robot_footprint_lethal_count_max': max(footprint) if footprint else None,
        'robot_footprint_lethal_count_last': footprint[-1] if footprint else None,
        'target_footprint_lethal_count_max': max(target) if target else None,
        'sample_age_sec_last': ages[-1] if ages else None,
    }


def _second_step_summary(second_dispatch: JSONDict | None, second_step: JSONDict, outcome: JSONDict) -> JSONDict:
    generated = bool(second_step.get('generated_after_fresh_evidence') or second_step.get('selected_candidate_target'))
    return {
        'second_step_forward_goal_generated': generated,
        'second_step_forward_goal': second_step or None,
        'second_step_nav2_dispatched': second_dispatch is not None,
        'second_step_dispatch_target': second_dispatch.get('target') if isinstance(second_dispatch, dict) else None,
        'second_step_timed_out_or_failed': bool(outcome.get('timed_out') or outcome.get('failed')),
        'second_step_outcome': outcome,
    }


def analyze_artifact_dir(artifact_dir: Path | str) -> JSONDict:
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

    dispatch, validation_rows, used_target_match = _select_goal2_dispatch(events)
    context = _phase92_dispatch_context(dispatch)
    initial_seq = _row_goal_sequence(dispatch) if isinstance(dispatch, dict) else None
    second_dispatch, second_step = _find_second_step(events, initial_seq)
    second_seq = _row_goal_sequence(second_dispatch) if isinstance(second_dispatch, dict) else None
    outcome_sequences = {seq for seq in [second_seq, initial_seq] if seq is not None}
    outcome = _outcome_for_sequences(events, outcome_sequences)
    feedback_summary = _feedback_summary(feedback, outcome_sequences)
    local_summary = _local_cost_summary(local_cost, outcome_sequences)
    trigger_summary = _trigger_summary(context)
    staging_summary = _staging_summary(context, events)
    fresh_summary = _fresh_evidence_summary(raw_capture if isinstance(raw_capture, dict) else {}, second_step)
    second_summary = _second_step_summary(second_dispatch, second_step, outcome)

    gaps: list[str] = []
    if not goal_events_path or not events:
        gaps.append('missing_goal_events_artifact')
    if dispatch is None:
        gaps.append('missing_goal2_equivalent_dispatch_event')
    if context is not None:
        for key in REQUIRED_PHASE92_DISPATCH_FIELDS:
            if key not in dispatch:
                gaps.append(f'missing_dispatch_field:{key}')
        if context.get('branch_scoring_changed') is not False:
            gaps.append('branch_scoring_changed_not_false')
        if context.get('fallback_terminal_acceptance_used') is not False:
            gaps.append('fallback_terminal_acceptance_used_not_false')
    if not trigger_summary['trigger_fields_present']:
        gaps.append('missing_or_invalid_trigger_bundle')
    if not feedback_path or not feedback:
        gaps.append('missing_nav2_feedback_artifact')
    if not raw_capture_path or not raw_capture:
        gaps.append('missing_raw_capture_artifact')
    if not local_cost_path or not local_cost:
        gaps.append('missing_local_costmap_samples_artifact')

    major_gap = (
        dispatch is None
        or any(g.startswith('missing_dispatch_field:') for g in gaps)
        or 'missing_or_invalid_trigger_bundle' in gaps
        or 'branch_scoring_changed_not_false' in gaps
        or 'fallback_terminal_acceptance_used_not_false' in gaps
    )

    if major_gap:
        classification = 'TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE'
    elif not trigger_summary['all_trigger_conditions_true']:
        classification = 'TWO_STEP_STAGING_NOT_TRIGGERED'
    elif not staging_summary['staging_applied']:
        classification = 'TWO_STEP_STAGING_REJECTED'
    elif second_summary['second_step_timed_out_or_failed']:
        classification = 'TWO_STEP_STAGING_APPLIED_BUT_SECOND_STEP_FAILED'
    elif staging_summary['staging_applied'] and second_summary['second_step_nav2_dispatched']:
        classification = 'TWO_STEP_STAGING_APPLIED_AND_SECOND_STEP_DISPATCHED'
    else:
        classification = 'TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE'
        if not second_summary['second_step_forward_goal_generated']:
            gaps.append('missing_second_step_forward_goal_generation')
        if not second_summary['second_step_nav2_dispatched']:
            gaps.append('missing_second_step_nav2_dispatch')

    local_cost_risk_present = bool(
        (local_summary.get('front_wedge_lethal_count_max') or 0) > 0
        or (local_summary.get('robot_footprint_lethal_count_max') or 0) > 0
    )
    result = {
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': gaps,
        'validation_goal_sequence': initial_seq,
        'second_step_goal_sequence': second_seq,
        'used_goal2_target_match_fallback': used_target_match,
        'goal2_dispatch_context': context,
        'trigger_bundle': trigger_summary,
        'staging_summary': staging_summary,
        'fresh_evidence_summary': fresh_summary,
        'second_step_summary': second_summary,
        'goal2_timeout_recovery_local_cost_risk': {
            'timed_out': bool(outcome.get('timed_out')),
            'failed': bool(outcome.get('failed')),
            'recoveries_max': feedback_summary.get('recoveries_max'),
            'distance_remaining_last': feedback_summary.get('distance_remaining_last'),
            'local_cost_risk_present': local_cost_risk_present,
            'front_wedge_lethal_count_max': local_summary.get('front_wedge_lethal_count_max'),
            'robot_footprint_lethal_count_max': local_summary.get('robot_footprint_lethal_count_max'),
        },
        'nav2_feedback_summary': feedback_summary,
        'local_costmap_summary': local_summary,
        'raw_capture_summary': {
            'raw_capture_available': bool(raw_capture),
            'scan_available': isinstance(raw_capture, dict) and raw_capture.get('scan') is not None,
            'local_costmap_available': isinstance(raw_capture, dict) and raw_capture.get('local_costmap') is not None,
            'odom_available': isinstance(raw_capture, dict) and raw_capture.get('odom') is not None,
            'tf_available': isinstance(raw_capture, dict) and bool(raw_capture.get('tf')),
            'footprint_available': isinstance(raw_capture, dict) and raw_capture.get('footprint') is not None,
        },
        'trigger': trigger,
        'screenshot_suggestions': [
            'Gazebo wide view: robot, Goal2-equivalent corridor, nearest wall, staging/second-step direction.',
            'RViz local costmap: robot footprint and front wedge around staging or timeout pose.',
            'RViz goal/tolerance view: original Goal2 target, staging target, and second-step target if visible.',
            'Recovery/timeout pose: show local-cost blockage or recovery loop if still present.',
        ],
        'guardrails': {
            'no_maze_explorer_strategy_changed': True,
            'no_phase92_staging_logic_changed': True,
            'no_branch_scoring_changed': context.get('branch_scoring_changed') is False if isinstance(context, dict) else False,
            'no_exploration_order_changed': True,
            'no_centerline_gate_changed': True,
            'no_directional_readiness_changed': True,
            'no_fallback_terminal_acceptance_changed': context.get('fallback_terminal_acceptance_used') is False if isinstance(context, dict) else False,
            'no_nav2_mppi_controller_tuning': True,
            'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
        },
        'autonomous_exploration_success_claimed': False,
        'exit_success_claimed': False,
        'phase94_entered': False,
        'source_files': {
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'nav2_feedback': str(feedback_path) if feedback_path else None,
            'local_costmap_samples': str(local_cost_path) if local_cost_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
            'trigger': str(trigger_path) if trigger_path else None,
        },
    }
    return result


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    staging = result.get('staging_summary') or {}
    trigger = result.get('trigger_bundle') or {}
    fresh = result.get('fresh_evidence_summary') or {}
    second = result.get('second_step_summary') or {}
    risk = result.get('goal2_timeout_recovery_local_cost_risk') or {}
    lines = [
        '# Phase93 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Validation goal sequence: {result.get('validation_goal_sequence')} second_step_goal_sequence={result.get('second_step_goal_sequence')}",
        f"Trigger bundle all true: {trigger.get('all_trigger_conditions_true')} conditions={trigger.get('trigger_conditions')}",
        f"Staging applied: {staging.get('staging_applied')}",
        f"Staging hard-safety-pass: {staging.get('staging_hard_safety_pass')}",
        f"Staging Nav2 dispatched: {staging.get('staging_nav2_dispatched')}",
        f"Staging reject reason: {staging.get('staging_reject_reason')}",
        f"Staging goal pose: {staging.get('staging_goal_pose')}",
        f"Fresh evidence all received: {fresh.get('all_fresh_evidence_received')} scan={fresh.get('fresh_scan_received')} local_costmap={fresh.get('fresh_local_costmap_received')} tf={fresh.get('fresh_tf_received')}",
        f"Second-step generated: {second.get('second_step_forward_goal_generated')}",
        f"Second-step Nav2 dispatched: {second.get('second_step_nav2_dispatched')}",
        f"Second-step timed out or failed: {second.get('second_step_timed_out_or_failed')}",
        f"Timeout/recovery/local-cost risk: timed_out={risk.get('timed_out')} recoveries_max={risk.get('recoveries_max')} local_cost_risk_present={risk.get('local_cost_risk_present')} front_wedge_lethal_count_max={risk.get('front_wedge_lethal_count_max')} robot_footprint_lethal_count_max={risk.get('robot_footprint_lethal_count_max')}",
        '',
        'Screenshot suggestions if the scene is held:',
        '- Gazebo wide view: robot, Goal2-equivalent corridor, nearest wall, and staging/second-step direction.',
        '- RViz local costmap: robot footprint/front wedge around staging or timeout pose.',
        '- RViz goal/tolerance: original target, staging target, and second-step target if visible.',
        '- Recovery/timeout moment: show local-cost blockage or recovery loop if still present.',
        '',
        'Guardrails: No maze_explorer strategy changed; No Phase92 staging logic changed; No branch scoring changed; No exploration order changed; No centerline gate changed; No directional readiness changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; No autonomous exploration success claimed; No exit success claimed; Phase94 not entered.',
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
