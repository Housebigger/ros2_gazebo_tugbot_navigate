#!/usr/bin/env python3
"""Phase102 carry-over bounded Goal1 staging validation analyzer.

Read-only analyzer for Phase101 carry-over validation. It consumes bounded
visible ingress-guided runtime artifacts and classifies whether Goal1
corridor evidence carry-over fixed the previous missing_two_side_wall_evidence
staging evidence path without claiming navigation/exit success.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

RUN_ID = 'phase102_carry_over_bounded_goal1_staging_validation'

ALLOWED_CLASSIFICATIONS = {
    'CARRY_OVER_APPLIED_STAGING_APPLIED',
    'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
    'CARRY_OVER_REJECTED',
    'CARRY_OVER_NOT_TRIGGERED',
    'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
}

JSONDict = dict[str, Any]


def _read_json(path: Path | None, default: Any = None) -> Any:
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
                row.setdefault('_recorder_seq', raw.get('seq'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path, suffix: str, run_id: str = RUN_ID) -> Path | None:
    candidates = [artifact_dir / f'{run_id}_{suffix}', artifact_dir / suffix]
    candidates.extend(sorted(artifact_dir.glob(f'*{suffix}')))
    for path in candidates:
        if path.exists():
            return path
    return None


def _goal_seq(row: JSONDict) -> int | None:
    try:
        value = row.get('goal_sequence')
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _boolish(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _dict(value: Any) -> JSONDict:
    return value if isinstance(value, dict) else {}


def _extract_refinement(row: JSONDict) -> JSONDict:
    nested = row.get('centerline_target_refinement')
    return nested if isinstance(nested, dict) else row


def _extract_two_step(row: JSONDict) -> JSONDict:
    nested = row.get('two_step_staging_plan')
    if isinstance(nested, dict):
        return nested
    ref_nested = _extract_refinement(row).get('two_step_staging_plan')
    return ref_nested if isinstance(ref_nested, dict) else {}


def _extract_multi_candidate(row: JSONDict) -> JSONDict:
    ref = _extract_refinement(row)
    nested = ref.get('multi_candidate_forward_search')
    return nested if isinstance(nested, dict) else _dict(row.get('multi_candidate_forward_search'))


def _terminal_outcome(rows: list[JSONDict]) -> tuple[str | None, JSONDict | None]:
    terminals = []
    for row in rows:
        event = str(row.get('event') or '').lower()
        if event in {'success', 'succeeded', 'timeout', 'failure', 'failed', 'cancel', 'canceled', 'cancelled'}:
            terminals.append(row)
    if not terminals:
        return None, None
    row = terminals[-1]
    event = str(row.get('event') or '').lower()
    if event in {'success', 'succeeded'}:
        return 'succeeded', row
    if event == 'timeout':
        return 'timeout', row
    if event in {'cancel', 'canceled', 'cancelled'}:
        return 'cancelled', row
    return 'failure', row


def _feedback_summary(feedback: list[JSONDict], seq: int) -> JSONDict:
    rows = [row for row in feedback if _goal_seq(row) == seq]
    distances = [_as_float(row.get('distance_remaining')) for row in rows]
    distances = [value for value in distances if value is not None]
    recoveries = [_as_int(row.get('number_of_recoveries')) or 0 for row in rows]
    timeline = []
    last = None
    for row in rows:
        rec = _as_int(row.get('number_of_recoveries')) or 0
        if last is None or rec != last:
            timeline.append({'elapsed_sec': row.get('elapsed_sec') or row.get('_recorder_elapsed_sec'), 'recoveries': rec, 'distance_remaining': row.get('distance_remaining')})
            last = rec
    first_nonzero = next((value for value in distances if value > 0.0), None)
    return {
        'sample_count': len(rows),
        'distance_remaining_first_nonzero': first_nonzero,
        'distance_remaining_last': distances[-1] if distances else None,
        'distance_remaining_min': min(distances) if distances else None,
        'distance_remaining_max': max(distances) if distances else None,
        'distance_remaining_progress_delta': (first_nonzero - distances[-1]) if first_nonzero is not None and distances else None,
        'recoveries_max': max(recoveries) if recoveries else 0,
        'recoveries_timeline': timeline,
    }


def _cost_summary(value: Any) -> JSONDict:
    if isinstance(value, dict) and isinstance(value.get('summary'), dict):
        return value['summary']
    return value if isinstance(value, dict) else {}


def _max_num(values: list[Any]) -> float | None:
    nums = [_as_float(v) for v in values]
    nums = [v for v in nums if v is not None]
    return max(nums) if nums else None


def _local_cost_summary(samples: list[JSONDict], seq: int) -> JSONDict:
    rows = [row for row in samples if _goal_seq(row) == seq]
    if not rows:
        return {
            'sample_count': 0,
            'front_wedge_lethal_count_max': None,
            'robot_footprint_lethal_count_max': None,
            'target_footprint_lethal_count_max': None,
            'terminal_local_cost_blocked': None,
        }
    front = [_cost_summary(row.get('front_wedge_cost')).get('lethal_count') for row in rows]
    robot = [_cost_summary(row.get('robot_footprint_cost')).get('lethal_count') for row in rows]
    target = [_cost_summary(row.get('target_footprint_cost')).get('lethal_count') for row in rows]
    front_max = _max_num(front)
    robot_max = _max_num(robot)
    target_max = _max_num(target)
    return {
        'sample_count': len(rows),
        'front_wedge_lethal_count_max': front_max,
        'robot_footprint_lethal_count_max': robot_max,
        'target_footprint_lethal_count_max': target_max,
        'front_wedge_clearance_min_m': min([v for v in (_as_float(row.get('front_wedge_clearance_m')) for row in rows) if v is not None], default=None),
        'terminal_sample_pose': rows[-1].get('robot_pose'),
        'terminal_local_cost_blocked': any((v or 0.0) >= 1.0 for v in [front_max, robot_max, target_max] if v is not None),
    }


def _field(row: JSONDict, key: str) -> Any:
    if key in row:
        return row.get(key)
    ref = _extract_refinement(row)
    if key in ref:
        return ref.get(key)
    two_step = _extract_two_step(row)
    if key in two_step:
        return two_step.get(key)
    return None


def _goal1_summary(goal_rows: list[JSONDict], feedback: list[JSONDict], local_samples: list[JSONDict]) -> JSONDict:
    dispatch = next((row for row in goal_rows if str(row.get('event') or '').lower() == 'dispatch'), goal_rows[0] if goal_rows else {})
    terminal_outcome, terminal_row = _terminal_outcome(goal_rows)
    two_step = _extract_two_step(dispatch)
    multi = _extract_multi_candidate(dispatch)
    check = _dict(dispatch.get('staging_executability_check') or two_step.get('staging_executability_check'))
    source_forward_window = _dict(_field(dispatch, 'source_forward_window'))
    staging_window = _dict(_field(dispatch, 'staging_window'))
    carry_eval = _dict(_field(dispatch, 'corridor_evidence_carry_over'))
    carry_applied = _boolish(_field(dispatch, 'carry_over_applied'))
    staging_applied = _boolish(_field(dispatch, 'staging_applied')) or _boolish(two_step.get('staging_applied'))
    safety_recomputed = _boolish(_field(dispatch, 'safety_evidence_recomputed')) or _boolish(check.get('safety_evidence_recomputed'))
    return {
        'dispatch_observed': bool(dispatch),
        'terminal_outcome': terminal_outcome,
        'terminal_event': terminal_row.get('event') if terminal_row else None,
        'target': dispatch.get('target') or dispatch.get('selected_candidate_target'),
        'selected_candidate_target': dispatch.get('selected_candidate_target'),
        'selected_candidate_yaw': dispatch.get('selected_candidate_yaw'),
        'refinement_applied': _boolish(_field(dispatch, 'refinement_applied')),
        'multi_candidate_forward_search': multi,
        'hard_safety_pass_candidate_count': _as_int(_field(dispatch, 'hard_safety_pass_candidate_count') if _field(dispatch, 'hard_safety_pass_candidate_count') is not None else multi.get('hard_safety_pass_candidate_count')),
        'two_step_staging_plan': two_step,
        'corridor_evidence_carry_over': carry_eval,
        'carry_over_source': _field(dispatch, 'carry_over_source'),
        'carry_over_applied': carry_applied,
        'carry_over_reject_reason': _field(dispatch, 'carry_over_reject_reason'),
        'source_forward_window': source_forward_window,
        'staging_window': staging_window,
        'safety_evidence_recomputed': safety_recomputed,
        'staging_executability_check': check,
        'staging_applied': staging_applied,
        'staging_reject_reason': _field(dispatch, 'staging_reject_reason') or check.get('reason'),
        'branch_scoring_changed': _boolish(_field(dispatch, 'branch_scoring_changed')),
        'fallback_terminal_acceptance_used': _boolish(_field(dispatch, 'fallback_terminal_acceptance_used')),
        'nav2_feedback': _feedback_summary(feedback, 1),
        'local_cost': _local_cost_summary(local_samples, 1),
    }


def _classify(goal1: JSONDict, gaps: list[str]) -> str:
    if gaps or not goal1.get('dispatch_observed'):
        return 'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE'
    carry_eval = _dict(goal1.get('corridor_evidence_carry_over')).get('evaluated')
    carry_applied = bool(goal1.get('carry_over_applied'))
    carry_reject = goal1.get('carry_over_reject_reason')
    if carry_applied and bool(goal1.get('staging_applied')):
        return 'CARRY_OVER_APPLIED_STAGING_APPLIED'
    if carry_applied and not bool(goal1.get('staging_applied')):
        return 'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED'
    if carry_eval and not carry_applied and carry_reject:
        return 'CARRY_OVER_REJECTED'
    if carry_eval is False or (not carry_applied and not carry_reject):
        return 'CARRY_OVER_NOT_TRIGGERED'
    return 'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE'


def analyze_artifact_dir(artifact_dir: Path, run_id: str = RUN_ID) -> JSONDict:
    goal_events = _read_jsonl(_glob_first(artifact_dir, 'goal_events.jsonl', run_id))
    feedback = _read_jsonl(_glob_first(artifact_dir, 'nav2_feedback.jsonl', run_id))
    local_samples = _read_jsonl(_glob_first(artifact_dir, 'local_costmap_samples.jsonl', run_id))
    raw_capture = _read_json(_glob_first(artifact_dir, 'raw_capture.json', run_id), {})
    ingress = _read_json(_glob_first(artifact_dir, 'ingress_result.json', run_id), {})
    ingress_preflight_artifact = _read_json(_glob_first(artifact_dir, 'ingress_preflight.json', run_id), {})
    ingress_preflight = _dict(ingress_preflight_artifact.get('ingress_preflight'))
    if not ingress_preflight and isinstance(ingress.get('ingress_preflight'), dict):
        ingress_preflight = _dict(ingress.get('ingress_preflight'))
    trigger = _read_json(_glob_first(artifact_dir, 'trigger_detected.json', run_id), {})
    goal1_rows = [row for row in goal_events if _goal_seq(row) == 1]
    goal1 = _goal1_summary(goal1_rows, feedback, local_samples)
    gaps: list[str] = []
    trigger_name = str(trigger.get('trigger') or '')
    ingress_status = str(ingress.get('status') or ingress.get('status_text') or '').lower()
    ingress_success = bool(ingress.get('success') is True or ingress_status in {'succeeded', 'status_succeeded'} or ingress_status.endswith('succeeded'))
    if trigger_name == 'ingress_preflight_rejected_explorer_not_started' or (ingress_preflight and ingress_preflight.get('passed') is False):
        gaps.append('ingress_preflight_rejected_explorer_not_started')
    if trigger_name == 'ingress_failed_explorer_not_started' or (ingress and not ingress_success):
        if 'ingress_preflight_rejected_explorer_not_started' not in gaps:
            gaps.append('ingress_failed_explorer_not_started')
    if not goal_events:
        gaps.append('missing_goal_events')
    if not goal1_rows:
        gaps.append('missing_goal1_events')
    if not goal1.get('dispatch_observed'):
        gaps.append('missing_goal1_dispatch')
    if not raw_capture:
        gaps.append('missing_raw_capture')
    classification = _classify(goal1, gaps)
    check = _dict(goal1.get('staging_executability_check'))
    source_window = _dict(goal1.get('source_forward_window'))
    staging_window = _dict(goal1.get('staging_window'))
    direct_answers = {
        'missing_two_side_wall_fixed_by_carry_over': bool(
            goal1.get('carry_over_applied')
            and check.get('two_side_wall_evidence') is True
            and (check.get('local_two_side_wall_evidence') is False or staging_window.get('two_side_wall_count') == 0)
        ),
        'carry_over_is_corridor_level_only': bool(
            goal1.get('carry_over_applied')
            and goal1.get('safety_evidence_recomputed') is True
            and source_window.get('two_side_wall_count') is not None
        ),
        'safety_evidence_recomputed_is_true': bool(goal1.get('safety_evidence_recomputed')),
        'staging_reject_reason_after_carry_over': goal1.get('staging_reject_reason'),
        'staging_safety_recompute_failed': goal1.get('staging_reject_reason') == 'staging_safety_recompute_failed',
    }
    return {
        'run_id': run_id,
        'artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'trigger': trigger,
        'ingress': ingress,
        'ingress_preflight': ingress_preflight,
        'goal_event_count': len(goal_events),
        'observed_goal_sequences': sorted({seq for seq in (_goal_seq(row) for row in goal_events) if seq is not None}),
        'goal1': goal1,
        'raw_capture_summary': {
            'available': bool(raw_capture),
            'scan_available': bool(_dict(raw_capture.get('scan')).get('available') or raw_capture.get('scan')),
            'map_available': bool(raw_capture.get('map')),
            'local_costmap_available': bool(raw_capture.get('local_costmap')),
            'odom_available': bool(raw_capture.get('odom')),
            'tf_available': bool(raw_capture.get('tf')),
        },
        'evidence_gaps': gaps,
        'direct_answers': direct_answers,
        'guardrails': {
            'algorithm_changed': False,
            'phase88_92_101_logic_changed': False,
            'branch_scoring_changed': bool(goal1.get('branch_scoring_changed')),
            'fallback_terminal_acceptance_used': bool(goal1.get('fallback_terminal_acceptance_used')),
            'nav2_config_changed': False,
            'no_success_claimed': True,
            'no_exit_success_claimed': True,
            'phase103_entered': False,
        },
    }


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    goal1 = result.get('goal1', {})
    direct = result.get('direct_answers', {})
    lines = [
        '# Phase102 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Artifact dir: {result.get('artifact_dir')}",
        f"Observed goal sequences: {result.get('observed_goal_sequences')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        '',
        'Goal1:',
        f"- carry_over_applied: {goal1.get('carry_over_applied')}",
        f"- carry_over_reject_reason: {goal1.get('carry_over_reject_reason')}",
        f"- safety_evidence_recomputed: {goal1.get('safety_evidence_recomputed')}",
        f"- staging_applied: {goal1.get('staging_applied')}",
        f"- staging_reject_reason: {goal1.get('staging_reject_reason')}",
        f"- branch_scoring_changed: {goal1.get('branch_scoring_changed')}",
        f"- fallback_terminal_acceptance_used: {goal1.get('fallback_terminal_acceptance_used')}",
        '',
        'Direct answers:',
        f"- missing_two_side_wall_fixed_by_carry_over: {direct.get('missing_two_side_wall_fixed_by_carry_over')}",
        f"- carry_over_is_corridor_level_only: {direct.get('carry_over_is_corridor_level_only')}",
        f"- safety_evidence_recomputed_is_true: {direct.get('safety_evidence_recomputed_is_true')}",
        f"- staging_reject_reason_after_carry_over: {direct.get('staging_reject_reason_after_carry_over')}",
        '',
        'Guardrails: no algorithm/config tuning, no autonomous/exit success claim, Phase103 not entered.',
    ]
    path.write_text('\n'.join(lines) + '\n')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    parser.add_argument('--run-id', default=RUN_ID)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--minimal-summary-output', type=Path)
    args = parser.parse_args()
    result = analyze_artifact_dir(args.artifact_dir, run_id=args.run_id)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    if args.minimal_summary_output:
        args.minimal_summary_output.parent.mkdir(parents=True, exist_ok=True)
        write_minimal_summary(result, args.minimal_summary_output)
    print(json.dumps({'classification': result['classification'], 'evidence_gaps': result['evidence_gaps']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
