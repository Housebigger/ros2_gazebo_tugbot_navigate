#!/usr/bin/env python3
"""Phase96 Phase88/92 refinement-chain bounded multi-goal smoke analyzer.

Read-only analyzer for a short visible smoke run with max_goals=2..3. It
summarizes each observed explore goal's Phase88 multi-candidate refinement and
Phase92 two-step staging fields, terminal outcome, Nav2 recoveries, distance
remaining trend, and local-cost risk.

This analyzer does not modify maze_explorer strategy, Phase88/92 logic, branch
scoring, exploration order, centerline gate, directional readiness,
fallback/terminal acceptance, Nav2, MPPI, controller, inflation, robot radius,
clearance radius, or map thresholds.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

RUN_ID = 'phase96_refinement_chain_bounded_multi_goal_smoke'
RECOVERY_DOMINANT_THRESHOLD = 5
LETHAL_THRESHOLD = 1

ALLOWED_CLASSIFICATIONS = {
    'REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS',
    'REFINEMENT_CHAIN_GOAL_TIMEOUT',
    'REFINEMENT_CHAIN_RECOVERY_DOMINANT',
    'REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW',
    'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE',
}

REQUIRED_GOAL_FIELDS = {
    'refinement_applied',
    'multi_candidate_forward_search',
    'hard_safety_pass_candidate_count',
    'two_step_staging_plan',
    'staging_applied',
    'second_step_forward_goal',
    'terminal_outcome',
    'recoveries',
    'local_cost_risk',
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
    if not isinstance(row, dict):
        return {}
    nested = _nested(row, 'centerline_target_refinement')
    return nested if nested else row


def _extract_multi_candidate(row: JSONDict | None) -> JSONDict:
    ref = _extract_refinement(row)
    nested = ref.get('multi_candidate_forward_search')
    if isinstance(nested, dict):
        return nested
    nested = row.get('multi_candidate_forward_search') if isinstance(row, dict) else None
    return nested if isinstance(nested, dict) else {}


def _extract_two_step(row: JSONDict | None) -> JSONDict:
    if not isinstance(row, dict):
        return {}
    nested = row.get('two_step_staging_plan')
    if isinstance(nested, dict):
        return nested
    nested = _extract_refinement(row).get('two_step_staging_plan')
    return nested if isinstance(nested, dict) else {}


def _boolish(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


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
    distances = [v for v in distances if v is not None]
    recoveries = [_as_int(row.get('number_of_recoveries')) or 0 for row in rows]
    timeline = []
    last = None
    for row in rows:
        rec = _as_int(row.get('number_of_recoveries')) or 0
        if last is None or rec != last:
            timeline.append({'elapsed_sec': row.get('elapsed_sec'), 'recoveries': rec})
            last = rec
    return {
        'sample_count': len(rows),
        'distance_remaining_first_nonzero': next((v for v in distances if v > 0.0), None),
        'distance_remaining_last': distances[-1] if distances else None,
        'distance_remaining_min': min(distances) if distances else None,
        'distance_remaining_max': max(distances) if distances else None,
        'distance_remaining_progress_delta': (next((v for v in distances if v > 0.0), distances[0]) - distances[-1]) if distances else None,
        'recoveries_max': max(recoveries) if recoveries else 0,
        'recoveries_timeline': timeline,
    }


def _cost_summary(value: Any) -> JSONDict:
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
    last = rows[-1]
    terminal_blocked = any((v or 0.0) >= LETHAL_THRESHOLD for v in [front_max, robot_max, target_max] if v is not None)
    clearances = [_as_float(row.get('front_wedge_clearance_m')) for row in rows]
    clearances = [value for value in clearances if value is not None]
    return {
        'sample_count': len(rows),
        'front_wedge_lethal_count_last': _as_float(_cost_summary(last.get('front_wedge_cost')).get('lethal_count')),
        'front_wedge_lethal_count_max': front_max,
        'robot_footprint_lethal_count_last': _as_float(_cost_summary(last.get('robot_footprint_cost')).get('lethal_count')),
        'robot_footprint_lethal_count_max': robot_max,
        'target_footprint_lethal_count_last': _as_float(_cost_summary(last.get('target_footprint_cost')).get('lethal_count')),
        'target_footprint_lethal_count_max': target_max,
        'front_wedge_clearance_min_m': min(clearances) if clearances else None,
        'terminal_sample_pose': last.get('robot_pose'),
        'terminal_local_cost_blocked': terminal_blocked,
    }


def _goal_summary(seq: int, rows: list[JSONDict], feedback: list[JSONDict], local_samples: list[JSONDict]) -> JSONDict:
    dispatch = next((row for row in rows if row.get('event') == 'dispatch'), rows[0] if rows else {})
    terminal, terminal_row = _terminal_outcome(rows)
    ref = _extract_refinement(dispatch)
    multi = _extract_multi_candidate(dispatch)
    two_step = _extract_two_step(dispatch)
    hard_count = _as_int(ref.get('hard_safety_pass_candidate_count'))
    if hard_count is None:
        hard_count = _as_int(multi.get('hard_safety_pass_candidate_count'))
    staging_applied = _boolish(dispatch.get('staging_applied')) or _boolish(two_step.get('staging_applied'))
    second_step = dispatch.get('second_step_forward_goal') or two_step.get('second_step_forward_goal') or ref.get('second_step_forward_goal')
    feedback_summary = _feedback_summary(feedback, seq)
    local_summary = _local_cost_summary(local_samples, seq)
    return {
        'goal_sequence': seq,
        'event_count': len(rows),
        'goal_kind': dispatch.get('goal_kind'),
        'target': dispatch.get('target') or dispatch.get('nav2_goal_target'),
        'refinement_applied': _boolish(ref.get('refinement_applied')) or _boolish(ref.get('applied')) or _boolish(dispatch.get('centerline_refinement_applied')),
        'multi_candidate_forward_search': {
            'present': bool(multi),
            'candidate_count': multi.get('candidate_count'),
            'candidate_family': multi.get('candidate_family'),
            'selected_candidate_index': multi.get('selected_candidate_index'),
            'selected_candidate_target': ref.get('selected_candidate_target') or multi.get('selected_candidate_target'),
            'selected_candidate_yaw': ref.get('selected_candidate_yaw') or multi.get('selected_candidate_yaw'),
            'selection_priority_trace': multi.get('selection_priority_trace') or ref.get('selection_priority_trace'),
        },
        'hard_safety_pass_candidate_count': hard_count,
        'two_step_staging_plan': {
            'present': bool(two_step),
            'staging_goal_pose': two_step.get('staging_goal_pose'),
            'staging_reject_reason': dispatch.get('staging_reject_reason') or two_step.get('staging_reject_reason') or ref.get('staging_reject_reason'),
            'eligible': two_step.get('eligible'),
        },
        'staging_applied': staging_applied,
        'second_step_forward_goal': second_step,
        'terminal_outcome': terminal,
        'terminal_event': terminal_row.get('event') if terminal_row else None,
        'terminal_result_status': terminal_row.get('result_status') if terminal_row else None,
        'terminal_pose': terminal_row.get('robot_pose') or terminal_row.get('terminal_pose') if terminal_row else None,
        'recoveries': {'max': feedback_summary['recoveries_max'], 'timeline': feedback_summary['recoveries_timeline']},
        'nav2_feedback': feedback_summary,
        'local_cost_risk': local_summary,
        'branch_scoring_changed': bool(dispatch.get('branch_scoring_changed')),
        'fallback_terminal_acceptance_used': bool(dispatch.get('fallback_terminal_acceptance_used')),
    }


def _group_by_goal(events: list[JSONDict]) -> dict[int, list[JSONDict]]:
    grouped: dict[int, list[JSONDict]] = {}
    for row in events:
        seq = _goal_seq(row)
        if seq is not None:
            grouped.setdefault(seq, []).append(row)
    return dict(sorted(grouped.items()))


def _required_evidence_gaps(feedback: list[JSONDict], local_samples: list[JSONDict], raw_capture: JSONDict) -> list[str]:
    gaps: list[str] = []
    if not feedback:
        gaps.append('nav2_feedback_missing')
    if not local_samples:
        gaps.append('local_costmap_samples_missing')
    if not raw_capture:
        gaps.append('raw_scan_odom_tf_capture_missing')
    else:
        if not raw_capture.get('scan'):
            gaps.append('scan_capture_missing')
        if not raw_capture.get('odom'):
            gaps.append('odom_capture_missing')
        if not raw_capture.get('tf'):
            gaps.append('tf_capture_missing')
    return gaps


def _classify(per_goal: dict[str, JSONDict], evidence_gaps: list[str], observed_goal_count: int) -> str:
    if evidence_gaps or observed_goal_count == 0:
        return 'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE'
    goals = list(per_goal.values())
    if any(goal.get('staging_applied') for goal in goals):
        return 'REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW'
    if any((goal.get('recoveries') or {}).get('max', 0) >= RECOVERY_DOMINANT_THRESHOLD for goal in goals):
        return 'REFINEMENT_CHAIN_RECOVERY_DOMINANT'
    if any(goal.get('terminal_outcome') in {'timeout', 'failure', 'cancelled'} for goal in goals):
        return 'REFINEMENT_CHAIN_GOAL_TIMEOUT'
    if observed_goal_count < 2:
        return 'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE'
    if any(goal.get('terminal_outcome') is None for goal in goals):
        return 'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE'
    return 'REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS'


def analyze_artifacts(artifact_dir: str | Path, *, run_id: str = RUN_ID, max_goals: int = 3) -> JSONDict:
    artifact = Path(artifact_dir)
    events = _read_jsonl(_glob_first(artifact, 'goal_events.jsonl', run_id))
    feedback = _read_jsonl(_glob_first(artifact, 'nav2_feedback.jsonl', run_id))
    local_samples = _read_jsonl(_glob_first(artifact, 'local_costmap_samples.jsonl', run_id))
    raw_capture = _read_json(_glob_first(artifact, 'raw_capture.json', run_id), {})
    grouped = _group_by_goal(events)
    selected_sequences = sorted(grouped)[:max_goals]
    per_goal = {str(seq): _goal_summary(seq, grouped[seq], feedback, local_samples) for seq in selected_sequences}
    evidence_gaps = _required_evidence_gaps(feedback, local_samples, raw_capture)
    classification = _classify(per_goal, evidence_gaps, len(selected_sequences))
    return {
        'run_id': run_id,
        'artifact_dir': str(artifact),
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'classification': classification,
        'max_goals_configured': max_goals,
        'observed_goal_count': len(selected_sequences),
        'observed_goal_sequences': selected_sequences,
        'evidence_gaps': evidence_gaps,
        'per_goal': per_goal,
        'raw_capture_summary': {
            'available': bool(raw_capture),
            'scan_available': bool(raw_capture.get('scan')) if isinstance(raw_capture, dict) else False,
            'odom_available': bool(raw_capture.get('odom')) if isinstance(raw_capture, dict) else False,
            'tf_available': bool(raw_capture.get('tf')) if isinstance(raw_capture, dict) else False,
        },
        'guardrails': {
            'algorithm_changed': False,
            'nav2_config_changed': False,
            'no_autonomous_exploration_success_claimed': True,
            'no_exit_success_claimed': True,
        },
    }


def _write_minimal_summary(result: JSONDict, path: Path) -> None:
    lines = [
        '# Phase96 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Observed goals: {result.get('observed_goal_sequences')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        '',
    ]
    for seq, goal in result.get('per_goal', {}).items():
        lines.extend([
            f"## Goal {seq}",
            f"refinement_applied: {goal.get('refinement_applied')}",
            f"multi_candidate_forward_search_present: {goal.get('multi_candidate_forward_search', {}).get('present')}",
            f"hard_safety_pass_candidate_count: {goal.get('hard_safety_pass_candidate_count')}",
            f"two_step_staging_plan_present: {goal.get('two_step_staging_plan', {}).get('present')}",
            f"staging_applied: {goal.get('staging_applied')}",
            f"second_step_forward_goal: {goal.get('second_step_forward_goal')}",
            f"terminal_outcome: {goal.get('terminal_outcome')}",
            f"recoveries_max: {goal.get('recoveries', {}).get('max')}",
            f"distance_remaining_last: {goal.get('nav2_feedback', {}).get('distance_remaining_last')}",
            f"front_wedge_lethal_count_max: {goal.get('local_cost_risk', {}).get('front_wedge_lethal_count_max')}",
            f"robot_footprint_lethal_count_max: {goal.get('local_cost_risk', {}).get('robot_footprint_lethal_count_max')}",
            '',
        ])
    lines.extend([
        'No autonomous exploration success claimed. No exit success claimed.',
        '',
    ])
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    parser.add_argument('--run-id', default=RUN_ID)
    parser.add_argument('--max-goals', type=int, default=3)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--minimal-summary-output', type=Path)
    args = parser.parse_args()
    result = analyze_artifacts(args.artifact_dir, run_id=args.run_id, max_goals=args.max_goals)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if args.minimal_summary_output:
        args.minimal_summary_output.parent.mkdir(parents=True, exist_ok=True)
        _write_minimal_summary(result, args.minimal_summary_output)
    print(json.dumps({'classification': result['classification'], 'observed_goal_sequences': result['observed_goal_sequences'], 'evidence_gaps': result['evidence_gaps']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
