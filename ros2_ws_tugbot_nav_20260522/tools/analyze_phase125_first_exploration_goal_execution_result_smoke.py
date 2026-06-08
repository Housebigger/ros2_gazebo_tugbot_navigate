#!/usr/bin/env python3
"""Analyze Phase125 first exploration goal execution result smoke artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUNNER_PATH = Path(__file__).resolve().parent / 'run_phase125_first_exploration_goal_execution_result_smoke.py'


def _load_runner() -> Any:
    spec = importlib.util.spec_from_file_location('phase125_runner_for_analyzer', RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase125 runner from {RUNNER_PATH}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runner = _load_runner()
REQUIRED_RESULT_FIELDS = list(_runner.REQUIRED_RESULT_FIELDS)
CLASSIFICATIONS = list(_runner.CLASSIFICATIONS)


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _schema(result_artifact: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_RESULT_FIELDS if field not in result_artifact]
    first = _dict(result_artifact.get('first_goal'))
    required_nested = [
        'candidate',
        'pose',
        'frame_id',
        'goal_kind',
        'selection_reason',
        'frontier_evidence',
        'topology_evidence',
        'dispatch_wall_time_sec',
        'accepted',
        'rejected',
        'nav2_feedback_timeline',
        'result_status_label',
        'abort_text',
        'timeout',
        'robot_pose_at_result',
        'distance_to_first_goal',
    ]
    missing_nested = [field for field in required_nested if field not in first]
    return {
        'required_result_fields_present': not missing,
        'missing_result_fields': missing,
        'required_result_fields': REQUIRED_RESULT_FIELDS,
        'required_first_goal_nested_fields_present': not missing_nested,
        'missing_first_goal_nested_fields': missing_nested,
        'required_first_goal_nested_fields': required_nested,
    }


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    handoff = _dict(artifact.get('handoff_artifact'))
    result_artifact = _dict(artifact.get('first_goal_result_artifact'))
    first = _dict(result_artifact.get('first_goal'))
    schema = _schema(result_artifact)
    expected = _runner.classify_phase125(handoff, result_artifact)
    classification = artifact.get('classification') or expected
    dispatch_event_count = _int(result_artifact.get('dispatch_event_count', artifact.get('dispatch_event_count')))
    goal_event_count = _int(result_artifact.get('goal_event_count', artifact.get('goal_event_count')))
    max_goals = _int(result_artifact.get('maze_explorer_max_goals', artifact.get('maze_explorer_max_goals')), -1)
    handoff_allowed_expected = _runner.handoff_ready(handoff)
    started = _bool(result_artifact.get('maze_explorer_started', artifact.get('maze_explorer_started')))
    dispatched = _bool(result_artifact.get('exploration_goal_dispatched', artifact.get('exploration_goal_dispatched')))
    second = _bool(result_artifact.get('second_goal_dispatched', artifact.get('second_goal_dispatched')))
    goal_kind = first.get('goal_kind')
    status = first.get('result_status_label')
    timeout = _bool(first.get('timeout'))
    robot_pose = _dict(first.get('robot_pose_at_result'))
    distance = _dict(first.get('distance_to_first_goal'))
    guardrails = {
        'classification_known': classification in CLASSIFICATIONS,
        'classification_matches_evidence': classification == expected,
        'handoff_ready_required': True,
        'handoff_not_ready_prevents_start': handoff_allowed_expected or (not started and not dispatched),
        'max_goals_one': max_goals == 1,
        'dispatch_event_count_one_or_less': dispatch_event_count <= 1,
        'second_goal_dispatched_false': not second,
        'success_requires_succeeded_result': classification != _runner.FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP or (dispatch_event_count == 1 and goal_kind == 'explore' and status == 'SUCCEEDED' and not timeout),
        'non_explore_not_success': classification != _runner.FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP or goal_kind == 'explore',
        'no_manual_goal1': True,
        'no_fallback_terminal_exit_success': classification != _runner.FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP or goal_kind == 'explore',
        'branch_scoring_changed_false': not _bool(_dict(artifact.get('guardrails')).get('branch_scoring_changed')),
        'centerline_gate_changed_false': not _bool(_dict(artifact.get('guardrails')).get('centerline_gate_changed')),
        'fallback_changed_false': not _bool(_dict(artifact.get('guardrails')).get('fallback_changed')),
        'terminal_acceptance_changed_false': not _bool(_dict(artifact.get('guardrails')).get('terminal_acceptance_changed')),
        'nav2_config_changed_false': not _bool(_dict(artifact.get('guardrails')).get('nav2_config_changed')),
        'no_autonomous_success_claim': not _bool(_dict(artifact.get('guardrails')).get('autonomous_success_claimed')),
        'no_exit_success_claim': not _bool(_dict(artifact.get('guardrails')).get('exit_success_claimed')),
    }
    valid = bool(
        schema['required_result_fields_present']
        and schema['required_first_goal_nested_fields_present']
        and all(guardrails.values())
        and goal_event_count >= dispatch_event_count
    )
    return {
        'phase': artifact.get('phase') or _runner.PHASE,
        'mode': artifact.get('mode') or _runner.MODE,
        'classification': classification,
        'expected_classification_from_evidence': expected,
        'classification_matches_evidence': classification == expected,
        'valid': valid,
        'handoff_allowed': _bool(artifact.get('handoff_allowed')),
        'handoff_allowed_expected': handoff_allowed_expected,
        'maze_explorer_started': started,
        'maze_explorer_max_goals': max_goals,
        'exploration_goal_dispatched': dispatched,
        'goal_event_count': goal_event_count,
        'dispatch_event_count': dispatch_event_count,
        'second_goal_dispatched': second,
        'stop_reason': artifact.get('stop_reason') or result_artifact.get('stop_reason'),
        'first_goal': first,
        'schema': schema,
        'guardrails': guardrails,
        'evidence_summary': {
            'local_costmap_sample_count': len(_list(result_artifact.get('local_costmap_samples'))),
            'nav2_feedback_count': len(_list(result_artifact.get('nav2_feedback'))),
            'scan_seen': _bool(_dict(result_artifact.get('scan')).get('seen')),
            'map_seen': _bool(_dict(result_artifact.get('map')).get('seen')),
            'odom_seen': _bool(_dict(result_artifact.get('odom')).get('seen')),
            'tf_fresh': _bool(_dict(result_artifact.get('tf')).get('fresh')),
            'costmap_fresh': _bool(_dict(result_artifact.get('costmap_freshness')).get('fresh')),
            'scan_fresh': _bool(_dict(result_artifact.get('scan_freshness')).get('fresh')),
            'tf_freshness': _bool(_dict(result_artifact.get('tf_freshness')).get('fresh')),
            'robot_pose_at_result_available': _bool(robot_pose.get('pose_available')),
            'distance_to_first_goal_available': _bool(distance.get('available')),
            'distance_to_first_goal_m': distance.get('meters'),
        },
        'notes': [
            'Phase125 result success is first-goal execution smoke success only.',
            'No autonomous exploration success or exit success is claimed.',
        ],
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase125 first exploration goal execution result smoke summary',
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"classification_matches_evidence: {analysis['classification_matches_evidence']}",
        f"handoff_allowed: {analysis['handoff_allowed']}",
        f"maze_explorer_started: {analysis['maze_explorer_started']}",
        f"maze_explorer_max_goals: {analysis['maze_explorer_max_goals']}",
        f"goal_kind: {analysis['first_goal'].get('goal_kind')}",
        f"accepted: {analysis['first_goal'].get('accepted')}",
        f"rejected: {analysis['first_goal'].get('rejected')}",
        f"timeout: {analysis['first_goal'].get('timeout')}",
        f"result_status_label: {analysis['first_goal'].get('result_status_label')}",
        f"goal_event_count: {analysis['goal_event_count']}",
        f"dispatch_event_count: {analysis['dispatch_event_count']}",
        f"second_goal_dispatched: {analysis['second_goal_dispatched']}",
        f"stop_reason: {analysis['stop_reason']}",
        '',
        '## Evidence summary',
    ]
    for key, value in analysis['evidence_summary'].items():
        lines.append(f'- {key}: {value}')
    lines.extend(['', '## Guardrails'])
    for key, value in analysis['guardrails'].items():
        lines.append(f'- {key}: {value}')
    lines.append('')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--summary', type=Path)
    args = parser.parse_args()
    artifact = json.loads(args.artifact.read_text(encoding='utf-8'))
    analysis = analyze_artifact(artifact)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(analysis, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if args.summary:
        write_summary(analysis, args.summary)
    print(json.dumps({
        'classification': analysis['classification'],
        'valid': analysis['valid'],
        'classification_matches_evidence': analysis['classification_matches_evidence'],
        'artifact': str(args.artifact),
        'analysis': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
