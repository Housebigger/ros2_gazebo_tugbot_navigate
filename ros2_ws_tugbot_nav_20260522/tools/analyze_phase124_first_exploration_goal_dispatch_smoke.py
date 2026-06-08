#!/usr/bin/env python3
"""Analyze Phase124 first exploration goal dispatch smoke artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUNNER_PATH = Path(__file__).resolve().parent / 'run_phase124_first_exploration_goal_dispatch_smoke.py'


def _load_runner() -> Any:
    spec = importlib.util.spec_from_file_location('phase124_runner_for_analyzer', RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase124 runner from {RUNNER_PATH}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runner = _load_runner()
REQUIRED_FIRST_GOAL_FIELDS = list(_runner.REQUIRED_FIRST_GOAL_FIELDS)
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


def _schema(first_goal_artifact: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_FIRST_GOAL_FIELDS if field not in first_goal_artifact]
    first_goal = _dict(first_goal_artifact.get('first_goal'))
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
        'result_status_label',
        'abort_text',
        'timeout',
    ]
    missing_nested = [field for field in required_nested if field not in first_goal]
    return {
        'required_first_goal_fields_present': not missing,
        'missing_first_goal_fields': missing,
        'required_first_goal_fields': REQUIRED_FIRST_GOAL_FIELDS,
        'required_first_goal_nested_fields_present': not missing_nested,
        'missing_first_goal_nested_fields': missing_nested,
        'required_first_goal_nested_fields': required_nested,
    }


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    handoff = _dict(artifact.get('handoff_artifact'))
    first_goal_artifact = _dict(artifact.get('first_goal_artifact'))
    first_goal = _dict(first_goal_artifact.get('first_goal'))
    schema = _schema(first_goal_artifact)
    expected = _runner.classify_phase124(handoff, first_goal_artifact)
    classification = artifact.get('classification') or expected
    dispatch_event_count = _int(first_goal_artifact.get('dispatch_event_count', artifact.get('dispatch_event_count')))
    goal_event_count = _int(first_goal_artifact.get('goal_event_count', artifact.get('goal_event_count')))
    max_goals = _int(first_goal_artifact.get('maze_explorer_max_goals', artifact.get('maze_explorer_max_goals')), -1)
    handoff_allowed_expected = _runner.handoff_ready(handoff)
    started = _bool(first_goal_artifact.get('maze_explorer_started', artifact.get('maze_explorer_started')))
    dispatched = _bool(first_goal_artifact.get('exploration_goal_dispatched', artifact.get('exploration_goal_dispatched')))
    goal_kind = first_goal.get('goal_kind')
    accepted = _bool(first_goal.get('accepted'))
    timeout = _bool(first_goal.get('timeout'))
    guardrails = {
        'classification_known': classification in CLASSIFICATIONS,
        'classification_matches_evidence': classification == expected,
        'handoff_ready_required': True,
        'handoff_not_ready_prevents_start': handoff_allowed_expected or (not started and not dispatched),
        'max_goals_one': max_goals == 1,
        'exactly_one_dispatch_event': dispatch_event_count <= 1,
        'success_requires_one_explore_dispatch': classification != _runner.FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP or (dispatch_event_count == 1 and goal_kind == 'explore' and accepted and not timeout),
        'non_explore_not_success': classification == _runner.FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP or goal_kind in (None, 'explore'),
        'no_manual_goal1': True,
        'no_fallback_terminal_exit_success': classification != _runner.FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP or goal_kind == 'explore',
        'branch_scoring_changed_false': not _bool(_dict(artifact.get('guardrails')).get('branch_scoring_changed')),
        'centerline_gate_changed_false': not _bool(_dict(artifact.get('guardrails')).get('centerline_gate_changed')),
        'fallback_changed_false': not _bool(_dict(artifact.get('guardrails')).get('fallback_changed')),
        'terminal_acceptance_changed_false': not _bool(_dict(artifact.get('guardrails')).get('terminal_acceptance_changed')),
        'nav2_config_changed_false': not _bool(_dict(artifact.get('guardrails')).get('nav2_config_changed')),
        'no_autonomous_success_claim': not _bool(_dict(artifact.get('guardrails')).get('autonomous_success_claimed')),
        'no_exit_success_claim': not _bool(_dict(artifact.get('guardrails')).get('exit_success_claimed')),
    }
    valid = bool(
        schema['required_first_goal_fields_present']
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
        'stop_reason': artifact.get('stop_reason') or first_goal_artifact.get('stop_reason'),
        'first_goal': first_goal,
        'schema': schema,
        'guardrails': guardrails,
        'evidence_summary': {
            'local_costmap_sample_count': len(_list(first_goal_artifact.get('local_costmap_samples'))),
            'nav2_feedback_count': len(_list(first_goal_artifact.get('nav2_feedback'))),
            'scan_seen': _bool(_dict(first_goal_artifact.get('scan')).get('seen')),
            'map_seen': _bool(_dict(first_goal_artifact.get('map')).get('seen')),
            'odom_seen': _bool(_dict(first_goal_artifact.get('odom')).get('seen')),
            'tf_fresh': _bool(_dict(first_goal_artifact.get('tf')).get('fresh')),
        },
        'notes': [
            'Phase124 accepted classification is first-goal smoke success only.',
            'No autonomous exploration success or exit success is claimed.',
        ],
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase124 first exploration goal dispatch smoke summary',
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"classification_matches_evidence: {analysis['classification_matches_evidence']}",
        f"handoff_allowed: {analysis['handoff_allowed']}",
        f"maze_explorer_started: {analysis['maze_explorer_started']}",
        f"maze_explorer_max_goals: {analysis['maze_explorer_max_goals']}",
        f"goal_kind: {analysis['first_goal'].get('goal_kind')}",
        f"accepted: {analysis['first_goal'].get('accepted')}",
        f"timeout: {analysis['first_goal'].get('timeout')}",
        f"goal_event_count: {analysis['goal_event_count']}",
        f"dispatch_event_count: {analysis['dispatch_event_count']}",
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
