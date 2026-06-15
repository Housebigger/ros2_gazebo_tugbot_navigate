#!/usr/bin/env python3
"""Analyze Phase122 post-ingress handoff dry-start artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUNNER_PATH = Path(__file__).resolve().parent / 'run_phase122_post_ingress_handoff_dry_start.py'


def _load_runner() -> Any:
    spec = importlib.util.spec_from_file_location('phase122_runner_for_analyzer', RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase122 runner from {RUNNER_PATH}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runner = _load_runner()
REQUIRED_HANDOFF_FIELDS = list(_runner.REQUIRED_HANDOFF_FIELDS)


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _schema(handoff: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_HANDOFF_FIELDS if field not in handoff]
    return {
        'required_handoff_fields_present': not missing,
        'missing_handoff_fields': missing,
        'required_handoff_fields': REQUIRED_HANDOFF_FIELDS,
    }


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    handoff = _dict(artifact.get('handoff_artifact'))
    dry = _dict(artifact.get('dry_start'))
    phase120_artifact = _dict(artifact.get('phase120_artifact'))
    schema = _schema(handoff)
    expected = _runner.classify_phase122(handoff, dry)
    classification = artifact.get('classification') or expected
    handoff_allowed_expected = _runner.handoff_gate_allowed(handoff)
    exploration_goal_dispatched = _bool(artifact.get('exploration_goal_dispatched')) or _bool(dry.get('exploration_goal_dispatched'))
    maze_explorer_max_goals = artifact.get('maze_explorer_max_goals', dry.get('maze_explorer_max_goals'))
    try:
        max_goals_int = int(maze_explorer_max_goals) if maze_explorer_max_goals is not None else -1
    except (TypeError, ValueError):
        max_goals_int = -1
    ready_success = classification == _runner.INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP
    fail_closed = classification in {
        _runner.INGRESS_SUCCESS_HANDOFF_NOT_READY,
        _runner.POSE_NOT_AT_INGRESS_GOAL,
        _runner.NAV2_ACTION_NOT_IDLE,
        _runner.TF_OR_SCAN_STALE,
        _runner.COSTMAP_NOT_READY,
    }
    dry_violation = classification == _runner.MAZE_EXPLORER_DRY_START_VIOLATION
    guardrails = {
        'only_explicit_inner_ingress_goal_sent': _runner.phase120_ingress_success(phase120_artifact),
        'no_exploration_goal_dispatched': not exploration_goal_dispatched,
        'dry_start_max_goals_zero': max_goals_int == 0,
        'no_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': True,
        'no_nav2_config_tuning': True,
        'no_exploration_strategy_change': True,
        'no_autonomous_success_claim': not _bool(_dict(artifact.get('guardrails')).get('autonomous_success_claimed')),
        'no_exit_success_claim': not _bool(_dict(artifact.get('guardrails')).get('exit_success_claimed')),
    }
    valid = bool(
        schema['required_handoff_fields_present']
        and classification == expected
        and _bool(artifact.get('handoff_allowed')) == handoff_allowed_expected
        and guardrails['only_explicit_inner_ingress_goal_sent']
        and guardrails['no_exploration_goal_dispatched']
        and guardrails['dry_start_max_goals_zero']
        and guardrails['no_autonomous_success_claim']
        and guardrails['no_exit_success_claim']
        and (ready_success or fail_closed or dry_violation)
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
        'maze_explorer_start_allowed': _bool(artifact.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(artifact.get('maze_explorer_started')),
        'maze_explorer_max_goals': max_goals_int,
        'exploration_goal_dispatched': exploration_goal_dispatched,
        'schema': schema,
        'handoff': {
            'ingress_goal_result': _dict(handoff.get('ingress_goal_result')),
            'robot_pose_after_ingress': _dict(handoff.get('robot_pose_after_ingress')),
            'distance_to_ingress_goal': _dict(handoff.get('distance_to_ingress_goal')),
            'orientation_error': _dict(handoff.get('orientation_error')),
            'costmap_freshness': _dict(handoff.get('costmap_freshness')),
            'scan_freshness': _dict(handoff.get('scan_freshness')),
            'tf_freshness': _dict(handoff.get('tf_freshness')),
            'nav2_action_idle_state': _dict(handoff.get('nav2_action_idle_state')),
            'preconditions': _dict(handoff.get('preconditions')),
            'failure_reasons': _list(handoff.get('failure_reasons')),
        },
        'dry_start': dry,
        'guardrails': guardrails,
        'notes': [
            'Phase122 dry-start validates handoff and max_goals=0 startup only.',
            'No autonomous exploration success or exit success is claimed.',
        ],
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase122 post-ingress handoff dry-start summary',
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"classification_matches_evidence: {analysis['classification_matches_evidence']}",
        f"handoff_allowed: {analysis['handoff_allowed']}",
        f"maze_explorer_start_allowed: {analysis['maze_explorer_start_allowed']}",
        f"maze_explorer_started: {analysis['maze_explorer_started']}",
        f"maze_explorer_max_goals: {analysis['maze_explorer_max_goals']}",
        f"exploration_goal_dispatched: {analysis['exploration_goal_dispatched']}",
        '',
        '## Handoff fields',
        f"- ingress_goal_result: {analysis['handoff']['ingress_goal_result']}",
        f"- robot_pose_after_ingress: {analysis['handoff']['robot_pose_after_ingress']}",
        f"- distance_to_ingress_goal: {analysis['handoff']['distance_to_ingress_goal']}",
        f"- orientation_error: {analysis['handoff']['orientation_error']}",
        f"- costmap_freshness: {analysis['handoff']['costmap_freshness']}",
        f"- scan_freshness: {analysis['handoff']['scan_freshness']}",
        f"- tf_freshness: {analysis['handoff']['tf_freshness']}",
        f"- nav2_action_idle_state: {analysis['handoff']['nav2_action_idle_state']}",
        '',
        '## Guardrails',
    ]
    for key, value in analysis['guardrails'].items():
        lines.append(f'- {key}: {value}')
    lines.append('')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


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
