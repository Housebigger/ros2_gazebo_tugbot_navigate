#!/usr/bin/env python3
"""Analyze Phase129 instrumented first-goal timeout diagnosis artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUNNER_PATH = Path(__file__).resolve().parent / 'run_phase129_instrumented_first_goal_timeout_diagnosis.py'


def _load_runner() -> Any:
    spec = importlib.util.spec_from_file_location('phase129_runner_for_analyzer', RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase129 runner from {RUNNER_PATH}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runner = _load_runner()
CLASSIFICATIONS = list(_runner.CLASSIFICATIONS)
REQUIRED_RESULT_FIELDS = list(_runner.REQUIRED_RESULT_FIELDS)
INSTRUMENTATION_FIELDS = list(_runner.INSTRUMENTATION_FIELDS)


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


def _schema(result_artifact: dict[str, Any], instrumentation: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_RESULT_FIELDS if field not in result_artifact]
    first = _dict(result_artifact.get('first_goal'))
    required_first = [
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
    missing_first = [field for field in required_first if field not in first]
    missing_instr = [field for field in INSTRUMENTATION_FIELDS if field not in instrumentation]
    return {
        'required_result_fields_present': not missing,
        'missing_result_fields': missing,
        'required_result_fields': REQUIRED_RESULT_FIELDS,
        'required_first_goal_nested_fields_present': not missing_first,
        'missing_first_goal_nested_fields': missing_first,
        'required_first_goal_nested_fields': required_first,
        'required_instrumentation_fields_present': not missing_instr,
        'missing_instrumentation_fields': missing_instr,
        'required_instrumentation_fields': INSTRUMENTATION_FIELDS,
    }


def _coverage(instrumentation: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return _runner._instrumentation_coverage(instrumentation)


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    result_artifact = _dict(artifact.get('first_goal_result_artifact')) or artifact
    first = _dict(artifact.get('first_goal')) or _dict(result_artifact.get('first_goal'))
    instrumentation = _dict(artifact.get('instrumentation')) or _dict(result_artifact.get('instrumentation'))
    schema = _schema(result_artifact, instrumentation)
    derived = _runner.analyze_instrumented_evidence({'first_goal': first, 'instrumentation': instrumentation})
    classification = derived.get('classification') or artifact.get('classification') or _runner.INSUFFICIENT_TIMEOUT_EVIDENCE
    if artifact.get('classification') == _runner.FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP:
        classification = artifact.get('classification')
    dispatch_event_count = _int(artifact.get('dispatch_event_count', result_artifact.get('dispatch_event_count')))
    max_goals = _int(artifact.get('maze_explorer_max_goals', result_artifact.get('maze_explorer_max_goals')), -1)
    second = _bool(artifact.get('second_goal_dispatched', result_artifact.get('second_goal_dispatched')))
    guard_src = _dict(artifact.get('guardrails'))
    forbidden = _dict(artifact.get('forbidden_actions'))
    guardrails = {
        'classification_known': classification in CLASSIFICATIONS,
        'max_goals_one': max_goals == 1,
        'dispatch_event_count_one_or_less': dispatch_event_count <= 1,
        'second_goal_dispatched_false': not second,
        'goal_kind_explore_or_no_dispatch': dispatch_event_count == 0 or first.get('goal_kind') == 'explore',
        'success_is_first_goal_only': classification != _runner.FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP or (first.get('goal_kind') == 'explore' and first.get('result_status_label') == 'SUCCEEDED'),
        'no_second_exploration_goal_forbidden': forbidden.get('second_exploration_goal') is False or not second,
        'no_manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': forbidden.get('manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal') is False,
        'no_nav2_tuning': forbidden.get('nav2_mppi_controller_goal_checker_config_tuning') is False and not _bool(guard_src.get('nav2_config_changed')),
        'no_strategy_change': forbidden.get('exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change') is False and not _bool(guard_src.get('branch_scoring_changed')) and not _bool(guard_src.get('centerline_gate_changed')) and not _bool(guard_src.get('fallback_changed')) and not _bool(guard_src.get('terminal_acceptance_changed')),
        'no_autonomous_success_claim': not _bool(guard_src.get('autonomous_success_claimed')),
        'no_exit_success_claim': not _bool(guard_src.get('exit_success_claimed')),
    }
    cov = _coverage(instrumentation)
    valid = bool(
        schema['required_result_fields_present']
        and schema['required_first_goal_nested_fields_present']
        and schema['required_instrumentation_fields_present']
        and guardrails['classification_known']
        and all(guardrails.values())
    )
    return {
        'phase': artifact.get('phase') or _runner.PHASE,
        'mode': artifact.get('mode') or _runner.MODE,
        'classification': classification,
        'valid': valid,
        'raw_artifact_classification': artifact.get('classification'),
        'diagnostic_analysis': derived,
        'schema': schema,
        'guardrails': guardrails,
        'coverage': cov,
        'evidence_gaps': derived.get('evidence_gaps', _runner._evidence_gaps(instrumentation)),
        'first_goal': first,
        'counts': {
            'maze_explorer_max_goals': max_goals,
            'dispatch_event_count': dispatch_event_count,
            'goal_event_count': _int(artifact.get('goal_event_count', result_artifact.get('goal_event_count'))),
            'second_goal_dispatched': second,
        },
        'notes': [
            'Phase129 is instrumentation-only first-goal diagnosis.',
            'No autonomous exploration success or exit success is claimed.',
            'Timeout classifications remain diagnostic and do not authorize tuning or repair.',
        ],
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase129 instrumented first-goal timeout diagnosis summary',
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"dispatch_event_count: {analysis['counts']['dispatch_event_count']}",
        f"second_goal_dispatched: {analysis['counts']['second_goal_dispatched']}",
        '',
        '## Coverage',
    ]
    for key, value in analysis['coverage'].items():
        lines.append(f"- {key}: present={value.get('present')} count={value.get('count')}")
    lines.extend(['', '## Evidence gaps'])
    for gap in analysis['evidence_gaps']:
        lines.append(f'- {gap}')
    lines.extend(['', '## Guardrails'])
    for key, value in analysis['guardrails'].items():
        lines.append(f'- {key}: {value}')
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
        'artifact': str(args.artifact),
        'analysis': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
