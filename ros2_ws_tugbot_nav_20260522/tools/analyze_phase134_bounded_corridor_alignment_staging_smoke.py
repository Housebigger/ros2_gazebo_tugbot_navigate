#!/usr/bin/env python3
"""Analyze Phase134 bounded corridor-alignment staging-contract smoke artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase134'
MODE = 'bounded_corridor_alignment_staging_smoke_analysis'
CLASSIFICATIONS = [
    'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
    'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
    'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
]
DEFAULT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT = DEFAULT_ROOT / 'log' / 'phase134_bounded_corridor_alignment_staging_smoke' / 'phase134_bounded_corridor_alignment_staging_smoke.json'
DEFAULT_OUTPUT_JSON = DEFAULT_ROOT / 'log' / 'phase134_bounded_corridor_alignment_staging_smoke' / 'phase134_bounded_corridor_alignment_staging_smoke_analysis.json'
DEFAULT_OUTPUT_MD = DEFAULT_ROOT / 'log' / 'phase134_bounded_corridor_alignment_staging_smoke' / 'phase134_bounded_corridor_alignment_staging_smoke_summary.md'


def _load_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase134_bounded_corridor_alignment_staging_smoke.py'
    spec = importlib.util.spec_from_file_location('phase134_runner_for_analysis', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase134 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_runner = _load_runner()


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _value(value: Any, source: str) -> dict[str, Any]:
    return {
        'available': value is not None,
        'value': value if value is not None else 'missing',
        'source': source,
    }


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _classification_reasons(artifact: dict[str, Any], classification: str) -> list[str]:
    first = _dict(artifact.get('first_literal_dispatch'))
    reasons: list[str] = []
    goal_kind = first.get('goal_kind')
    if goal_kind == 'explore':
        reasons.append('first literal dispatch is goal_kind=explore; Phase134 stops after first literal dispatch classification')
    elif goal_kind == 'corridor_alignment_staging':
        reasons.append('first literal dispatch is goal_kind=corridor_alignment_staging under the Phase132 contract')
        if first.get('staging_applied') is True:
            reasons.append('staging_applied=true')
        if first.get('two_step_stage_dispatch_requested') is True:
            reasons.append('two_step_stage_dispatch_requested=true')
        if first.get('pending_corridor_alignment_second_step'):
            reasons.append('pending second-step relationship was recorded or inferred, but Phase134 does not attempt it')
        if classification == 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP':
            reasons.append('bounded staging dispatch had acceptance/result evidence and the smoke stopped before any second-step explore')
        elif classification == 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL':
            reasons.append('bounded staging observation timed out or reported timeout; this remains diagnostic')
        elif classification == 'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL':
            reasons.append('bounded staging dispatch was rejected/aborted/canceled; this remains diagnostic')
        elif classification == 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP':
            reasons.append('staging dispatch was observed, and second-step explore was intentionally not attempted')
    else:
        reasons.append(f'first literal dispatch goal_kind={goal_kind!r} is ambiguous under the Phase132 contract')
    if artifact.get('second_step_attempted') is False and artifact.get('second_goal_dispatched') is False:
        reasons.append('second_step_attempted=false and second_goal_dispatched=false')
    return reasons


def _staging_relationship(first: dict[str, Any]) -> dict[str, Any]:
    return {
        'original_target': _value(first.get('original_target'), 'first_literal_dispatch.original_target'),
        'refined_target': _value(first.get('refined_target'), 'first_literal_dispatch.refined_target'),
        'staging_target': _value(first.get('staging_target'), 'first_literal_dispatch.staging_target'),
        'staging_applied': _value(first.get('staging_applied'), 'first_literal_dispatch.staging_applied'),
        'two_step_stage_dispatch_requested': _value(first.get('two_step_stage_dispatch_requested'), 'first_literal_dispatch.two_step_stage_dispatch_requested'),
        'staging_reason': _value(first.get('staging_reason'), 'first_literal_dispatch.staging_reason'),
        'lateral_residual_before_m': _value(first.get('lateral_residual_before_m'), 'first_literal_dispatch.lateral_residual_before_m'),
        'lateral_residual_after_m': _value(first.get('lateral_residual_after_m'), 'first_literal_dispatch.lateral_residual_after_m'),
        'front_wedge_risk': _value(first.get('front_wedge_risk'), 'first_literal_dispatch.front_wedge_risk'),
        'staging_executability_check': _value(first.get('staging_executability_check'), 'first_literal_dispatch.staging_executability_check'),
        'pending_corridor_alignment_second_step': _value(first.get('pending_corridor_alignment_second_step'), 'first_literal_dispatch.pending_corridor_alignment_second_step'),
        'second_step_forward_goal': _value(first.get('second_step_forward_goal'), 'first_literal_dispatch.second_step_forward_goal'),
    }


def _guardrails(artifact: dict[str, Any]) -> dict[str, bool]:
    source = _dict(artifact.get('guardrails'))
    first = _dict(artifact.get('first_literal_dispatch'))
    guardrails = {
        'max_goals_one': source.get('max_goals_one') is True and artifact.get('maze_explorer_max_goals') == 1,
        'first_literal_dispatch_only': source.get('first_literal_dispatch_only') is True and artifact.get('dispatch_event_count', 0) <= 1,
        'second_step_attempted_false': source.get('second_step_attempted_false') is True and artifact.get('second_step_attempted') is False and first.get('second_step_attempted') is False,
        'second_goal_dispatched_false': source.get('second_goal_dispatched_false') is True and artifact.get('second_goal_dispatched') is False and first.get('second_goal_dispatched') is False,
        'no_manual_goal1_or_related_goal': source.get('no_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_manual_goal') is True,
        'nav2_config_not_changed': source.get('nav2_config_changed') is False,
        'strategy_not_changed': source.get('branch_scoring_changed') is False and source.get('centerline_gate_changed') is False and source.get('fallback_changed') is False and source.get('terminal_acceptance_changed') is False,
        'direct_staging_not_disabled': source.get('direct_staging_disablement') is False,
        'no_autonomous_success_claim': source.get('no_autonomous_success_claim') is True,
        'no_exit_success_claim': source.get('no_exit_success_claim') is True,
    }
    return guardrails


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    first = _dict(artifact.get('first_literal_dispatch'))
    classification = _runner.classify_bounded_record(_dict(artifact.get('bounded_staging_record')) or artifact)
    if artifact.get('classification') in CLASSIFICATIONS and artifact.get('classification') != classification:
        # Prefer the artifact classification only if it is a guard violation that
        # the runner cannot reconstruct from a stripped top-level artifact.
        classification = str(artifact.get('classification'))
    guardrails = _guardrails(artifact)
    coherent_staging = _runner._staging_fields_are_coherent(first) if first.get('goal_kind') == 'corridor_alignment_staging' else True
    classification_in_vocab = classification in CLASSIFICATIONS
    guardrails_ok = all(guardrails.values())
    valid = bool(classification_in_vocab and guardrails_ok and (coherent_staging or classification == 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'))
    if first.get('goal_kind') == 'corridor_alignment_staging' and not coherent_staging:
        classification = 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
        valid = False
    result = {
        'phase': PHASE,
        'mode': MODE,
        'valid': valid,
        'classification': classification,
        'classification_reasons': _classification_reasons(artifact, classification),
        'created_wall_time_sec': time.time(),
        'schema': {
            'classification_vocabulary': CLASSIFICATIONS,
            'contract_source': 'Phase132 corridor alignment staging contract and Phase134 bounded smoke scope',
        },
        'first_literal_dispatch': {
            'available': bool(first.get('available')),
            'goal_kind': first.get('goal_kind'),
            'goal_sequence': first.get('goal_sequence'),
            'is_first_exploration_goal': bool(first.get('is_first_exploration_goal')),
            'accepted': bool(first.get('accepted')),
            'rejected': bool(first.get('rejected')),
            'timeout': bool(first.get('timeout')),
            'result_status_label': first.get('result_status_label'),
            'abort_text': first.get('abort_text'),
        },
        'staging_relationship': _staging_relationship(first),
        'second_step_policy': {
            'second_step_attempted': bool(artifact.get('second_step_attempted')),
            'second_goal_dispatched': bool(artifact.get('second_goal_dispatched')),
            'attempt_allowed_by_phase134': False,
            'policy': 'Phase134 stops before any second-step goal_kind=explore or second exploration goal',
        },
        'guardrails': guardrails,
        'evidence_counts': {
            'goal_event_count': artifact.get('goal_event_count'),
            'dispatch_event_count': artifact.get('dispatch_event_count'),
            'explorer_state_count': len(_list(artifact.get('explorer_states'))),
            'nav2_feedback_count': len(_list(artifact.get('nav2_feedback'))),
            'action_status_sample_count': len(_list(artifact.get('action_status_samples'))),
            'cmd_vel_sample_count': len(_list(artifact.get('cmd_vel_timeline'))),
            'odom_velocity_sample_count': len(_list(artifact.get('odom_velocity_timeline'))),
        },
        'claims': {
            'staging_is_exploration_success': False,
            'staging_is_exit_success': False,
            'mixed_with_phase127_timeout_local_cost_replay': False,
            'autonomous_exploration_success': False,
            'exit_success': False,
        },
        'artifact_stop_reason': artifact.get('stop_reason'),
    }
    return result


def analyze_path(path: Path) -> dict[str, Any]:
    return analyze_artifact(_read_json(path))


def _render_summary(result: dict[str, Any]) -> str:
    first = result['first_literal_dispatch']
    rel = result['staging_relationship']
    guard = result['guardrails']
    lines = [
        '# Phase134 bounded corridor-alignment staging smoke summary',
        '',
        f"Classification: `{result['classification']}`",
        f"Valid: `{result['valid']}`",
        '',
        '## First literal dispatch',
        '',
        f"- goal_kind: `{first.get('goal_kind')}`",
        f"- accepted: `{first.get('accepted')}`",
        f"- rejected: `{first.get('rejected')}`",
        f"- timeout: `{first.get('timeout')}`",
        f"- result_status_label: `{first.get('result_status_label')}`",
        '',
        '## Staging contract fields',
        '',
        f"- original_target: `{rel['original_target']['value']}`",
        f"- refined_target: `{rel['refined_target']['value']}`",
        f"- staging_target: `{rel['staging_target']['value']}`",
        f"- staging_applied: `{rel['staging_applied']['value']}`",
        f"- two_step_stage_dispatch_requested: `{rel['two_step_stage_dispatch_requested']['value']}`",
        f"- staging_reason: `{rel['staging_reason']['value']}`",
        f"- lateral_residual_before_m: `{rel['lateral_residual_before_m']['value']}`",
        f"- lateral_residual_after_m: `{rel['lateral_residual_after_m']['value']}`",
        f"- front_wedge_risk: `{rel['front_wedge_risk']['value']}`",
        f"- staging_executability_check: `{rel['staging_executability_check']['value']}`",
        f"- pending_corridor_alignment_second_step: `{rel['pending_corridor_alignment_second_step']['value']}`",
        f"- second_step_forward_goal: `{rel['second_step_forward_goal']['value']}`",
        '',
        '## Guardrails',
        '',
    ]
    lines.extend(f'- {key}: `{value}`' for key, value in guard.items())
    lines.extend([
        '',
        '## Classification reasons',
        '',
    ])
    lines.extend(f'- {reason}' for reason in result['classification_reasons'])
    lines.extend([
        '',
        '## Contract assertions',
        '',
        '- second_step_attempted=false is required.',
        '- second_goal_dispatched=false is required.',
        '- No second-step goal_kind=explore is allowed in Phase134.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED is not mixed into this staging-contract smoke.',
        '',
    ])
    return '\n'.join(lines)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Analyze Phase134 bounded staging-contract smoke artifact')
    parser.add_argument('--artifact', type=Path, default=DEFAULT_ARTIFACT)
    parser.add_argument('--output-json', type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument('--output-md', type=Path, default=DEFAULT_OUTPUT_MD)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    result = analyze_path(args.artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    args.output_md.write_text(_render_summary(result), encoding='utf-8')
    print(json.dumps({'classification': result['classification'], 'valid': result['valid'], 'artifact': str(args.artifact), 'analysis': str(args.output_json)}, sort_keys=True))
    return 0 if result['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
