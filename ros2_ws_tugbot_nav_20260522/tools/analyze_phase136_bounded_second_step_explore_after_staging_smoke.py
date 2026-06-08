#!/usr/bin/env python3
"""Analyze Phase136 bounded second-step explore after staging smoke artifacts."""
from __future__ import annotations

import argparse
import importlib.util
import json
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase136'
MODE = 'bounded_second_step_explore_after_staging_smoke_analysis'
CLASSIFICATIONS = [
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP',
    'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
    'SECOND_STEP_CONTRACT_AMBIGUOUS',
]
DEFAULT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT = DEFAULT_ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke.json'
DEFAULT_OUTPUT_JSON = DEFAULT_ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke_analysis.json'
DEFAULT_OUTPUT_MD = DEFAULT_ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke_summary.md'


def _load_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase136_bounded_second_step_explore_after_staging_smoke.py'
    spec = importlib.util.spec_from_file_location('phase136_runner_for_analysis', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase136 runner from {path}')
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
    return {'available': value is not None, 'value': value if value is not None else 'missing', 'source': source}


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _finite_pair(value: Any) -> bool:
    return _runner._finite_pair(value)


def _classification_reasons(artifact: dict[str, Any], classification: str) -> list[str]:
    first = _dict(artifact.get('first_literal_dispatch'))
    second = _dict(artifact.get('second_step_dispatch'))
    reasons: list[str] = []
    if first.get('goal_kind') == 'corridor_alignment_staging':
        reasons.append('first literal dispatch was goal_kind=corridor_alignment_staging')
        if first.get('result_status_label') == 'SUCCEEDED':
            reasons.append('staging result_status_label=SUCCEEDED')
    else:
        reasons.append(f"first literal dispatch goal_kind={first.get('goal_kind')!r} is not the required staging dispatch")
    if second.get('available'):
        reasons.append(f"second-step dispatch goal_kind={second.get('goal_kind')!r}")
        if second.get('recursion_guard') is True:
            reasons.append('second-step recursion guard present')
        if _dict(second.get('second_step_forward_goal')).get('generated_after_fresh_evidence') is True:
            reasons.append('second_step_forward_goal.generated_after_fresh_evidence=true')
    else:
        reasons.append('no second-step dispatch observed; fail closed if staging succeeded')
    if artifact.get('third_goal_dispatched') is True:
        reasons.append('third goal dispatch guard violation')
    if classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP':
        reasons.append('bounded smoke stopped at second-step acceptance without claiming success')
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP':
        reasons.append('bounded smoke observed a second-step Nav2 success result and stopped')
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL':
        reasons.append('second-step explore was rejected/aborted/canceled; diagnostic failure')
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL':
        reasons.append('second-step explore timed out; diagnostic failure')
    elif classification == 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED':
        reasons.append('staging succeeded but second-step readiness was not proved; no replacement goal permitted')
    elif classification == 'SECOND_STEP_CONTRACT_AMBIGUOUS':
        reasons.append('Phase136 second-step contract or guardrail was ambiguous/violated')
    return reasons


def _staging_relationship(first: dict[str, Any]) -> dict[str, Any]:
    return {
        'original_target': _value(first.get('original_target'), 'first_literal_dispatch.original_target'),
        'refined_target': _value(first.get('refined_target'), 'first_literal_dispatch.refined_target'),
        'staging_target': _value(first.get('staging_target'), 'first_literal_dispatch.staging_target'),
        'staging_applied': _value(first.get('staging_applied'), 'first_literal_dispatch.staging_applied'),
        'two_step_stage_dispatch_requested': _value(first.get('two_step_stage_dispatch_requested'), 'first_literal_dispatch.two_step_stage_dispatch_requested'),
        'staging_reason': _value(first.get('staging_reason'), 'first_literal_dispatch.staging_reason'),
        'staging_executability_check': _value(first.get('staging_executability_check'), 'first_literal_dispatch.staging_executability_check'),
        'front_wedge_risk_before_staging': _value(first.get('front_wedge_risk'), 'first_literal_dispatch.front_wedge_risk'),
        'lateral_residual_after': _value(first.get('lateral_residual_after_m'), 'first_literal_dispatch.lateral_residual_after_m'),
        'staging_result_status_label': _value(first.get('result_status_label'), 'first_literal_dispatch.result_status_label'),
    }


def _second_step_relationship(artifact: dict[str, Any]) -> dict[str, Any]:
    second = _dict(artifact.get('second_step_dispatch'))
    payload = _dict(second.get('second_step_forward_goal'))
    return {
        'pending_corridor_alignment_second_step': _value(artifact.get('pending_corridor_alignment_second_step'), 'artifact.pending_corridor_alignment_second_step'),
        'freshness_after_staging': _value(artifact.get('freshness_after_staging'), 'artifact.freshness_after_staging'),
        'front_wedge_risk_after_staging': _value(artifact.get('front_wedge_risk_after_staging'), 'artifact.front_wedge_risk_after_staging'),
        'lateral_residual_after': _value(artifact.get('lateral_residual_after'), 'artifact.lateral_residual_after'),
        'second_step_forward_goal': _value(second.get('second_step_forward_goal'), 'second_step_dispatch.second_step_forward_goal'),
        'generated_after_fresh_evidence': _value(payload.get('generated_after_fresh_evidence'), 'second_step_forward_goal.generated_after_fresh_evidence'),
        'selected_candidate_target': _value(payload.get('selected_candidate_target'), 'second_step_forward_goal.selected_candidate_target'),
        'goal_kind': _value(second.get('goal_kind'), 'second_step_dispatch.goal_kind'),
        'skip_two_step_staging': _value(second.get('skip_two_step_staging'), 'second_step_dispatch.skip_two_step_staging'),
        'recursion_guard': _value(second.get('recursion_guard'), 'second_step_dispatch.recursion_guard'),
        'accepted': _value(second.get('accepted'), 'second_step_dispatch.accepted'),
        'rejected': _value(second.get('rejected'), 'second_step_dispatch.rejected'),
        'timeout': _value(second.get('timeout'), 'second_step_dispatch.timeout'),
        'result_status_label': _value(second.get('result_status_label'), 'second_step_dispatch.result_status_label'),
        'abort_text': _value(second.get('abort_text'), 'second_step_dispatch.abort_text'),
    }


def _guardrails(artifact: dict[str, Any]) -> dict[str, bool]:
    source = _dict(artifact.get('guardrails'))
    first = _dict(artifact.get('first_literal_dispatch'))
    second = _dict(artifact.get('second_step_dispatch'))
    payload = _dict(second.get('second_step_forward_goal'))
    second_available = bool(second.get('available'))
    guardrails = {
        'max_goals_two': source.get('max_goals_two') is True and artifact.get('maze_explorer_max_goals') == 2,
        'staging_and_second_step_only': source.get('staging_and_second_step_only') is True and int(artifact.get('dispatch_event_count') or 0) <= 2,
        'one_second_step_goal_only': source.get('one_second_step_goal_only') is True and int(artifact.get('second_step_goal_count') or 0) <= 1,
        'third_goal_dispatched_false': source.get('third_goal_dispatched_false') is True and artifact.get('third_goal_dispatched') is False,
        'staging_succeeded_before_second_step': source.get('staging_succeeded_before_second_step') is True and _runner._staging_succeeded(first),
        'pending_second_step_present': source.get('pending_second_step_present') is True,
        'freshness_after_staging_recorded': source.get('freshness_after_staging_recorded') is True,
        'second_step_goal_kind_explore': source.get('second_step_goal_kind_explore') is True and ((not second_available) or second.get('goal_kind') == 'explore'),
        'second_step_recursion_guard_present': source.get('second_step_recursion_guard_present') is True and ((not second_available) or second.get('recursion_guard') is True),
        'second_step_generated_after_fresh_evidence': source.get('second_step_generated_after_fresh_evidence') is True and ((not second_available) or payload.get('generated_after_fresh_evidence') is True),
        'second_step_selected_candidate_target_present': source.get('second_step_selected_candidate_target_present') is True and ((not second_available) or _finite_pair(payload.get('selected_candidate_target'))),
        'no_manual_goal1_or_related_goal': source.get('no_goal1_carry_over_branch_centerline_fallback_terminal_exit_manual_goal') is True,
        'nav2_config_not_changed': source.get('nav2_config_changed') is False,
        'strategy_not_changed': source.get('branch_scoring_changed') is False and source.get('centerline_gate_changed') is False and source.get('fallback_changed') is False and source.get('terminal_acceptance_changed') is False,
        'direct_staging_not_disabled': source.get('direct_staging_disablement') is False,
        'no_autonomous_success_claim': source.get('no_autonomous_success_claim') is True,
        'no_exit_success_claim': source.get('no_exit_success_claim') is True,
        'no_phase127_timeout_fixed_claim': source.get('phase127_timeout_fixed_claimed') is False,
    }
    return guardrails


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    classification = _runner.classify_bounded_record(_dict(artifact.get('bounded_second_step_record')) or artifact)
    guardrails = _guardrails(artifact)
    if not all(guardrails.values()):
        classification = 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    classification_in_vocab = classification in CLASSIFICATIONS
    valid = bool(classification_in_vocab and all(guardrails.values()))
    result = {
        'phase': PHASE,
        'mode': MODE,
        'valid': valid,
        'classification': classification,
        'classification_reasons': _classification_reasons(artifact, classification),
        'created_wall_time_sec': time.time(),
        'schema': {
            'classification_vocabulary': CLASSIFICATIONS,
            'contract_source': 'Phase135 second-step explore after staging design and Phase136 bounded runtime scope',
        },
        'first_literal_dispatch': {
            'available': bool(_dict(artifact.get('first_literal_dispatch')).get('available')),
            'goal_kind': _dict(artifact.get('first_literal_dispatch')).get('goal_kind'),
            'goal_sequence': _dict(artifact.get('first_literal_dispatch')).get('goal_sequence'),
            'accepted': bool(_dict(artifact.get('first_literal_dispatch')).get('accepted')),
            'rejected': bool(_dict(artifact.get('first_literal_dispatch')).get('rejected')),
            'timeout': bool(_dict(artifact.get('first_literal_dispatch')).get('timeout')),
            'result_status_label': _dict(artifact.get('first_literal_dispatch')).get('result_status_label'),
        },
        'staging_relationship': _staging_relationship(_dict(artifact.get('first_literal_dispatch'))),
        'second_step_relationship': _second_step_relationship(artifact),
        'second_step_policy': {
            'second_step_attempted': bool(artifact.get('second_step_attempted')),
            'second_step_goal_count': artifact.get('second_step_goal_count'),
            'third_goal_dispatched': bool(artifact.get('third_goal_dispatched')),
            'policy': 'Phase136 permits exactly one second-step goal_kind=explore after staging succeeded and stops at accepted/rejected/timeout/result',
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
            'second_step_is_autonomous_exploration_success': False,
            'second_step_is_exit_success': False,
            'phase127_timeout_fixed': False,
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
    rel = result['second_step_relationship']
    lines = [
        '# Phase136 bounded second-step explore after staging smoke summary',
        '',
        f"Classification: `{result['classification']}`",
        f"Valid: `{result['valid']}`",
        '',
        '## First literal staging dispatch',
        '',
        f"- goal_kind: `{first.get('goal_kind')}`",
        f"- result_status_label: `{first.get('result_status_label')}`",
        '',
        '## Second-step dispatch',
        '',
        f"- goal_kind: `{rel['goal_kind']['value']}`",
        f"- generated_after_fresh_evidence: `{rel['generated_after_fresh_evidence']['value']}`",
        f"- selected_candidate_target: `{rel['selected_candidate_target']['value']}`",
        f"- recursion_guard: `{rel['recursion_guard']['value']}`",
        f"- result_status_label: `{rel['result_status_label']['value']}`",
        f"- abort_text: `{rel['abort_text']['value']}`",
        '',
        '## Guardrails',
        '',
    ]
    for key, value in result['guardrails'].items():
        lines.append(f'- {key}: `{value}`')
    lines.extend([
        '',
        '## Boundary claims',
        '',
        '- No autonomous exploration success is claimed.',
        '- No exit success is claimed.',
        '- Phase127 timeout fixed claim is false.',
        '- Phase137 not entered.',
        '',
    ])
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, default=DEFAULT_ARTIFACT)
    parser.add_argument('--output-json', type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument('--output-md', type=Path, default=DEFAULT_OUTPUT_MD)
    args = parser.parse_args()
    result = analyze_path(args.artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    args.output_md.write_text(_render_summary(result), encoding='utf-8')
    print(json.dumps({
        'classification': result.get('classification'),
        'valid': result.get('valid'),
        'artifact': str(args.artifact),
        'output_json': str(args.output_json),
        'output_md': str(args.output_md),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
