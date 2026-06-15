#!/usr/bin/env python3
"""Phase106 preflighted carry-over bounded Goal1 staging validation analyzer.

Read-only analyzer for the Phase105-preflighted rerun of the Phase102 wrapper.
It preserves the Phase102 Goal1 field extraction, but emits Phase106-specific
classifications that distinguish preflight reject, ingress failure after a pass,
and actual Goal1 carry-over/staging evidence.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUN_ID = 'phase106_preflighted_carry_over_bounded_goal1_staging_validation'
ROOT = Path(__file__).resolve().parents[1]
PHASE102_ANALYZER = ROOT / 'tools' / 'analyze_phase102_carry_over_bounded_goal1_staging_validation.py'

ALLOWED_CLASSIFICATIONS = {
    'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED',
    'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
    'PREFLIGHTED_CARRY_OVER_REJECTED',
    'PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED',
    'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS',
    'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS',
    'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
}

CLASSIFICATION_MAP = {
    'CARRY_OVER_APPLIED_STAGING_APPLIED': 'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED',
    'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED': 'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
    'CARRY_OVER_REJECTED': 'PREFLIGHTED_CARRY_OVER_REJECTED',
    'CARRY_OVER_NOT_TRIGGERED': 'PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED',
    'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE': 'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
}

JSONDict = dict[str, Any]


def _load_phase102():
    spec = importlib.util.spec_from_file_location('phase102_analyzer_for_phase106', PHASE102_ANALYZER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load {PHASE102_ANALYZER}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _dict(value: Any) -> JSONDict:
    return value if isinstance(value, dict) else {}


def _boolish(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y', 'passed', 'succeeded'}
    return bool(value)


def _ingress_success(ingress: JSONDict) -> bool:
    status = str(ingress.get('status') or ingress.get('status_text') or '').lower()
    return bool(ingress.get('success') is True or status in {'succeeded', 'status_succeeded'} or status.endswith('succeeded'))


def _preflight_passed(preflight: JSONDict) -> bool | None:
    if 'passed' in preflight:
        return _boolish(preflight.get('passed'))
    if 'status' in preflight:
        status = str(preflight.get('status') or '').lower()
        if status in {'passed', 'pass', 'ok'}:
            return True
        if status in {'rejected', 'failed', 'timeout'}:
            return False
    return None


def _ingress_goal_sent(preflight: JSONDict, ingress: JSONDict) -> bool:
    if 'ingress_goal_sent' in preflight:
        return bool(_boolish(preflight.get('ingress_goal_sent')))
    if 'ingress_goal_sent' in ingress:
        return bool(_boolish(ingress.get('ingress_goal_sent')))
    return bool(ingress and str(ingress.get('status') or '').lower() != 'preflight_rejected')


def _only_after_preflight_pass(preflight_pass: bool | None, sent: bool) -> bool:
    if sent:
        return preflight_pass is True
    if preflight_pass is False:
        return True
    return True


def _corridor_level_only(goal1: JSONDict) -> bool:
    if not goal1.get('carry_over_applied'):
        return False
    if goal1.get('safety_evidence_recomputed') is not True:
        return False
    source = _dict(goal1.get('carry_over_source'))
    source_window = _dict(goal1.get('source_forward_window'))
    check = _dict(goal1.get('staging_executability_check'))
    # The positive evidence should be corridor evidence from Phase88/source window;
    # hard safety remains recomputed locally and may fail independently.
    has_corridor_source = bool(
        source.get('two_side_wall_evidence') is True
        or source_window.get('two_side_wall_count') not in (None, 0)
    )
    safety_not_inferred_from_source = not bool(source.get('hard_safety_pass') is True and check.get('hard_safety_pass') is not True)
    return bool(has_corridor_source and safety_not_inferred_from_source)


def _phase106_classification(base: JSONDict, preflight_pass: bool | None, ingress_ok: bool, sent: bool) -> str:
    trigger = _dict(base.get('trigger'))
    trigger_name = str(trigger.get('trigger') or '')
    preflight = _dict(base.get('ingress_preflight'))
    if preflight_pass is False or trigger_name == 'ingress_preflight_rejected_explorer_not_started':
        return 'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS'
    if preflight_pass is True and sent and not ingress_ok:
        return 'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS'
    return CLASSIFICATION_MAP.get(str(base.get('classification')), 'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE')


def analyze_artifact_dir(artifact_dir: Path, run_id: str = RUN_ID) -> JSONDict:
    phase102 = _load_phase102()
    base = phase102.analyze_artifact_dir(artifact_dir, run_id=run_id)
    preflight = _dict(base.get('ingress_preflight'))
    ingress = _dict(base.get('ingress'))
    goal1 = _dict(base.get('goal1'))
    preflight_pass = _preflight_passed(preflight)
    ingress_ok = _ingress_success(ingress)
    sent = _ingress_goal_sent(preflight, ingress)
    classification = _phase106_classification(base, preflight_pass, ingress_ok, sent)

    evidence_gaps = list(base.get('evidence_gaps') or [])
    if classification == 'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS':
        if 'phase105_preflight_rejected_before_ingress' not in evidence_gaps:
            evidence_gaps.insert(0, 'phase105_preflight_rejected_before_ingress')
    elif classification == 'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS':
        if 'ingress_failed_after_preflight_pass' not in evidence_gaps:
            evidence_gaps.insert(0, 'ingress_failed_after_preflight_pass')

    check = _dict(goal1.get('staging_executability_check'))
    staging_window = _dict(goal1.get('staging_window'))
    direct_answers = dict(base.get('direct_answers') or {})
    direct_answers.update({
        'phase105_preflight_passed': preflight_pass,
        'ingress_goal_sent_only_after_preflight_pass': _only_after_preflight_pass(preflight_pass, sent),
        'ingress_success': ingress_ok,
        'goal1_dispatched': bool(goal1.get('dispatch_observed')),
        'missing_two_side_wall_fixed_by_carry_over': bool(
            goal1.get('carry_over_applied')
            and check.get('two_side_wall_evidence') is True
            and (check.get('local_two_side_wall_evidence') is False or staging_window.get('two_side_wall_count') == 0)
        ),
        'carry_over_is_corridor_level_only': _corridor_level_only(goal1),
        'safety_evidence_recomputed_is_true': bool(goal1.get('safety_evidence_recomputed')),
        'staging_reject_reason_after_carry_over': goal1.get('staging_reject_reason'),
        'staging_safety_recompute_failed': goal1.get('staging_reject_reason') == 'staging_safety_recompute_failed',
    })

    result = dict(base)
    result.update({
        'run_id': run_id,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'evidence_gaps': evidence_gaps,
        'direct_answers': direct_answers,
        'ingress_goal_sent': sent,
        'maze_explorer_started': bool(_boolish(preflight.get('maze_explorer_started'))),
        'ingress_preflight_reject_reason': preflight.get('ingress_preflight_reject_reason') or ingress.get('ingress_preflight_reject_reason'),
        'guardrails': {
            'algorithm_changed': False,
            'phase88_92_101_105_logic_changed': False,
            'branch_scoring_changed': bool(goal1.get('branch_scoring_changed')),
            'fallback_terminal_acceptance_used': bool(goal1.get('fallback_terminal_acceptance_used')),
            'nav2_config_changed': False,
            'no_success_claimed': True,
            'no_exit_success_claimed': True,
            'phase107_entered': False,
        },
    })
    return result


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    goal1 = _dict(result.get('goal1'))
    direct = _dict(result.get('direct_answers'))
    lines = [
        '# Phase106 minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Artifact dir: {result.get('artifact_dir')}",
        f"Ingress preflight reject reason: {result.get('ingress_preflight_reject_reason')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        '',
        'Preflight/ingress:',
        f"- phase105_preflight_passed: {direct.get('phase105_preflight_passed')}",
        f"- ingress_goal_sent_only_after_preflight_pass: {direct.get('ingress_goal_sent_only_after_preflight_pass')}",
        f"- ingress_success: {direct.get('ingress_success')}",
        f"- goal1_dispatched: {direct.get('goal1_dispatched')}",
        '',
        'Goal1 carry-over/staging:',
        f"- corridor_evidence_carry_over: {goal1.get('corridor_evidence_carry_over')}",
        f"- carry_over_source: {goal1.get('carry_over_source')}",
        f"- carry_over_applied: {goal1.get('carry_over_applied')}",
        f"- carry_over_reject_reason: {goal1.get('carry_over_reject_reason')}",
        f"- source_forward_window: {goal1.get('source_forward_window')}",
        f"- staging_window: {goal1.get('staging_window')}",
        f"- safety_evidence_recomputed: {goal1.get('safety_evidence_recomputed')}",
        f"- two_step_staging_plan: {goal1.get('two_step_staging_plan')}",
        f"- staging_executability_check: {goal1.get('staging_executability_check')}",
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
        'Guardrails: no algorithm/config tuning, no autonomous/exit success claim, Phase107 not entered.',
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
