#!/usr/bin/env python3
"""Phase133 corridor alignment staging contract artifact replay.

Offline analyzer only. It reads existing Phase129 and Phase131 artifacts plus the
existing maze_explorer stderr log, then classifies the Phase129 first literal
staging dispatch under the Phase132 contract vocabulary.

This file intentionally has no runtime launch or goal-sending path.
"""
from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase133'
MODE = 'corridor_alignment_staging_contract_replay'
MISSING = 'missing'

CLASSIFICATIONS = [
    'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
    'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
    'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
]

DEFAULT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PHASE129_ARTIFACT = DEFAULT_ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun.json'
DEFAULT_PHASE131_ANALYSIS = DEFAULT_ROOT / 'log' / 'phase131_first_dispatch_kind_artifact_replay' / 'phase131_first_dispatch_kind_artifact_replay_analysis.json'
DEFAULT_PHASE129_STDERR = DEFAULT_ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun_maze_explorer_stderr.log'
DEFAULT_OUTPUT_JSON = DEFAULT_ROOT / 'log' / 'phase133_corridor_alignment_staging_contract_replay' / 'phase133_corridor_alignment_staging_contract_replay_analysis.json'
DEFAULT_OUTPUT_MD = DEFAULT_ROOT / 'log' / 'phase133_corridor_alignment_staging_contract_replay' / 'phase133_corridor_alignment_staging_contract_replay_summary.md'


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _read_text_if_exists(path: Path) -> str:
    if path.exists():
        return path.read_text(encoding='utf-8', errors='replace')
    return ''


def _safe_get(obj: Any, path: list[Any], default: Any = None) -> Any:
    cur = obj
    for key in path:
        if isinstance(cur, dict):
            if key not in cur:
                return default
            cur = cur[key]
        elif isinstance(cur, list) and isinstance(key, int) and 0 <= key < len(cur):
            cur = cur[key]
        else:
            return default
    return cur


def _value_field(value: Any, source: str | None = None) -> dict[str, Any]:
    available = value is not None
    return {
        'available': available,
        'value': value if available else MISSING,
        'source': source,
    }


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _first_dispatch_event(artifact: dict[str, Any]) -> dict[str, Any]:
    for path in (
        ['first_goal_result_artifact', 'goal_events'],
        ['goal_events'],
        ['first_goal_artifact', 'goal_events'],
    ):
        events = _safe_get(artifact, path, [])
        if isinstance(events, list):
            for event in events:
                if isinstance(event, dict) and event.get('event') == 'dispatch':
                    return event
    return {}


def _find_result_events(artifact: dict[str, Any]) -> list[dict[str, Any]]:
    events = _safe_get(artifact, ['first_goal_result_artifact', 'goal_events'], [])
    if not isinstance(events, list):
        return []
    return [event for event in events if isinstance(event, dict) and event.get('event') in {'success', 'failure', 'timeout', 'cancel'}]


def _status_samples_after_dispatch(artifact: dict[str, Any], dispatch_time: float | None) -> list[dict[str, Any]]:
    samples = _safe_get(artifact, ['first_goal_result_artifact', 'action_status_samples'], [])
    if not isinstance(samples, list):
        return []
    if dispatch_time is None:
        return [sample for sample in samples if isinstance(sample, dict)]
    after: list[dict[str, Any]] = []
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        stamp = _finite_number(sample.get('received_wall_time_sec'))
        if stamp is not None and stamp >= dispatch_time:
            after.append(sample)
    return after


def _stderr_evidence(text: str) -> dict[str, Any]:
    send_line = None
    for line in text.splitlines():
        if 'sending corridor_alignment_staging goal' in line:
            send_line = line
            break
    parsed = None
    if send_line:
        m = re.search(r'goal #(\d+) seq=(\d+): x=([-0-9.]+) y=([-0-9.]+) yaw=([-0-9.]+)', send_line)
        if m:
            parsed = {
                'goal_number': int(m.group(1)),
                'sequence_id': int(m.group(2)),
                'x': float(m.group(3)),
                'y': float(m.group(4)),
                'yaw': float(m.group(5)),
            }
    return {
        'available': bool(send_line),
        'send_line': send_line,
        'parsed_send_line': parsed,
        'external_shutdown_seen': 'ExternalShutdownException' in text,
        'shutdown_error_seen': 'failed to shutdown' in text,
    }


def _field_matrix(dispatch: dict[str, Any]) -> dict[str, Any]:
    pose_any = dispatch.get('staging_goal_pose')
    pose: dict[str, Any] = pose_any if isinstance(pose_any, dict) else {}
    check_any = dispatch.get('staging_executability_check')
    check: dict[str, Any] = check_any if isinstance(check_any, dict) else {}
    front_wedge = dispatch.get('phase62_front_wedge_cost')
    return {
        'original_target': _value_field(dispatch.get('original_target'), 'dispatch.original_target'),
        'refined_target': _value_field(dispatch.get('refined_target'), 'dispatch.refined_target'),
        'staging_target': _value_field(dispatch.get('target'), 'dispatch.target'),
        'staging_goal_pose': _value_field(dispatch.get('staging_goal_pose'), 'dispatch.staging_goal_pose'),
        'staging_applied': _value_field(dispatch.get('staging_applied'), 'dispatch.staging_applied'),
        'two_step_stage_dispatch_requested': _value_field(dispatch.get('two_step_stage_dispatch_requested'), 'dispatch.two_step_stage_dispatch_requested'),
        'staging_reason': _value_field(dispatch.get('staging_reason'), 'dispatch.staging_reason'),
        'staging_reject_reason': _value_field(dispatch.get('staging_reject_reason'), 'dispatch.staging_reject_reason'),
        'staging_lateral_residual_before_m': _value_field(pose.get('lateral_residual_before_m'), 'dispatch.staging_goal_pose.lateral_residual_before_m'),
        'staging_lateral_residual_after_m': _value_field(pose.get('lateral_residual_after_m'), 'dispatch.staging_goal_pose.lateral_residual_after_m'),
        'staging_distance_m': _value_field(pose.get('staging_distance_m'), 'dispatch.staging_goal_pose.staging_distance_m'),
        'front_wedge_risk': _value_field(front_wedge, 'dispatch.phase62_front_wedge_cost'),
        'staging_executability_check': _value_field(dispatch.get('staging_executability_check'), 'dispatch.staging_executability_check'),
        'hard_safety_pass': _value_field(check.get('hard_safety_pass'), 'dispatch.staging_executability_check.hard_safety_pass'),
        'lateral_residual_reduced': _value_field(check.get('lateral_residual_reduced'), 'dispatch.staging_executability_check.lateral_residual_reduced'),
        'source_forward_window': _value_field(dispatch.get('source_forward_window'), 'dispatch.source_forward_window'),
        'staging_window': _value_field(dispatch.get('staging_window'), 'dispatch.staging_window'),
        'target_local_cost_max_radius': _value_field(dispatch.get('dispatch_target_local_cost_max_radius'), 'dispatch.dispatch_target_local_cost_max_radius'),
        'path_corridor_min_clearance_m': _value_field(dispatch.get('path_corridor_min_clearance_m'), 'dispatch.path_corridor_min_clearance_m'),
    }


def _actual_explore_dispatch_after_first(artifact: dict[str, Any]) -> bool:
    events = _safe_get(artifact, ['first_goal_result_artifact', 'goal_events'], [])
    if not isinstance(events, list):
        return False
    dispatches = [event for event in events if isinstance(event, dict) and event.get('event') == 'dispatch']
    return any(event.get('goal_kind') == 'explore' for event in dispatches[1:])


def _pending_second_step_evidence(artifact: dict[str, Any], dispatch: dict[str, Any]) -> dict[str, Any]:
    # Phase129 artifacts preserve the plan fields, but they do not serialize the
    # node's in-memory pending relationship object after first dispatch.
    pending = _safe_get(artifact, ['first_goal_result_artifact', 'pending_corridor_alignment_second_step'])
    if pending is None:
        pending = _safe_get(artifact, ['pending_corridor_alignment_second_step'])
    second_step = dispatch.get('second_step_forward_goal')
    actual_explore_after_first = _actual_explore_dispatch_after_first(artifact)
    return {
        'pending_corridor_alignment_second_step': _value_field(pending, 'artifact.pending_corridor_alignment_second_step'),
        'second_step_forward_goal': _value_field(second_step, 'dispatch.second_step_forward_goal'),
        'second_goal_dispatched': _value_field(_safe_get(artifact, ['first_goal_result_artifact', 'second_goal_dispatched']), 'first_goal_result_artifact.second_goal_dispatched'),
        'legacy_exploration_goal_dispatched': _value_field(_safe_get(artifact, ['first_goal_result_artifact', 'exploration_goal_dispatched']), 'first_goal_result_artifact.exploration_goal_dispatched'),
        'actual_explore_dispatch_after_first': _value_field(actual_explore_after_first, 'goal_events after first dispatch'),
        'stop_reason': _value_field(_safe_get(artifact, ['first_goal_result_artifact', 'stop_reason']), 'first_goal_result_artifact.stop_reason'),
    }


def _staging_fields_are_coherent(dispatch: dict[str, Any]) -> bool:
    return (
        dispatch.get('goal_kind') == 'corridor_alignment_staging'
        and dispatch.get('staging_applied') is True
        and dispatch.get('two_step_stage_dispatch_requested') is True
        and dispatch.get('original_target') is not None
        and dispatch.get('target') is not None
        and dispatch.get('staging_reason') is not None
        and isinstance(dispatch.get('staging_goal_pose'), dict)
    )


def _nav_outcome_evidence(artifact: dict[str, Any], dispatch: dict[str, Any]) -> dict[str, Any]:
    dispatch_time = _finite_number(dispatch.get('received_wall_time_sec'))
    result_events = _find_result_events(artifact)
    status_after = _status_samples_after_dispatch(artifact, dispatch_time)
    first_goal = artifact.get('first_goal') if isinstance(artifact.get('first_goal'), dict) else {}
    legacy_label = first_goal.get('result_status_label')
    legacy_rejection_is_contract_guard = legacy_label == 'REJECTED_NON_EXPLORE_GOAL_KIND'
    result_summary = dispatch.get('phase62_nav2_result_summary')
    feedback_summary = dispatch.get('phase62_nav2_feedback_summary')
    acceptance_available = bool(result_events or status_after or result_summary)
    return {
        'dispatch_wall_time_sec': dispatch_time,
        'result_events_after_dispatch': result_events,
        'status_samples_after_dispatch': status_after,
        'phase62_nav2_result_summary': _value_field(result_summary, 'dispatch.phase62_nav2_result_summary'),
        'phase62_nav2_feedback_summary': _value_field(feedback_summary, 'dispatch.phase62_nav2_feedback_summary'),
        'nav2_acceptance_evidence': _value_field('present' if acceptance_available else None, 'result/status evidence after dispatch'),
        'nav2_rejection_evidence': _value_field(None, 'no Nav2 rejection event after staging dispatch'),
        'legacy_non_explore_rejection_is_contract_guard': legacy_rejection_is_contract_guard,
    }


def _phase131_context(phase131: dict[str, Any]) -> dict[str, Any]:
    return {
        'classification': phase131.get('classification'),
        'phase': phase131.get('phase'),
        'mode': phase131.get('mode'),
        'dispatch_kind_sequence': phase131.get('dispatch_kind_sequence'),
        'diagnosis_summary': phase131.get('diagnosis_summary'),
    }


def _classify(*, dispatch: dict[str, Any], artifact: dict[str, Any], nav: dict[str, Any], pending: dict[str, Any], coherent: bool) -> tuple[str, list[str], bool]:
    reasons: list[str] = []
    goal_kind = dispatch.get('goal_kind')
    second_dispatched = _safe_get(artifact, ['first_goal_result_artifact', 'second_goal_dispatched']) is True
    actual_explore_after_first = pending['actual_explore_dispatch_after_first']['value'] is True
    observation_timed_out = _safe_get(artifact, ['first_goal_result_artifact', 'observation_timed_out']) is True

    if goal_kind == 'explore':
        reasons.append('first literal dispatch is goal_kind=explore under the Phase132 vocabulary')
        return 'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP', reasons, True

    if goal_kind != 'corridor_alignment_staging':
        reasons.append(f'first literal dispatch goal_kind={goal_kind!r} is neither explore nor corridor_alignment_staging')
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS', reasons, False

    reasons.append('Phase129 first literal dispatch is corridor_alignment_staging')
    if not coherent:
        reasons.append('required staging relationship fields are missing or inconsistent')
        return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS', reasons, False

    reasons.append('staging fields are coherent: original target, staging target, staging_applied, two_step request, and staging_reason are present')
    if nav.get('legacy_non_explore_rejection_is_contract_guard'):
        reasons.append('legacy Phase129 REJECTED_NON_EXPLORE_GOAL_KIND is a first-explore replay guard, not Nav2 rejection of the staging dispatch')

    pending_available = bool(pending['pending_corridor_alignment_second_step']['available'])
    second_step_available = bool(pending['second_step_forward_goal']['available'])
    if not second_dispatched and not actual_explore_after_first:
        reasons.append('second-step explore goal was not attempted in the bounded Phase129 artifact')
        if not pending_available:
            reasons.append('pending_corridor_alignment_second_step is not serialized in the Phase129 artifact and remains explicit missing evidence')
        if not second_step_available:
            reasons.append('second_step_forward_goal is missing in the Phase129 dispatch event')
        return 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP', reasons, True

    if observation_timed_out:
        reasons.append('bounded observation timed out after a staging dispatch')
        return 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL', reasons, True

    if nav['nav2_rejection_evidence']['available']:
        reasons.append('Nav2 rejection evidence exists after the staging dispatch')
        return 'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL', reasons, True

    if nav['nav2_acceptance_evidence']['available']:
        reasons.append('staging dispatch has acceptance/result evidence and the smoke stops at staging outcome')
        return 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP', reasons, True

    reasons.append('staging dispatch is coherent but Nav2 acceptance/rejection/result evidence is absent')
    return 'DISPATCH_KIND_CONTRACT_AMBIGUOUS', reasons, False


def analyze_artifacts(*, phase129_artifact_path: Path | str, phase131_analysis_path: Path | str, phase129_stderr_path: Path | str) -> dict[str, Any]:
    phase129_path = Path(phase129_artifact_path)
    phase131_path = Path(phase131_analysis_path)
    stderr_path = Path(phase129_stderr_path)
    phase129 = _read_json(phase129_path)
    phase131 = _read_json(phase131_path)
    stderr_text = _read_text_if_exists(stderr_path)

    dispatch = _first_dispatch_event(phase129)
    coherent = _staging_fields_are_coherent(dispatch)
    pending = _pending_second_step_evidence(phase129, dispatch)
    nav = _nav_outcome_evidence(phase129, dispatch)
    classification, reasons, valid = _classify(dispatch=dispatch, artifact=phase129, nav=nav, pending=pending, coherent=coherent)

    first_goal_any = phase129.get('first_goal')
    first_goal: dict[str, Any] = first_goal_any if isinstance(first_goal_any, dict) else {}
    fgra_any = phase129.get('first_goal_result_artifact')
    fgra: dict[str, Any] = fgra_any if isinstance(fgra_any, dict) else {}

    result = {
        'phase': PHASE,
        'mode': MODE,
        'valid': valid,
        'classification': classification,
        'classification_reasons': reasons,
        'created_wall_time_sec': time.time(),
        'schema': {
            'classification_vocabulary': CLASSIFICATIONS,
            'contract_source': 'Phase132 corridor alignment staging contract design',
            'artifact_sources': {
                'phase129_artifact': str(phase129_path),
                'phase131_analysis': str(phase131_path),
                'phase129_stderr': str(stderr_path),
            },
        },
        'first_literal_dispatch': {
            'available': bool(dispatch),
            'goal_kind': dispatch.get('goal_kind'),
            'goal_sequence': dispatch.get('goal_sequence'),
            'target': dispatch.get('target'),
            'original_target': dispatch.get('original_target'),
            'refined_target': dispatch.get('refined_target'),
            'received_wall_time_sec': dispatch.get('received_wall_time_sec'),
            'is_first_exploration_goal': dispatch.get('goal_kind') == 'explore',
            'stderr_evidence': _stderr_evidence(stderr_text),
        },
        'first_exploration_goal': {
            'attempted': bool(pending['actual_explore_dispatch_after_first']['value'] is True),
            'legacy_phase129_exploration_goal_dispatched_flag': bool(fgra.get('exploration_goal_dispatched')),
            'second_step_dispatched': bool(fgra.get('second_goal_dispatched')),
            'evidence_source': 'goal_events after first dispatch plus first_goal_result_artifact.second_goal_dispatched',
            'note': 'A corridor_alignment_staging dispatch is not the first exploration goal under the Phase132 contract; the Phase129 legacy exploration_goal_dispatched flag means a goal was sent, not that a later explore dispatch existed.',
        },
        'staging_relationship': _field_matrix(dispatch),
        'pending_second_step_evidence': pending,
        'nav2_outcome_evidence': nav,
        'legacy_phase129_contract': {
            'result_status_label': first_goal.get('result_status_label'),
            'abort_text': first_goal.get('abort_text'),
            'stop_reason': fgra.get('stop_reason'),
            'interpretation': 'Phase129 was written as first-explore timeout instrumentation and fail-closed on non-explore dispatch; Phase133 replays it under the Phase132 staging contract.',
        },
        'phase131_context': _phase131_context(phase131),
        'guardrails': {
            'runtime_started_by_phase133': False,
            'goal_sent_by_phase133': False,
            'maze_explorer_started_by_phase133': False,
            'nav2_or_controller_config_tuned': False,
            'exploration_strategy_changed': False,
            'phase127_timeout_replay_modified': False,
            'phase134_entered': False,
        },
        'claims': {
            'staging_is_exploration_success': False,
            'staging_is_exit_success': False,
            'mixed_with_phase127_timeout_local_cost_replay': False,
            'first_literal_staging_is_first_exploration_goal': False,
            'autonomous_exploration_success': False,
            'exit_success': False,
        },
        'phase134_boundary': {
            'allowed_if_runtime_is_later_authorized': 'bounded staging-contract smoke only',
            'forbidden': [
                'full autonomous exploration',
                'direct staging disablement',
                'Nav2 parameter tuning',
                'treating staging accepted/succeeded as exploration success or exit success',
            ],
        },
    }
    return result


def _render_summary(result: dict[str, Any]) -> str:
    rel = result['staging_relationship']
    pending = result['pending_second_step_evidence']
    lines = [
        '# Phase133 corridor alignment staging contract replay summary',
        '',
        f"Classification: `{result['classification']}`",
        '',
        '## First literal dispatch',
        '',
        f"- goal_kind: `{result['first_literal_dispatch'].get('goal_kind')}`",
        f"- is first exploration goal: `{result['first_literal_dispatch'].get('is_first_exploration_goal')}`",
        f"- stderr send line present: `{result['first_literal_dispatch']['stderr_evidence'].get('available')}`",
        '',
        '## Staging relationship evidence',
        '',
        f"- staging_applied: `{rel['staging_applied']['value']}`",
        f"- two_step_stage_dispatch_requested: `{rel['two_step_stage_dispatch_requested']['value']}`",
        f"- staging_reason: `{rel['staging_reason']['value']}`",
        f"- original_target: `{rel['original_target']['value']}`",
        f"- staging_target: `{rel['staging_target']['value']}`",
        f"- second_step_forward_goal: `{pending['second_step_forward_goal']['value']}`",
        f"- pending_corridor_alignment_second_step: `{pending['pending_corridor_alignment_second_step']['value']}`",
        '',
        '## Classification reasons',
        '',
    ]
    lines.extend(f'- {reason}' for reason in result['classification_reasons'])
    lines.extend([
        '',
        '## Contract assertions',
        '',
        '- Staging is not exploration success.',
        '- Staging is not exit success.',
        '- Staging is not Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED.',
        '- The Phase129 first literal staging dispatch is not treated as the first exploration goal.',
        '',
        '## Phase134 boundary',
        '',
        'If Phase134 performs runtime after human acceptance, it may only be a bounded staging-contract smoke. It must not run full autonomous exploration, must not tune Nav2 parameters, and must not disable staging.',
        '',
        'Phase134 not entered by this replay.',
        '',
    ])
    return '\n'.join(lines)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Offline Phase133 corridor alignment staging contract artifact replay analyzer')
    parser.add_argument('--phase129-artifact', type=Path, default=DEFAULT_PHASE129_ARTIFACT)
    parser.add_argument('--phase131-analysis', type=Path, default=DEFAULT_PHASE131_ANALYSIS)
    parser.add_argument('--phase129-stderr', type=Path, default=DEFAULT_PHASE129_STDERR)
    parser.add_argument('--output-json', type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument('--output-md', type=Path, default=DEFAULT_OUTPUT_MD)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    result = analyze_artifacts(
        phase129_artifact_path=args.phase129_artifact,
        phase131_analysis_path=args.phase131_analysis,
        phase129_stderr_path=args.phase129_stderr,
    )
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    args.output_md.write_text(_render_summary(result), encoding='utf-8')
    print(json.dumps({'analysis': str(args.output_json), 'summary': str(args.output_md), 'classification': result['classification'], 'valid': result['valid']}, sort_keys=True))
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
