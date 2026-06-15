#!/usr/bin/env python3
"""Analyze Phase139 instrumented second-step contract runtime artifacts.

Phase139 is a bounded runtime contract verification pass only.  It verifies that
Phase138's artifact/serialization fields are present in a real visible-stack
run, and it never converts second-step accepted/succeeded evidence into
autonomous exploration success, exit success, or a Phase127 timeout fix.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase139'
MODE = 'instrumented_second_step_contract_runtime_verification_analysis'
CLASSIFICATIONS = [
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_ACCEPTED_STOP',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
    'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS',
    'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
]
DEFAULT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = DEFAULT_ROOT / 'log' / 'phase139_instrumented_second_step_contract_runtime_verification'
DEFAULT_ARTIFACT = DEFAULT_LOG_DIR / 'phase139_instrumented_second_step_contract_runtime_verification.json'
DEFAULT_OUTPUT_JSON = DEFAULT_LOG_DIR / 'phase139_instrumented_second_step_contract_runtime_verification_analysis.json'
DEFAULT_OUTPUT_MD = DEFAULT_LOG_DIR / 'phase139_instrumented_second_step_contract_runtime_verification_summary.md'


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite_pair(value: Any) -> bool:
    if isinstance(value, dict):
        xs = [value.get('x'), value.get('y')]
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        xs = [value[0], value[1]]
    else:
        return False
    return all(_finite(v) for v in xs)


def _finite_pose_or_target(value: Any) -> bool:
    if not _finite_pair(value):
        return False
    if isinstance(value, dict):
        yaw = value.get('yaw')
        return yaw is None or _finite(yaw)
    return True


def _value(value: Any, source: str) -> dict[str, Any]:
    return {
        'available': value is not None,
        'value': value if value is not None else 'missing',
        'source': source,
    }


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _latest_second_dispatch_from_events(artifact: dict[str, Any]) -> dict[str, Any]:
    dispatches = [
        event for event in _list(artifact.get('goal_events'))
        if isinstance(event, dict) and str(event.get('event', '')).lower() == 'dispatch'
    ]
    for event in dispatches[1:]:
        if event.get('goal_kind') == 'explore':
            return event
    return {}


def _second_dispatch(artifact: dict[str, Any]) -> dict[str, Any]:
    second = _dict(artifact.get('second_step_dispatch'))
    raw = _latest_second_dispatch_from_events(artifact)
    merged = dict(second)
    for key, value in raw.items():
        if key not in merged or merged.get(key) in (None, {}, [], False):
            merged[key] = value
    # Prefer Phase138 raw dispatch context for fields that Phase136 normalization
    # intentionally collapsed before Phase138 existed.
    for key in [
        'pending_corridor_alignment_second_step', 'second_step_forward_goal',
        'skip_two_step_staging', 'phase138_recursion_guard', 'phase136_recursion_guard',
        'recursion_guard', 'two_step_stage_dispatch_requested', 'staging_applied',
        'prior_staging_applied', 'freshness_after_staging', 'front_wedge_risk_after_staging',
        'lateral_residual_after',
    ]:
        if key in raw:
            merged[key] = raw[key]
    return merged


def _pending_snapshot(artifact: dict[str, Any], second: dict[str, Any]) -> dict[str, Any]:
    pending = _dict(artifact.get('pending_corridor_alignment_second_step'))
    if pending.get('runtime_serialized') is True:
        return pending
    second_pending = _dict(second.get('pending_corridor_alignment_second_step'))
    if second_pending:
        return second_pending
    return pending


def _freshness(artifact: dict[str, Any], second: dict[str, Any], payload: dict[str, Any], pending: dict[str, Any]) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    for source in [_dict(artifact.get('freshness_after_staging')), _dict(second.get('freshness_after_staging')), pending, payload]:
        for key in [
            'fresh_scan_received', 'fresh_local_costmap_received', 'fresh_tf_received',
            'fresh_scan_sample_time_sec', 'fresh_local_costmap_sample_time_sec', 'fresh_tf_sample_time_sec',
            'generation_wall_time_sec',
        ]:
            if key in source and source.get(key) is not None:
                merged[key] = source.get(key)
    return merged


def _timestamp_ordering(freshness: dict[str, Any], payload: dict[str, Any], pending: dict[str, Any]) -> dict[str, Any]:
    generation = payload.get('generation_wall_time_sec') or freshness.get('generation_wall_time_sec')
    samples = [
        freshness.get('fresh_scan_sample_time_sec') or payload.get('fresh_scan_sample_time_sec') or pending.get('fresh_scan_sample_time_sec'),
        freshness.get('fresh_local_costmap_sample_time_sec') or payload.get('fresh_local_costmap_sample_time_sec') or pending.get('fresh_local_costmap_sample_time_sec'),
        freshness.get('fresh_tf_sample_time_sec') or payload.get('fresh_tf_sample_time_sec') or pending.get('fresh_tf_sample_time_sec'),
    ]
    finite_samples = [_finite(v) for v in samples]
    generation_value = float(generation) if _finite(generation) else None
    ordered = bool(
        generation_value is not None
        and all(finite_samples)
        and all(generation_value >= float(v) for v in samples if _finite(v))
    )
    return {
        'generation_wall_time_sec': _value(generation, 'second_step_forward_goal.generation_wall_time_sec'),
        'fresh_sample_times': _value(samples, 'freshness_after_staging sample timestamps'),
        'generated_after_all_fresh_evidence': _value(ordered, 'generation_wall_time_sec >= all fresh sample timestamps'),
    }


def _first_staging_succeeded(artifact: dict[str, Any]) -> bool:
    first = _dict(artifact.get('first_literal_dispatch'))
    staging = _dict(artifact.get('staging_result'))
    return bool(
        first.get('goal_kind') == 'corridor_alignment_staging'
        and (first.get('result_status_label') == 'SUCCEEDED' or staging.get('result_status_label') == 'SUCCEEDED')
        and first.get('rejected') is not True
        and first.get('timeout') is not True
    )


def _runtime_contract(artifact: dict[str, Any]) -> tuple[dict[str, Any], dict[str, bool]]:
    second = _second_dispatch(artifact)
    payload = _dict(second.get('second_step_forward_goal'))
    pending = _pending_snapshot(artifact, second)
    freshness = _freshness(artifact, second, payload, pending)
    front = _dict(payload.get('front_wedge_risk_after_staging')) or _dict(second.get('front_wedge_risk_after_staging')) or _dict(artifact.get('front_wedge_risk_after_staging'))
    lateral = payload.get('lateral_residual_after')
    if lateral is None:
        lateral = second.get('lateral_residual_after', artifact.get('lateral_residual_after'))
    timestamp = _timestamp_ordering(freshness, payload, pending)
    target = payload.get('selected_candidate_target')
    yaw = payload.get('selected_candidate_yaw')
    explicit_xy_yaw = True
    if payload.get('x') is not None or payload.get('y') is not None or payload.get('yaw') is not None:
        explicit_xy_yaw = _finite(payload.get('x')) and _finite(payload.get('y')) and _finite(payload.get('yaw'))
    contract = {
        'pending_corridor_alignment_second_step': {
            'exists': _value(pending.get('exists'), 'pending_corridor_alignment_second_step.exists'),
            'runtime_serialized': _value(pending.get('runtime_serialized'), 'pending_corridor_alignment_second_step.runtime_serialized'),
            'original_goal_kind': _value(pending.get('original_goal_kind'), 'pending_corridor_alignment_second_step.original_goal_kind'),
            'original_target': _value(pending.get('original_target'), 'pending_corridor_alignment_second_step.original_target'),
            'direction_rad': _value(pending.get('direction_rad'), 'pending_corridor_alignment_second_step.direction_rad'),
            'start_node_id': _value(pending.get('start_node_id'), 'pending_corridor_alignment_second_step.start_node_id'),
            'active_branch': _value(pending.get('active_branch'), 'pending_corridor_alignment_second_step.active_branch'),
            'staging_goal_pose': _value(pending.get('staging_goal_pose'), 'pending_corridor_alignment_second_step.staging_goal_pose'),
            'staging_plan': _value(pending.get('staging_plan'), 'pending_corridor_alignment_second_step.staging_plan'),
            'staging_result_status_label': _value(pending.get('staging_result_status_label'), 'pending_corridor_alignment_second_step.staging_result_status_label'),
            'staging_succeeded_wall_time_sec': _value(pending.get('staging_succeeded_wall_time_sec'), 'pending_corridor_alignment_second_step.staging_succeeded_wall_time_sec'),
        },
        'freshness_after_staging': {
            'fresh_scan_received': _value(freshness.get('fresh_scan_received'), 'freshness_after_staging.fresh_scan_received'),
            'fresh_local_costmap_received': _value(freshness.get('fresh_local_costmap_received'), 'freshness_after_staging.fresh_local_costmap_received'),
            'fresh_tf_received': _value(freshness.get('fresh_tf_received'), 'freshness_after_staging.fresh_tf_received'),
            'fresh_scan_sample_time_sec': _value(freshness.get('fresh_scan_sample_time_sec'), 'freshness_after_staging.fresh_scan_sample_time_sec'),
            'fresh_local_costmap_sample_time_sec': _value(freshness.get('fresh_local_costmap_sample_time_sec'), 'freshness_after_staging.fresh_local_costmap_sample_time_sec'),
            'fresh_tf_sample_time_sec': _value(freshness.get('fresh_tf_sample_time_sec'), 'freshness_after_staging.fresh_tf_sample_time_sec'),
        },
        'second_step_forward_goal': {
            'valid': _value(payload.get('valid'), 'second_step_forward_goal.valid'),
            'map_frame_id': _value(payload.get('map_frame_id'), 'second_step_forward_goal.map_frame_id'),
            'selected_candidate_target': _value(target, 'second_step_forward_goal.selected_candidate_target'),
            'selected_candidate_yaw': _value(yaw, 'second_step_forward_goal.selected_candidate_yaw'),
            'finite_x_y_yaw': _value(bool(_finite_pair(target) and _finite(yaw) and explicit_xy_yaw), 'second_step_forward_goal finite x/y/yaw'),
            'generated_after_fresh_evidence': _value(payload.get('generated_after_fresh_evidence'), 'second_step_forward_goal.generated_after_fresh_evidence'),
            'front_wedge_risk_after_staging': _value(front, 'second_step_forward_goal.front_wedge_risk_after_staging'),
            'lateral_residual_after': _value(lateral, 'second_step_forward_goal.lateral_residual_after'),
            'candidate_count': _value(payload.get('candidate_count'), 'second_step_forward_goal.candidate_count'),
            'hard_safety_pass_candidate_count': _value(payload.get('hard_safety_pass_candidate_count'), 'second_step_forward_goal.hard_safety_pass_candidate_count'),
            'selected_candidate_index': _value(payload.get('selected_candidate_index'), 'second_step_forward_goal.selected_candidate_index'),
            'selection_priority_trace': _value(payload.get('selection_priority_trace'), 'second_step_forward_goal.selection_priority_trace'),
            'rejected_candidate_summaries': _value(payload.get('rejected_candidate_summaries'), 'second_step_forward_goal.rejected_candidate_summaries'),
        },
        'timestamp_ordering': timestamp,
        'outgoing_second_step_artifact': {
            'skip_two_step_staging': _value(second.get('skip_two_step_staging'), 'second_step_dispatch.skip_two_step_staging'),
            'recursion_guard': _value(second.get('recursion_guard') or second.get('phase138_recursion_guard') or second.get('phase136_recursion_guard'), 'second_step_dispatch recursion guard'),
            'two_step_stage_dispatch_requested': _value(second.get('two_step_stage_dispatch_requested'), 'second_step_dispatch.two_step_stage_dispatch_requested'),
            'staging_applied': _value(second.get('staging_applied'), 'second_step_dispatch.staging_applied'),
            'prior_staging_applied': _value(second.get('prior_staging_applied'), 'second_step_dispatch.prior_staging_applied'),
        },
    }
    checks = {
        'staging_succeeded': _first_staging_succeeded(artifact),
        'second_step_available': bool(second.get('available') or second.get('goal_kind') == 'explore'),
        'second_step_goal_kind_explore': second.get('goal_kind') == 'explore',
        'pending_exists': pending.get('exists') is True,
        'pending_runtime_serialized': pending.get('runtime_serialized') is True,
        'pending_origin_fields_present': bool(
            pending.get('original_goal_kind') == 'explore'
            and pending.get('original_target') is not None
            and _finite(pending.get('direction_rad'))
            and pending.get('start_node_id') is not None
            and isinstance(pending.get('active_branch'), dict)
            and isinstance(pending.get('staging_goal_pose'), dict)
            and isinstance(pending.get('staging_plan'), dict)
        ),
        'staging_result_status_and_time_present': bool(
            pending.get('staging_result_status_label') == 'SUCCEEDED'
            and _finite(pending.get('staging_succeeded_wall_time_sec'))
        ),
        'fresh_scan_local_costmap_tf_booleans_true': bool(
            freshness.get('fresh_scan_received') is True
            and freshness.get('fresh_local_costmap_received') is True
            and freshness.get('fresh_tf_received') is True
        ),
        'fresh_sample_timestamps_present': bool(
            _finite(freshness.get('fresh_scan_sample_time_sec'))
            and _finite(freshness.get('fresh_local_costmap_sample_time_sec'))
            and _finite(freshness.get('fresh_tf_sample_time_sec'))
        ),
        'second_step_forward_goal_valid': payload.get('valid') is True,
        'second_step_map_frame_map': payload.get('map_frame_id') == 'map',
        'second_step_selected_candidate_finite': bool(_finite_pair(target) and _finite(yaw) and explicit_xy_yaw),
        'generated_after_fresh_evidence_true': payload.get('generated_after_fresh_evidence') is True,
        'timestamp_ordering_generated_after_fresh_evidence': timestamp['generated_after_all_fresh_evidence']['value'] is True,
        'front_wedge_risk_after_staging_present': isinstance(front, dict) and bool(front) and front.get('max') is not None,
        'lateral_residual_after_present': _finite(lateral),
        'candidate_diagnostics_present': bool(
            payload.get('candidate_count') is not None
            and payload.get('hard_safety_pass_candidate_count') is not None
            and payload.get('selected_candidate_index') is not None
            and isinstance(payload.get('selection_priority_trace'), list)
            and isinstance(payload.get('rejected_candidate_summaries'), list)
        ),
        'outgoing_recursion_guard_present': bool(second.get('skip_two_step_staging') is True or second.get('recursion_guard') is True or second.get('phase138_recursion_guard') is True or second.get('phase136_recursion_guard') is True),
        'outgoing_no_second_staging_request': second.get('two_step_stage_dispatch_requested') is False,
        'outgoing_staging_not_reapplied': second.get('staging_applied') is False,
        'outgoing_prior_staging_applied': second.get('prior_staging_applied') is True,
        'third_goal_dispatched_false': artifact.get('third_goal_dispatched') is False,
        'max_goals_two': artifact.get('maze_explorer_max_goals') == 2,
        'one_second_step_goal_only': artifact.get('second_step_goal_count') == 1,
        'dispatch_count_bounded': int(artifact.get('dispatch_event_count') or 0) <= 2,
        'no_success_claims': _dict(artifact.get('claims')).get('autonomous_exploration_success') is False and _dict(artifact.get('claims')).get('exit_success') is False and _dict(artifact.get('claims')).get('phase127_timeout_fixed') is False,
    }
    return contract, checks


def _classify(artifact: dict[str, Any], checks: dict[str, bool]) -> str:
    second = _second_dispatch(artifact)
    if checks.get('staging_succeeded') and not checks.get('second_step_available'):
        return 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED'
    if not all(checks.values()):
        return 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'
    label = second.get('result_status_label')
    if _bool(second.get('timeout')) or label == 'TIMEOUT':
        return 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL'
    if _bool(second.get('rejected')) or label in {'REJECTED', 'ABORTED', 'CANCELED', 'UNKNOWN'}:
        return 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_REJECTED_DIAGNOSTIC_FAIL'
    if label == 'SUCCEEDED':
        return 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP'
    if _bool(second.get('accepted')) or label == 'ACCEPTED':
        return 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_ACCEPTED_STOP'
    return 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    contract, checks = _runtime_contract(artifact)
    classification = _classify(artifact, checks)
    valid = bool(classification in CLASSIFICATIONS and classification != 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS' and all(checks.values()))
    if classification == 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED':
        valid = False
    return {
        'phase': PHASE,
        'mode': MODE,
        'valid': valid,
        'classification': classification,
        'classification_vocabulary': CLASSIFICATIONS,
        'created_wall_time_sec': time.time(),
        'contract_checks': checks,
        'missing_or_failed_checks': [key for key, ok in checks.items() if not ok],
        'runtime_contract': contract,
        'second_step_policy': {
            'second_step_attempted': bool(artifact.get('second_step_attempted')),
            'second_step_goal_count': artifact.get('second_step_goal_count'),
            'third_goal_dispatched': bool(artifact.get('third_goal_dispatched')),
            'policy': 'Phase139 permits only staging plus one second-step explore goal and stops before any third goal/full exploration',
        },
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


def analyze_path(path: Path) -> dict[str, Any]:
    return analyze_artifact(_read_json(path))


def _render_summary(result: dict[str, Any]) -> str:
    lines = [
        '# Phase139 instrumented second-step contract runtime verification summary',
        '',
        f"Classification: `{result['classification']}`",
        f"Valid: `{result['valid']}`",
        '',
        '## Contract checks',
        '',
    ]
    for key, value in result['contract_checks'].items():
        lines.append(f'- {key}: `{value}`')
    lines.extend([
        '',
        '## Boundary claims',
        '',
        '- autonomous exploration success: `False`',
        '- exit success: `False`',
        '- Phase127 timeout fixed: `False`',
        '',
    ])
    return '\n'.join(lines) + '\n'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact', nargs='?', type=Path, default=DEFAULT_ARTIFACT)
    parser.add_argument('--output-json', type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument('--output-md', type=Path, default=DEFAULT_OUTPUT_MD)
    args = parser.parse_args()
    result = analyze_path(args.artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    args.output_md.write_text(_render_summary(result), encoding='utf-8')
    print(json.dumps({'classification': result['classification'], 'valid': result['valid'], 'output_json': str(args.output_json)}, sort_keys=True))
    return 0 if result['classification'] in CLASSIFICATIONS else 1


if __name__ == '__main__':
    raise SystemExit(main())
