#!/usr/bin/env python3
"""Phase110 synthetic replay analyzer for Phase109 ingress preflight artifacts.

This is offline-only tooling. It constructs synthetic Phase109 preflight artifacts,
replays their artifact contract, and writes a JSON analysis plus a human-readable
minimal summary. It must not start Gazebo/RViz/SLAM/Nav2/maze_explorer, send an
ingress goal, rerun Phase106, tune Nav2/controller config, or alter exploration
strategy.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

RUN_ID = 'phase110_preflight_synthetic_replay_contract'
ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_PATH = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
DEFAULT_OUTPUT_DIR = ROOT / 'log' / RUN_ID


def _load_preflight():
    spec = importlib.util.spec_from_file_location('phase105_preflight_for_phase110', PREFLIGHT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load {PREFLIGHT_PATH}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _missing_scan() -> dict[str, Any]:
    return {
        'scan_available': False,
        'scan_frame_id': None,
        'target_frame': 'map',
        'transform_available': False,
        'stable': False,
        'cache_drop_detected': False,
        'exception_count': 0,
    }


def _controller_with_lifecycle(module: Any, state: str) -> dict[str, Any]:
    if state == 'active_confirmed':
        controller_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 0, 'state': 'active'},
            node_graph_present=True,
            action_available=True,
        )
        bt_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 0, 'state': 'active'},
            node_graph_present=True,
            action_available=True,
        )
        action_ready = True
        active = True
    elif state == 'inactive_confirmed':
        controller_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 0, 'state': 'inactive'},
            node_graph_present=False,
            action_available=False,
        )
        bt_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 0, 'state': 'inactive'},
            node_graph_present=False,
            action_available=False,
        )
        action_ready = False
        active = False
    elif state == 'ambiguous':
        controller_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
            node_graph_present=True,
            action_available=True,
        )
        bt_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
            node_graph_present=True,
            action_available=True,
        )
        action_ready = True
        active = False
    elif state == 'query_error':
        controller_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
            node_graph_present=False,
            action_available=False,
        )
        bt_state = module.derive_lifecycle_confirmation(
            lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
            node_graph_present=False,
            action_available=False,
        )
        action_ready = False
        active = False
    else:
        raise ValueError(f'unknown lifecycle state {state}')
    return {
        'controller_server_active': active,
        'bt_navigator_active': active,
        'navigate_to_pose_action_ready': action_ready,
        'robot_pose_available': True,
        'robot_pose_unavailable_log_count': 0,
        'lifecycle_sources': {
            'controller_server': controller_state,
            'bt_navigator': bt_state,
        },
    }


def _sample(module: Any, **overrides: Any) -> dict[str, Any]:
    sample = module.make_synthetic_sample()
    sample.update(overrides)
    return sample


def _artifact_from_samples(module: Any, samples: list[dict[str, Any]], config: Any) -> dict[str, Any]:
    result = module.evaluate_preflight_samples(samples, config)
    return result.to_artifact()


def generate_synthetic_replay_artifacts(output_dir: Path) -> dict[str, Path]:
    """Generate Phase110 synthetic preflight artifacts and return case path map."""
    module = _load_preflight()
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts: dict[str, Path] = {}

    def write_case(name: str, artifact: dict[str, Any]) -> None:
        path = output_dir / f'{RUN_ID}_{name}.json'
        preflight = artifact.setdefault('ingress_preflight', {})
        preflight['phase110_case'] = name
        preflight['phase110_synthetic_replay'] = True
        _write_json(path, artifact)
        artifacts[name] = path

    lifecycle_config = module.PreflightConfig(startup_grace_sec=0.0, tf_stability_window_sec=0.0, timeout_sec=1.0)
    for state in ['active_confirmed', 'inactive_confirmed', 'ambiguous', 'query_error']:
        case = f'lifecycle_{state}' if state != 'query_error' else 'lifecycle_query_error'
        # Avoid duplicating confirmed in the name for active/inactive.
        case = case.replace('_confirmed_confirmed', '_confirmed')
        write_case(
            case,
            _artifact_from_samples(
                module,
                [_sample(module, elapsed_sec=0.0, controller_pose_check=_controller_with_lifecycle(module, state))],
                lifecycle_config,
            ),
        )

    scan_grace_config = module.PreflightConfig(startup_grace_sec=2.0, tf_stability_window_sec=1.0, timeout_sec=3.0)
    write_case(
        'first_scan_waiting_only',
        _artifact_from_samples(
            module,
            [_sample(module, elapsed_sec=0.0, scan_transform_check=_missing_scan())],
            scan_grace_config,
        ),
    )
    write_case(
        'first_scan_timeout',
        _artifact_from_samples(
            module,
            [
                _sample(module, elapsed_sec=0.0, scan_transform_check=_missing_scan()),
                _sample(module, elapsed_sec=1.0, scan_transform_check=_missing_scan()),
                _sample(module, elapsed_sec=2.5, scan_transform_check=_missing_scan()),
            ],
            scan_grace_config,
        ),
    )

    tf_config = module.PreflightConfig(startup_grace_sec=0.0, tf_stability_window_sec=1.0, timeout_sec=3.0)
    write_case(
        'early_tf_miss_recovered',
        _artifact_from_samples(
            module,
            [
                _sample(module, elapsed_sec=0.0, map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}),
                _sample(module, elapsed_sec=0.5),
                _sample(module, elapsed_sec=1.6),
            ],
            tf_config,
        ),
    )
    write_case(
        'stable_window_not_met',
        _artifact_from_samples(
            module,
            [
                _sample(module, elapsed_sec=0.0, map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}),
                _sample(module, elapsed_sec=0.5),
                _sample(module, elapsed_sec=0.9),
            ],
            tf_config,
        ),
    )
    write_case(
        'raw_snapshot_contradiction',
        _artifact_from_samples(
            module,
            [
                _sample(
                    module,
                    elapsed_sec=0.0,
                    map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None},
                    raw_style_snapshot_override={
                        'present': True,
                        'scan_available': True,
                        'map_available': True,
                        'local_costmap_available': True,
                        'odom_available': True,
                        'tf_pairs_available': {'map->base_link': True, 'map->odom': True, 'odom->base_link': True},
                        'navigate_to_pose_action_ready': True,
                        'lifecycle_node_graph_present': {'controller_server': True, 'bt_navigator': True},
                    },
                )
            ],
            tf_config,
        ),
    )
    return artifacts


def _case_name(path: Path, preflight: dict[str, Any]) -> str:
    if preflight.get('phase110_case'):
        return str(preflight['phase110_case'])
    stem = path.stem
    prefix = f'{RUN_ID}_'
    return stem[len(prefix):] if stem.startswith(prefix) else stem


def _sample_tokens(preflight: dict[str, Any]) -> list[list[str]]:
    return [list(sample.get('failed_gates') or []) for sample in preflight.get('sample_history') or []]


def _lifecycle_state(preflight: dict[str, Any]) -> str | None:
    sample_history = preflight.get('sample_history') or []
    source = None
    if sample_history:
        source = ((sample_history[0].get('lifecycle_sources') or {}).get('controller_server') or {})
    if not source:
        source = (((preflight.get('controller_pose_check') or {}).get('lifecycle_sources') or {}).get('controller_server') or {})
    return source.get('state')


def _classify_case(name: str, preflight: dict[str, Any]) -> str:
    reject_reason = preflight.get('ingress_preflight_reject_reason')
    passed = bool(preflight.get('passed'))
    tokens = set(preflight.get('failed_gates') or [])
    sample_tokens = _sample_tokens(preflight)
    raw = preflight.get('raw_style_snapshot_cross_check') or {}

    if name.startswith('lifecycle_'):
        return f"LIFECYCLE_{str(_lifecycle_state(preflight) or 'unknown').upper()}_REPLAYED"
    if name == 'first_scan_waiting_only' and sample_tokens and sample_tokens[0] == ['waiting_for_first_scan'] and reject_reason != 'ingress_first_scan_timeout':
        return 'WAITING_FOR_FIRST_SCAN_OBSERVED_NO_FINAL_SCAN_TIMEOUT'
    if reject_reason == 'ingress_first_scan_timeout':
        return 'INGRESS_FIRST_SCAN_TIMEOUT_REPLAYED'
    if name == 'early_tf_miss_recovered' and passed:
        return 'EARLY_TF_MISS_RECOVERED_STABLE_WINDOW_PASS'
    if reject_reason == 'ingress_tf_stable_window_not_met':
        return 'INGRESS_TF_STABLE_WINDOW_NOT_MET_REPLAYED'
    if raw.get('ambiguous') and 'ingress_raw_snapshot_cross_check_failed' in tokens and not passed:
        return 'RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED'
    return 'PHASE110_SYNTHETIC_REPLAY_UNCLASSIFIED'


def _case_summary(name: str, preflight: dict[str, Any]) -> dict[str, Any]:
    sample_history = list(preflight.get('sample_history') or [])
    reject_reason = preflight.get('ingress_preflight_reject_reason')
    final_failure_not_early_missing = reject_reason not in {'ingress_map_base_tf_missing'}
    return {
        'name': name,
        'classification': _classify_case(name, preflight),
        'passed': bool(preflight.get('passed')),
        'reject_reason': reject_reason,
        'last_specific_reject_reason': preflight.get('last_specific_reject_reason'),
        'failed_gates': list(preflight.get('failed_gates') or []),
        'ingress_goal_sent': bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': bool(preflight.get('maze_explorer_started')),
        'sample_count': int(preflight.get('sample_count') or len(sample_history)),
        'sample_history': sample_history,
        'sample_history_tokens': _sample_tokens(preflight),
        'sample_history_failure_reasons': [sample.get('failure_reasons_by_gate') for sample in sample_history],
        'lifecycle_state': _lifecycle_state(preflight),
        'first_scan_seen': preflight.get('first_scan_seen'),
        'first_scan_wait_elapsed_sec': preflight.get('first_scan_wait_elapsed_sec'),
        'raw_style_snapshot_cross_check': preflight.get('raw_style_snapshot_cross_check') or {},
        'final_failure_not_early_missing': final_failure_not_early_missing,
    }


def analyze_replay_dir(replay_dir: Path) -> dict[str, Any]:
    cases: dict[str, Any] = {}
    missing_fields: list[dict[str, Any]] = []
    for path in sorted(replay_dir.glob(f'{RUN_ID}_*.json')):
        data = _read_json(path)
        preflight = data.get('ingress_preflight') or {}
        name = _case_name(path, preflight)
        summary = _case_summary(name, preflight)
        cases[name] = summary
        for idx, sample in enumerate(summary['sample_history']):
            for field in ['failed_gates', 'failure_reasons_by_gate']:
                if field not in sample:
                    missing_fields.append({'case': name, 'sample_index': idx, 'field': field})

    lifecycle_replay_states = {
        name: case.get('lifecycle_state')
        for name, case in cases.items()
        if name.startswith('lifecycle_')
    }
    classifications = {name: case.get('classification') for name, case in cases.items()}
    all_samples_have_failed_gates = not any(item['field'] == 'failed_gates' for item in missing_fields)
    all_samples_have_failure_reasons = not any(item['field'] == 'failure_reasons_by_gate' for item in missing_fields)
    required_cases = {
        'lifecycle_active_confirmed',
        'lifecycle_inactive_confirmed',
        'lifecycle_ambiguous',
        'lifecycle_query_error',
        'first_scan_waiting_only',
        'first_scan_timeout',
        'early_tf_miss_recovered',
        'stable_window_not_met',
        'raw_snapshot_contradiction',
    }
    expected_classifications = {
        'first_scan_waiting_only': 'WAITING_FOR_FIRST_SCAN_OBSERVED_NO_FINAL_SCAN_TIMEOUT',
        'first_scan_timeout': 'INGRESS_FIRST_SCAN_TIMEOUT_REPLAYED',
        'early_tf_miss_recovered': 'EARLY_TF_MISS_RECOVERED_STABLE_WINDOW_PASS',
        'stable_window_not_met': 'INGRESS_TF_STABLE_WINDOW_NOT_MET_REPLAYED',
        'raw_snapshot_contradiction': 'RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED',
    }
    missing_cases = sorted(required_cases - set(cases))
    mismatched_classifications = {
        name: {'expected': expected, 'actual': classifications.get(name)}
        for name, expected in expected_classifications.items()
        if classifications.get(name) != expected
    }
    contract_valid = (
        not missing_cases
        and not missing_fields
        and not mismatched_classifications
        and lifecycle_replay_states.get('lifecycle_active_confirmed') == 'active_confirmed'
        and lifecycle_replay_states.get('lifecycle_inactive_confirmed') == 'inactive_confirmed'
        and lifecycle_replay_states.get('lifecycle_ambiguous') == 'ambiguous'
        and lifecycle_replay_states.get('lifecycle_query_error') == 'query_error'
    )
    return {
        'run_id': RUN_ID,
        'phase': 'Phase110',
        'mode': 'synthetic_replay_only_no_runtime',
        'contract_valid': contract_valid,
        'cases': cases,
        'classifications': classifications,
        'lifecycle_replay_states': lifecycle_replay_states,
        'sample_history_contract': {
            'all_samples_have_failed_gates': all_samples_have_failed_gates,
            'all_samples_have_failure_reasons_by_gate': all_samples_have_failure_reasons,
            'missing_fields': missing_fields,
        },
        'missing_cases': missing_cases,
        'mismatched_classifications': mismatched_classifications,
        'guardrails': {
            'no_runtime_stack_started': True,
            'phase106_rerun': False,
            'ingress_goal_sent': False,
            'nav2_config_tuned': False,
            'exploration_strategy_changed': False,
            'preflight_removed': False,
            'phase111_entered': False,
        },
    }


def render_minimal_summary(analysis: dict[str, Any]) -> str:
    lines = [
        '# Phase110 synthetic replay contract summary',
        '',
        f"contract_valid: {analysis['contract_valid']}",
        'mode: synthetic replay only; No runtime stack was started.',
        '',
        '## Lifecycle replay states',
    ]
    for name in sorted(analysis.get('lifecycle_replay_states') or {}):
        lines.append(f"- {name}: {analysis['lifecycle_replay_states'][name]}")
    lines.extend(['', '## Case classifications'])
    for name in sorted(analysis.get('cases') or {}):
        case = analysis['cases'][name]
        lines.append(f"- {name}: {case['classification']} (passed={case['passed']}, reject_reason={case['reject_reason']})")
    lines.extend([
        '',
        '## Contract checks',
        f"- all samples have failed_gates: {analysis['sample_history_contract']['all_samples_have_failed_gates']}",
        f"- all samples have failure_reasons_by_gate: {analysis['sample_history_contract']['all_samples_have_failure_reasons_by_gate']}",
        '- waiting_for_first_scan and ingress_first_scan_timeout are distinguished.',
        '- ingress_tf_stable_window_not_met is replayed for insufficient stable window.',
        '- RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED remains fail-closed.',
        '',
        '## Guardrails',
        '- No runtime stack was started.',
        '- No Phase106 rerun was performed.',
        '- No ingress goal was sent.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- Phase111 not entered.',
    ])
    return '\n'.join(lines) + '\n'


def write_outputs(output_dir: Path) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts_dir = output_dir / 'synthetic_artifacts'
    generate_synthetic_replay_artifacts(artifacts_dir)
    analysis = analyze_replay_dir(artifacts_dir)
    analysis_path = output_dir / f'{RUN_ID}_analysis.json'
    summary_path = output_dir / f'{RUN_ID}_minimal_summary.md'
    _write_json(analysis_path, analysis)
    summary_path.write_text(render_minimal_summary(analysis), encoding='utf-8')
    return {'analysis': analysis_path, 'summary': summary_path, 'artifacts_dir': artifacts_dir}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    paths = write_outputs(args.output_dir)
    analysis = _read_json(paths['analysis'])
    print(json.dumps({
        'contract_valid': analysis.get('contract_valid'),
        'analysis': str(paths['analysis']),
        'summary': str(paths['summary']),
        'artifacts_dir': str(paths['artifacts_dir']),
    }, sort_keys=True))
    return 0 if analysis.get('contract_valid') else 2


if __name__ == '__main__':
    raise SystemExit(main())
