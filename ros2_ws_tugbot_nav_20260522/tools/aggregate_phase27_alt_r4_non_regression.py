#!/usr/bin/env python3
"""Aggregate Phase27-alt-R4 fallback-enabled non-regression repeats.

Consumes per-run `analyze_phase27_alt_fallback_runtime.py` JSON outputs and
checks that enabling the default-off fallback does not create non-near-exit
triggers, topology pollution, schema gaps, or unrecorded final outcomes.

Diagnostics-only: this does not run Gazebo/Nav2, modify Nav2/MPPI/controller
parameters, change branch selection, relax local-cost gates, promote/reject
CostCritic=2.75, or explain MPPI selected-control near-zero root cause.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

NEAR_EXIT_TRIGGER_RADIUS_M = 0.9
PASS = 'PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT'
FAIL = 'FAIL_R4_FALLBACK_ENABLED_NON_REGRESSION'


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def integer(value: Any) -> int | None:
    num = number(value)
    return int(num) if num is not None else None


def infer_run_id(path: Path, data: dict[str, Any]) -> str:
    goal_events = ((data.get('inputs') or {}).get('goal_events'))
    if isinstance(goal_events, str) and goal_events:
        name = Path(goal_events).name
        suffix = '_goal_events.jsonl'
        if name.endswith(suffix):
            return name[:-len(suffix)]
    stem = path.stem
    suffix = '_phase27_alt_fallback_analysis'
    if stem.endswith(suffix):
        return stem[:-len(suffix)]
    return stem


def fallback_events(data: dict[str, Any]) -> list[dict[str, Any]]:
    fb = data.get('fallback') or {}
    events = []
    for key in ('triggered_events', 'no_action_events'):
        value = fb.get(key) or []
        if isinstance(value, list):
            events.extend(row for row in value if isinstance(row, dict))
    return events


def run_summary(path: Path, data: dict[str, Any]) -> dict[str, Any]:
    schema = data.get('schema') or {}
    fallback = data.get('fallback') or {}
    topology = data.get('topology_non_pollution') or {}
    final_state = data.get('final_state') or {}
    events = fallback_events(data)
    triggered = [row for row in events if row.get('near_exit_fallback_triggered') is True]
    non_near_trigger = []
    for row in triggered:
        dist = number(row.get('robot_exit_dist'))
        if dist is None or dist > NEAR_EXIT_TRIGGER_RADIUS_M:
            non_near_trigger.append({
                'run_id': infer_run_id(path, data),
                'action': row.get('action'),
                'fallback_reason': row.get('fallback_reason'),
                'robot_exit_dist': dist,
                'event': row.get('event'),
            })
    first_blocked = integer(topology.get('first_blocked_branch_count'))
    last_blocked = integer(topology.get('last_blocked_branch_count'))
    first_blacklisted = integer(topology.get('first_blacklisted_goal_count'))
    last_blacklisted = integer(topology.get('last_blacklisted_goal_count'))
    fallback_counter_increase = False
    if first_blocked is not None and last_blocked is not None and last_blocked > first_blocked and int(fallback.get('triggered_count') or 0) > 0:
        fallback_counter_increase = True
    if first_blacklisted is not None and last_blacklisted is not None and last_blacklisted > first_blacklisted and int(fallback.get('triggered_count') or 0) > 0:
        fallback_counter_increase = True
    return {
        'run_id': infer_run_id(path, data),
        'analysis_json': str(path),
        'goal_events': (data.get('inputs') or {}).get('goal_events'),
        'explorer_state': (data.get('inputs') or {}).get('explorer_state'),
        'goal_event_rows': (data.get('counts') or {}).get('goal_event_rows'),
        'explorer_state_rows': (data.get('counts') or {}).get('explorer_state_rows'),
        'schema_complete': bool(schema.get('required_fields_present')) and bool(schema.get('triggered_event_required_fields_present', True)),
        'schema_missing_required_fields': schema.get('missing_required_fields') or [],
        'triggered_schema_violations': schema.get('triggered_event_required_field_violations') or [],
        'fallback_event_count': int(fallback.get('event_count') or 0),
        'fallback_triggered_count': int(fallback.get('triggered_count') or 0),
        'fallback_no_action_count': int(fallback.get('no_action_event_count') or 0),
        'fallback_actions': fallback.get('actions') or {},
        'fallback_reasons': fallback.get('fallback_reasons') or {},
        'non_near_exit_trigger_violations': non_near_trigger,
        'topology_non_pollution': bool(topology.get('passed')),
        'topology_violations': topology.get('violations') or [],
        'first_blocked_branch_count': first_blocked,
        'last_blocked_branch_count': last_blocked,
        'first_blacklisted_goal_count': first_blacklisted,
        'last_blacklisted_goal_count': last_blacklisted,
        'blocked_blacklisted_increased_by_fallback': fallback_counter_increase,
        'final_mode': final_state.get('final_mode'),
        'final_exit_distance_m': number(final_state.get('final_exit_distance_m')),
        'run_conclusion': data.get('conclusion'),
        'mppi_root_cause_claim': data.get('mppi_root_cause_claim'),
    }


def aggregate(paths: list[Path]) -> dict[str, Any]:
    runs = [run_summary(path, load_json(path)) for path in paths]
    final_modes: dict[str, int] = {}
    final_exit_distances: dict[str, float | None] = {}
    for run in runs:
        mode = str(run.get('final_mode'))
        final_modes[mode] = final_modes.get(mode, 0) + 1
        final_exit_distances[run['run_id']] = run.get('final_exit_distance_m')

    non_near = [item for run in runs for item in run['non_near_exit_trigger_violations']]
    schema_gaps = [run for run in runs if not run['schema_complete']]
    topology_bad = [run for run in runs if not run['topology_non_pollution']]
    fallback_counter_bad = [run for run in runs if run['blocked_blacklisted_increased_by_fallback']]
    missing_final = [run for run in runs if run.get('final_mode') is None or run.get('final_exit_distance_m') is None]

    acceptance = {
        'no_non_near_exit_fallback_trigger': not non_near,
        'topology_non_pollution': not topology_bad,
        'blocked_blacklisted_not_increased_by_fallback': not fallback_counter_bad,
        'all_fallback_events_schema_complete': not schema_gaps,
        'final_modes_and_exit_distances_recorded': not missing_final,
        'src_tugbot_navigation_config_no_diff': None,
    }
    passed = all(value is True for key, value in acceptance.items() if key != 'src_tugbot_navigation_config_no_diff')
    failed_runs = [run for run in runs if run.get('final_mode') != 'EXIT_REACHED']
    return {
        'phase': 'Phase27-alt-R4',
        'run_count': len(runs),
        'runs': runs,
        'acceptance': acceptance,
        'non_near_exit_fallback_trigger_violations': non_near,
        'schema_gap_runs': [{'run_id': run['run_id'], 'missing': run['schema_missing_required_fields'], 'triggered_violations': run['triggered_schema_violations']} for run in schema_gaps],
        'topology_pollution_runs': [{'run_id': run['run_id'], 'violations': run['topology_violations']} for run in topology_bad],
        'fallback_counter_increase_runs': [{'run_id': run['run_id'], 'first_blocked': run['first_blocked_branch_count'], 'last_blocked': run['last_blocked_branch_count'], 'first_blacklisted': run['first_blacklisted_goal_count'], 'last_blacklisted': run['last_blacklisted_goal_count']} for run in fallback_counter_bad],
        'final_modes': final_modes,
        'final_exit_distances_m': final_exit_distances,
        'failed_or_non_exit_runs': [{'run_id': run['run_id'], 'final_mode': run.get('final_mode'), 'final_exit_distance_m': run.get('final_exit_distance_m'), 'fallback_triggered_count': run.get('fallback_triggered_count'), 'topology_non_pollution': run.get('topology_non_pollution')} for run in failed_runs],
        'runtime_interpretation': {
            'do_not_tune_on_failed_run': True,
            'failed_runs_require_evidence_whether_fallback_caused': True,
            'do_not_force_terminal_acceptance_runtime_coverage': True,
            'fallback_enabled_non_regression_only': True,
        },
        'guardrails': {
            'phase26y_or_26z_entered': False,
            'nav2_mppi_controller_source_fetched': False,
            'nav2_mppi_controller_params_modified': False,
            'costcritic_275_promoted_or_rejected': False,
            'branch_selection_strategy_modified': False,
            'local_cost_gate_relaxed': False,
            'terminal_acceptance_runtime_forced': False,
        },
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r4',
        'conclusion': PASS if passed else FAIL,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('analysis_json', nargs='+', type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = aggregate(args.analysis_json)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(args.output_json),
        'run_count': result['run_count'],
        'conclusion': result['conclusion'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
