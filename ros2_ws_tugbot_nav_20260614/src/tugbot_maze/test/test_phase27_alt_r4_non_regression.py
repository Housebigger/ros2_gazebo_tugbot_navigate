import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'aggregate_phase27_alt_r4_non_regression.py'


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def _analysis(run_id: str, *, final_mode='EXIT_REACHED', final_exit_distance=0.55,
              triggered_events=None, no_action_events=None, schema_complete=True,
              topology_passed=True, blocked_first=0, blocked_last=0,
              blacklisted_first=0, blacklisted_last=0) -> dict:
    triggered_events = triggered_events or []
    no_action_events = no_action_events or []
    missing_fields = [] if schema_complete else ['robot_exit_dist']
    return {
        'phase': 'Phase27-alt-R1',
        'counts': {'goal_event_rows': 10, 'explorer_state_rows': 50},
        'schema': {
            'required_fields_present': schema_complete,
            'missing_required_fields': missing_fields,
            'triggered_event_required_fields_present': True,
            'triggered_event_required_field_violations': [],
            'field_row_count': len(triggered_events) + len(no_action_events),
        },
        'fallback': {
            'event_count': len(triggered_events) + len(no_action_events),
            'triggered_count': len(triggered_events),
            'no_action_event_count': len(no_action_events),
            'triggered_events': triggered_events,
            'no_action_events': no_action_events,
            'fallback_reasons': {},
            'actions': {},
            'no_trigger_reason': 'no_near_exit_fallback_events_observed' if not triggered_events and not no_action_events else None,
        },
        'topology_non_pollution': {
            'passed': topology_passed,
            'violations': [] if topology_passed else [{'counter': 'blocked_branch_count'}],
            'first_blocked_branch_count': blocked_first,
            'last_blocked_branch_count': blocked_last,
            'first_blacklisted_goal_count': blacklisted_first,
            'last_blacklisted_goal_count': blacklisted_last,
        },
        'final_state': {
            'final_mode': final_mode,
            'final_exit_distance_m': final_exit_distance,
            'blocked_branch_count': blocked_last,
            'blacklisted_goal_count': blacklisted_last,
        },
        'conclusion': 'runtime_no_fallback_trigger_observed',
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r1',
        'inputs': {
            'goal_events': f'log/{run_id}_goal_events.jsonl',
            'explorer_state': f'log/{run_id}_explorer_state.jsonl',
        },
    }


def _run(tmp_path: Path, analyses: list[dict]) -> dict:
    paths = []
    for i, analysis in enumerate(analyses, start=1):
        path = tmp_path / f'run{i}.json'
        _write_json(path, analysis)
        paths.append(path)
    output = tmp_path / 'summary.json'
    subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--output-json',
        str(output),
        *[str(path) for path in paths],
    ], check=True)
    return json.loads(output.read_text())


def test_r4_aggregator_passes_two_enabled_runs_with_only_near_exit_no_action_events(tmp_path):
    near_exit_no_action = {
        'event': 'near_exit_fallback',
        'near_exit_fallback_triggered': False,
        'action': 'no_action',
        'fallback_reason': 'micro_goal_local_cost_lethal',
        'robot_exit_dist': 0.77,
        'near_exit_fallback_attempts': 0,
        'near_exit_fallback_max_attempts': 1,
    }
    summary = _run(tmp_path, [
        _analysis('phase27_alt_r4_enabled_run1', no_action_events=[near_exit_no_action], final_exit_distance=0.54),
        _analysis('phase27_alt_r4_enabled_run2', no_action_events=[], final_exit_distance=0.58),
    ])

    assert summary['phase'] == 'Phase27-alt-R4'
    assert summary['run_count'] == 2
    assert summary['acceptance']['no_non_near_exit_fallback_trigger'] is True
    assert summary['acceptance']['topology_non_pollution'] is True
    assert summary['acceptance']['all_fallback_events_schema_complete'] is True
    assert summary['acceptance']['blocked_blacklisted_not_increased_by_fallback'] is True
    assert summary['conclusion'] == 'PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT'
    assert summary['mppi_root_cause_claim'] == 'not_evaluated_by_phase27_alt_r4'


def test_r4_aggregator_detects_non_near_exit_trigger_and_schema_gap(tmp_path):
    bad_trigger = {
        'event': 'near_exit_fallback',
        'near_exit_fallback_triggered': True,
        'action': 'micro_goal',
        'fallback_reason': 'micro_goal_candidate',
        'robot_exit_dist': 1.25,
        'near_exit_fallback_attempts': 0,
    }
    summary = _run(tmp_path, [
        _analysis('phase27_alt_r4_enabled_run1', triggered_events=[bad_trigger], schema_complete=False),
        _analysis('phase27_alt_r4_enabled_run2'),
    ])

    assert summary['acceptance']['no_non_near_exit_fallback_trigger'] is False
    assert summary['acceptance']['all_fallback_events_schema_complete'] is False
    assert summary['non_near_exit_fallback_trigger_violations'][0]['robot_exit_dist'] == 1.25
    assert summary['conclusion'] == 'FAIL_R4_FALLBACK_ENABLED_NON_REGRESSION'


def test_r4_aggregator_records_failed_final_mode_without_tuning_recommendation(tmp_path):
    summary = _run(tmp_path, [
        _analysis('phase27_alt_r4_enabled_run1', final_mode='FAILED_EXHAUSTED', final_exit_distance=1.4),
        _analysis('phase27_alt_r4_enabled_run2', final_mode='EXIT_REACHED', final_exit_distance=0.56),
    ])

    assert summary['final_modes'] == {'EXIT_REACHED': 1, 'FAILED_EXHAUSTED': 1}
    assert summary['acceptance']['final_modes_and_exit_distances_recorded'] is True
    assert summary['runtime_interpretation']['do_not_tune_on_failed_run'] is True
    assert summary['runtime_interpretation']['failed_runs_require_evidence_whether_fallback_caused'] is True
    assert summary['guardrails']['nav2_mppi_controller_params_modified'] is False
