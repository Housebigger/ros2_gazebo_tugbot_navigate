import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_phase27_alt_fallback_runtime.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n')


def _goal_event(event: str, **payload) -> dict:
    state = {
        'event': event,
        'goal_sequence': payload.pop('goal_sequence', 1),
        'near_exit_fallback_triggered': payload.pop('near_exit_fallback_triggered', False),
        'fallback_reason': payload.pop('fallback_reason', None),
        'robot_exit_dist': payload.pop('robot_exit_dist', None),
        'cmd_near_zero_duration': payload.pop('cmd_near_zero_duration', None),
        'last_nav2_result': payload.pop('last_nav2_result', None),
        'robot_to_path_distance': payload.pop('robot_to_path_distance', None),
        'action': payload.pop('action', None),
        'near_exit_fallback_attempts': payload.pop('near_exit_fallback_attempts', 0),
        'near_exit_fallback_max_attempts': payload.pop('near_exit_fallback_max_attempts', 1),
        'micro_goal_geometry': payload.pop('micro_goal_geometry', None),
        'micro_goal_local_cost': payload.pop('micro_goal_local_cost', None),
        'blocked_branch_count': payload.pop('blocked_branch_count', 0),
        'blacklisted_goal_count': payload.pop('blacklisted_goal_count', 0),
    }
    state.update(payload)
    return {'wall_time': float(payload.get('wall_time', 100.0)), 'state': state}


def _state(mode: str, exit_distance: float, blocked: int = 0, blacklisted: int = 0) -> dict:
    return {
        'wall_time': 200.0,
        'state': {
            'mode': mode,
            'exit_distance_m': exit_distance,
            'blocked_branch_count': blocked,
            'blacklisted_goal_count': blacklisted,
            'near_exit_fallback_enabled': True,
        },
    }


def _run(tmp_path: Path, goal_rows: list[dict], state_rows: list[dict] | None = None) -> dict:
    goal_events = tmp_path / 'run_goal_events.jsonl'
    explorer_state = tmp_path / 'run_explorer_state.jsonl'
    output = tmp_path / 'analysis.json'
    _write_jsonl(goal_events, goal_rows)
    if state_rows is not None:
        _write_jsonl(explorer_state, state_rows)
    cmd = [sys.executable, str(SCRIPT), '--goal-events', str(goal_events), '--output-json', str(output)]
    if state_rows is not None:
        cmd.extend(['--explorer-state', str(explorer_state)])
    subprocess.run(cmd, check=True)
    return json.loads(output.read_text())


def test_phase27_alt_runtime_analyzer_accepts_triggered_micro_goal_without_topology_pollution(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event(
                'near_exit_fallback',
                goal_sequence=2,
                near_exit_fallback_triggered=True,
                fallback_reason='micro_goal_candidate',
                robot_exit_dist=0.82,
                cmd_near_zero_duration=None,
                last_nav2_result='GOAL_TIMEOUT',
                robot_to_path_distance=0.08,
                action='micro_goal',
                near_exit_fallback_attempts=0,
                micro_goal_geometry={'line_of_sight_occupied_count': 0, 'target_clearance_m': 0.42},
                micro_goal_local_cost={'dispatch_target_local_cost': 12, 'dispatch_path_local_cost_max': 34},
                blocked_branch_count=0,
                blacklisted_goal_count=0,
            ),
            _goal_event('dispatch', goal_sequence=3, goal_kind='near_exit_micro_goal', blocked_branch_count=0, blacklisted_goal_count=0),
            _goal_event('failure', goal_sequence=3, goal_kind='near_exit_micro_goal', result_reason='BLOCKED_NAV2', blocked_branch_count=0, blacklisted_goal_count=0),
        ],
        [_state('FAILED_EXHAUSTED', 0.71)],
    )

    assert data['schema']['required_fields_present'] is True
    assert data['fallback']['event_count'] == 1
    assert data['fallback']['triggered_count'] == 1
    assert data['fallback']['actions']['micro_goal'] == 1
    assert data['fallback']['triggered_events'][0]['fallback_reason'] == 'micro_goal_candidate'
    assert data['topology_non_pollution']['passed'] is True
    assert data['conclusion'] == 'runtime_fallback_triggered_without_topology_pollution'
    assert data['mppi_root_cause_claim'] == 'not_evaluated_by_phase27_alt_r1'


def test_phase27_alt_runtime_analyzer_reports_no_trigger_reason_when_fields_present_but_no_fallback(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event('dispatch', goal_sequence=1, robot_exit_dist=2.4, blocked_branch_count=0, blacklisted_goal_count=0),
            _goal_event('failure', goal_sequence=1, result_reason='GOAL_TIMEOUT', robot_exit_dist=1.4, blocked_branch_count=0, blacklisted_goal_count=0),
        ],
        [_state('FAILED_EXHAUSTED', 1.4)],
    )

    assert data['schema']['required_fields_present'] is True
    assert data['fallback']['event_count'] == 0
    assert data['fallback']['triggered_count'] == 0
    assert data['fallback']['no_trigger_reason'] == 'no_near_exit_fallback_events_observed'
    assert data['fallback']['min_robot_exit_dist_observed'] == 1.4
    assert data['topology_non_pollution']['passed'] is True
    assert data['conclusion'] == 'runtime_no_fallback_trigger_observed'


def test_phase27_alt_runtime_analyzer_flags_micro_goal_topology_pollution(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event('near_exit_fallback', goal_sequence=4, near_exit_fallback_triggered=True, action='micro_goal', fallback_reason='micro_goal_candidate', robot_exit_dist=0.83, blocked_branch_count=0, blacklisted_goal_count=0),
            _goal_event('dispatch', goal_sequence=5, goal_kind='near_exit_micro_goal', blocked_branch_count=0, blacklisted_goal_count=0),
            _goal_event('failure', goal_sequence=5, goal_kind='near_exit_micro_goal', result_reason='BLOCKED_NAV2', blocked_branch_count=1, blacklisted_goal_count=0),
        ],
    )

    assert data['topology_non_pollution']['passed'] is False
    assert data['topology_non_pollution']['violations'][0]['goal_sequence'] == 5
    assert data['conclusion'] == 'runtime_topology_pollution_detected'
