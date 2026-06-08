import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_phase27_alt_terminal_acceptance_coverage.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n')


def _goal_event(event: str, elapsed: float, **payload) -> dict:
    state = {'event': event}
    state.update(payload)
    return {'elapsed_sec': elapsed, 'wall_time': 1000.0 + elapsed, 'state': state}


def _state(elapsed: float, mode: str, exit_distance: float, **payload) -> dict:
    state = {
        'mode': mode,
        'exit_distance_m': exit_distance,
        'near_exit_fallback_enabled': True,
        'blocked_branch_count': 0,
        'blacklisted_goal_count': 0,
    }
    state.update(payload)
    return {'elapsed_sec': elapsed, 'wall_time': 1000.0 + elapsed, 'state': state}


def _run(tmp_path: Path, goal_rows: list[dict], state_rows: list[dict]) -> dict:
    goal_events = tmp_path / 'goal_events.jsonl'
    explorer_state = tmp_path / 'explorer_state.jsonl'
    output = tmp_path / 'coverage.json'
    _write_jsonl(goal_events, goal_rows)
    _write_jsonl(explorer_state, state_rows)
    subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--goal-events',
        str(goal_events),
        '--explorer-state',
        str(explorer_state),
        '--output-json',
        str(output),
    ], check=True)
    return json.loads(output.read_text())


def test_terminal_acceptance_coverage_detects_triggered_branch_and_exit_reached(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event('timeout', 20.0, goal_sequence=4, goal_kind='explore'),
            _goal_event('timeout_cancel_result', 20.2, goal_sequence=4, goal_kind='explore'),
            _goal_event(
                'near_exit_fallback',
                22.0,
                goal_sequence=None,
                goal_kind='none',
                near_exit_fallback_triggered=True,
                fallback_reason='terminal_acceptance_radius',
                action='terminal_acceptance',
                robot_exit_dist=0.54,
                last_nav2_result='goal_canceled_after_timeout',
                near_exit_fallback_attempts=0,
                near_exit_fallback_max_attempts=1,
            ),
            _goal_event('terminal_cancel', 22.1, goal_sequence=4, goal_kind='explore', mode='EXIT_REACHED'),
        ],
        [
            _state(19.0, 'SETTLING', 0.62),
            _state(22.05, 'EXIT_REACHED', 0.54),
        ],
    )

    assert data['coverage_status'] == 'covered'
    assert data['conclusion'] == 'PASS_TERMINAL_ACCEPTANCE_BRANCH_VALIDATED'
    assert data['terminal_acceptance_event']['robot_exit_dist'] == 0.54
    assert data['terminal_acceptance_event']['last_nav2_result_is_recent_failure_evidence'] is True
    assert data['final_state']['final_mode'] == 'EXIT_REACHED'
    assert data['mppi_root_cause_claim'] == 'not_evaluated_by_phase27_alt_r2'


def test_terminal_acceptance_coverage_reports_not_covered_when_r1_like_logs_enter_exit_radius_without_branch_event(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event('timeout', 154.7, goal_sequence=8, goal_kind='explore'),
            _goal_event('timeout_cancel_result', 154.73, goal_sequence=8, goal_kind='explore'),
            _goal_event(
                'near_exit_fallback',
                156.7,
                near_exit_fallback_triggered=False,
                fallback_reason='micro_goal_local_cost_lethal',
                action='no_action',
                robot_exit_dist=0.77,
                last_nav2_result='goal_canceled_after_timeout',
                near_exit_fallback_attempts=0,
                near_exit_fallback_max_attempts=1,
            ),
            _goal_event('terminal_cancel', 180.7, goal_sequence=9, goal_kind='explore', mode='EXIT_REACHED'),
        ],
        [
            _state(156.7, 'AT_NODE_ANALYZE', 0.77),
            _state(179.7, 'NAVIGATING', 0.587),
            _state(180.7, 'EXIT_REACHED', 0.545),
        ],
    )

    assert data['coverage_status'] == 'not_covered'
    assert data['conclusion'] == 'NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH'
    assert data['terminal_acceptance_event'] is None
    assert data['nearest_exit_radius_state_window']['min_exit_distance_m'] == 0.545
    assert data['nearest_exit_radius_state_window']['state_count_at_or_below_terminal_radius'] == 2
    assert data['not_covered_reason'] == 'exit_radius_state_seen_but_no_terminal_acceptance_event'
    assert data['bounded_runtime_smoke_recommendation']['recommended'] is True
    assert data['bounded_runtime_smoke_recommendation']['do_not_relax_local_cost_gate'] is True


def test_terminal_acceptance_coverage_flags_missing_recent_failure_evidence(tmp_path):
    data = _run(
        tmp_path,
        [
            _goal_event(
                'near_exit_fallback',
                10.0,
                near_exit_fallback_triggered=True,
                fallback_reason='terminal_acceptance_radius',
                action='terminal_acceptance',
                robot_exit_dist=0.55,
                last_nav2_result=None,
            ),
        ],
        [_state(10.2, 'EXIT_REACHED', 0.55)],
    )

    assert data['coverage_status'] == 'invalid_terminal_acceptance_evidence'
    assert data['conclusion'] == 'TERMINAL_ACCEPTANCE_EVENT_MISSING_RECENT_FAILURE_EVIDENCE'
    assert data['terminal_acceptance_event']['last_nav2_result_is_recent_failure_evidence'] is False
