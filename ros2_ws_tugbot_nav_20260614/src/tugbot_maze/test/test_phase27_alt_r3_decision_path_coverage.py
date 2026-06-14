import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_phase27_alt_r3_decision_path_coverage.py'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def _run(tmp_path: Path, source_text: str, r2_text: str | None = None) -> dict:
    src = tmp_path / 'maze_explorer.py'
    r2 = tmp_path / 'r2.json'
    out = tmp_path / 'r3.json'
    _write(src, source_text)
    if r2_text is None:
        r2_text = json.dumps({
            'coverage_status': 'not_covered',
            'conclusion': 'NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH',
            'nearest_exit_radius_state_window': {'state_count_at_or_below_terminal_radius': 13},
            'final_state': {'final_mode': 'EXIT_REACHED'},
            'counts': {'terminal_acceptance_event_count': 0},
        })
    _write(r2, r2_text)
    subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--maze-explorer',
        str(src),
        '--r2-coverage-json',
        str(r2),
        '--output-json',
        str(out),
    ], check=True)
    return json.loads(out.read_text())


def test_r3_analyzer_reports_normal_exit_monitor_precedes_fallback_and_explains_r1_bypass(tmp_path):
    source = MAZE_EXPLORER.read_text()
    data = _run(tmp_path, source)

    assert data['phase'] == 'Phase27-alt-R3'
    assert data['decision_path_audit']['normal_exit_monitor_precedes_near_exit_fallback'] is True
    assert data['decision_path_audit']['near_exit_fallback_called_only_after_no_active_goal_and_settle'] is True
    assert data['decision_path_audit']['fallback_terminal_acceptance_requires_recent_problem_evidence'] is True
    assert data['r1_r2_interpretation']['normal_exit_monitor_natural_bypass_likely'] is True
    assert data['r1_r2_interpretation']['why_r1_entered_radius_without_fallback_event'] == 'robot_entered_terminal_radius_while_goal_active_normal_exit_monitor_executed_before_fallback_decision'
    assert data['runtime_coverage_design']['execute_runtime_now'] is False
    assert data['runtime_coverage_design']['recommended_path'] == 'host_level_coverage_sufficient_for_abnormal_branch'
    assert data['mppi_root_cause_claim'] == 'not_evaluated_by_phase27_alt_r3'


def test_r3_analyzer_flags_missing_normal_precedence_if_order_is_reversed(tmp_path):
    source = MAZE_EXPLORER.read_text()
    source = source.replace(
        "        if self._exit_reached(robot_pose):\n            self._enter_terminal_state(EXIT_REACHED, terminal_reason='exit_reached', robot_pose=robot_pose)\n            self._publish_state()\n            return\n\n        if self.goal_active:",
        "        if self.goal_active:",
    )
    data = _run(tmp_path, source)

    assert data['decision_path_audit']['normal_exit_monitor_precedes_near_exit_fallback'] is False
    assert data['runtime_coverage_design']['targeted_runtime_needed'] is True
    assert data['runtime_coverage_design']['execute_runtime_now'] is False


def test_host_level_terminal_acceptance_branch_contract_is_present_in_phase27_tests():
    source = (ROOT / 'src' / 'tugbot_maze' / 'test' / 'test_phase27_alt_near_exit_fallback.py').read_text()
    required = [
        'terminal_acceptance',
        'near_exit_fallback_enabled',
        'GOAL_CANCELED_AFTER_TIMEOUT',
        'robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m',
        "'fallback_reason': 'terminal_acceptance_radius'",
        "'action': 'terminal_acceptance'",
    ]
    for token in required:
        assert token in source
