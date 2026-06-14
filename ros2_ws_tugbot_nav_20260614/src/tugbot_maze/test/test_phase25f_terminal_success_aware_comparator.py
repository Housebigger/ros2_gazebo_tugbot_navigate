import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
COMPARE = ROOT / 'tools' / 'compare_phase25e_metrics.py'


def _write(path: Path, *, final_mode: str, goal_count: int, successes: int, timeouts: int, footprint: int, exit_distance: float, blocked: int = 0, blacklisted: int = 0):
    path.write_text(json.dumps({
        'run_summary': {
            'goal_count': goal_count,
            'timeout_cancel_count': timeouts,
            'goal_success_count': successes,
            'blocked_branch_count': blocked,
            'blacklisted_goal_count': blacklisted,
            'final_mode': final_mode,
            'exit_distance_m': exit_distance,
        },
        'timeout_subtypes': {
            'summary': {
                'controller_subtype_counts': {'footprint_path_blocked_late_silent': footprint},
            }
        }
    }), encoding='utf-8')


def _run_compare(tmp_path: Path, *, exp_mode: str, exp_goals: int, exp_successes: int, exp_timeouts: int, exp_footprint: int, exp_blocked: int = 0):
    baseline = tmp_path / 'baseline.json'
    experiment = tmp_path / 'experiment.json'
    output = tmp_path / 'decision.json'
    _write(
        baseline,
        final_mode='FAILED_EXHAUSTED',
        goal_count=12,
        successes=8,
        timeouts=4,
        footprint=2,
        exit_distance=0.75,
    )
    _write(
        experiment,
        final_mode=exp_mode,
        goal_count=exp_goals,
        successes=exp_successes,
        timeouts=exp_timeouts,
        footprint=exp_footprint,
        exit_distance=0.60,
        blocked=exp_blocked,
    )
    result = subprocess.run([
        sys.executable,
        str(COMPARE),
        '--baseline', str(baseline),
        '--experiment', str(experiment),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    return json.loads(output.read_text(encoding='utf-8'))


def test_phase25f_exit_reached_replaces_raw_success_count_with_terminal_success_gate(tmp_path):
    data = _run_compare(
        tmp_path,
        exp_mode='EXIT_REACHED',
        exp_goals=9,
        exp_successes=6,
        exp_timeouts=2,
        exp_footprint=0,
    )
    assert data['accepted'] is True
    success_check = data['checks']['success_gate']
    assert success_check['passed'] is True
    assert success_check['criterion'] == 'experiment EXIT_REACHED bypasses raw success count gate'
    assert data['advisory']['success_efficiency']['baseline_success_ratio'] == 8 / 12
    assert data['advisory']['success_efficiency']['experiment_success_ratio'] == 6 / 9
    assert data['advisory']['success_efficiency']['raw_success_count_regressed'] is True
    assert data['recommendation'] == 'candidate_baseline_or_repeat_validation'


def test_phase25f_non_terminal_run_keeps_strict_success_no_regression(tmp_path):
    data = _run_compare(
        tmp_path,
        exp_mode='FAILED_EXHAUSTED',
        exp_goals=12,
        exp_successes=6,
        exp_timeouts=2,
        exp_footprint=0,
    )
    assert data['accepted'] is False
    success_check = data['checks']['success_gate']
    assert success_check['passed'] is False
    assert success_check['criterion'] == 'experiment >= baseline unless experiment EXIT_REACHED'


def test_phase25f_exit_reached_still_respects_safety_gates(tmp_path):
    data = _run_compare(
        tmp_path,
        exp_mode='EXIT_REACHED',
        exp_goals=9,
        exp_successes=6,
        exp_timeouts=2,
        exp_footprint=0,
        exp_blocked=1,
    )
    assert data['accepted'] is False
    assert data['checks']['blocked_branch_no_regression']['passed'] is False
