import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_goal_controller_dynamics.py'
LOG = ROOT / 'log'


def _run_analyzer(tmp_path: Path) -> dict:
    output = tmp_path / 'phase21c_windowed_controller.json'
    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--goal-events', str(LOG / 'phase21_run1_goal_events.jsonl'),
        '--controller-dynamics', str(LOG / 'phase21_run1_controller_dynamics.jsonl'),
        '--nav2-analysis', str(LOG / 'phase21_run1_goal_nav2_analysis.json'),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    return json.loads(output.read_text(encoding='utf-8'))


def test_phase22a_phase21c_timeouts_are_late_stall_not_plain_healthy_motion(tmp_path):
    data = _run_analyzer(tmp_path)
    by_seq = {row['goal_sequence']: row for row in data['goals']}

    assert data['summary']['healthy_motion_but_late_stall_count'] == 3
    assert data['summary']['late_controller_silent_count'] == 3
    assert data['summary']['healthy_motion_but_timed_out_count'] == 0

    for seq in (2, 4, 7):
        row = by_seq[seq]
        assert row['outcome'] == 'timeout'
        assert row['classification'] == 'healthy_motion_but_late_stall'
        assert row['healthy_motion_but_late_stall'] is True
        assert row['late_controller_silent'] is True
        assert row['last_10s_odom_distance_m'] < 0.05
        assert row['last_10s_cmd_linear_abs_mean'] < 0.01
        assert row['timeout_or_failure_late_stall'] is True

    for seq in (3, 5, 6):
        row = by_seq[seq]
        assert row['outcome'] == 'success'
        assert row['classification'] == 'healthy_motion'
        assert row['healthy_motion_but_late_stall'] is False
        assert row['late_controller_silent'] is False
        assert row['timeout_or_failure_late_stall'] is False

    terminal = by_seq[8]
    assert terminal['outcome'] == 'terminal_cancel'
    assert terminal['terminal_cancel_after_exit'] is True
    assert terminal['timeout_or_failure_late_stall'] is False
