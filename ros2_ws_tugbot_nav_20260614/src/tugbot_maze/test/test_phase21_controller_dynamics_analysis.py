import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_goal_controller_dynamics.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def _goal_event(seq: int, event: str, wall_time: float, **extra) -> dict:
    payload = {
        'goal_sequence': seq,
        'event': event,
        'mode': 'NAVIGATING',
        'wall_time': wall_time,
    }
    payload.update(extra)
    return {'wall_time': wall_time, 'state': payload}


def _odom_sample(t: float, x: float, y: float = 0.0, yaw: float = 0.0) -> dict:
    return {'wall_time': t, 'source': 'odom', 'x': x, 'y': y, 'yaw': yaw}


def _cmd_sample(t: float, linear: float, angular: float = 0.0) -> dict:
    return {'wall_time': t, 'source': 'cmd_vel', 'linear_x': linear, 'angular_z': angular}


def test_phase21_controller_dynamics_classifies_synthetic_goal_patterns(tmp_path):
    goal_events = tmp_path / 'goal_events.jsonl'
    dynamics = tmp_path / 'controller_dynamics.jsonl'
    nav2 = tmp_path / 'nav2.json'
    output = tmp_path / 'controller_summary.json'

    _write_jsonl(goal_events, [
        _goal_event(1, 'dispatch', 0.0),
        _goal_event(1, 'success', 10.0),
        _goal_event(2, 'dispatch', 20.0),
        _goal_event(2, 'timeout', 55.0),
        _goal_event(3, 'dispatch', 70.0),
        _goal_event(3, 'failure', 90.0, result_reason='blocked_nav2'),
        _goal_event(4, 'dispatch', 100.0),
        _goal_event(4, 'timeout', 135.0),
    ])
    _write_jsonl(dynamics, [
        # seq 1: healthy success, enough odom distance with normal forward commands.
        _odom_sample(0.0, 0.0), _cmd_sample(0.0, 0.20, 0.02),
        _odom_sample(5.0, 0.8), _cmd_sample(5.0, 0.20, 0.01),
        _odom_sample(10.0, 1.6), _cmd_sample(10.0, 0.10, 0.00),
        # seq 2: stuck_with_cmd, commands continue but odom barely changes.
        _odom_sample(20.0, 2.0), _cmd_sample(20.0, 0.22, 0.0),
        _odom_sample(37.0, 2.03), _cmd_sample(37.0, 0.22, 0.0),
        _odom_sample(55.0, 2.05), _cmd_sample(55.0, 0.22, 0.0),
        # seq 3: blocked oscillation, angular commands/yaw change high but xy movement low.
        _odom_sample(70.0, 3.0, yaw=0.0), _cmd_sample(70.0, 0.01, 0.75),
        _odom_sample(80.0, 3.01, yaw=1.2), _cmd_sample(80.0, 0.01, -0.8),
        _odom_sample(90.0, 3.02, yaw=-1.4), _cmd_sample(90.0, 0.01, 0.9),
        # seq 4: controller_silent, neither cmd nor odom moves.
        _odom_sample(100.0, 4.0), _cmd_sample(100.0, 0.0, 0.0),
        _odom_sample(117.0, 4.0), _cmd_sample(117.0, 0.0, 0.0),
        _odom_sample(135.0, 4.0), _cmd_sample(135.0, 0.0, 0.0),
    ])
    nav2.write_text(json.dumps({
        'summary': {'goal_count': 4, 'success_count': 1, 'timeout_count': 2},
        'goals': [
            {'goal_sequence': 1, 'outcome': 'success', 'progress_failure_count': 0, 'recovery_count': 0, 'controller_abort_count': 0},
            {'goal_sequence': 2, 'outcome': 'timeout', 'progress_failure_count': 2, 'recovery_count': 3, 'controller_abort_count': 2},
            {'goal_sequence': 3, 'outcome': 'failure', 'result_reason': 'blocked_nav2', 'progress_failure_count': 1, 'recovery_count': 1, 'controller_abort_count': 3},
            {'goal_sequence': 4, 'outcome': 'timeout', 'progress_failure_count': 1, 'recovery_count': 1, 'controller_abort_count': 1},
        ],
    }), encoding='utf-8')

    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--goal-events', str(goal_events),
        '--controller-dynamics', str(dynamics),
        '--nav2-analysis', str(nav2),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    assert data['summary']['goal_count'] == 4
    assert data['summary']['stuck_with_cmd_count'] == 1
    assert data['summary']['oscillation_candidate_count'] == 1
    assert data['summary']['controller_silent_count'] == 1
    assert data['summary']['healthy_motion_count'] == 1

    by_seq = {row['goal_sequence']: row for row in data['goals']}
    assert by_seq[1]['classification'] == 'healthy_motion'
    assert by_seq[1]['odom_distance_m'] > 1.5
    assert by_seq[1]['cmd_linear_abs_mean'] > 0.1

    assert by_seq[2]['classification'] == 'stuck_with_cmd'
    assert by_seq[2]['stuck_with_cmd'] is True
    assert by_seq[2]['odom_distance_m'] < 0.1
    assert by_seq[2]['cmd_linear_abs_mean'] > 0.15

    assert by_seq[3]['classification'] == 'oscillation_candidate'
    assert by_seq[3]['oscillation_candidate'] is True
    assert by_seq[3]['odom_distance_m'] < 0.1
    assert by_seq[3]['odom_yaw_delta_abs_sum_rad'] > 2.0
    assert by_seq[3]['cmd_angular_abs_mean'] > 0.7

    assert by_seq[4]['classification'] == 'controller_silent'
    assert by_seq[4]['controller_silent'] is True
    assert by_seq[4]['near_zero_cmd_ratio'] == 1.0
    assert by_seq[4]['odom_distance_m'] == 0.0

    assert 'goal_sequence,outcome,classification' in result.stdout


def test_phase22a_synthetic_windowed_late_motion_patterns(tmp_path):
    goal_events = tmp_path / 'goal_events_windowed.jsonl'
    dynamics = tmp_path / 'controller_dynamics_windowed.jsonl'
    nav2 = tmp_path / 'nav2_windowed.json'
    output = tmp_path / 'controller_windowed_summary.json'

    _write_jsonl(goal_events, [
        _goal_event(10, 'dispatch', 0.0),
        _goal_event(10, 'timeout', 40.0),
        _goal_event(11, 'dispatch', 50.0),
        _goal_event(11, 'failure', 90.0, result_reason='blocked_nav2'),
        _goal_event(12, 'dispatch', 100.0),
        _goal_event(12, 'timeout', 140.0),
        _goal_event(13, 'dispatch', 150.0),
        _goal_event(13, 'timeout', 190.0),
    ])
    _write_jsonl(dynamics, [
        # seq 10: full interval moves, last 10s silent/stalled.
        _odom_sample(0.0, 0.0), _cmd_sample(0.0, 0.25, 0.0),
        _odom_sample(20.0, 1.2), _cmd_sample(20.0, 0.20, 0.0),
        _odom_sample(30.0, 1.25), _cmd_sample(30.0, 0.0, 0.0),
        _odom_sample(40.0, 1.26), _cmd_sample(40.0, 0.0, 0.0),
        # seq 11: full interval moves, last 10s cmd active but odom stalled.
        _odom_sample(50.0, 2.0), _cmd_sample(50.0, 0.25, 0.0),
        _odom_sample(70.0, 2.8), _cmd_sample(70.0, 0.25, 0.0),
        _odom_sample(80.0, 2.82), _cmd_sample(80.0, 0.22, 0.0),
        _odom_sample(90.0, 2.83), _cmd_sample(90.0, 0.22, 0.0),
        # seq 12: full interval moves, last 10s oscillates in yaw with little xy.
        _odom_sample(100.0, 4.0, yaw=0.0), _cmd_sample(100.0, 0.25, 0.0),
        _odom_sample(120.0, 4.8, yaw=0.2), _cmd_sample(120.0, 0.25, 0.0),
        _odom_sample(130.0, 4.82, yaw=1.2), _cmd_sample(130.0, 0.0, 0.8),
        _odom_sample(140.0, 4.83, yaw=-1.2), _cmd_sample(140.0, 0.0, -0.8),
        # seq 13: full interval moves and last 10s still moves, but times out.
        _odom_sample(150.0, 6.0), _cmd_sample(150.0, 0.20, 0.0),
        _odom_sample(170.0, 6.8), _cmd_sample(170.0, 0.20, 0.0),
        _odom_sample(180.0, 7.1), _cmd_sample(180.0, 0.20, 0.0),
        _odom_sample(190.0, 7.4), _cmd_sample(190.0, 0.20, 0.0),
    ])
    nav2.write_text(json.dumps({
        'summary': {'goal_count': 4, 'timeout_count': 3},
        'goals': [
            {'goal_sequence': 10, 'outcome': 'timeout'},
            {'goal_sequence': 11, 'outcome': 'failure', 'result_reason': 'blocked_nav2'},
            {'goal_sequence': 12, 'outcome': 'timeout'},
            {'goal_sequence': 13, 'outcome': 'timeout'},
        ],
    }), encoding='utf-8')

    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--goal-events', str(goal_events),
        '--controller-dynamics', str(dynamics),
        '--nav2-analysis', str(nav2),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    by_seq = {row['goal_sequence']: row for row in data['goals']}

    assert data['summary']['late_controller_silent_count'] == 1
    assert data['summary']['late_stuck_with_cmd_count'] == 1
    assert data['summary']['late_oscillation_count'] == 1
    assert data['summary']['healthy_motion_but_timed_out_count'] == 1

    assert by_seq[10]['classification'] == 'healthy_motion_but_late_stall'
    assert by_seq[10]['late_controller_silent'] is True
    assert by_seq[10]['healthy_motion_but_late_stall'] is True

    assert by_seq[11]['classification'] == 'healthy_motion_but_late_stall'
    assert by_seq[11]['late_stuck_with_cmd'] is True

    assert by_seq[12]['classification'] == 'healthy_motion_but_late_stall'
    assert by_seq[12]['late_oscillation'] is True

    assert by_seq[13]['classification'] == 'healthy_motion_but_timed_out'
    assert by_seq[13]['healthy_motion_but_timed_out'] is True
    assert by_seq[13]['timeout_or_failure_late_stall'] is False
