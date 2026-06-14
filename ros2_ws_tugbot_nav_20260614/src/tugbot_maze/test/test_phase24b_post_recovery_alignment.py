import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_post_recovery_alignment.py'
LOG = ROOT / 'log'


def _write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def _run(args: list[str], output: Path) -> dict:
    result = subprocess.run([sys.executable, str(SCRIPT), *args, '--output-json', str(output)], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'goal_sequence,outcome,last_recovery_time,path_updates_after_recovery' in result.stdout
    return json.loads(output.read_text(encoding='utf-8'))


def test_phase24b_synthetic_post_recovery_alignment(tmp_path):
    failure_windows = tmp_path / 'failure_windows.json'
    nav2 = tmp_path / 'nav2.json'
    dynamics = tmp_path / 'dynamics.jsonl'
    launch_log = tmp_path / 'launch.log'
    output = tmp_path / 'alignment.json'

    _write_json(failure_windows, {
        'failure_windows': [{
            'goal_sequence': 1,
            'outcome': 'timeout',
            'classification': 'healthy_motion_but_late_stall',
            'failure_window_start_time': 105.0,
            'failure_window_end_time': 115.0,
            'near_zero_cmd_start_time': 110.0,
            'timeout_footprint_cost_max': 99,
            'timeout_path_ahead_0_5m_cost_max': 100,
            'timeout_path_ahead_1_0m_cost_max': 100,
        }]
    })
    _write_json(nav2, {'goals': [{
        'goal_sequence': 1,
        'outcome': 'timeout',
        'dispatch_wall_time': 90.0,
        'outcome_wall_time': 120.0,
        'clear_costmap_times': [100.0],
        'controller_abort_times': [99.8],
        'progress_failure_times': [99.7],
    }]})
    _write_jsonl(dynamics, [
        {'source': 'odom', 'wall_time': 109.0, 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        {'source': 'cmd_vel', 'wall_time': 106.0, 'linear_x': 0.2, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'wall_time': 111.0, 'linear_x': 0.002, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'wall_time': 112.0, 'linear_x': 0.003, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'wall_time': 113.0, 'linear_x': 0.004, 'angular_z': 0.0},
    ])
    launch_log.write_text('\n'.join([
        '[controller_server-7] [INFO] [101.000000000] [controller_server]: Passing new path to controller.',
        '[controller_server-7] [INFO] [108.000000000] [controller_server]: Passing new path to controller.',
        '[controller_server-7] [INFO] [112.000000000] [controller_server]: Passing new path to controller.',
    ]), encoding='utf-8')

    data = _run([
        '--failure-windows', str(failure_windows),
        '--nav2-analysis', str(nav2),
        '--controller-dynamics', str(dynamics),
        '--launch-log', str(launch_log),
    ], output)

    row = data['post_recovery_alignment'][0]
    assert row['goal_sequence'] == 1
    assert row['last_recovery_time'] == 100.0
    assert row['path_updates_after_recovery'] == 3
    assert row['path_updates_after_recovery_before_near_zero'] == 2
    assert row['first_path_update_after_recovery_time'] == 101.0
    assert row['last_path_update_before_near_zero_time'] == 108.0
    assert row['near_zero_after_last_recovery_sec'] == 10.0
    assert row['path_update_to_near_zero_sec'] == 2.0
    assert row['near_zero_cmd_samples_after_recovery'] == 3
    assert row['path_updates_while_cmd_near_zero'] == 1
    assert row['controller_received_path_but_cmd_near_zero'] is True
    assert row['timeout_footprint_cost_max'] == 99
    assert row['timeout_path_ahead_1_0m_cost_max'] == 100
    assert row['available_diagnostics']['path_update_times'] is True
    assert row['available_diagnostics']['local_cost_timeout_snapshot'] is True
    assert data['summary']['controller_received_path_but_cmd_near_zero_count'] == 1


def test_phase24b_phase24_fixture_alignment_has_post_recovery_fields(tmp_path):
    required = [
        LOG / 'phase24_run1_failure_windows.json',
        LOG / 'phase24_run1_goal_nav2_analysis.json',
        LOG / 'phase24_run1_controller_dynamics.jsonl',
        LOG / 'phase24_run1_launch.log',
    ]
    for path in required:
        assert path.exists(), f'missing fixture {path}'

    data = _run([
        '--failure-windows', str(required[0]),
        '--nav2-analysis', str(required[1]),
        '--controller-dynamics', str(required[2]),
        '--launch-log', str(required[3]),
    ], tmp_path / 'phase24_alignment.json')

    rows = data['post_recovery_alignment']
    assert rows, 'expected phase24 fixture alignment rows'
    for row in rows:
        assert 'path_updates_after_recovery' in row
        assert 'path_updates_after_recovery_before_near_zero' in row
        assert 'controller_received_path_but_cmd_near_zero' in row
        assert 'timeout_path_ahead_1_0m_cost_max' in row
        assert 'available_diagnostics' in row
    assert data['summary']['row_count'] >= 3
