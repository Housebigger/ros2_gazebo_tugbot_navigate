import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_failure_windows.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def test_phase23a_failure_window_introspection_contract(tmp_path):
    controller = tmp_path / 'controller.json'
    dynamics = tmp_path / 'dynamics.jsonl'
    local_cost = tmp_path / 'local_cost.json'
    nav2 = tmp_path / 'nav2.json'
    output = tmp_path / 'failure_windows.json'

    controller.write_text(json.dumps({
        'summary': {'goal_count': 2},
        'goals': [
            {
                'goal_sequence': 1,
                'outcome': 'timeout',
                'classification': 'healthy_motion_but_late_stall',
                'late_controller_silent': True,
                'start_time': 0.0,
                'end_time': 40.0,
                'last_10s_odom_distance_m': 0.03,
                'last_10s_cmd_linear_abs_mean': 0.004,
                'last_10s_cmd_angular_abs_mean': 0.01,
                'progress_failure_count': 2,
                'recovery_count': 3,
                'controller_abort_count': 2,
            },
            {
                'goal_sequence': 2,
                'outcome': 'success',
                'classification': 'healthy_motion',
                'late_controller_silent': False,
                'start_time': 50.0,
                'end_time': 60.0,
            },
        ],
    }), encoding='utf-8')

    _write_jsonl(dynamics, [
        {'wall_time': 30.0, 'source': 'cmd_vel', 'linear_x': 0.20, 'angular_z': 0.0},
        {'wall_time': 34.0, 'source': 'cmd_vel', 'linear_x': 0.02, 'angular_z': 0.0},
        {'wall_time': 36.0, 'source': 'cmd_vel', 'linear_x': 0.004, 'angular_z': 0.01},
        {'wall_time': 38.0, 'source': 'cmd_vel', 'linear_x': 0.0, 'angular_z': 0.0},
        {'wall_time': 40.0, 'source': 'cmd_vel', 'linear_x': 0.0, 'angular_z': 0.0},
        {'wall_time': 30.0, 'source': 'odom', 'x': 1.0, 'y': 1.0, 'yaw': 0.0},
        {'wall_time': 40.0, 'source': 'odom', 'x': 1.03, 'y': 1.0, 'yaw': 0.0},
    ])

    local_cost.write_text(json.dumps({
        'summary': {},
        'goals': [
            {
                'goal_sequence': 1,
                'outcome': 'timeout',
                'dispatch_path_local_cost_max': 42,
                'timeout_robot_local_cost_max': 100,
                'timeout_robot_local_cost_mean': 70.5,
                'timeout_robot_obstacle_cluster_count': 123,
                'footprint_corridor_inflation_squeezed': True,
                'timeout_robot_in_local_costmap_bounds': True,
            }
        ],
    }), encoding='utf-8')

    nav2.write_text(json.dumps({
        'summary': {},
        'goals': [
            {
                'goal_sequence': 1,
                'outcome': 'timeout',
                'progress_failure_count': 2,
                'recovery_count': 3,
                'controller_abort_count': 2,
                'progress_failure_times': [33.0, 39.0],
                'clear_costmap_times': [34.0, 39.5],
                'controller_abort_times': [34.1, 39.8],
            }
        ],
    }), encoding='utf-8')

    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--controller-analysis', str(controller),
        '--controller-dynamics', str(dynamics),
        '--local-cost-summary', str(local_cost),
        '--nav2-analysis', str(nav2),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    assert data['summary']['failure_window_count'] == 1
    assert data['summary']['late_controller_silent_count'] == 1
    assert data['summary']['squeezed_count'] == 1
    assert data['summary']['high_timeout_robot_cost_count'] == 1

    row = data['failure_windows'][0]
    assert row['goal_sequence'] == 1
    assert row['classification'] == 'healthy_motion_but_late_stall'
    assert row['failure_window_start_time'] == 30.0
    assert row['failure_window_end_time'] == 40.0
    assert row['cmd_linear_abs_min'] == 0.0
    assert row['cmd_linear_abs_max'] == 0.2
    assert row['cmd_linear_abs_p95'] == 0.2
    assert row['last_nonzero_cmd_time'] == 34.0
    assert row['near_zero_cmd_start_time'] == 36.0
    assert row['near_zero_cmd_duration_sec'] == 4.0
    assert row['timeout_robot_local_cost_max'] == 100
    assert row['footprint_corridor_inflation_squeezed'] is True
    assert row['progress_failure_count'] == 2
    assert row['first_progress_failure_time'] == 33.0
    assert row['last_controller_abort_time'] == 39.8
    assert row['near_zero_cmd_to_first_progress_failure_sec'] == 3.0
    assert row['available_diagnostics']['footprint_cost_stats'] is False
    assert row['available_diagnostics']['path_ahead_cost'] is False
    assert 'goal_sequence,classification' in result.stdout
