import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'summarize_goal_geometry_nav2.py'


def _write_goal_events(path: Path) -> None:
    rows = [
        {
            'seq': 1,
            'wall_time': 100.0,
            'state': {
                'event': 'dispatch',
                'goal_sequence': 1,
                'goal_kind': 'explore',
                'local_topology': 'junction',
                'branch_angle': 0.1,
                'target_exit_dist': 3.0,
                'target_clearance_m': 1.0,
                'path_corridor_min_clearance_m': 0.9,
                'line_of_sight_occupied_count': 0,
                'line_of_sight_unknown_count': 0,
                'target_crosses_narrow_passage': False,
                'target_crosses_wall_corner': False,
                'target_near_wall': False,
                'target_yaw_corridor_conflict': False,
            },
        },
        {'seq': 2, 'wall_time': 106.0, 'state': {'event': 'success', 'goal_sequence': 1, 'elapsed_sec': 6.0}},
        {
            'seq': 3,
            'wall_time': 120.0,
            'state': {
                'event': 'dispatch',
                'goal_sequence': 2,
                'goal_kind': 'explore',
                'local_topology': 'corridor',
                'branch_angle': 1.57,
                'target_exit_dist': 4.5,
                'target_clearance_m': 0.31,
                'path_corridor_min_clearance_m': 0.24,
                'line_of_sight_occupied_count': 2,
                'line_of_sight_unknown_count': 1,
                'target_crosses_narrow_passage': True,
                'target_crosses_wall_corner': True,
                'target_near_wall': True,
                'target_yaw_corridor_conflict': True,
            },
        },
        {'seq': 4, 'wall_time': 156.0, 'state': {'event': 'timeout', 'goal_sequence': 2, 'elapsed_sec': 36.0}},
    ]
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')


def _write_nav2_analysis(path: Path) -> None:
    data = {
        'summary': {'goal_count': 2, 'success_count': 1, 'timeout_count': 1},
        'goals': [
            {'goal_sequence': 1, 'outcome': 'success', 'progress_failure_count': 0, 'recovery_count': 0, 'controller_abort_count': 0},
            {'goal_sequence': 2, 'outcome': 'timeout', 'progress_failure_count': 3, 'recovery_count': 2, 'controller_abort_count': 1},
        ],
    }
    path.write_text(json.dumps(data) + '\n')


def test_phase18_join_script_outputs_success_vs_timeout_geometry_summary(tmp_path):
    events = tmp_path / 'goal_events.jsonl'
    nav2 = tmp_path / 'nav2_analysis.json'
    output = tmp_path / 'summary.json'
    _write_goal_events(events)
    _write_nav2_analysis(nav2)

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--goal-events', str(events), '--nav2-analysis', str(nav2), '--output-json', str(output)],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    assert data['summary']['success']['count'] == 1
    assert data['summary']['timeout']['count'] == 1
    assert data['summary']['timeout']['target_clearance_m']['mean'] == 0.31
    assert data['summary']['timeout']['progress_failure_count']['sum'] == 3
    assert data['summary']['timeout']['target_crosses_narrow_passage']['true_count'] == 1
    timeout_row = [row for row in data['joined_goals'] if row['outcome'] == 'timeout'][0]
    assert timeout_row['path_corridor_min_clearance_m'] == 0.24
    assert timeout_row['line_of_sight_occupied_count'] == 2
    assert 'success,1' in result.stdout
    assert 'timeout,1' in result.stdout
