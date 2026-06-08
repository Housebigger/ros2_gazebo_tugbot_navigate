import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'record_local_costmap_diagnostics.py'


def _write_goal_events(path: Path) -> None:
    rows = [
        {
            'seq': 1,
            'wall_time': 10.0,
            'state': {
                'event': 'dispatch',
                'goal_sequence': 7,
                'dispatch_pose': [0.0, 0.0, 0.0],
                'target': [1.0, 0.0],
                'target_clearance_m': 0.8,
            },
        },
        {
            'seq': 2,
            'wall_time': 45.0,
            'state': {'event': 'timeout', 'goal_sequence': 7, 'elapsed_sec': 35.0},
        },
    ]
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')


def test_phase18_costmap_script_supports_offline_sample_join_contract(tmp_path):
    events = tmp_path / 'goal_events.jsonl'
    costmap = tmp_path / 'local_costmap_samples.jsonl'
    output = tmp_path / 'local_cost_summary.json'
    _write_goal_events(events)
    samples = [
        {
            'wall_time': 10.2,
            'topic': '/local_costmap/costmap_raw',
            'info': {'width': 5, 'height': 5, 'resolution': 0.5, 'origin': [-1.25, -1.25]},
            'data': [0] * 25,
        },
        {
            'wall_time': 44.0,
            'topic': '/local_costmap/costmap_raw',
            'info': {'width': 5, 'height': 5, 'resolution': 0.5, 'origin': [-1.25, -1.25]},
            # High costs near robot at timeout, but target corridor stays mostly free.
            'data': [0, 0, 0, 0, 0, 0, 80, 90, 80, 0, 0, 90, 100, 90, 0, 0, 80, 90, 80, 0, 0, 0, 0, 0, 0],
        },
    ]
    costmap.write_text('\n'.join(json.dumps(row) for row in samples) + '\n')

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--goal-events',
            str(events),
            '--costmap-samples',
            str(costmap),
            '--output-json',
            str(output),
            '--target-radius-m',
            '0.30',
            '--robot-cluster-radius-m',
            '0.75',
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    assert data['summary']['goal_count'] == 1
    goal = data['goals'][0]
    assert goal['goal_sequence'] == 7
    assert goal['dispatch_target_local_cost'] == 0
    assert goal['dispatch_path_local_cost_max'] == 0
    assert goal['dispatch_path_local_cost_mean'] == 0
    assert goal['dispatch_sample_age_sec'] == 0.2
    assert goal['outcome_sample_age_sec'] == 1.0
    assert goal['target_in_local_costmap_bounds'] is True
    assert goal['robot_pose_in_local_costmap_bounds'] is True
    assert goal['dispatch_path_sample_count'] == 5
    assert goal['local_cost_sample_coverage_ratio'] == 1.0
    assert goal['timeout_robot_local_cost_max'] == 100
    assert goal['timeout_robot_obstacle_cluster_count'] == 9
    assert goal['footprint_corridor_inflation_squeezed'] is True
    assert 'goal_sequence,dispatch_target_local_cost' in result.stdout
