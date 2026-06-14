import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'summarize_post_recovery_snapshots.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def test_phase24c_post_recovery_snapshot_summary_schema(tmp_path):
    snapshots = tmp_path / 'snapshots.jsonl'
    output = tmp_path / 'summary.json'
    _write_jsonl(snapshots, [
        {
            'event': 'snapshot',
            'snapshot_type': 'pre_recovery',
            'goal_sequence': 1,
            'wall_time': 100.0,
            'path_ahead_0_5m_cost_max': 80,
            'path_ahead_1_0m_cost_max': 90,
            'robot_to_path_distance_m': 0.2,
        },
        {
            'event': 'snapshot',
            'snapshot_type': 'post_recovery',
            'goal_sequence': 1,
            'wall_time': 101.0,
            'path_ahead_0_5m_cost_max': 30,
            'path_ahead_1_0m_cost_max': 40,
            'robot_to_path_distance_m': 0.25,
        },
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': 1,
            'wall_time': 110.0,
            'path_ahead_0_5m_cost_max': 99,
            'path_ahead_1_0m_cost_max': 100,
            'robot_to_path_distance_m': 0.9,
            'controller_received_path_but_cmd_near_zero': True,
            'post_recovery_path_update_count': 3,
        },
    ])

    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--snapshots', str(snapshots),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    assert 'goal_sequence,pre_recovery_path_ahead_1_0m_cost_max,post_recovery_path_ahead_1_0m_cost_max,near_zero_path_ahead_1_0m_cost_max' in result.stdout
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['summary']['goal_count'] == 1
    row = data['goals'][0]
    assert row['goal_sequence'] == 1
    assert row['pre_recovery_path_ahead_1_0m_cost_max'] == 90
    assert row['post_recovery_path_ahead_1_0m_cost_max'] == 40
    assert row['near_zero_path_ahead_1_0m_cost_max'] == 100
    assert row['near_zero_robot_to_path_distance_m'] == 0.9
    assert row['controller_received_path_but_cmd_near_zero'] is True
    assert row['post_recovery_path_update_count'] == 3
