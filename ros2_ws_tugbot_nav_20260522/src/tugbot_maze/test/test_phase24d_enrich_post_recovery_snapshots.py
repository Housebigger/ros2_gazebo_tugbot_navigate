import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'enrich_post_recovery_snapshots.py'
LOG = ROOT / 'log'


def _write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def _run(snapshots: Path, nav2: Path, output: Path) -> dict:
    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--snapshots', str(snapshots),
        '--nav2-analysis', str(nav2),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'goal_sequence,recovery_time,pre_recovery_age_sec,post_recovery_age_sec,near_zero_age_sec' in result.stdout
    return json.loads(output.read_text(encoding='utf-8'))


def test_phase24d_synthetic_enriches_pre_post_recovery_and_near_zero(tmp_path):
    snapshots = tmp_path / 'snapshots.jsonl'
    nav2 = tmp_path / 'nav2.json'
    output = tmp_path / 'enriched.json'

    _write_jsonl(snapshots, [
        {'event': 'snapshot', 'snapshot_type': 'near_zero_onset', 'goal_sequence': 1, 'wall_time': 95.0, 'path_ahead_1_0m_cost_max': 10, 'robot_to_path_distance_m': 0.1},
        {'event': 'snapshot', 'snapshot_type': 'path_update', 'goal_sequence': 1, 'wall_time': 99.0, 'path_ahead_1_0m_cost_max': 80, 'robot_to_path_distance_m': 0.2},
        {'event': 'snapshot', 'snapshot_type': 'near_zero_onset', 'goal_sequence': 1, 'wall_time': 101.5, 'path_ahead_1_0m_cost_max': 90, 'robot_to_path_distance_m': 0.3, 'controller_received_path_but_cmd_near_zero': True},
        {'event': 'snapshot', 'snapshot_type': 'path_update', 'goal_sequence': 1, 'wall_time': 102.0, 'path_ahead_1_0m_cost_max': 40, 'robot_to_path_distance_m': 0.25},
        {'event': 'path_update', 'snapshot_type': 'path_update', 'goal_sequence': 1, 'wall_time': 103.0, 'path_point_count': 20},
        {'event': 'snapshot', 'snapshot_type': 'near_zero_onset', 'goal_sequence': 2, 'wall_time': 210.0, 'path_ahead_1_0m_cost_max': 100, 'robot_to_path_distance_m': 0.5},
    ])
    _write_json(nav2, {'goals': [
        {'goal_sequence': 1, 'outcome': 'timeout', 'clear_costmap_times': [100.0]},
        {'goal_sequence': 2, 'outcome': 'timeout', 'clear_costmap_times': [200.0]},
    ]})

    data = _run(snapshots, nav2, output)
    rows = data['enriched_recovery_snapshots']
    by_seq = {row['goal_sequence']: row for row in rows}

    assert by_seq[1]['recovery_time'] == 100.0
    assert by_seq[1]['pre_recovery_snapshot']['wall_time'] == 99.0
    assert by_seq[1]['post_recovery_snapshot']['wall_time'] == 101.5
    assert by_seq[1]['near_zero_snapshot']['wall_time'] == 101.5
    assert by_seq[1]['pre_recovery_age_sec'] == 1.0
    assert by_seq[1]['post_recovery_age_sec'] == 1.5
    assert by_seq[1]['near_zero_age_sec'] == 1.5
    assert by_seq[1]['pre_recovery_path_ahead_1_0m_cost_max'] == 80
    assert by_seq[1]['post_recovery_path_ahead_1_0m_cost_max'] == 90
    assert by_seq[1]['near_zero_path_ahead_1_0m_cost_max'] == 90
    assert by_seq[1]['near_zero_robot_to_path_distance_m'] == 0.3
    assert by_seq[1]['path_update_count_after_recovery'] == 1
    assert by_seq[1]['snapshot_density_sufficient'] is True

    assert by_seq[2]['snapshot_density_sufficient'] is False
    assert by_seq[2]['pre_recovery_snapshot'] is None
    assert by_seq[2]['post_recovery_snapshot']['wall_time'] == 210.0
    assert data['summary']['recovery_count'] == 2
    assert data['summary']['sufficient_density_count'] == 1
    assert data['summary']['needs_periodic_snapshot_count'] == 1


def test_phase24d_phase24c_fixture_enrichment_outputs_recovery_rows(tmp_path):
    snapshots = LOG / 'phase24c_run1_post_recovery_snapshots.jsonl'
    nav2 = LOG / 'phase24c_run1_goal_nav2_analysis.json'
    assert snapshots.exists(), 'Phase 24C snapshot fixture required'
    assert nav2.exists(), 'Phase 24C Nav2 fixture required'

    data = _run(snapshots, nav2, tmp_path / 'phase24c_enriched.json')
    assert data['summary']['recovery_count'] >= 1
    for row in data['enriched_recovery_snapshots']:
        assert 'goal_sequence' in row
        assert 'recovery_time' in row
        assert 'pre_recovery_snapshot' in row
        assert 'post_recovery_snapshot' in row
        assert 'near_zero_snapshot' in row
        assert 'snapshot_density_sufficient' in row
