import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26m_timing_windows.py'


def write_json(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True) + '\n', encoding='utf-8')


def append_goal_json(path: Path, goal_key: str, goal: dict, summary: dict | None = None):
    payload = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {goal_key: [], 'summary': {}}
    payload.setdefault(goal_key, []).append(goal)
    if summary:
        payload.setdefault('summary', {}).update(summary)
    write_json(path, payload)


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def prepare_run(log_dir: Path, run_id: str, seq: int, *, near_zero_delay: float, path_updates: int, first_high_cost: float | None, robot_to_path: float, chosen_cost: int, explored_cost: int | None):
    append_goal_json(log_dir / f'{run_id}_goal_nav2_analysis.json', 'goals', {
            'goal_sequence': seq,
            'outcome': 'timeout',
            'clear_costmap_times': [100.0],
            'controller_abort_times': [104.0],
            'progress_failure_times': [103.5],
        })
    append_goal_json(log_dir / f'{run_id}_goal_controller_dynamics.json', 'goals', {
            'goal_sequence': seq,
            'classification': 'healthy_motion_but_late_stall',
            'last_window_classification': 'late_controller_silent',
            'cmd_near_zero_fraction_last_window': 1.0,
        }, {'timeout_or_failure_late_stall_count': 1, 'late_controller_silent_count': 1})
    append_goal_json(log_dir / f'{run_id}_timeout_subtypes.json', 'timeouts', {
            'goal_sequence': seq,
            'combined_subtype': 'side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort',
            'controller_subtype': 'side_cost_or_timing_late_silent',
            'timing_subtype': 'cmd_silent_after_recovery_abort',
        }, {'timeout_count': 1})
    rows = [
        {
            'event': 'path_update',
            'goal_sequence': seq,
            'wall_time': 101.0 + index,
            'post_recovery_path_update_count': index + 1,
        }
        for index in range(path_updates)
    ]
    rows.extend([
        {
            'event': 'snapshot',
            'snapshot_type': 'post_recovery',
            'goal_sequence': seq,
            'wall_time': 100.2,
            'path_ahead_1_0m_cost_max': 30,
            'path_ahead_1_0m_first_high_cost_distance_m': None,
            'robot_to_path_distance_m': 0.02,
            'chosen_route_corridor_cost_max': 20,
            'explored_candidate_corridor_cost_max': 15,
        },
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': seq,
            'wall_time': 100.0 + near_zero_delay,
            'path_ahead_1_0m_cost_max': 100 if first_high_cost is not None else 30,
            'path_ahead_1_0m_high_cost_count': 4 if first_high_cost is not None else 0,
            'path_ahead_1_0m_first_high_cost_distance_m': first_high_cost,
            'robot_to_path_distance_m': robot_to_path,
            'chosen_route_corridor_cost_max': chosen_cost,
            'chosen_route_corridor_cost_mean': chosen_cost / 2,
            'chosen_route_first_high_cost_distance_m': first_high_cost if chosen_cost >= 70 else None,
            'explored_candidate_corridor_cost_max': explored_cost,
            'explored_candidate_corridor_cost_mean': explored_cost / 2 if explored_cost is not None else None,
            'explored_candidate_first_high_cost_distance_m': None if explored_cost is None or explored_cost < 70 else 0.8,
            'controller_received_path_but_cmd_near_zero': path_updates > 0,
            'post_recovery_path_update_count': path_updates,
        },
    ])
    path = log_dir / f'{run_id}_post_recovery_snapshots.jsonl'
    existing = path.read_text(encoding='utf-8').splitlines() if path.exists() else []
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(existing + [json.dumps(row, sort_keys=True) for row in rows]) + '\n', encoding='utf-8')
    dynamics_path = log_dir / f'{run_id}_controller_dynamics.jsonl'
    existing_dynamics = dynamics_path.read_text(encoding='utf-8').splitlines() if dynamics_path.exists() else []
    dynamics_rows = [
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 100.5, 'linear_x': 0.12, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 100.0 + near_zero_delay, 'linear_x': 0.0, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 100.0 + near_zero_delay + 0.5, 'linear_x': 0.0, 'angular_z': 0.0},
    ]
    dynamics_path.write_text('\n'.join(existing_dynamics + [json.dumps(row, sort_keys=True) for row in dynamics_rows]) + '\n', encoding='utf-8')


def test_phase26m_analyzer_compares_matched_pairs_and_recommends_timing_diagnostics(tmp_path):
    log_dir = tmp_path / 'log'
    prepare_run(log_dir, 'baseline', 2, near_zero_delay=6.5, path_updates=5, first_high_cost=0.65, robot_to_path=0.08, chosen_cost=40, explored_cost=None)
    prepare_run(log_dir, 'candidate', 2, near_zero_delay=6.2, path_updates=6, first_high_cost=0.7, robot_to_path=0.09, chosen_cost=40, explored_cost=None)
    prepare_run(log_dir, 'baseline', 10, near_zero_delay=5.8, path_updates=4, first_high_cost=None, robot_to_path=0.03, chosen_cost=0, explored_cost=99)
    prepare_run(log_dir, 'candidate', 7, near_zero_delay=5.6, path_updates=8, first_high_cost=None, robot_to_path=0.04, chosen_cost=46, explored_cost=0)
    output = tmp_path / 'phase26m.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--pair', 'early:baseline:2:candidate:2',
        '--pair', 'near_exit:baseline:10:candidate:7',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26M'
    assert data['analysis_only'] is True
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['next_recommendation'] == 'continue_controller_local_cost_timing_diagnostics'
    assert data['summary']['common_combined_subtype'] == 'side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort'

    early = data['pairs']['early']
    assert early['matched_pattern'] == 'persistent_cmd_silent_after_recovery_with_path_updates'
    assert early['baseline']['near_zero_after_recovery_sec'] == 6.5
    assert early['candidate']['near_zero_after_recovery_sec'] == 6.2
    assert early['baseline']['cmd_near_zero_after_recovery_sec'] == 6.5
    assert early['candidate']['cmd_near_zero_after_recovery_sec'] == 6.2
    assert early['baseline']['cmd_near_zero_ratio_5s_after_recovery'] == 0.0
    assert early['baseline']['path_updates_after_recovery'] == 5
    assert early['candidate']['path_updates_after_recovery'] == 6
    assert early['interpretation'] == 'timing_or_controller_silence_not_branch_selection'

    near_exit = data['pairs']['near_exit']
    assert near_exit['matched_pattern'] == 'non_divergent_or_shared_near_exit_timing_window'
    assert near_exit['baseline']['explored_candidate_corridor_cost_max'] == 99
    assert near_exit['candidate']['explored_candidate_corridor_cost_max'] == 0
    assert near_exit['interpretation'] == 'near_exit_pair_is_not_phase27_branch_evidence'
