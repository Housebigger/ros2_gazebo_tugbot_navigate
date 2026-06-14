from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_post_recovery_snapshots.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'

REQUIRED_TOKENS = [
    'PostRecoverySnapshotRecorder',
    'OccupancyGrid',
    'Path',
    'Odometry',
    'Twist',
    '/local_costmap/costmap',
    '/plan',
    '/odom',
    'cmd_vel_nav',
    '/maze/goal_events',
    'pre_recovery',
    'post_recovery',
    'near_zero_onset',
    'path_update',
    'robot_to_path_distance_m',
    'path_ahead_0_5m_cost_max',
    'path_ahead_1_0m_cost_max',
    'controller_received_path_but_cmd_near_zero',
    'post_recovery_path_update_count',
]


def test_phase24c_runtime_post_recovery_recorder_source_contract():
    assert RECORDER.exists(), 'runtime post-recovery recorder script must exist'
    source = RECORDER.read_text(encoding='utf-8')
    for token in REQUIRED_TOKENS:
        assert token in source, token
    assert 'local_costmap/costmap_raw' not in source


def test_phase24c_smoke_wrapper_runs_post_recovery_recorder_and_outputs_json():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'POST_RECOVERY_SNAPSHOTS' in source
    assert 'record_post_recovery_snapshots.py' in source
    assert '--local-costmap-topic /local_costmap/costmap' in source
    assert '--path-topic /plan' in source
    assert '--goal-events-topic /maze/goal_events' in source
    assert 'phase24c_run' in source
