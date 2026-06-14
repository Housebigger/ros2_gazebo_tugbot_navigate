from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_phase39_startup_map_sufficiency.py'
WRAPPER = ROOT / 'tools' / 'run_phase39_startup_map_sufficiency_diagnostics.sh'
OUT_DIR = ROOT / 'log' / 'phase39_startup_map_sufficiency'
OUT_JSON = OUT_DIR / 'phase39_startup_map_sufficiency.json'

ALLOWED = {
    'MAP_SUFFICIENCY_DELAY_REQUIRED',
    'FRAME_ALIGNMENT_ISSUE_CONFIRMED',
    'START_POSE_ALIGNMENT_OK_BUT_MAP_NOT_READY',
    'SENSOR_FLOW_NOT_READY_AT_TOPOLOGY_TIME',
    'INSUFFICIENT_EVIDENCE',
}


def test_phase39_recorder_contract_mentions_full_ros_topics_and_no_cli_echo_truncation():
    assert RECORDER.exists(), 'Phase39 full-data rclpy recorder must exist'
    source = RECORDER.read_text(encoding='utf-8')
    assert 'PHASE39_ALLOWED_CONCLUSIONS' in source
    for label in ALLOWED:
        assert label in source
    for topic in [
        '/map',
        '/local_costmap/costmap',
        '/global_costmap/costmap',
        '/scan',
        '/tf',
        '/maze/explorer_state',
        '/maze/goal_events',
    ]:
        assert topic in source
    for token in [
        'OccupancyGrid',
        'LaserScan',
        'TFMessage',
        'tf2_ros.Buffer',
        'map->base_link',
        'odom->base_link',
        'map->odom',
        'robot_vs_entrance_frame_alignment_overlay.png',
        'local_map_known_free_overlay.png',
        'topology_sampling_after_full_map_overlay.png',
    ]:
        assert token in source
    forbidden = [
        'ros2 topic echo',
        'ros2 topic pub',
        'ros2 action send_goal',
        'NavigateToPose',
        'controller_server',
        'nav2_slam_phase',
    ]
    for token in forbidden:
        assert token not in source


def test_phase39_wrapper_is_bounded_active_world_no_strategy_or_nav2_tuning():
    assert WRAPPER.exists(), 'Phase39 bounded startup diagnostics wrapper must exist'
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'phase39_startup_map_sufficiency' in source
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in source
    assert 'maze_20260528_scaled_instance.yaml' in source
    assert 'PHASE39_RUN_TIMEOUT_SEC' in source
    assert 'PHASE39_MAX_GOALS' in source
    assert 'max_goals:="$MAX_GOALS"' in source
    assert 'allows at most one bounded analysis/dispatch attempt' in source
    assert '--snapshots 30,60,90' in source
    assert 'record_phase39_startup_map_sufficiency.py' in source
    assert 'near_exit_fallback_enabled:=false' in source
    assert 'explorer_type:=maze_dfs' in source
    assert 'git diff -- src/tugbot_navigation/config' in source
    assert 'cleanup_processes_after.txt' in source
    for forbidden in [
        'nav2_slam_phase16_progress_params.yaml',
        'nav2_slam_phase25',
        'candidate_costcritic_275_profile:=true',
        'phase26p_mppi_diagnostics_profile:=true',
        'explorer_type:=frontier',
    ]:
        assert forbidden not in source


def test_phase39_recorder_generates_summary_from_synthetic_full_data_without_ros_runtime(tmp_path):
    synthetic = tmp_path / 'synthetic_phase39_full_data.json'
    output_dir = tmp_path / 'out'
    payload = {
        'metadata': {
            'active_world': 'src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf',
            'active_metadata': 'src/tugbot_maze/config/maze_20260528_scaled_instance.yaml',
            'guardrails': {'bounded_startup_diagnostics_only': True},
        },
        'active_truth': {
            'entrance': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
            'exit': {'x_m': 10.0, 'y_m': 10.0, 'radius_m': 1.2},
        },
        'snapshots': [
            {
                'elapsed_sec': 30.0,
                'map': {
                    'header': {'frame_id': 'map'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [-1] * 400,
                },
                'local_costmap': {
                    'header': {'frame_id': 'odom'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [-1] * 400,
                },
                'global_costmap': {
                    'header': {'frame_id': 'map'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [-1] * 400,
                },
                'scan': {'ranges': [float('inf'), 1.5, 2.0], 'range_min': 0.1, 'range_max': 10.0},
                'tf_lookups': {
                    'map->base_link': {'available': True, 'translation': {'x': 0.05, 'y': 0.02, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                    'odom->base_link': {'available': True, 'translation': {'x': 0.05, 'y': 0.02, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                    'map->odom': {'available': True, 'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                },
                'explorer_state_samples': [],
                'goal_event_samples': [],
            },
            {
                'elapsed_sec': 60.0,
                'map': {
                    'header': {'frame_id': 'map'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [0] * 400,
                },
                'local_costmap': {
                    'header': {'frame_id': 'odom'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [0] * 400,
                },
                'global_costmap': {
                    'header': {'frame_id': 'map'},
                    'info': {'width': 20, 'height': 20, 'resolution': 0.1, 'origin': {'position': {'x': -1.0, 'y': -1.0}}},
                    'data': [0] * 400,
                },
                'scan': {'ranges': [2.5, 3.0, float('nan')], 'range_min': 0.1, 'range_max': 10.0},
                'tf_lookups': {
                    'map->base_link': {'available': True, 'translation': {'x': 0.05, 'y': 0.02, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                    'odom->base_link': {'available': True, 'translation': {'x': 0.05, 'y': 0.02, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                    'map->odom': {'available': True, 'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                },
                'explorer_state_samples': [
                    {'elapsed_sec': 45.0, 'state': {'mode': 'FAILED_EXHAUSTED', 'last_terminal_reason': 'no untried branches remain', 'goal_count': 0}}
                ],
                'goal_event_samples': [],
            },
        ],
    }
    synthetic.write_text(json.dumps(payload), encoding='utf-8')
    result = subprocess.run(
        [
            sys.executable,
            str(RECORDER),
            '--analyze-existing',
            str(synthetic),
            '--output-dir',
            str(output_dir),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    data = json.loads(result.stdout)
    assert data['phase'] == 'Phase39 Startup Map Sufficiency and Entrance Frame Alignment Evidence'
    assert data['conclusion'] in ALLOWED
    assert data['conclusion'] == 'MAP_SUFFICIENCY_DELAY_REQUIRED'
    assert data['autonomous_success_claimed'] is False
    assert data['guardrails']['bounded_startup_diagnostics_only'] is True
    assert data['guardrails']['nav2_mppi_controller_params_modified'] is False
    assert data['first_explorer_state']['elapsed_sec'] == 45.0
    assert data['failed_exhausted_before_map_sufficient'] is True
    assert data['map_sufficiency_timeline'][0]['map']['known_ratio_near_robot'] < 0.5
    assert data['map_sufficiency_timeline'][1]['map']['known_ratio_near_robot'] >= 0.5
    assert data['frame_alignment']['robot_to_active_entrance_distance_m'] < 0.5
    assert data['sensor_flow']['first_explorer_state_time_scan_finite_count'] >= 1
    for name in [
        'robot_vs_entrance_frame_alignment_overlay.png',
        'local_map_known_free_overlay.png',
        'topology_sampling_after_full_map_overlay.png',
        'phase39_startup_map_sufficiency.json',
    ]:
        path = output_dir / name
        assert path.exists() and path.stat().st_size > 0
