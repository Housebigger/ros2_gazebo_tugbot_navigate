from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase44_manual_nav2_baseline_recovery.sh'
RECORDER = ROOT / 'tools' / 'record_phase44_manual_nav2_baseline_evidence.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase44_manual_nav2_baseline_recovery_report.md'
SLAM_NAV_LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam_nav.launch.py'
NAV2_CONFIG_DIR = ROOT / 'src' / 'tugbot_navigation' / 'config'


def test_phase44_wrapper_exists_and_uses_manual_slam_nav_only():
    text = WRAPPER.read_text()
    assert 'phase44_manual_nav2_baseline_recovery' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'use_rviz:=true' in text
    assert 'headless:=false' in text
    assert 'tugbot_maze_explore.launch.py' not in text
    launch_line = next(line for line in text.splitlines() if line.startswith('ros2 launch '))
    assert 'tugbot_maze_slam_nav.launch.py' in launch_line
    assert 'tugbot_maze_explore.launch.py' not in launch_line
    assert 'maze_explorer' in text  # allowed only as forbidden-node guardrail/evidence token
    assert 'FORBIDDEN_PATTERN' in text


def test_phase44_wrapper_records_required_artifacts_and_cleanup():
    text = WRAPPER.read_text()
    required = [
        'launch.log',
        'nodes.txt',
        'topics.txt',
        'actions.txt',
        'navigate_to_pose_action_info.txt',
        'map_scan_tf_odom_sample.json',
        'cleanup_processes_after.txt',
    ]
    for token in required:
        assert token in text
    assert 'pgrep -af' in text
    assert 'git diff -- src/tugbot_navigation/config' in text


def test_phase44_recorder_collects_map_scan_tf_odom_costmaps_and_action():
    text = RECORDER.read_text()
    for topic in ['/map', '/scan', '/odom', '/local_costmap/costmap', '/global_costmap/costmap']:
        assert topic in text
    for frame_pair in [('map', 'base_link'), ('odom', 'base_link'), ('map', 'odom')]:
        assert repr(frame_pair) in text or f'"{frame_pair[0]}", "{frame_pair[1]}"' in text
    assert 'navigate_to_pose' in text
    assert 'MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK' in text


def test_phase44_does_not_modify_nav2_configs_or_maze_explorer_contractually():
    assert NAV2_CONFIG_DIR.exists()
    wrapper_text = WRAPPER.read_text()
    recorder_text = RECORDER.read_text()
    combined = wrapper_text + recorder_text
    forbidden_strategy_tokens = ['max_goals:=', 'near_exit_fallback_enabled:=true', 'maze_explorer.py']
    for token in forbidden_strategy_tokens:
        assert token not in combined


def test_existing_manual_slam_nav_launch_is_explorer_free_and_active_world_default():
    text = SLAM_NAV_LAUNCH.read_text()
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'nav2_slam_params.yaml' in text
    for forbidden in ['maze_explorer', 'maze_goal_monitor', 'frontier_explorer']:
        assert forbidden not in text


def test_phase44_report_documents_human_rviz_steps_and_classification_set():
    text = REPORT.read_text()
    for token in ['2D Pose Estimate', 'Nav2 Goal', 'MANUAL_NAV2_BASELINE_OK', 'MANUAL_NAV2_BASELINE_FAIL', 'MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK']:
        assert token in text
    assert '不得写成自主探索成功' in text or '不声明自主探索成功' in text
