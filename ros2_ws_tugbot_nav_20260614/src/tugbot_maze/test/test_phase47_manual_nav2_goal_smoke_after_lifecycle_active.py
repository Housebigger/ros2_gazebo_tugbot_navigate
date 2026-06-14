from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase47_manual_nav2_goal_smoke_after_lifecycle_active.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase47_manual_nav2_goal_smoke_after_lifecycle_active_report.md'


def test_phase47_wrapper_exists_and_uses_manual_slam_nav_launch():
    text = WRAPPER.read_text()
    assert 'phase47_manual_nav2_goal_smoke_after_lifecycle_active' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'headless:=false' in text
    assert 'use_rviz:=true' in text
    assert 'autostart:=true' in text
    assert 'tugbot_maze_explore.launch.py' not in text


def test_phase47_wrapper_waits_for_required_readiness_gates_before_manual_goal():
    text = WRAPPER.read_text()
    required = [
        '/controller_server',
        '/planner_server',
        '/bt_navigator',
        '/navigate_to_pose',
        '/goal_pose',
        'Action servers: 1',
        'Subscription count: 1',
        'active [3]',
        'PHASE47_READY_FOR_MANUAL_NAV2_GOAL',
    ]
    for token in required:
        assert token in text
    assert text.index('/controller_server') < text.index('PHASE47_READY_FOR_MANUAL_NAV2_GOAL')
    assert text.index('/navigate_to_pose') < text.index('PHASE47_READY_FOR_MANUAL_NAV2_GOAL')
    assert text.index('/goal_pose') < text.index('PHASE47_READY_FOR_MANUAL_NAV2_GOAL')


def test_phase47_wrapper_records_manual_smoke_artifacts_and_keeps_run_alive():
    text = WRAPPER.read_text()
    artifacts = [
        'launch.log',
        'nodes.txt',
        'topics.txt',
        'actions.txt',
        'lifecycle_readiness.txt',
        'navigate_to_pose_action_info.txt',
        'goal_pose_topic_info.txt',
        'phase47_manual_nav2_goal_smoke_readiness.json',
        'manual_goal_observation_notes.md',
    ]
    for artifact in artifacts:
        assert artifact in text
    assert 'KEEP_ALIVE_FOR_HUMAN' in text
    assert 'RUN_ACTIVE_PENDING_HUMAN_MANUAL_GOAL_ACCEPTANCE' in text


def test_phase47_report_declares_manual_only_non_autonomous_status():
    text = REPORT.read_text()
    assert 'MANUAL_NAV2_GOAL_BASELINE_OK' in text
    assert 'NAV2_LIFECYCLE_ACTIVE_RECOVERED' in text
    assert 'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING' in text
    assert 'TOPOLOGY_REJECTION_CAUSE_IDENTIFIED' in text
    assert 'Gazebo/SLAM/Nav2/TF/manual RViz Nav2 Goal 基线可用' in text
    assert 'maze_explorer 自动拓扑采样/dispatch 入口问题' in text
    assert '不声明自主探索成功' in text
    assert '未修改 Nav2/MPPI/controller 参数' in text
    assert '未修改 `maze_explorer`' in text
    assert 'cleanup check: empty' in text
    assert 'Phase47' in text
