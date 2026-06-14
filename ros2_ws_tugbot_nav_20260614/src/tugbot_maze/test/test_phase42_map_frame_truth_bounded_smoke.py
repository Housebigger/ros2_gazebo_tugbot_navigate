from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PHASE37_WRAPPER = ROOT / 'tools' / 'run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh'
PHASE42_WRAPPER = ROOT / 'tools' / 'run_phase42_map_frame_truth_bounded_smoke.sh'
PHASE42_ANALYZER = ROOT / 'tools' / 'analyze_phase42_map_frame_truth_bounded_smoke.py'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
RUN_ID = 'phase42_map_frame_truth_bounded_smoke'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase42_wrapper_exists_and_uses_phase_specific_run_id():
    source = _read(PHASE42_WRAPPER)
    assert 'set -euo pipefail' in source
    assert f'RUN_ID="{RUN_ID}"' in source
    assert f'log/${{RUN_ID}}' in source
    assert 'Phase42 Map-frame Truth Bounded Autonomous Smoke' in source
    assert 'Phase37' not in source.split('RUN_ID=', 1)[0]


def test_phase42_wrapper_keeps_bounded_autonomous_smoke_limits_and_guardrails():
    source = _read(PHASE42_WRAPPER)
    assert 'RUN_TIMEOUT_SEC="${PHASE42_RUN_TIMEOUT_SEC:-900}"' in source
    assert 'MAX_GOALS="${PHASE42_MAX_GOALS:-4}"' in source
    assert 'timeout --preserve-status "$RUN_TIMEOUT_SEC"' in source
    assert 'max_goals:="$MAX_GOALS"' in source
    assert 'near_exit_fallback_enabled:=false' in source
    assert 'headless:=true' in source
    assert 'use_rviz:="${USE_RVIZ}"' in source
    assert 'complete_autonomous_success_claimed' in source
    assert 'false' in source


def test_phase42_wrapper_explicitly_uses_active_world_metadata_and_map_frame_truth():
    source = _read(PHASE42_WRAPPER)
    assert ACTIVE_WORLD in source
    assert ACTIVE_METADATA in source
    for token in [
        'entrance_x:="0.0"',
        'entrance_y:="0.0"',
        'entrance_yaw:="0.0"',
        'exit_x:="21.072562"',
        'exit_y:="18.083566"',
        'exit_radius:="1.2"',
        'active_truth_frame=map',
    ]:
        assert token in source
    launch_block = source[source.index('timeout --preserve-status "$RUN_TIMEOUT_SEC"'):source.index('2>&1 | tee "$LAUNCH_LOG"')]
    for legacy_token in [
        'maze_config:="$ROOT/src/tugbot_maze/config/maze_instance.yaml"',
        'world_sdf:="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world.sdf"',
        'entrance_x:="-4.0"',
        'entrance_y:="-3.0"',
        'exit_x:="4.0"',
        'exit_y:="3.0"',
        'exit_radius:="0.6"',
    ]:
        assert legacy_token not in launch_block


def test_phase42_wrapper_records_required_artifacts_and_cleanup():
    source = _read(PHASE42_WRAPPER)
    for artifact in [
        'launch.log',
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'map_once.txt',
        'scan_once.txt',
        'odom_once.txt',
        'tf_once.txt',
        'local_costmap_once.txt',
        'global_costmap_once.txt',
        'navigate_to_pose_action_info.txt',
        'summary.json',
        'acceptance_analysis.json',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in source
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in source
    assert 'record_explorer_state_series.py --topic /maze/explorer_state' in source
    assert 'ros2 action info /navigate_to_pose' in source
    assert 'trap cleanup EXIT' in source
    for cleanup_token in [
        'maze_explorer',
        'maze_goal_monitor',
        'ros_gz_bridge',
        'static_transform_publisher',
        'slam_toolbox',
        'controller_server',
        'planner_server',
        'bt_navigator',
        'gz sim',
    ]:
        assert cleanup_token in source


def test_phase42_analyzer_classifies_dispatch_and_no_dispatch_without_claiming_success():
    source = _read(PHASE42_ANALYZER)
    for token in [
        'PHASE42_ALLOWED_CLASSIFICATIONS',
        'BOUNDED_SMOKE_DISPATCHED_GOAL',
        'BOUNDED_SMOKE_NO_DISPATCH_MAP_INSUFFICIENCY',
        'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING',
        'BOUNDED_SMOKE_NO_DISPATCH_COSTMAP_BLOCKED',
        'BOUNDED_SMOKE_NO_DISPATCH_TF_ALIGNMENT',
        'BOUNDED_SMOKE_NO_DISPATCH_OTHER',
        'active_map_frame_truth_used',
        'dispatch_expected',
        'complete_autonomous_success_claimed',
    ]:
        assert token in source
    assert 'autonomous_success' not in source.lower().replace('complete_autonomous_success_claimed', '')


def test_phase37_wrapper_is_not_overwritten_by_phase42_run_id():
    source = _read(PHASE37_WRAPPER)
    assert 'RUN_ID="phase37_scaled_clean_world_maze_explorer_bounded_smoke"' in source
    assert RUN_ID not in source
