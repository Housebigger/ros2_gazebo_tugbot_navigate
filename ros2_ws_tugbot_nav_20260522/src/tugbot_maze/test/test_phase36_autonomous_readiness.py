import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / 'tools' / 'check_phase36_autonomous_readiness.py'
ACTIVE_WORLD_NAME = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'


def _run_check():
    assert TOOL.exists(), 'Phase36 readiness check tool must exist'
    result = subprocess.run(
        [sys.executable, str(TOOL), '--workspace-root', str(ROOT), '--json'],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    return json.loads(result.stdout)


def test_phase36_active_metadata_matches_scaled2x_truth():
    metadata_path = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
    data = yaml.safe_load(metadata_path.read_text())
    assert data['output_world'] == f'src/tugbot_gazebo/worlds/{ACTIVE_WORLD_NAME}'
    assert data['scale_factor'] == 2.0
    assert data['source_image'] == 'package://tugbot_maze/assets/maze_20260528.png'
    assert data['runtime_policy']['image_allowed_for_runtime_path_planning'] is False
    assert data['entrance']['status'] == 'candidate_aligned_to_detected_boundary_opening'
    assert data['exit']['status'] == 'candidate_aligned_to_detected_boundary_opening'
    assert data['entrance']['x_m'] == -11.011281
    assert data['entrance']['y_m'] == -9.02507
    assert data['entrance']['yaw_rad'] == 0.0
    assert data['exit']['x_m'] == 10.061281
    assert data['exit']['y_m'] == 9.058496
    assert data['exit']['radius_m'] == 1.2


def test_phase36_current_maze_launch_defaults_use_active_scaled_world():
    result = _run_check()
    launch = result['launch_defaults']
    assert set(launch) == {
        'src/tugbot_bringup/launch/tugbot_maze_slam.launch.py',
        'src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py',
        'src/tugbot_bringup/launch/tugbot_maze_explore.launch.py',
    }
    for rel_path, info in launch.items():
        assert info['active_world_default_present'] is True, rel_path
        assert info['old_scaffold_default_present'] is False, rel_path
        assert info['legacy_static_map_default_present'] is False, rel_path


def test_phase36b_explorer_launch_defaults_are_wired_to_active_scaled2x_metadata():
    result = _run_check()
    params = result['explorer_active_truth']
    assert params['source_status'] == 'launch_arguments_not_metadata_loaded'
    assert params['metadata_file_declared'] == 'maze_20260528_scaled_instance.yaml'
    assert params['metadata_file_exists'] is True
    assert params['uses_active_metadata_defaults'] is True
    assert params['launch_defaults']['entrance_x'] == '0.0'
    assert params['launch_defaults']['entrance_y'] == '0.0'
    assert params['launch_defaults']['entrance_yaw'] == '0.0'
    assert params['launch_defaults']['exit_x'] == '21.072562'
    assert params['launch_defaults']['exit_y'] == '18.083566'
    assert params['launch_defaults']['exit_radius'] == '1.2'
    assert params['expected_from_metadata']['entrance_x'] == 0.0
    assert params['expected_from_metadata']['entrance_y'] == 0.0
    assert params['expected_from_metadata']['entrance_yaw'] == 0.0
    assert params['expected_from_metadata']['exit_x'] == 21.072562
    assert params['expected_from_metadata']['exit_y'] == 18.083566
    assert params['expected_from_metadata']['exit_radius'] == 1.2


def test_phase36_goal_event_and_state_schema_is_sufficient_for_bounded_smoke():
    result = _run_check()
    events = result['goal_events_and_state_schema']
    assert events['goal_events_topic_configurable'] is True
    assert events['state_topic_configurable'] is True
    assert events['required_goal_event_fields_present'] is True
    assert events['required_state_fields_present'] is True
    assert not events['missing_goal_event_fields']
    assert not events['missing_state_fields']


def test_phase36_legacy_current_workflow_references_are_not_active_defaults():
    result = _run_check()
    refs = result['legacy_references']
    assert refs['active_source_legacy_files_absent'] is True
    assert refs['current_launch_legacy_default_refs'] == []
    assert refs['nav2_config_diff_empty'] is True
    assert refs['decorative_route_archived'] is True
    assert refs['old_scaffold_archived'] is True
    assert refs['legacy_static_maps_archived'] is True


def test_phase36b_readiness_is_ready_after_active_metadata_wiring_fix():
    result = _run_check()
    assert result['decision'] == 'READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE'
    assert result['blockers'] == []
    assert result['guardrails']['autonomous_exploration_started'] is False
    assert result['guardrails']['maze_explorer_started'] is False
    assert result['guardrails']['nav2_params_modified'] is False
    assert result['guardrails']['strategy_modified'] is False
