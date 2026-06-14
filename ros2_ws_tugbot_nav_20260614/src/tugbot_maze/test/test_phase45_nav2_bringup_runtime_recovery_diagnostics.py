import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase45_nav2_bringup_runtime_recovery_diagnostics.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase45_nav2_bringup_runtime_recovery.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase45_nav2_bringup_runtime_recovery_diagnostics_report.md'
MANUAL_LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam_nav.launch.py'


def test_phase45_wrapper_contract_and_guardrails():
    text = WRAPPER.read_text()
    assert 'phase45_nav2_bringup_runtime_recovery_diagnostics' in text
    assert 'log/${RUN_ID}' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'nav2_slam_params.yaml' in text
    assert 'slam_toolbox_params.yaml' in text
    assert 'maze_explorer' not in _launch_command_section(text)
    assert 'maze_goal_monitor' not in _launch_command_section(text)
    assert 'frontier_explorer' not in _launch_command_section(text)
    for artifact in [
        'launch.log',
        'nodes.txt',
        'topics.txt',
        'actions.txt',
        'lifecycle_states.txt',
        'params_snapshot',
        'navigate_to_pose_action_info.txt',
        'goal_pose_topic_info.txt',
        'nav2_process_tree.txt',
        'nav2_launch_static_analysis.json',
        'phase45_nav2_bringup_diagnostics.json',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in text


def test_phase45_analyzer_classification_vocabulary_and_inputs():
    text = ANALYZER.read_text()
    for label in [
        'NAV2_BRINGUP_NOT_INCLUDED',
        'NAV2_NODES_NOT_STARTED',
        'BT_NAVIGATOR_MISSING',
        'NAVIGATE_TO_POSE_SERVER_MISSING',
        'LIFECYCLE_NOT_ACTIVE',
        'PARAMS_OR_LAUNCH_ARGUMENT_ERROR',
        'NAV2_BASELINE_RECOVERED',
        'INCONCLUSIVE_NEEDS_MANUAL_LOG_REVIEW',
    ]:
        assert label in text
    for required in [
        'navigation_launch.py',
        'bringup_launch.py',
        'lifecycle_manager_navigation',
        'bt_navigator',
        'Action servers:',
        'params_file',
        'autostart',
        'use_sim_time',
    ]:
        assert required in text


def test_phase45_static_analysis_reports_nav2_include_and_launch_arguments(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location('phase45_analyzer', ANALYZER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.static_analyze(ROOT)
    assert result['manual_launch']['exists'] is True
    assert result['manual_launch']['includes_nav2_bringup'] is True
    assert result['manual_launch']['nav2_launch_file'] == 'navigation_launch.py'
    assert result['manual_launch']['uses_active_scaled2x_world'] is True
    assert result['manual_launch']['passes_params_file'] is True
    assert result['manual_launch']['passes_autostart'] is True
    assert result['manual_launch']['passes_use_sim_time'] is True
    assert result['manual_launch']['forbidden_explorer_references'] == []


def test_phase45_report_records_phase44_failure_and_no_autonomy_claim():
    text = REPORT.read_text()
    assert 'MANUAL_NAV2_BASELINE_FAIL' in text
    assert 'Phase44' in text
    assert '不得写成自主探索成功' in text or '不声明自主探索成功' in text
    assert 'phase45_nav2_bringup_runtime_recovery_diagnostics' in text
    assert '不进入 Phase46' in text


def test_phase45_manual_launch_static_contract_still_nav2_slam_only():
    text = MANUAL_LAUNCH.read_text()
    assert 'navigation_launch.py' in text
    assert 'nav2_bringup' in text
    assert 'online_async_launch.py' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'maze_explorer' not in text
    assert 'maze_goal_monitor' not in text
    assert 'frontier_explorer' not in text


def _launch_command_section(text: str) -> str:
    start = text.rfind('ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py')
    if start < 0:
        return text
    end = text.find('LAUNCH_PID=', start)
    return text[start:end if end >= 0 else len(text)]
