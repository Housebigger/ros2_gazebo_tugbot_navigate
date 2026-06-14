import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase46_nav2_lifecycle_activation_root_cause.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase46_nav2_lifecycle_activation_root_cause.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase46_nav2_lifecycle_activation_root_cause_report.md'
LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam_nav.launch.py'

RUN_ID = 'phase46_nav2_lifecycle_activation_root_cause'
CLASSES = {
    'LIFECYCLE_MANAGER_NAMESPACE_MISMATCH',
    'LIFECYCLE_MANAGER_EXITED',
    'MANAGED_NODE_LIST_MISMATCH',
    'LIFECYCLE_TRANSITION_BLOCKED_BY_NODE',
    'AUTOSTART_NOT_EFFECTIVE',
    'NAV2_LIFECYCLE_ACTIVE_RECOVERED',
    'INCONCLUSIVE',
}


def test_phase46_wrapper_contract_and_guardrails():
    text = WRAPPER.read_text()
    assert f'RUN_ID="{RUN_ID}"' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'use_sim_time:=true' in text
    assert 'autostart:=true' in text
    assert 'maze_explorer' not in _launch_command_section(text)
    assert 'maze_goal_monitor' not in _launch_command_section(text)
    assert 'frontier_explorer' not in _launch_command_section(text)
    for artifact in [
        'launch.log',
        'nodes.txt',
        'topics.txt',
        'actions.txt',
        'lifecycle_states_before_after.txt',
        'manager_params.txt',
        'managed_nodes.txt',
        'transition_service_results.txt',
        'diagnostics.json',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in text


def test_phase46_analyzer_classifications_and_inputs():
    text = ANALYZER.read_text()
    for cls in CLASSES:
        assert cls in text
    assert 'lifecycle_manager_navigation' in text
    assert 'node_names' in text
    assert 'transition_service_results' in text
    assert 'managed_nodes' in text
    assert 'docking_server' in text
    assert 'route_server' in text
    assert 'collision_monitor' in text


def test_phase46_static_analysis_detects_launch_and_params():
    import importlib.util
    spec = importlib.util.spec_from_file_location('phase46', ANALYZER)
    assert spec is not None
    assert spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    result = mod.static_analysis(ROOT)
    assert result['run_id'] == RUN_ID
    assert result['manual_launch']['exists'] is True
    assert result['manual_launch']['includes_nav2_bringup'] is True
    assert result['manual_launch']['mentions_navigation_launch.py'] is True
    assert result['manual_launch']['passes_autostart'] is True
    assert result['manual_launch']['passes_use_sim_time'] is True
    assert result['manual_launch']['forbidden_explorer_references'] == []
    assert result['nav2_params']['exists'] is True
    assert 'docking_server' in result['nav2_params']['declared_sections']
    assert 'route_server' in result['nav2_params']['declared_sections']


def test_phase46_report_scaffold_preserves_phase45_and_forbids_autonomy_claims():
    text = REPORT.read_text()
    assert 'Phase45' in text
    assert 'LIFECYCLE_NOT_ACTIVE' in text
    assert RUN_ID in text
    assert '不得写成自主探索成功' in text or '不声明自主探索成功' in text
    assert 'maze_explorer' in text
    assert 'Phase47' in text
    assert CLASSES & set(text.replace('`', '').replace(',', ' ').split())


def test_phase46_launch_still_uses_manual_slam_nav_without_explorer():
    text = LAUNCH.read_text()
    assert 'navigation_launch.py' in text
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
