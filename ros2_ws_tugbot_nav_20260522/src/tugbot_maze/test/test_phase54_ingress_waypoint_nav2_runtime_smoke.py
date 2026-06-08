from pathlib import Path
import ast
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase54_ingress_waypoint_nav2_runtime_smoke.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase54_ingress_waypoint_nav2_runtime_smoke_report.md'
ARTIFACT = ROOT / 'log' / 'phase54_ingress_waypoint_nav2_runtime_smoke' / 'phase54_ingress_waypoint_nav2_runtime_smoke.json'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase54_wrapper_uses_slam_nav_not_explorer_and_targets_ingress_goal():
    text = _read(WRAPPER)
    assert 'RUN_ID="phase54_ingress_waypoint_nav2_runtime_smoke"' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tugbot_maze_explore.launch.py' not in text
    launch_start = text.index('timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py')
    launch_end = text.index('2>&1 | tee "$LAUNCH_LOG"', launch_start)
    launch_command = text[launch_start:launch_end]
    assert 'maze_goal_monitor' not in launch_command
    assert 'frontier_explorer' not in launch_command
    assert 'maze_explorer' not in launch_command
    assert 'INGRESS_X="1.0"' in text
    assert 'INGRESS_Y="0.0"' in text
    assert 'INGRESS_YAW="0.0"' in text
    assert 'PHASE54_ACCEPTANCE_RADIUS_M' in text
    assert 'ros2 action info /navigate_to_pose' in text
    assert 'ros2 topic info /goal_pose' in text
    assert 'ros2 lifecycle get /controller_server' in text
    assert 'ros2 lifecycle get /planner_server' in text
    assert 'ros2 lifecycle get /bt_navigator' in text
    assert 'analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --record-runtime' in text
    assert 'analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --analyze' in text


def test_phase54_analyzer_contract_and_guardrail_classification():
    tree = ast.parse(_read(ANALYZER))
    constants = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            try:
                constants[node.targets[0].id] = ast.literal_eval(node.value)
            except Exception:
                pass
    assert constants['RUN_ID'] == 'phase54_ingress_waypoint_nav2_runtime_smoke'
    assert constants['INGRESS_WAYPOINT_MAP'] == {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert constants['ACCEPTANCE_RADIUS_M'] <= 0.35
    assert constants['ALLOWED_CLASSIFICATIONS'] == {
        'INGRESS_NAV2_GOAL_REACHED',
        'INGRESS_NAV2_GOAL_FAILED',
        'INGRESS_NAV2_INCONCLUSIVE_RUNTIME_DATA_GAP',
        'GUARDRAIL_VIOLATION_EXPLORER_STARTED',
    }
    text = _read(ANALYZER)
    for token in [
        'NavigateToPose',
        '/plan',
        '/map',
        '/scan',
        '/local_costmap/costmap',
        'map->base_link',
        'inclusive_near_robot',
        'distance_to_ingress_m',
        'robot_moved_distance_m',
        'plan_evidence',
        'complete_autonomous_success_claimed',
    ]:
        assert token in text


def test_phase54_offline_analyzer_classifies_reached_fixture(tmp_path):
    runtime = tmp_path / 'runtime.json'
    action = tmp_path / 'action.json'
    cleanup = tmp_path / 'cleanup.txt'
    runtime.write_text(json.dumps({
        'samples': [
            {'elapsed_sec': 1.0, 'robot_pose_map': [0.0, 0.0, 0.0], 'map': {'available': True}, 'scan': {'available': True, 'finite_count': 200}, 'local_costmap': {'available': True}, 'tf_lookups': {'map->base_link': {'available': True}}, 'plan_evidence': {'available': False}},
            {'elapsed_sec': 20.0, 'robot_pose_map': [0.92, 0.02, 0.0], 'map': {'available': True, 'inclusive_near_robot': {'known_ratio': 1.0, 'free_ratio': 1.0}}, 'scan': {'available': True, 'finite_count': 220}, 'local_costmap': {'available': True, 'inclusive_near_robot': {'known_ratio': 1.0, 'free_ratio': 0.95}}, 'tf_lookups': {'map->base_link': {'available': True}}, 'plan_evidence': {'available': True, 'pose_count': 8}},
        ]
    }), encoding='utf-8')
    action.write_text(json.dumps({'goal_accepted': True, 'result_received': True, 'status': 4, 'error_code': 0, 'success': True}), encoding='utf-8')
    cleanup.write_text('', encoding='utf-8')
    out = tmp_path / 'summary.json'
    subprocess.run([
        sys.executable, str(ANALYZER), '--analyze', '--artifact-dir', str(tmp_path), '--output', str(out),
        '--runtime-evidence', str(runtime), '--action-result', str(action), '--cleanup-processes-after', str(cleanup)
    ], cwd=ROOT, check=True)
    data = json.loads(out.read_text(encoding='utf-8'))
    assert data['classification'] == 'INGRESS_NAV2_GOAL_REACHED'
    assert data['goal_reached_within_acceptance_radius'] is True
    assert data['robot_moved'] is True
    assert data['plan_generated'] is True
    assert data['complete_autonomous_success_claimed'] is False


def test_phase54_report_exists_after_runtime_analysis():
    text = _read(REPORT)
    data = json.loads(_read(ARTIFACT))
    assert data['classification'] in {
        'INGRESS_NAV2_GOAL_REACHED',
        'INGRESS_NAV2_GOAL_FAILED',
        'INGRESS_NAV2_INCONCLUSIVE_RUNTIME_DATA_GAP',
        'GUARDRAIL_VIOLATION_EXPLORER_STARTED',
    }
    assert data['ingress_waypoint_map'] == {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert 'Phase54: Ingress Waypoint Nav2 Runtime Smoke' in text
    assert data['classification'] in text
    assert 'autonomous exploration success' in text
    assert 'Phase55' in text
