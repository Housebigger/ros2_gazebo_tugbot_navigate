from pathlib import Path
import ast
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_phase53_entrance_ingress_waypoint.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase53_entrance_ingress_waypoint_design_report.md'
ARTIFACT = ROOT / 'log' / 'phase53_entrance_ingress_waypoint_design' / 'phase53_entrance_ingress_waypoint_design.json'
OVERLAY = ROOT / 'log' / 'phase53_entrance_ingress_waypoint_design' / 'phase53_entrance_ingress_waypoint_overlay.png'
NAV2_CONFIG = ROOT / 'src' / 'tugbot_navigation' / 'config'


def test_phase53_static_analyzer_exists_and_is_guarded():
    assert SCRIPT.exists()
    text = SCRIPT.read_text()
    forbidden_runtime_or_tuning_tokens = [
        'ros2 launch',
        'NavigateToPose',
        'max_goals',
        'clearance_radius_m =',
        'dispatch_readiness_min_map_known_ratio =',
        'src/tugbot_navigation/config',
        'tugbot_maze_world.sdf',
        'maze_20260522.jpg',
    ]
    for token in forbidden_runtime_or_tuning_tokens:
        assert token not in text
    assert '"maze_explorer_started": False' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'maze_20260528_scaled_instance.yaml' in text
    assert 'phase53_entrance_ingress_waypoint_design.json' in text
    assert 'phase53_entrance_ingress_waypoint_overlay.png' in text


def test_phase53_analyzer_ast_contract_mentions_required_classifications():
    assert SCRIPT.exists()
    tree = ast.parse(SCRIPT.read_text())
    constants = {node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)}
    for classification in [
        'INGRESS_WAYPOINT_GEOMETRY_VALIDATED',
        'INGRESS_WAYPOINT_BLOCKED',
        'INCONCLUSIVE_NEEDS_RUNTIME_CHECK',
    ]:
        assert classification in constants


def test_phase53_analyzer_generates_json_overlay_and_validated_candidate(tmp_path):
    result = subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr + result.stdout
    assert ARTIFACT.exists()
    assert OVERLAY.exists()
    data = json.loads(ARTIFACT.read_text())
    assert data['classification'] in {
        'INGRESS_WAYPOINT_GEOMETRY_VALIDATED',
        'INGRESS_WAYPOINT_BLOCKED',
        'INCONCLUSIVE_NEEDS_RUNTIME_CHECK',
    }
    assert data['guardrails']['maze_explorer_started'] is False
    assert data['guardrails']['dispatch_goal_sent'] is False
    assert data['active_world'].endswith('tugbot_maze_world_20260528_clean_scaled2x.sdf')
    assert data['active_config'].endswith('maze_20260528_scaled_instance.yaml')
    assert data['map_frame_truth']['entrance'] == {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    assert data['map_frame_truth']['exit']['x_m'] == 21.072562
    assert data['entrance_geometry']['inward_direction'] == '+X'
    assert data['selected_candidate'] is not None
    selected = data['selected_candidate']
    assert 0.8 <= selected['distance_from_entrance_m'] <= 1.5
    assert selected['point_map']['x_m'] > 0.0
    assert abs(selected['point_map']['y_m']) < 1e-9
    assert selected['inside_map_boundary'] is True
    assert selected['occupied_or_unknown'] is False
    assert selected['min_clearance_to_wall_m'] >= data['criteria']['min_wall_clearance_m']
    assert selected['short_straight_line_reachable'] is True


def test_phase53_report_exists_and_preserves_non_success_semantics():
    assert REPORT.exists()
    text = REPORT.read_text()
    assert 'Phase53' in text
    assert 'Entrance Ingress Waypoint Design and Geometry Validation' in text
    assert 'INGRESS_WAYPOINT_GEOMETRY_VALIDATED' in text
    assert 'ingress_waypoint_map' in text
    assert '不启动 maze_explorer' in text or 'maze_explorer not started' in text
    assert '不声明自主探索成功' in text or 'No autonomous exploration success claim' in text
    assert 'Phase54' in text
