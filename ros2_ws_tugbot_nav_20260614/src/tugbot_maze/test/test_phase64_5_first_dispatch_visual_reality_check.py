from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'phase64_5_first_dispatch_visual_overlay.py'
LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'phase64_5_first_dispatch_visual_reality_check.launch.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase64_5_first_dispatch_visual_reality_check_report.md'


def _load_node_module():
    spec = importlib.util.spec_from_file_location('phase64_5_first_dispatch_visual_overlay', NODE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase64_5_overlay_node_contract_exists_and_preserves_guardrails():
    assert NODE.exists(), 'Phase64.5 overlay marker node must exist'
    text = NODE.read_text()
    assert 'phase64_5_first_dispatch_visual_overlay' in text
    assert 'visualization_msgs.msg' in text
    assert 'Marker' in text and 'MarkerArray' in text
    assert '/phase64_5/first_dispatch_visual_markers' in text
    assert 'first_dispatch_target' in text
    assert 'robot_radius_circle' in text
    assert 'inflation_envelope_circle' in text
    assert 'nearest_wall' in text
    assert 'corridor_width' in text
    assert 'no runtime dispatch integration' in text
    assert 'no Nav2/MPPI/controller parameter edits' in text
    assert 'first dispatch is not exit success' in text
    assert 'autonomous exploration success' in text


def test_phase64_5_loads_phase64_artifact_into_overlay_payload():
    module = _load_node_module()
    artifact = ROOT / 'log' / 'phase64_corridor_width_robot_footprint_feasibility_decision' / 'phase64_corridor_width_robot_footprint_feasibility_decision.json'
    payload = module.load_overlay_payload(artifact)
    assert payload['classification'] == 'GEOMETRY_FEASIBILITY_BLOCKED'
    assert payload['target_map'] == [0.558242117171631, 0.5108474753216039]
    assert payload['dispatch_pose_map'] == [0.8740772410891823, 0.020696297436460016, 0.03198252951540831]
    assert payload['robot_radius_m'] == 0.35
    assert payload['inflation_radius_m'] == 0.7
    assert payload['inflation_full_radius_m'] == 1.05
    assert payload['first_dispatch_effective_width_m'] == 0.806226
    assert payload['target_clearance_m'] == 0.403113
    assert payload['nearest_wall_name'] == 'maze_wall_outer_026_outer_003'
    assert payload['phase62_classification_preserved'] == 'CORRIDOR_TOO_NARROW'
    assert payload['phase64_classification_preserved'] == 'GEOMETRY_FEASIBILITY_BLOCKED'


def test_phase64_5_marker_spec_contains_required_visual_elements():
    module = _load_node_module()
    artifact = ROOT / 'log' / 'phase64_corridor_width_robot_footprint_feasibility_decision' / 'phase64_corridor_width_robot_footprint_feasibility_decision.json'
    payload = module.load_overlay_payload(artifact)
    specs = module.build_marker_specs(payload)
    names = {spec['name'] for spec in specs}
    assert {
        'first_dispatch_target',
        'dispatch_pose',
        'robot_radius_circle_at_target',
        'mesh_radius_circle_at_target',
        'inflation_envelope_circle_at_target',
        'effective_corridor_width_line',
        'nearest_wall_distance_line',
        'manual_goal_comparison_note',
    }.issubset(names)
    target = next(spec for spec in specs if spec['name'] == 'first_dispatch_target')
    assert target['frame_id'] == 'map'
    assert target['position'][:2] == payload['target_map']
    inflation = next(spec for spec in specs if spec['name'] == 'inflation_envelope_circle_at_target')
    assert inflation['radius_m'] == 1.05
    assert inflation['purpose'] == 'show robot_radius + local inflation radius envelope'
    width = next(spec for spec in specs if spec['name'] == 'effective_corridor_width_line')
    assert width['width_m'] == 0.806226


def test_phase64_5_launch_contract_manual_visual_only():
    assert LAUNCH.exists(), 'Phase64.5 overlay launch must exist'
    text = LAUNCH.read_text()
    assert 'phase64_5_first_dispatch_visual_reality_check' in text
    assert 'phase64_5_first_dispatch_visual_overlay' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'headless' in text and 'false' in text
    assert 'use_rviz' in text and 'true' in text
    assert 'world_sdf' in text and 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'params_file' in text and 'nav2_slam_params.yaml' in text
    assert 'maze_explorer' not in text
    assert 'target_projection' not in text


def test_phase64_5_report_contract_and_manual_classification_pending():
    assert REPORT.exists(), 'Phase64.5 report must exist'
    text = REPORT.read_text()
    assert 'Phase64.5 First Dispatch Visual Reality Check / Manual Nav Comparison' in text
    assert 'VISUAL_EVIDENCE_INCONCLUSIVE' in text
    assert 'manual Nav2 Goal' in text
    assert 'first dispatch target marker' in text
    assert 'robot_radius' in text
    assert 'inflation envelope' in text
    assert 'nearest wall' in text
    assert 'corridor width' in text
    assert '不调 Nav2/MPPI/controller 参数' in text
    assert '不接入 target projection' in text
    assert '不声明 autonomous exploration success' in text
    assert '不声明 exit success' in text
