from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase64_corridor_width_robot_footprint_feasibility_decision.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase64_corridor_width_robot_footprint_feasibility_decision_report.md'


def load_module():
    spec = importlib.util.spec_from_file_location('phase64_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase64_analyzer_declares_static_geometry_only_guardrails():
    module = load_module()
    assert module.RUN_ID == 'phase64_corridor_width_robot_footprint_feasibility_decision'
    assert set(module.ALLOWED_CLASSIFICATIONS) == {
        'GEOMETRY_FEASIBILITY_BLOCKED',
        'GEOMETRY_FEASIBILITY_MARGINAL',
        'GEOMETRY_FEASIBILITY_OK',
        'INSUFFICIENT_EVIDENCE',
    }
    guardrail_text = ' '.join(module.GUARDRAILS)
    assert 'static geometry/config/log replay only' in guardrail_text
    assert 'no runtime dispatch integration' in guardrail_text
    assert 'no Nav2/MPPI/controller parameter edits' in guardrail_text
    assert 'no clearance_radius_m tuning' in guardrail_text
    assert 'no map sufficiency threshold tuning' in guardrail_text
    assert 'no branch selection scoring change' in guardrail_text
    assert 'no entrance/fallback/terminal acceptance change' in guardrail_text
    assert 'no autonomous exploration success claim' in guardrail_text
    assert 'first dispatch is not exit success' in guardrail_text


def test_phase64_extracts_nav2_radius_and_inflation_envelope_from_active_config():
    module = load_module()
    config = module.load_nav2_geometry_config(ROOT / 'src/tugbot_navigation/config/nav2_slam_params.yaml')
    assert config['config_file'].endswith('nav2_slam_params.yaml')
    assert config['local_costmap']['robot_radius_m'] == 0.35
    assert config['local_costmap']['inflation_radius_m'] == 0.46
    assert config['global_costmap']['robot_radius_m'] == 0.35
    assert config['global_costmap']['inflation_radius_m'] == 0.40
    envelope = module.compute_required_widths(config, mesh_turning_diameter_m=0.7382442013625243)
    assert envelope['robot_radius_diameter_m'] == 0.70
    assert envelope['mesh_turning_outer_diameter_m'] > envelope['robot_radius_diameter_m']
    assert envelope['inflation_full_envelope_width_m'] == 1.62
    assert envelope['robot_body_width_required_m'] >= 0.738
    assert envelope['comfort_contract_width_m'] >= 1.476


def test_phase64_world_geometry_and_first_dispatch_replay_evidence_contract():
    module = load_module()
    result = module.analyze_phase64(
        root=ROOT,
        phase62_artifact=ROOT / 'log/phase62_first_dispatch_local_cost_traversability_diagnostics/phase62_first_dispatch_local_cost_traversability_diagnostics.json',
        phase63_artifact=ROOT / 'log/phase63_first_dispatch_target_safety_projection_static_replay/phase63_first_dispatch_target_safety_projection_static_replay.json',
    )
    assert result['active_world'].endswith('tugbot_maze_world_20260528_clean_scaled2x.sdf')
    assert result['active_metadata'].endswith('maze_20260528_scaled_instance.yaml')
    assert result['nav2_geometry_config']['local_costmap']['robot_radius_m'] == 0.35
    assert result['robot_geometry']['mesh_turning_outer_diameter_m'] > 0.73
    assert result['world_corridor_geometry']['global_min_positive_wall_gap_m'] > 1.6
    assert result['first_dispatch_corridor_evidence']['phase62_classification_preserved'] == 'CORRIDOR_TOO_NARROW'
    assert result['first_dispatch_corridor_evidence']['phase63_classification_preserved'] == 'NO_SAFE_PROJECTION_IN_CORRIDOR'
    assert result['first_dispatch_corridor_evidence']['target_clearance_m'] < 0.5
    assert result['first_dispatch_corridor_evidence']['path_corridor_min_clearance_m'] < 0.5
    assert result['first_dispatch_corridor_evidence']['footprint_max_cost'] == 99
    assert result['first_dispatch_corridor_evidence']['local_radius_max_cost'] == 99
    assert result['first_dispatch_corridor_evidence']['target_crosses_narrow_passage'] is True


def test_phase64_feasibility_classification_is_geometry_blocked_or_marginal_with_margin_fields():
    module = load_module()
    result = module.analyze_phase64(
        root=ROOT,
        phase62_artifact=ROOT / 'log/phase62_first_dispatch_local_cost_traversability_diagnostics/phase62_first_dispatch_local_cost_traversability_diagnostics.json',
        phase63_artifact=ROOT / 'log/phase63_first_dispatch_target_safety_projection_static_replay/phase63_first_dispatch_target_safety_projection_static_replay.json',
    )
    assert result['classification'] in module.ALLOWED_CLASSIFICATIONS
    assert result['classification'] in {'GEOMETRY_FEASIBILITY_BLOCKED', 'GEOMETRY_FEASIBILITY_MARGINAL'}
    margins = result['feasibility_margins']
    assert margins['first_dispatch_clearance_minus_robot_radius_m'] < 0.10
    assert margins['first_dispatch_clearance_minus_mesh_radius_m'] < 0.05
    assert margins['first_dispatch_effective_width_minus_inflation_full_envelope_m'] < 0.0
    assert result['geometry_mismatch_escalated'] is True
    assert result['runtime_dispatch_changed'] is False
    assert result['complete_autonomous_success_claimed'] is False
    assert result['first_dispatch_is_not_exit_success'] is True


def test_phase64_report_mentions_static_only_and_stop_condition():
    assert REPORT.exists()
    text = REPORT.read_text()
    assert 'Phase64 Corridor Width / Robot Footprint Feasibility Decision' in text
    assert 'GEOMETRY_FEASIBILITY_' in text or 'INSUFFICIENT_EVIDENCE' in text
    assert 'CORRIDOR_TOO_NARROW' in text
    assert 'LOCAL_COSTMAP_INFLATION_DOMINANT' in text
    assert 'NO_SAFE_PROJECTION_IN_CORRIDOR' in text
    assert 'no runtime dispatch integration' in text
    assert 'not autonomous exploration success' in text
    assert 'not exit success' in text
    assert 'Do not enter Phase65' in text or '不进入 Phase65' in text
