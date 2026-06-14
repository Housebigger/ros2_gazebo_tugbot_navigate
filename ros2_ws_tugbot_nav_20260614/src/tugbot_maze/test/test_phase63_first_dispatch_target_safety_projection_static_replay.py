from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase63_first_dispatch_target_safety_projection_static_replay.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase63_first_dispatch_target_safety_projection_static_replay_report.md'
PHASE62_ARTIFACT = ROOT / 'log' / 'phase62_first_dispatch_local_cost_traversability_diagnostics' / 'phase62_first_dispatch_local_cost_traversability_diagnostics.json'
RUN_ID = 'phase63_first_dispatch_target_safety_projection_static_replay'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase63_analyzer_contract_static_only_and_preserves_phase62_phase61():
    source = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in source
    assert 'PHASE62_RUN_ID' in source
    assert 'phase62_first_dispatch_local_cost_traversability_diagnostics' in source
    assert 'CORRIDOR_TOO_NARROW' in source
    assert 'LOCAL_COSTMAP_INFLATION_DOMINANT' in source
    assert 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED' in source
    for guardrail in [
        'no runtime dispatch integration',
        'no Nav2/MPPI/controller parameter edits',
        'no clearance_radius_m tuning',
        'no map sufficiency threshold tuning',
        'no branch selection scoring change',
        'no entrance/fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'first dispatch is not exit success',
    ]:
        assert guardrail in source
    assert 'rclpy' not in source, 'Phase63 is static/log replay only and must not start ROS runtime'
    assert 'NavigateToPose' not in source, 'Phase63 must not send runtime goals'


def test_phase63_projection_functions_and_classifications_are_present():
    source = _read(ANALYZER)
    for name in [
        'extract_phase62_first_dispatch',
        'project_candidates_along_corridor',
        'evaluate_projection_candidate',
        'classify_phase63',
        'analyze_phase63',
    ]:
        assert f'def {name}' in source
    for classification in [
        'SAFE_PROJECTION_FOUND',
        'NO_SAFE_PROJECTION_IN_CORRIDOR',
        'INSUFFICIENT_REPLAY_EVIDENCE',
    ]:
        assert classification in source
    for evidence in [
        'projection_distance_m',
        'same_open_direction',
        'same_corridor',
        'occupancy',
        'local_radius_cost_summary',
        'footprint_cost_summary',
        'front_wedge_cost_summary',
        'safety_score',
        'improves_over_original',
    ]:
        assert evidence in source


def test_phase63_projection_candidate_geometry_helper_prefers_same_direction():
    spec = importlib.util.spec_from_file_location('phase63_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    dispatch = {'target': [1.0, 0.0], 'branch_angle': 0.0, 'dispatch_pose': [0.0, 0.0, 0.0]}
    candidates = module.project_candidates_along_corridor(dispatch, distances=[-0.2, 0.0, 0.2])
    assert len(candidates) == 3
    assert candidates[0]['projection_distance_m'] == -0.2
    assert candidates[0]['target'][0] < 1.0
    assert candidates[1]['target'] == [1.0, 0.0]
    assert candidates[2]['target'][0] > 1.0
    assert all(row['same_open_direction'] for row in candidates)


def test_phase63_analyzer_uses_phase62_artifact_and_emits_static_replay_schema(tmp_path):
    assert PHASE62_ARTIFACT.exists(), 'Phase63 must replay the accepted Phase62 artifact'
    spec = importlib.util.spec_from_file_location('phase63_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = tmp_path / 'phase63.json'
    result = module.analyze_phase63(PHASE62_ARTIFACT, tmp_path, output)
    assert output.exists()
    assert result['run_id'] == RUN_ID
    assert result['phase62_classification_preserved'] == 'CORRIDOR_TOO_NARROW'
    assert result['phase61_classification_preserved'] == 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED'
    assert result['runtime_dispatch_changed'] is False
    assert result['complete_autonomous_success_claimed'] is False
    assert result['first_dispatch_is_not_exit_success'] is True
    assert result['classification'] in {
        'SAFE_PROJECTION_FOUND',
        'NO_SAFE_PROJECTION_IN_CORRIDOR',
        'INSUFFICIENT_REPLAY_EVIDENCE',
    }
    assert isinstance(result['projection_candidates'], list)
    assert result['original_target_evidence']['target']
    if result['classification'] == 'SAFE_PROJECTION_FOUND':
        assert result['best_projection']['improves_over_original'] is True
        assert result['best_projection']['same_corridor'] is True


def test_phase63_report_contract_stop_condition():
    report = _read(REPORT)
    assert 'Phase63' in report
    assert 'First Dispatch Target Safety Projection Design / Static Replay' in report
    assert RUN_ID in report
    assert 'CORRIDOR_TOO_NARROW' in report
    assert 'LOCAL_COSTMAP_INFLATION_DOMINANT' in report
    assert 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED' in report
    assert 'static replay' in report.lower() or '静态' in report
    assert 'not autonomous exploration success' in report or '不是自主探索成功' in report
    assert 'not exit success' in report or '不是出口成功' in report
    assert '不进入 Phase64' in report or 'Do not enter Phase64' in report
