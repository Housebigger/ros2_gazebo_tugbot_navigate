import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase141_staging_gate_variability_artifact_source.py'
LOG_DIR = ROOT / 'log' / 'phase141_staging_gate_variability_artifact_source'
ANALYSIS_JSON = LOG_DIR / 'phase141_staging_gate_variability_artifact_source_analysis.json'
ANALYSIS_MD = LOG_DIR / 'phase141_staging_gate_variability_artifact_source_analysis.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase141_staging_gate_variability_artifact_source_report.md'
PHASE134 = ROOT / 'log' / 'phase134_bounded_corridor_alignment_staging_smoke' / 'phase134_bounded_corridor_alignment_staging_smoke.json'
PHASE136 = ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke.json'
PHASE139 = ROOT / 'log' / 'phase139_instrumented_second_step_contract_runtime_verification' / 'phase139_instrumented_second_step_contract_runtime_verification.json'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase141_analyzer', ANALYZER)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _run_analyzer():
    assert ANALYZER.exists(), f'missing analyzer: {ANALYZER}'
    result = subprocess.run(
        [sys.executable, str(ANALYZER), '--root', str(ROOT)],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    assert 'PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_ANALYSIS_COMPLETE' in result.stdout
    assert ANALYSIS_JSON.exists()
    return json.loads(ANALYSIS_JSON.read_text(encoding='utf-8'))


def test_phase141_analyzer_declares_artifact_source_only_scope_and_inputs():
    module = _load_analyzer()
    assert module.PHASE == 'phase141_staging_gate_variability_artifact_source'
    assert module.ALLOWED_CLASSIFICATIONS == {
        'STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE',
        'STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT',
        'STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY',
        'STAGING_GATE_CONTRACT_AMBIGUOUS',
        'INSUFFICIENT_STAGING_GATE_EVIDENCE',
    }
    assert [entry.phase for entry in module.ARTIFACTS] == ['Phase134', 'Phase136', 'Phase139']
    assert str(PHASE134.relative_to(ROOT)) in [str(entry.path) for entry in module.ARTIFACTS]
    assert str(PHASE136.relative_to(ROOT)) in [str(entry.path) for entry in module.ARTIFACTS]
    assert str(PHASE139.relative_to(ROOT)) in [str(entry.path) for entry in module.ARTIFACTS]
    assert module.SOURCE_FILES == [
        Path('src/tugbot_maze/tugbot_maze/maze_explorer.py'),
        Path('src/tugbot_maze/tugbot_maze/maze_perception.py'),
    ]


def test_phase141_analyzer_extracts_required_normalized_gate_rows_from_existing_artifacts():
    module = _load_analyzer()
    rows = module.analyze(ROOT)['normalized_rows']
    by_phase = {row['phase']: row for row in rows}
    assert set(by_phase) == {'Phase134', 'Phase136', 'Phase139'}

    for phase, row in by_phase.items():
        for field in [
            'lateral_residual_before_m',
            'lateral_residual_after_m',
            'front_wedge_risk',
            'source_single_step',
            'same_corridor',
            'two_side_wall_evidence',
            'candidate_count',
            'hard_safety_pass_candidate_count',
            'target_local_cost',
            'target_local_cost_max_radius',
            'path_corridor_min_clearance_m',
            'target_clearance_m',
            'candidate_branch_count',
            'last_open_direction_count',
            'last_candidate_count',
            'selected_branch_geometry',
            'branch_angle',
            'staging_reject_reason',
            'staging_reason',
            'evidence_gaps',
        ]:
            assert field in row, f'{phase} missing normalized field {field}'

    assert by_phase['Phase134']['classification'] == 'STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT'
    assert by_phase['Phase134']['goal_kind'] == 'corridor_alignment_staging'
    assert by_phase['Phase134']['staging_applied'] is True
    assert by_phase['Phase134']['candidate_count'] == 63
    assert by_phase['Phase134']['hard_safety_pass_candidate_count'] == 0
    assert by_phase['Phase134']['front_wedge_risk']['max'] == 99
    assert by_phase['Phase134']['same_corridor'] is True
    assert by_phase['Phase134']['two_side_wall_evidence'] is True

    assert by_phase['Phase136']['classification'] == 'STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT'
    assert by_phase['Phase136']['goal_kind'] == 'corridor_alignment_staging'
    assert by_phase['Phase136']['staging_applied'] is True
    assert by_phase['Phase136']['candidate_count'] == 63
    assert by_phase['Phase136']['hard_safety_pass_candidate_count'] == 0
    assert by_phase['Phase136']['front_wedge_risk']['max'] == 99
    assert by_phase['Phase136']['same_corridor'] is True
    assert by_phase['Phase136']['two_side_wall_evidence'] is True

    assert by_phase['Phase139']['classification'] == 'STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE'
    assert by_phase['Phase139']['goal_kind'] == 'explore'
    assert by_phase['Phase139']['staging_applied'] is False
    assert by_phase['Phase139']['staging_reject_reason'] == 'single_step_forward_search_had_hard_safe_candidate'
    assert by_phase['Phase139']['candidate_count'] == 63
    assert by_phase['Phase139']['hard_safety_pass_candidate_count'] == 17
    assert by_phase['Phase139']['trigger_conditions']['single_step_forward_search_no_hard_safety_pass'] is False


def test_phase141_analyzer_judges_cross_phase_variability_by_candidate_cost_geometry_evidence_with_gaps():
    module = _load_analyzer()
    result = module.analyze(ROOT)
    cross = result['cross_phase_assessment']
    assert cross['classification'] == 'STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY'
    assert cross['supported'] is True
    assert cross['confidence'] == 'supported_with_explicit_evidence_gaps'
    assert cross['decision_boundary'] == 'diagnostic_artifact_source_only_not_runtime_success'
    assert cross['triggered_phases'] == ['Phase134', 'Phase136']
    assert cross['direct_explore_phases'] == ['Phase139']
    required_support = [
        'Phase134/Phase136 source single-step candidate_count=63 with hard_safety_pass_candidate_count=0',
        'Phase139 source single-step candidate_count=63 with hard_safety_pass_candidate_count=17',
        'Phase139 reject reason single_step_forward_search_had_hard_safe_candidate matches source trigger false condition',
        'all compared samples retain high front_wedge risk evidence',
        'same corridor and two-side wall evidence are present for staging-triggered samples',
    ]
    for phrase in required_support:
        assert phrase in cross['supporting_evidence']
    required_gaps = [
        'Phase139 terminal branch geometry/local cost/clearance fields are absent from the Phase139 bounded artifact',
        'Phase139 lateral residual scalar is absent even though trigger_conditions.near_goal_lateral_residual is true',
    ]
    for phrase in required_gaps:
        assert phrase in cross['evidence_gaps']


def test_phase141_analyzer_source_scan_records_gate_logic_anchors_without_modifying_sources():
    module = _load_analyzer()
    result = module.analyze(ROOT)
    source = result['source_review']
    assert source['read_only_files'] == [str(MAZE_EXPLORER.relative_to(ROOT)), str(MAZE_PERCEPTION.relative_to(ROOT))]
    assert source['all_required_anchors_present'] is True
    required = [
        'def _maybe_plan_corridor_alignment_staging',
        'plan_two_step_corridor_alignment_staging_goal',
        'def _two_step_staging_trigger_conditions',
        "'near_goal_lateral_residual'",
        "'single_step_forward_search_no_hard_safety_pass'",
        "'safety_floor_dominant_blocker'",
        "'execution_time_footprint_front_wedge_risk'",
        "return 'single_step_forward_search_had_hard_safe_candidate'",
    ]
    for anchor in required:
        assert anchor in source['present_anchors']


def test_phase141_cli_writes_json_markdown_and_report_without_success_or_tuning_claims():
    analysis = _run_analyzer()
    assert analysis['phase'] == 'phase141_staging_gate_variability_artifact_source'
    assert analysis['status'] == 'PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_COMPLETE_STOP_BEFORE_PHASE142'
    assert analysis['scope']['runtime_started'] is False
    assert analysis['scope']['goals_sent'] is False
    assert analysis['scope']['config_tuned'] is False
    assert analysis['scope']['strategy_changed'] is False
    assert analysis['scope']['success_claimed'] is False
    assert ANALYSIS_MD.exists()
    assert REPORT.exists()
    report = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_COMPLETE_STOP_BEFORE_PHASE142',
        'artifact/source analyzer only',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No autonomous exploration success or exit success is claimed',
        'Phase142 not entered',
    ]:
        assert phrase in report
    for forbidden in [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase127 timeout fixed',
        'Phase138 runtime contract verified',
        'disable staging',
        'tune MPPI',
    ]:
        assert forbidden not in report
