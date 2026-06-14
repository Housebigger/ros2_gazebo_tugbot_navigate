import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase140_staging_gate_variability_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase140_staging_gate_variability_design_report.md'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE134 = ROOT / 'log' / 'phase134_bounded_corridor_alignment_staging_smoke' / 'phase134_bounded_corridor_alignment_staging_smoke.json'
PHASE136 = ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke.json'
PHASE139 = ROOT / 'log' / 'phase139_instrumented_second_step_contract_runtime_verification' / 'phase139_instrumented_second_step_contract_runtime_verification.json'


def _read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase140 artifact: {path}'
    return path.read_text(encoding='utf-8')


def _load(path: Path) -> dict:
    assert path.exists(), f'missing expected artifact: {path}'
    return json.loads(path.read_text(encoding='utf-8'))


def test_phase140_proposal_declares_design_only_scope_and_runtime_guardrails():
    text = _read(PROPOSAL)
    required = [
        'Status: DESIGN_ONLY',
        'doc-only/design-only',
        'No Gazebo/RViz/Nav2 runtime may be launched',
        'No NavigateToPose goal may be sent',
        'No maze_explorer may be started',
        'No staging/explore/third goal may be sent',
        'No Nav2/MPPI/controller/goal checker/config tuning may be performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made',
        'No direct staging disablement is authorized',
        'No autonomous exploration success or exit success may be claimed',
        'Phase141 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase140_artifact_comparison_records_staging_vs_direct_explore_inputs():
    text = _read(PROPOSAL)
    required = [
        'Phase134 first literal: goal_kind=corridor_alignment_staging',
        'Phase134 lateral_residual_before_m=0.175000002607703',
        'Phase134 front_wedge_risk max=99 lethal_count=9',
        'Phase134 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true',
        'Phase136 first literal: goal_kind=corridor_alignment_staging',
        'Phase136 lateral_residual_before_m=0.22500000335276146',
        'Phase136 front_wedge_risk max=99 lethal_count=9',
        'Phase136 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true',
        'Phase139 first literal: goal_kind=explore',
        'Phase139 first_literal_staging_applied=false',
        'Phase139 staging_reject_reason=single_step_forward_search_had_hard_safe_candidate',
        'Phase139 staging_executability_check checked=false',
        'candidate_count',
        'hard_safety_pass_candidate_count',
        'target_local_cost_max_radius',
        'path_corridor_min_clearance',
        'branch geometry',
    ]
    for phrase in required:
        assert phrase in text


def test_phase140_proposal_defines_classification_vocabulary_and_decision_boundaries():
    text = _read(PROPOSAL)
    required = [
        'STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE',
        'STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT',
        'STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY',
        'STAGING_GATE_CONTRACT_AMBIGUOUS',
        'INSUFFICIENT_STAGING_GATE_EVIDENCE',
        'staging_not_needed_direct_explore',
        'staging_expected_but_not_triggered',
        'single_step_forward_search_had_hard_safe_candidate means direct explore can be a valid single-step path',
        'first literal explore in Phase139 is not a second-step contract failure',
        'Phase138 serialization contract remains unverified but not invalidated',
        'Phase136 remains SECOND_STEP_CONTRACT_AMBIGUOUS',
    ]
    for phrase in required:
        assert phrase in text


def test_phase140_proposal_defines_phase141_artifact_source_analyzer_only_scope():
    text = _read(PROPOSAL)
    required = [
        'Phase141 scope: artifact/source analyzer only',
        'No Phase141 runtime',
        'No Gazebo/RViz/Nav2',
        'No NavigateToPose goal',
        'No maze_explorer',
        'Phase141 must compare Phase134/136/139 artifacts plus source gate logic only',
        'force observation',
        'classification matrix',
        'force observation must not disable staging and must not tune thresholds',
        'classification matrix must classify naturally observed artifacts without changing behavior',
        'do not force staging by disabling hard-safe direct explore',
    ]
    for phrase in required:
        assert phrase in text


def test_phase140_source_review_anchors_actual_staging_gate_logic_without_modifying_it():
    perception = _read(MAZE_PERCEPTION)
    explorer = _read(MAZE_EXPLORER)
    required_perception_terms = [
        'def _two_step_staging_trigger_conditions',
        "'near_goal_lateral_residual'",
        "'single_step_forward_search_no_hard_safety_pass'",
        "'safety_floor_dominant_blocker'",
        "'execution_time_footprint_front_wedge_risk'",
        'hard_count == 0',
        'not refinement_applied',
        'original_preserved',
        'def _two_step_trigger_reject_reason',
        "return 'single_step_forward_search_had_hard_safe_candidate'",
        'def plan_two_step_corridor_alignment_staging_goal',
        "'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal'",
    ]
    for phrase in required_perception_terms:
        assert phrase in perception
    required_explorer_terms = [
        'def _maybe_plan_corridor_alignment_staging',
        'skip_two_step_staging',
        'plan_two_step_corridor_alignment_staging_goal',
        'two_step_stage_dispatch_requested',
        'staging_reject_reason',
    ]
    for phrase in required_explorer_terms:
        assert phrase in explorer
    proposal = _read(PROPOSAL)
    assert 'Phase140 does not modify maze_explorer.py or maze_perception.py' in proposal


def test_phase140_existing_artifacts_support_observed_variability_claims():
    p134 = _load(PHASE134)
    p136 = _load(PHASE136)
    p139 = _load(PHASE139)
    first134 = p134['first_literal_dispatch']
    first136 = p136['first_literal_dispatch']
    first139 = p139['first_literal_dispatch']

    assert first134['goal_kind'] == 'corridor_alignment_staging'
    assert first134['staging_applied'] is True
    assert first134['staging_executability_check']['hard_safety_pass'] is True
    assert first134['staging_executability_check']['same_corridor'] is True
    assert first134['staging_executability_check']['two_side_wall_evidence'] is True

    assert first136['goal_kind'] == 'corridor_alignment_staging'
    assert first136['staging_applied'] is True
    assert first136['staging_executability_check']['hard_safety_pass'] is True
    assert p136['classification'] == 'SECOND_STEP_CONTRACT_AMBIGUOUS'

    assert first139['goal_kind'] == 'explore'
    assert first139['staging_applied'] is False
    assert first139['staging_reject_reason'] == 'single_step_forward_search_had_hard_safe_candidate'
    assert p139['classification'] == 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'
    assert p139['valid'] is False
    assert p139['third_goal_dispatched'] is False


def test_phase140_report_records_completion_without_runtime_or_forbidden_claims():
    text = _read(REPORT)
    required = [
        'PHASE140_STAGING_GATE_VARIABILITY_DESIGN_COMPLETE_STOP_BEFORE_PHASE141',
        'Staging gate variability design review',
        'DESIGN_ONLY',
        'focused static tests',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No autonomous exploration success or exit success is claimed',
        'Phase141 not entered',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase127 timeout fixed',
        'Phase138 contract verified by Phase139',
        'disable corridor_alignment_staging now',
        'tune MPPI now',
        'run Phase141 runtime',
    ]
    for phrase in forbidden:
        assert phrase not in text
