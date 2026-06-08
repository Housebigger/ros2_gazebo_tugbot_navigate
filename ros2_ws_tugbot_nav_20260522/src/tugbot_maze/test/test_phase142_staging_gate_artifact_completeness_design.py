from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase142_staging_gate_artifact_completeness_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase142_staging_gate_artifact_completeness_design_report.md'
PHASE141_REPORT = ROOT / 'doc' / 'doc_report' / 'phase141_staging_gate_variability_artifact_source_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'


def _read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase142 artifact: {path}'
    return path.read_text(encoding='utf-8')


def test_phase142_proposal_declares_design_only_scope_and_runtime_guardrails():
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
        'No staging may be disabled',
        'No autonomous exploration success or exit success may be claimed',
        'Phase143 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase142_proposal_preserves_phase141_diagnostic_conclusion_and_gaps():
    text = _read(PROPOSAL)
    phase141 = PHASE141_REPORT.read_text(encoding='utf-8')
    required = [
        'Phase141 conclusion: STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY, supported=true',
        'Phase134/136 triggered corridor_alignment_staging',
        'candidate_count=63',
        'hard_safety_pass_candidate_count=0',
        'front_wedge_risk.max=99',
        'same_corridor=true',
        'two_side_wall_evidence=true',
        'Phase139 direct explore',
        'staging_reject_reason=single_step_forward_search_had_hard_safe_candidate',
        'hard_safety_pass_candidate_count=17',
        'lateral residual scalar',
        'target/local cost',
        'path_corridor_min_clearance',
        'target_clearance',
        'candidate_branch_count',
        'branch_angle',
        'diagnostic_artifact_source_only_not_runtime_success',
    ]
    for phrase in required:
        assert phrase in text
    assert 'STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY' in phase141
    assert 'supported: `true`' in phase141


def test_phase142_proposal_defines_required_artifact_completeness_contract_for_both_gate_paths():
    text = _read(PROPOSAL)
    required = [
        'direct-explore reject staging path',
        'staging-triggered path',
        'lateral_residual_before_m',
        'lateral_residual_after_m',
        'target_local_cost',
        'target_local_cost_max_radius',
        'path_corridor_min_clearance_m',
        'target_clearance_m',
        'candidate_branch_count',
        'last_open_direction_count',
        'last_candidate_count',
        'branch_angle',
        'selected_branch_geometry',
        'source_single_step.candidate_count',
        'source_single_step.hard_safety_pass_candidate_count',
        'source_single_step.hard_safe_candidate_summaries',
        'trigger_conditions.near_goal_lateral_residual',
        'trigger_conditions.single_step_forward_search_no_hard_safety_pass',
        'trigger_conditions.safety_floor_dominant_blocker',
        'trigger_conditions.execution_time_footprint_front_wedge_risk',
        'staging_reason',
        'staging_reject_reason',
    ]
    for phrase in required:
        assert phrase in text


def test_phase142_proposal_defines_direct_explore_classification_boundary():
    text = _read(PROPOSAL)
    required = [
        'staging_not_needed_direct_explore',
        'staging_expected_but_not_triggered',
        'single_step_forward_search_had_hard_safe_candidate',
        'must have source_single_step.hard_safety_pass_candidate_count > 0',
        'must have trigger_conditions.single_step_forward_search_no_hard_safety_pass=false',
        'missing hard-safe candidate summaries means STAGING_GATE_CONTRACT_AMBIGUOUS',
        'missing lateral/cost/clearance/branch geometry means INSUFFICIENT_STAGING_GATE_EVIDENCE',
    ]
    for phrase in required:
        assert phrase in text


def test_phase142_proposal_defines_phase143_minimal_implementation_scope_without_behavior_change():
    text = _read(PROPOSAL)
    required = [
        'Phase143 minimal implementation scope',
        'artifact/diagnostic serialization only',
        'No gate logic changes',
        'No threshold tuning',
        'No staging disablement',
        'No exploration strategy change',
        'No branch scoring change',
        'No centerline/fallback/terminal acceptance change',
        'do not convert timeout or direct explore into success',
        'Phase143 runtime is not authorized by Phase142',
    ]
    for phrase in required:
        assert phrase in text


def test_phase142_source_review_anchors_existing_gate_logic_without_requiring_implementation():
    proposal = _read(PROPOSAL)
    explorer = MAZE_EXPLORER.read_text(encoding='utf-8')
    perception = MAZE_PERCEPTION.read_text(encoding='utf-8')
    required_source_anchors = [
        'def _maybe_plan_corridor_alignment_staging',
        'plan_two_step_corridor_alignment_staging_goal',
        'def _two_step_staging_trigger_conditions',
        "'near_goal_lateral_residual'",
        "'single_step_forward_search_no_hard_safety_pass'",
        "'safety_floor_dominant_blocker'",
        "'execution_time_footprint_front_wedge_risk'",
        "return 'single_step_forward_search_had_hard_safe_candidate'",
        'source_single_step',
        'staging_reject_reason',
    ]
    combined = explorer + '\n' + perception
    for phrase in required_source_anchors:
        assert phrase in combined
    assert 'Phase142 is a design review and does not implement the serialization changes' in proposal
    assert 'maze_explorer.py and maze_perception.py were only read' in proposal


def test_phase142_report_records_completion_without_runtime_or_forbidden_claims():
    text = _read(REPORT)
    required = [
        'PHASE142_STAGING_GATE_ARTIFACT_COMPLETENESS_DESIGN_COMPLETE_STOP_BEFORE_PHASE143',
        'Staging gate artifact completeness design review',
        'DESIGN_ONLY',
        'focused static tests',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No staging was disabled',
        'No autonomous exploration success or exit success is claimed',
        'Phase143 not entered',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase127 timeout fixed',
        'Phase141 proved runtime success',
        'Phase143 implemented',
        'disable staging',
        'tune MPPI',
        'run Phase143 runtime',
    ]
    for phrase in forbidden:
        assert phrase not in text
