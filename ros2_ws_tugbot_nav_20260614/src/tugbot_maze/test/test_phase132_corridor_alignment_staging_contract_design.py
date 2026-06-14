from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase132_corridor_alignment_staging_contract_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase132_corridor_alignment_staging_contract_design_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'


def _read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase132 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase132_proposal_declares_design_only_scope_and_runtime_guardrails():
    text = _read(PROPOSAL)
    required = [
        'Status: DESIGN_ONLY',
        'doc-only/design-only',
        'No Gazebo/RViz/Nav2 runtime may be launched',
        'No NavigateToPose goal may be sent',
        'No maze_explorer may be started',
        'No exploration/corridor/staging goal may be sent',
        'No Nav2/MPPI/controller/goal checker/config tuning may be performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made',
        'No autonomous exploration success or exit success may be claimed',
        'Phase133 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase132_proposal_defines_goal_kind_relationship_and_staging_contract():
    text = _read(PROPOSAL)
    relationship_terms = [
        'corridor_alignment_staging is not goal_kind=explore',
        'front-end staging dispatch before a possible second-step exploration dispatch',
        'first exploration goal is the later second-step goal_kind=explore dispatch',
        'first dispatch may be a staging dispatch',
        'must not be mixed with first explore-goal timeout replay',
    ]
    contract_terms = [
        'original_target',
        'refined_target',
        'staging target',
        'pending_corridor_alignment_second_step',
        'second_step_forward_goal',
        'staging_reason',
        'staging_reject_reason',
        'staging_lateral_residual_before_m',
        'staging_lateral_residual_after_m',
        'front_wedge_risk',
        'staging_executability_check',
        'source_forward_window',
        'staging_window',
        'two_step_stage_dispatch_requested',
        'staging_applied',
    ]
    for phrase in relationship_terms + contract_terms:
        assert phrase in text


def test_phase132_proposal_defines_future_smoke_classification_vocabulary():
    text = _read(PROPOSAL)
    classifications = [
        'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
        'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
        'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
        'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
        'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
        'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
    ]
    for classification in classifications:
        assert classification in text
    assert 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP is not exploration success' in text
    assert 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL is not FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED' in text


def test_phase132_proposal_bounds_phase133_to_staging_contract_smoke_or_artifact_replay():
    text = _read(PROPOSAL)
    required = [
        'Phase133 may only be a staging-contract smoke or artifact replay',
        'Phase133 must not run full autonomous exploration',
        'Phase133 must not send a second exploration goal unless the accepted design explicitly chooses a bounded second-step contract smoke',
        'Phase133 must preserve max_goals or an equivalent first-dispatch/staging budget guard',
        'Phase133 must stop after the first classified staging or first classified explore dispatch outcome',
        'No direct staging disablement is authorized',
        'No Nav2 parameter tuning is authorized',
    ]
    for phrase in required:
        assert phrase in text


def test_phase132_report_records_completion_and_no_success_claims():
    text = _read(REPORT)
    required = [
        'PHASE132_CORRIDOR_ALIGNMENT_STAGING_CONTRACT_DESIGN_COMPLETE_STOP_BEFORE_PHASE133',
        'Corridor alignment staging contract design review',
        'DESIGN_ONLY',
        'corridor_alignment_staging is not goal_kind=explore',
        'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
        'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
        'staging accepted/succeeded is not exploration success',
        'staging accepted/succeeded is not exit success',
        'Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED remains diagnostic-only',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No exploration/corridor/staging goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'Phase133 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase132_source_semantics_support_design_without_source_modification():
    source = _read(MAZE_EXPLORER)
    required_source_terms = [
        'def _maybe_plan_corridor_alignment_staging',
        "goal_kind != 'explore'",
        'plan_two_step_corridor_alignment_staging_goal',
        'def _dispatch_second_step_after_corridor_alignment_staging',
        'pending_corridor_alignment_second_step',
        "goal_kind = 'corridor_alignment_staging'",
        "'original_goal_kind': requested_goal_kind",
        "'original_target': self._point_to_payload(original_target_xy)",
        "'two_step_stage_dispatch_requested'",
        "'staging_reason'",
        "'second_step_forward_goal'",
        "'staging_applied'",
        "corridor-alignment staging succeeded; waiting for fresh scan/local_costmap/TF before second-step forward goal",
    ]
    for phrase in required_source_terms:
        assert phrase in source


def test_phase132_does_not_add_runtime_runner_or_modify_nav_config_contract():
    forbidden_runtime_runner = ROOT / 'tools' / 'run_phase132_corridor_alignment_staging_contract_design.py'
    assert not forbidden_runtime_runner.exists()
    for doc in (PROPOSAL, REPORT):
        text = _read(doc)
        forbidden_phrases = [
            'autonomous exploration success achieved',
            'exit success achieved',
            'disable corridor_alignment_staging now',
            'tune MPPI now',
            'increase goal checker tolerance now',
        ]
        for phrase in forbidden_phrases:
            assert phrase not in text
