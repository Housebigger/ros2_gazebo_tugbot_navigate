from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase130_first_dispatch_kind_discrepancy_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase130_first_dispatch_kind_discrepancy_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase130 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase130_proposal_is_design_only_and_preserves_guardrails():
    text = _text(PROPOSAL)
    required = [
        'Phase130',
        'DESIGN_ONLY',
        'doc-only/design-only',
        'do not launch Gazebo/RViz/Nav2 runtime',
        'do not send NavigateToPose goal',
        'do not start maze_explorer',
        'do not send exploration/corridor/staging goal',
        'do not tune Nav2/MPPI/controller/goal checker/config',
        'do not change exploration strategy',
        'do not change branch scoring',
        'do not change centerline gate',
        'do not change fallback',
        'do not change terminal acceptance',
        'do not claim autonomous exploration success',
        'do not claim exit success',
        'Phase131 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_preserves_phase129_result_and_phase127_prior_diagnosis_boundary():
    text = _text(PROPOSAL)
    required = [
        'INSUFFICIENT_TIMEOUT_EVIDENCE',
        'goal_kind=corridor_alignment_staging',
        'first_dispatch_not_explore_diagnostic_stop',
        'second_goal_dispatched=false',
        'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'artifact replay diagnosis',
        'Phase129 did not strengthen, repair, or override Phase127',
        'do not mix staging/corridor alignment with timeout replay',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_artifact_comparison_matrix_contract_is_explicit():
    text = _text(PROPOSAL)
    required = [
        'Phase124/125/129 artifact comparison matrix',
        'robot pose after ingress',
        'current_node_id',
        'topology state',
        'candidate_family',
        'candidate_rank',
        'candidate_count',
        'near_exit',
        'post_ingress flags',
        'active_edge/state machine',
        'goal_kind',
        'raw_target',
        'refined_target',
        'original_target',
        'last_open_direction_count',
        'last_candidate_count',
        'candidate_branch_count',
        'selection_reason',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_staging_trigger_hypotheses_are_defined_without_repair():
    text = _text(PROPOSAL)
    required = [
        'corridor_alignment_staging trigger conditions',
        'pose/yaw drift hypothesis',
        'centerline or corridor alignment hypothesis',
        'post-ingress context hypothesis',
        'topology consistency hypothesis',
        'candidate refinement hypothesis',
        'goal_count/max_goals interaction hypothesis',
        'state-machine or active-edge hypothesis',
        'diagnosis only; no repair',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_first_dispatch_kind_contract_and_classifications_are_explicit():
    text = _text(PROPOSAL)
    required = [
        'first dispatch kind contract',
        'Phase124/125 first-goal smoke only allows goal_kind=explore',
        'staging/corridor alignment must be classified separately',
        'FIRST_DISPATCH_KIND_STABLE_EXPLORE',
        'FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS',
        'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT',
        'FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS',
        'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_phase131_boundary_is_artifact_replay_only():
    text = _text(PROPOSAL)
    required = [
        'Phase131 allowed scope',
        'artifact replay/analyzer only',
        'compare existing Phase124/125/129 artifacts',
        'do not rerun runtime',
        'do not launch Gazebo/RViz/Nav2',
        'do not start maze_explorer',
        'do not send any goal',
        'do not tune or repair',
        'bounded offline artifact',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_report_summarizes_design_review_and_static_verification():
    text = _text(REPORT)
    required = [
        'PHASE130_FIRST_DISPATCH_KIND_DISCREPANCY_DESIGN_COMPLETE_STOP_BEFORE_PHASE131',
        'No implementation was performed',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No exploration/corridor/staging goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy was changed',
        'No autonomous exploration success or exit success is claimed',
        'focused static tests',
        'process guard',
        'protected config diff guard',
        'Phase131 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase130_does_not_add_runtime_runner_or_analyzer():
    forbidden = [
        ROOT / 'tools' / 'run_phase130_first_dispatch_kind_discrepancy_design.py',
        ROOT / 'tools' / 'analyze_phase130_first_dispatch_kind_discrepancy_design.py',
        ROOT / 'tools' / 'run_phase131_first_dispatch_kind_discrepancy_replay.py',
    ]
    for path in forbidden:
        assert not path.exists(), f'Phase130 is design-only; unexpected runtime file exists: {path}'
