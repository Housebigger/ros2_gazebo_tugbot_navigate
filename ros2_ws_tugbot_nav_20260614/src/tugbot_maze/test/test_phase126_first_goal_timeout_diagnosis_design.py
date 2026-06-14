from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase126_first_goal_timeout_diagnosis_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase126_first_goal_timeout_diagnosis_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase126 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase126_proposal_is_doc_only_design_only_and_no_runtime():
    text = _text(PROPOSAL)
    required = [
        'Phase126',
        'DESIGN_ONLY',
        'doc-only/design-only',
        'do not launch Gazebo/RViz/Nav2 runtime',
        'do not send NavigateToPose goal',
        'do not start maze_explorer',
        'do not tune Nav2/MPPI/controller/config',
        'do not change exploration strategy',
        'do not change branch scoring',
        'do not change centerline gate',
        'do not change fallback',
        'do not change terminal acceptance',
        'do not claim autonomous exploration success',
        'do not claim exit success',
        'Phase127 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_phase125_timeout_facts_are_preserved_without_success_overclaim():
    text = _text(PROPOSAL)
    required = [
        'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL',
        'Phase120 ingress succeeded',
        'Phase122 handoff ready',
        'maze_explorer max_goals=1',
        'exactly one current-algorithm goal_kind=explore',
        'dispatch accepted',
        'goal_timeout',
        'robot_pose_at_result x=2.4438,y=1.0153',
        'distance_to_first_goal≈0.356m',
        'footprint_max=99',
        'front_max=100',
        'path_0_5m_max=73',
        'path_1_0m_max=84',
        'recoveries≈4',
        'timeout is not success',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_root_cause_taxonomy_covers_required_distinctions():
    text = _text(PROPOSAL)
    required = [
        'goal too close to obstacle',
        'local cost high',
        'path locally blocked',
        'controller oscillation/stall',
        'goal checker tolerance',
        'BT recovery loop',
        'candidate selection near wall',
        'target refinement landed in high-cost area',
        'distance-to-goal curve',
        'velocity-stall evidence',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_required_evidence_contract_is_explicit():
    text = _text(PROPOSAL)
    required = [
        'Nav2 feedback timeline',
        'cmd_vel',
        'odom velocity',
        'robot pose trace',
        'local costmap windows',
        'global costmap windows',
        'footprint/front/path cost sequence',
        'planned path',
        'recovery count',
        'distance-to-goal curve',
        'controller_server logs',
        'bt_navigator logs',
        'goal checker state',
        'tf freshness',
        'scan freshness',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_target_selection_diagnostics_are_defined():
    text = _text(PROPOSAL)
    required = [
        'candidate pose clearance',
        'frontier/topology evidence',
        'junction branch geometry',
        'candidate_family=junction',
        'candidate_rank=1',
        'target_local_cost',
        'target_local_cost_max_radius',
        'path_corridor_min_clearance_m',
        'branch_angle',
        'score_components',
        'target refinement',
        'selected branch vs lower-ranked branches',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_classification_contract_is_complete_and_evidence_gated():
    text = _text(PROPOSAL)
    required = [
        'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL',
        'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE',
        'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY',
        'INSUFFICIENT_TIMEOUT_EVIDENCE',
        'classification precedence',
        'evidence thresholds',
        'do not convert timeout/failure into success',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_phase127_scope_is_diagnosis_only_replay_or_offline_analyzer():
    text = _text(PROPOSAL)
    required = [
        'Phase127 allowed scope',
        'diagnosis-only replay/analyzer',
        'at most reuse Phase125 artifacts',
        'no parameter tuning',
        'no runtime exploration rerun',
        'no second exploration goal',
        'no MPPI/controller/goal checker change',
        'no Nav2 config change',
        'no algorithm change',
        'stop before repair',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_report_summarizes_design_only_work_and_guards():
    text = _text(REPORT)
    required = [
        'PHASE126_FIRST_GOAL_TIMEOUT_DIAGNOSIS_DESIGN_COMPLETE_STOP_BEFORE_PHASE127',
        'No implementation was performed',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'No exploration strategy was changed',
        'No autonomous exploration success or exit success is claimed',
        'Phase127 not entered',
        'focused static tests',
        'Nav2 config diff guard',
        'process guard',
    ]
    for phrase in required:
        assert phrase in text


def test_phase126_does_not_add_runtime_runner_or_analyzer():
    forbidden = [
        ROOT / 'tools' / 'run_phase126_first_goal_timeout_diagnosis_design.py',
        ROOT / 'tools' / 'analyze_phase126_first_goal_timeout_diagnosis_design.py',
    ]
    for path in forbidden:
        assert not path.exists(), f'Phase126 is doc-only; unexpected runtime file exists: {path}'
