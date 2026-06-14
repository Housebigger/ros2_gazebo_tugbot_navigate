from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase128_first_goal_timeout_instrumentation_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase128_first_goal_timeout_instrumentation_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase128 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase128_proposal_is_design_only_and_preserves_runtime_guardrails():
    text = _text(PROPOSAL)
    required = [
        'Phase128',
        'DESIGN_ONLY',
        'doc-only/design-only',
        'do not launch Gazebo/RViz/Nav2 runtime',
        'do not send NavigateToPose goal',
        'do not start maze_explorer',
        'do not send exploration goal',
        'do not tune Nav2/MPPI/controller/goal checker/config',
        'do not change exploration strategy',
        'do not change branch scoring',
        'do not change centerline gate',
        'do not change fallback',
        'do not change terminal acceptance',
        'do not claim autonomous exploration success',
        'do not claim exit success',
        'Phase129 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_preserves_phase127_diagnostic_classification_and_evidence_gaps():
    text = _text(PROPOSAL)
    required = [
        'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'diagnosis-only',
        'distance_remaining plateau',
        'recoveries=4',
        'footprint_max=99',
        'front_max=100',
        'path_0_5m_max=73',
        'path_1_0m_max=84',
        'local/global costmap windows',
        'cmd_vel timeline',
        'odom velocity timeline',
        'robot pose trace',
        'goal checker state',
        'classification remains diagnostic',
        'timeout is not success',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_motion_and_tolerance_instrumentation_contract_is_explicit():
    text = _text(PROPOSAL)
    required = [
        'cmd_vel timeline',
        'odom velocity timeline',
        'controller stall vs local-cost blocked',
        'robot pose trace',
        'distance error curve',
        'yaw error curve',
        'goal checker state',
        'tolerance edge',
        'commanded vs measured velocity',
        'distance-progress slope',
        'near-tolerance band',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_cost_path_and_log_instrumentation_contract_is_explicit():
    text = _text(PROPOSAL)
    required = [
        'local costmap windows',
        'global costmap windows',
        'footprint cost snapshot',
        'front wedge cost snapshot',
        'path corridor cost snapshot',
        'planned path',
        'BT recovery events',
        'controller_server logs',
        'bt_navigator logs',
        'dispatch-to-timeout',
        'frame and timestamp alignment',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_first_goal_candidate_risk_contract_is_explicit():
    text = _text(PROPOSAL)
    required = [
        'first goal candidate risk',
        'clearance',
        'target_local_cost_max_radius',
        'path_corridor_min_clearance',
        'branch geometry',
        'candidate_family',
        'candidate_rank',
        'selected branch vs lower-ranked branches',
        'near wall',
        'high-cost band',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_phase129_boundary_allows_only_bounded_first_goal_instrumentation():
    text = _text(PROPOSAL)
    required = [
        'Phase129 allowed scope',
        'repeat Phase125 bounded first-goal result smoke',
        'instrumentation only',
        'max_goals=1',
        'no second goal',
        'no parameter tuning',
        'no repair',
        'do not convert timeout into success',
        'stop after first goal terminal result',
        'bounded artifact',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_report_summarizes_design_review_and_static_verification():
    text = _text(REPORT)
    required = [
        'PHASE128_FIRST_GOAL_TIMEOUT_INSTRUMENTATION_DESIGN_COMPLETE_STOP_BEFORE_PHASE129',
        'No implementation was performed',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No exploration goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy was changed',
        'No autonomous exploration success or exit success is claimed',
        'focused static tests',
        'process guard',
        'Nav2 config diff guard',
        'Phase129 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase128_does_not_add_runtime_runner_or_analyzer():
    forbidden = [
        ROOT / 'tools' / 'run_phase128_first_goal_timeout_instrumentation_design.py',
        ROOT / 'tools' / 'analyze_phase128_first_goal_timeout_instrumentation_design.py',
        ROOT / 'tools' / 'run_phase129_first_goal_timeout_instrumented_smoke.py',
    ]
    for path in forbidden:
        assert not path.exists(), f'Phase128 is design-only; unexpected runtime file exists: {path}'
