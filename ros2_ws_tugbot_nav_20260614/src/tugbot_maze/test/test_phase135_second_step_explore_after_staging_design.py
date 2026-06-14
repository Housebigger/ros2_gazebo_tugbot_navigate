from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase135_second_step_explore_after_staging_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase135_second_step_explore_after_staging_design_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'


def _read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase135 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase135_proposal_declares_design_only_scope_and_runtime_guardrails():
    text = _read(PROPOSAL)
    required = [
        'Status: DESIGN_ONLY',
        'doc-only/design-only',
        'No Gazebo/RViz/Nav2 runtime may be launched',
        'No NavigateToPose goal may be sent',
        'No maze_explorer may be started',
        'No staging/explore/second-step goal may be sent',
        'No Nav2/MPPI/controller/goal checker/config tuning may be performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made',
        'No direct staging disablement is authorized',
        'No autonomous exploration success or exit success may be claimed',
        'Phase136 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase135_proposal_defines_second_step_preconditions_after_staging_success():
    text = _read(PROPOSAL)
    required = [
        'staging accepted/succeeded',
        'fresh scan/local_costmap/TF',
        'pending_corridor_alignment_second_step present',
        'second_step_forward_goal valid',
        'generated_after_fresh_evidence=true',
        'selected_candidate_target',
        'goal_kind=explore',
        'fail closed',
        'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
    ]
    for phrase in required:
        assert phrase in text


def test_phase135_proposal_defines_second_step_artifact_contract():
    text = _read(PROPOSAL)
    required_fields = [
        'original_target',
        'staging target',
        'second_step_forward_goal',
        'freshness after staging',
        'front_wedge_risk_after_staging',
        'lateral residual after',
        'goal_kind=explore',
        'pending_corridor_alignment_second_step',
        'fresh_scan_received',
        'fresh_local_costmap_received',
        'fresh_tf_received',
        'accepted',
        'rejected',
        'timeout',
        'result_status_label',
        'abort_text',
        'second_step_goal_count=1',
        'third_goal_dispatched=false',
    ]
    for phrase in required_fields:
        assert phrase in text


def test_phase135_proposal_bounds_phase136_to_one_second_step_explore_goal():
    text = _read(PROPOSAL)
    required = [
        'Phase136 allowed scope',
        'only after a bounded staging result succeeded',
        'send exactly one second-step goal_kind=explore',
        'stop after accepted/rejected/timeout/result',
        'must not send a third goal',
        'must not run full autonomous exploration',
        'must preserve the first staging dispatch evidence',
        'must not tune Nav2 parameters',
        'must not change exploration strategy',
        'must not disable staging',
    ]
    for phrase in required:
        assert phrase in text


def test_phase135_proposal_defines_classification_vocabulary():
    text = _read(PROPOSAL)
    classifications = [
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP',
        'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
        'SECOND_STEP_CONTRACT_AMBIGUOUS',
    ]
    for classification in classifications:
        assert classification in text
    forbidden_interpretations = [
        'second-step accepted is not autonomous exploration success',
        'second-step succeeded is not exit success',
        'second-step result must not be used to claim Phase127 timeout is fixed',
    ]
    for phrase in forbidden_interpretations:
        assert phrase in text


def test_phase135_report_records_completion_and_static_verification_boundaries():
    text = _read(REPORT)
    required = [
        'PHASE135_SECOND_STEP_EXPLORE_AFTER_STAGING_DESIGN_COMPLETE_STOP_BEFORE_PHASE136',
        'Second-step explore dispatch after staging design review',
        'DESIGN_ONLY',
        'focused static tests',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No staging/explore/second-step goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No autonomous exploration success or exit success is claimed',
        'Phase136 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase135_source_semantics_support_pending_second_step_design_without_source_modification():
    source = _read(MAZE_EXPLORER)
    required_source_terms = [
        'def _dispatch_second_step_after_corridor_alignment_staging',
        'pending_corridor_alignment_second_step',
        "pending['fresh_tf_received'] = True",
        'fresh_scan_received=bool(pending.get',
        'fresh_local_costmap_received=bool(pending.get',
        'fresh_tf_received=bool(pending.get',
        "generated_after_fresh_evidence",
        "selected_candidate_target",
        "self._send_goal((float(selected[0]), float(selected[1])), float(yaw if isinstance(yaw, (int, float)) else direction_rad), 'explore', skip_two_step_staging=True)",
        "'original_goal_kind': requested_goal_kind",
        "'fresh_scan_received': False",
        "'fresh_local_costmap_received': False",
        "'fresh_tf_received': False",
        "corridor-alignment staging succeeded; waiting for fresh scan/local_costmap/TF before second-step forward goal",
    ]
    for phrase in required_source_terms:
        assert phrase in source


def test_phase135_design_only_does_not_add_runtime_tools_or_forbidden_claims():
    forbidden_paths = [
        ROOT / 'tools' / 'run_phase135_second_step_explore_after_staging_design.py',
        ROOT / 'tools' / 'analyze_phase135_second_step_explore_after_staging_design.py',
        ROOT / 'tools' / 'run_phase136_second_step_explore_after_staging_smoke.py',
    ]
    for path in forbidden_paths:
        assert not path.exists(), f'Phase135 is design-only; unexpected runtime tool exists: {path}'

    for doc in (PROPOSAL, REPORT):
        text = _read(doc)
        forbidden_phrases = [
            'autonomous exploration success achieved',
            'exit success achieved',
            'Phase127 timeout fixed',
            'disable corridor_alignment_staging now',
            'tune MPPI now',
            'increase goal checker tolerance now',
            'run full autonomous exploration in Phase136',
        ]
        for phrase in forbidden_phrases:
            assert phrase not in text
