from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase137_second_step_contract_serialization_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase137_second_step_contract_serialization_design_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE136_REPORT = ROOT / 'doc' / 'doc_report' / 'phase136_bounded_second_step_explore_after_staging_smoke_report.md'
PHASE136_ANALYSIS = ROOT / 'log' / 'phase136_bounded_second_step_explore_after_staging_smoke' / 'phase136_bounded_second_step_explore_after_staging_smoke_analysis.json'


def _read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase137 artifact: {path}'
    return path.read_text(encoding='utf-8')


def test_phase137_proposal_declares_design_only_scope_and_runtime_guardrails():
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
        'Phase138 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase137_proposal_anchors_phase136_gap_and_preserves_ambiguous_classification():
    text = _read(PROPOSAL)
    phase136 = _read(PHASE136_REPORT)
    analysis = _read(PHASE136_ANALYSIS)
    required = [
        'Phase136 final classification: SECOND_STEP_CONTRACT_AMBIGUOUS',
        'staging succeeded',
        'fresh scan/local_costmap/TF recorded',
        'unique second-step goal_kind=explore dispatch',
        'accepted=true',
        'third_goal_dispatched=false',
        'pending_corridor_alignment_second_step present',
        'second_step_forward_goal valid',
        'generated_after_fresh_evidence=true',
        'selected_candidate_target present',
        'skip_two_step_staging=True or equivalent recursion guard',
        'accepted must not be interpreted as success',
    ]
    for phrase in required:
        assert phrase in text
    assert 'SECOND_STEP_CONTRACT_AMBIGUOUS' in phase136
    assert '"pending_second_step_present": false' in analysis
    assert '"second_step_recursion_guard_present": false' in analysis


def test_phase137_proposal_defines_pending_second_step_serialization_contract():
    text = _read(PROPOSAL)
    required_fields = [
        'pending_corridor_alignment_second_step',
        'exists=true',
        'runtime_serialized=true',
        'original_goal_kind=explore',
        'original_target',
        'direction_rad',
        'start_node_id',
        'active_branch',
        'staging_goal_pose',
        'staging_plan',
        'staging_result_status_label',
        'staging_succeeded_wall_time_sec',
        'fresh_scan_received',
        'fresh_local_costmap_received',
        'fresh_tf_received',
        'freshness_sample_time_sec',
        'invalid_reason',
        'cleared_reason',
    ]
    for phrase in required_fields:
        assert phrase in text


def test_phase137_proposal_defines_second_step_forward_goal_validity_contract():
    text = _read(PROPOSAL)
    required_fields = [
        'second_step_forward_goal valid',
        'generated_after_fresh_evidence=true',
        'selected_candidate_target',
        'selected_candidate_yaw',
        'front_wedge_risk_after_staging',
        'lateral_residual_after',
        'candidate_count',
        'hard_safety_pass_candidate_count',
        'selected_candidate_index',
        'selection_priority_trace',
        'rejected_candidate_summaries',
        'map_frame_id=map',
        'finite numeric x/y/yaw',
        'goal_kind=explore',
        'valid=false if any required field is missing',
    ]
    for phrase in required_fields:
        assert phrase in text


def test_phase137_proposal_defines_freshness_and_recursion_guard_evidence_chain():
    text = _read(PROPOSAL)
    required = [
        'freshness evidence chain',
        'staging_result_wall_time_sec',
        'fresh_scan_sample_time_sec > staging_result_wall_time_sec',
        'fresh_local_costmap_sample_time_sec > staging_result_wall_time_sec',
        'fresh_tf_sample_time_sec >= staging_result_wall_time_sec',
        'generated_after_fresh_evidence=true only after all freshness booleans are true',
        'selected_candidate_target must be copied from second_step_forward_goal',
        'skip_two_step_staging=True',
        'recursion_guard={',
        'source=phase92_corridor_alignment_staging_second_step',
        'two_step_stage_dispatch_requested=false',
        'staging_applied=false on outgoing second-step dispatch',
        'prevent second-step from triggering staging again',
    ]
    for phrase in required:
        assert phrase in text


def test_phase137_proposal_defines_phase138_minimal_implementation_boundaries():
    text = _read(PROPOSAL)
    required = [
        'Phase138 minimal implementation scope',
        'artifact/serialization/diagnostic fields only',
        'No strategy change',
        'No branch scoring change',
        'No centerline change',
        'No fallback change',
        'No terminal acceptance change',
        'No Nav2/MPPI/controller/goal checker/config change',
        'No direct staging disablement',
        'do not change target selection',
        'do not change goal timing',
        'focused static tests first',
        'future bounded runtime must still stop after one second-step goal',
    ]
    for phrase in required:
        assert phrase in text


def test_phase137_report_records_completion_without_runtime_or_forbidden_claims():
    text = _read(REPORT)
    required = [
        'PHASE137_SECOND_STEP_CONTRACT_SERIALIZATION_DESIGN_COMPLETE_STOP_BEFORE_PHASE138',
        'Second-step contract serialization gap design review',
        'DESIGN_ONLY',
        'focused static tests',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No autonomous exploration success or exit success is claimed',
        'Phase138 not entered',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase127 timeout fixed',
        'disable corridor_alignment_staging now',
        'tune MPPI now',
        'increase goal checker tolerance now',
        'run full autonomous exploration in Phase138',
    ]
    for phrase in forbidden:
        assert phrase not in text


def test_phase137_source_review_terms_exist_but_phase137_does_not_modify_runtime_semantics():
    source = _read(MAZE_EXPLORER)
    required_source_terms = [
        'self.pending_corridor_alignment_second_step = {',
        "'original_goal_kind': requested_goal_kind",
        "'original_target': self._point_to_payload(original_target_xy)",
        "'fresh_scan_received': False",
        "'fresh_local_costmap_received': False",
        "'fresh_tf_received': False",
        'def _dispatch_second_step_after_corridor_alignment_staging',
        "pending['fresh_tf_received'] = True",
        'generate_second_step_forward_goal_after_staging',
        "generated_after_fresh_evidence",
        "selected_candidate_target",
        "'second_step_forward_goal': serialized_second_step",
        "self._send_goal((float(selected[0]), float(selected[1])), float(yaw if isinstance(yaw, (int, float)) else direction_rad), 'explore', skip_two_step_staging=True)",
    ]
    for phrase in required_source_terms:
        assert phrase in source

    proposal = _read(PROPOSAL)
    assert 'Phase137 does not modify maze_explorer.py' in proposal
