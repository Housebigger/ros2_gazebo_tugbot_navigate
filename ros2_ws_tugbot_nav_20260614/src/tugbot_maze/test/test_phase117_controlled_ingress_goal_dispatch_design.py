from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase117_controlled_ingress_goal_dispatch_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase117_controlled_ingress_goal_dispatch_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase117 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase117_proposal_exists_and_is_design_only_no_goal():
    text = _text(PROPOSAL)
    required = [
        'Phase117',
        'DESIGN_ONLY',
        'doc-only',
        'do not send NavigateToPose goal',
        'do not start maze_explorer',
        'do not tune Nav2/MPPI/controller/config',
        'do not change exploration strategy',
        'do not remove preflight',
        'do not claim autonomous exploration success',
        'do not claim exit success',
        'Phase118 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_proposal_requires_strict_preflight_pass_before_dispatch():
    text = _text(PROPOSAL)
    required = [
        'preflight.passed == true',
        'preflight.failed_gates == []',
        'dispatch_precondition_failed',
        'no dispatch when failed_gates is non-empty',
        'no dispatch when reject_reason is present',
        'fail-closed',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_proposal_locks_goal_identity_and_no_exploration_goal():
    text = _text(PROPOSAL)
    required = [
        'explicit inner-ingress goal',
        'frame_id=map',
        'x=2.0',
        'y=0.0',
        'yaw=0.0',
        'do not add exploration goal',
        'do not start maze_explorer automatically',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_dispatch_artifact_schema_and_state_transition_contract():
    text = _text(PROPOSAL)
    required = [
        'goal_pose',
        'frame_id',
        'stamp',
        'action_server_ready',
        'send_time',
        'accepted',
        'rejected',
        'result_status',
        'abort_text',
        'bounded_goal_result_wait_sec',
        'ingress_goal_sent=false before send',
        'ingress_goal_sent=true only after accepted',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_outcome_classifications_and_cleanup_rules():
    text = _text(PROPOSAL)
    required = [
        'PREFLIGHT_FAILED_NO_DISPATCH',
        'INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL',
        'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER',
        'INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL',
        'INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL',
        'INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL',
        'INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE',
        'timeout/abort/cancel are diagnostic fail',
        'cleanup/stop',
        'never auto-start maze_explorer',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_phase118_scope_is_single_goal_dispatch_smoke_only():
    text = _text(PROPOSAL)
    required = [
        'Phase118 allowed scope',
        'single-goal dispatch smoke only',
        'does not validate exploration success',
        'does not validate Goal1 carry-over',
        'does not validate staging',
        'does not validate exit success',
        'max_goals=0',
    ]
    for phrase in required:
        assert phrase in text


def test_phase117_report_summarizes_doc_only_work_and_verification():
    text = _text(REPORT)
    required = [
        'PHASE117_CONTROLLED_INGRESS_GOAL_DISPATCH_DESIGN_COMPLETE_STOP_BEFORE_PHASE118',
        'No implementation was performed',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'No exploration strategy was changed',
        'No autonomous exploration success or exit success is claimed',
        'Phase118 not entered',
        'focused static tests',
    ]
    for phrase in required:
        assert phrase in text
