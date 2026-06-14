from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase121_post_ingress_handoff_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase121_post_ingress_handoff_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase121 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase121_proposal_is_doc_only_design_only_and_no_runtime():
    text = _text(PROPOSAL)
    required = [
        'Phase121',
        'DESIGN_ONLY',
        'doc-only/design-only',
        'do not launch Gazebo/RViz/Nav2 runtime',
        'do not send NavigateToPose goal',
        'do not start maze_explorer',
        'do not tune Nav2/MPPI/controller/config',
        'do not change exploration strategy',
        'do not remove preflight',
        'do not claim autonomous exploration success',
        'do not claim exit success',
        'Phase122 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_handoff_preconditions_after_phase120_success():
    text = _text(PROPOSAL)
    required = [
        'same-run readiness wait passed',
        'preflight passed',
        'inner-ingress result SUCCEEDED',
        'robot pose near ingress goal',
        'no residual active goal',
        'frame_id=map',
        'x=2.0',
        'y=0.0',
        'yaw=0.0',
        'INGRESS_SUCCESS_HANDOFF_NOT_READY',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_handoff_artifact_schema_is_defined():
    text = _text(PROPOSAL)
    required = [
        'handoff_artifact',
        'ingress_goal_result',
        'robot_pose_after_ingress',
        'distance_to_ingress_goal',
        'orientation_error',
        'costmap_freshness',
        'scan_freshness',
        'tf_freshness',
        'nav2_action_idle_state',
        'costmap/scan/TF freshness',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_failure_classifications_are_complete():
    text = _text(PROPOSAL)
    required = [
        'INGRESS_SUCCESS_HANDOFF_NOT_READY',
        'POSE_NOT_AT_INGRESS_GOAL',
        'NAV2_ACTION_NOT_IDLE',
        'TF_OR_SCAN_STALE',
        'COSTMAP_NOT_READY',
        'fail-closed',
        'no maze_explorer startup on failed handoff',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_future_phase122_scope_is_handoff_smoke_only():
    text = _text(PROPOSAL)
    required = [
        'Phase122 allowed scope',
        'post-ingress handoff smoke only',
        'may start maze_explorer only after explicit Phase122+ authorization',
        'max_goals=0',
        'dry-start',
        'do not dispatch exploration goal',
        'does not validate Goal1',
        'does not validate carry-over',
        'does not validate staging',
        'does not validate exit success',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_proposal_preserves_strategy_and_branch_guardrails():
    text = _text(PROPOSAL)
    required = [
        'do not change branch scoring',
        'do not change centerline gate',
        'do not change fallback',
        'do not change terminal acceptance',
        'Phase120 single-goal success is not exploration success',
        'not autonomous exploration success',
        'not exit success',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_report_summarizes_design_only_work_and_guards():
    text = _text(REPORT)
    required = [
        'PHASE121_POST_INGRESS_HANDOFF_DESIGN_COMPLETE_STOP_BEFORE_PHASE122',
        'No implementation was performed',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'No exploration strategy was changed',
        'No autonomous exploration success or exit success is claimed',
        'Phase122 not entered',
        'focused static tests',
    ]
    for phrase in required:
        assert phrase in text


def test_phase121_does_not_add_runtime_runner_or_analyzer():
    forbidden = [
        ROOT / 'tools' / 'run_phase121_post_ingress_handoff_design.py',
        ROOT / 'tools' / 'analyze_phase121_post_ingress_handoff_design.py',
        ROOT / 'tools' / 'run_phase122_post_ingress_handoff_smoke.py',
    ]
    for path in forbidden:
        assert not path.exists(), f'Phase121 is doc-only; unexpected runtime file exists: {path}'
