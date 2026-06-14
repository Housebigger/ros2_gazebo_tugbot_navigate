from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase104_inner_ingress_tf_controller_preflight_hardening_design_review.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase104_inner_ingress_tf_controller_preflight_hardening_design_review_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase104 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase104_design_document_declares_design_only_scope_and_decision() -> None:
    proposal = _text(PROPOSAL)
    assert 'Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`' in proposal
    assert 'DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION' in proposal
    assert 'Phase105 not entered' in proposal
    assert 'No runtime code is changed' in proposal
    assert 'No Phase88/92/101 logic is changed' in proposal
    assert 'This design does not validate Phase101 carry-over' in proposal


def test_phase104_fail_closed_preflight_gates_are_explicit() -> None:
    proposal = _text(PROPOSAL)
    required_gates = [
        'map->base_link TF continuous stability window',
        'map->odom timestamp age gate',
        'odom->base_link timestamp age gate',
        'scan frame transform stability gate',
        'controller robot pose availability gate',
        'inner-ingress goal pose transform gate',
        'TF jump / cache-drop / robot-pose-unavailable detector gate',
        'bounded wait then fail-closed',
    ]
    for token in required_gates:
        assert token in proposal
    assert 'Do not send the ingress goal until every hard gate passes in the same bounded preflight window' in proposal


def test_phase104_reject_tokens_are_complete_and_fail_closed() -> None:
    proposal = _text(PROPOSAL)
    reject_tokens = [
        'ingress_tf_unstable',
        'ingress_map_base_tf_missing',
        'ingress_map_odom_tf_stale',
        'ingress_odom_base_tf_stale',
        'ingress_scan_transform_unstable',
        'ingress_controller_robot_pose_unavailable',
        'ingress_goal_pose_transform_unavailable',
        'ingress_preflight_timeout',
    ]
    for token in reject_tokens:
        assert token in proposal
    assert 'ingress_goal_sent=false' in proposal
    assert 'preflight rejection is not maze_explorer failure' in proposal


def test_phase104_future_artifacts_and_goal_diagnostics_contract() -> None:
    proposal = _text(PROPOSAL)
    required_fields = [
        'ingress_preflight',
        'tf_stability_window_sec',
        'tf_jump_count',
        'scan_transform_check',
        'controller_pose_check',
        'goal_pose_transform_check',
        'ingress_goal_sent=false when rejected',
        'ingress_preflight_reject_reason',
        'map_base_tf_age_sec',
        'map_odom_tf_age_sec',
        'odom_base_tf_age_sec',
        'bounded_wait_elapsed_sec',
    ]
    for token in required_fields:
        assert token in proposal


def test_phase104_guardrails_preserve_targets_strategy_and_nav2_params() -> None:
    proposal = _text(PROPOSAL)
    guardrails = [
        'Do not change the inner-ingress target',
        'No Nav2/MPPI/controller tuning',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'No maze_explorer strategy change',
        'No branch scoring change',
        'No exploration order change',
        'No centerline gate change',
        'No directional readiness change',
        'No fallback/terminal acceptance change',
        'No autonomous exploration success claim',
        'No exit success claim',
    ]
    for token in guardrails:
        assert token in proposal


def test_phase104_report_records_static_validation_and_stop_condition() -> None:
    report = _text(REPORT)
    assert 'Status: FINAL_DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION' in report
    assert 'Phase104 is design review only' in report
    assert 'Focused static tests' in report
    assert 'Nav2 config diff guard' in report
    assert 'maze runtime logic diff guard' in report
    assert 'Phase105 not entered' in report
