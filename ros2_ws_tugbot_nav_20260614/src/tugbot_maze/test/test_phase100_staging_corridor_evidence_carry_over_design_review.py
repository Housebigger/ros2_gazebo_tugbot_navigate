from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase100_staging_corridor_evidence_carry_over_design_review.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase100_staging_corridor_evidence_carry_over_design_review_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase100 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase100_design_document_declares_design_only_scope_and_decision() -> None:
    proposal = _text(PROPOSAL)
    assert 'Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`' in proposal
    assert 'DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION' in proposal
    assert 'Phase101 not entered' in proposal
    assert 'No runtime code is changed' in proposal
    assert 'No Phase88/92 logic is changed' in proposal


def test_phase100_carry_over_allows_only_corridor_level_evidence() -> None:
    proposal = _text(PROPOSAL)
    allowed = [
        'same_corridor',
        'two_side_wall_evidence',
        'corridor_heading',
        'wall clearance context',
        'source_forward_window',
    ]
    forbidden = [
        'hard_safety_pass',
        'safety_floor_ok',
        'occupancy_free',
        'target_has_clearance',
        'footprint_lethal_not_increased',
        'front_wedge_lethal_not_increased',
        'fresh scan/local_costmap/TF',
    ]
    for token in allowed:
        assert token in proposal
    for token in forbidden:
        assert token in proposal
    assert 'Carry-over must never set or imply hard_safety_pass=true' in proposal
    assert 'staging itself must recompute' in proposal


def test_phase100_trigger_and_reject_conditions_are_explicit() -> None:
    proposal = _text(PROPOSAL)
    trigger_tokens = [
        'staging window missing two-side-wall',
        'Phase88 forward window has same-corridor/two-side-wall',
        'robot/staging/forward target geometry coherent',
    ]
    reject_tokens = [
        'carry_over_source_stale',
        'frame_mismatch',
        'heading_mismatch',
        'forward_window_not_trustworthy',
        'staging_not_consistent_with_source_corridor',
    ]
    for token in trigger_tokens + reject_tokens:
        assert token in proposal


def test_phase100_future_goal_events_contract_and_guardrails() -> None:
    proposal = _text(PROPOSAL)
    required_fields = [
        'corridor_evidence_carry_over',
        'carry_over_source',
        'carry_over_applied',
        'carry_over_reject_reason',
        'source_forward_window',
        'staging_window',
        'safety_evidence_recomputed=true',
        'branch_scoring_changed=false',
        'fallback_terminal_acceptance_used=false',
    ]
    guardrails = [
        'No Nav2/MPPI/controller tuning',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'No branch scoring change',
        'No exploration order change',
        'No centerline gate change',
        'No directional readiness change',
        'No fallback/terminal acceptance change',
        'No autonomous exploration success claim',
        'No exit success claim',
    ]
    for token in required_fields + guardrails:
        assert token in proposal


def test_phase100_report_records_static_validation_and_stop_condition() -> None:
    report = _text(REPORT)
    assert 'Status: FINAL_DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION' in report
    assert 'Phase100 is design review only' in report
    assert 'Focused static tests' in report
    assert 'Nav2 config diff guard' in report
    assert 'runtime files changed: false' in report
    assert 'Phase101 not entered' in report
