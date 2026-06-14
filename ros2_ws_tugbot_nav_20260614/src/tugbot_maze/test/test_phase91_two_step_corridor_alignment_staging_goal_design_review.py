from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase91_two_step_corridor_alignment_staging_goal_design_review.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase91_two_step_corridor_alignment_staging_goal_design_review_report.md'
PHASE90_JSON = ROOT / 'log' / 'phase90_rejected_candidate_failure_landscape_diagnosis' / 'phase90_rejected_candidate_failure_landscape_diagnosis.json'

TRIGGER_TOKENS = [
    'near-goal lateral residual',
    'single-step forward-search no hard-safety-pass',
    'safety_floor dominant blocker',
    'execution-time footprint/front-wedge risk',
]

STAGING_GENERATION_TOKENS = [
    'short-distance staging pose',
    'low-risk staging motion',
    'corridor heading',
    'reduce lateral residual',
    'reduce front-wedge risk',
]

SECOND_STEP_TOKENS_IN_ORDER = [
    'staging success',
    're-acquire scan',
    're-acquire local costmap',
    're-acquire TF',
    'forward goal refinement',
]

GOAL_EVENT_FIELDS = [
    'two_step_staging_plan',
    'staging_goal_pose',
    'staging_reason',
    'staging_executability_check',
    'second_step_forward_goal',
    'staging_applied',
    'staging_reject_reason',
    'branch_scoring_changed=false',
    'fallback_terminal_acceptance_used=false',
]

GUARDRAIL_TOKENS = [
    'Design review only',
    'Phase92 not entered',
    'staging is not fallback',
    'staging is not terminal acceptance',
    'timeout is not success',
    'original target preserved on reject',
    'stop and wait for visual diagnosis',
    'do not lower safety floor',
    'No Nav2/MPPI/controller tuning',
    'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'No branch scoring changed',
    'No exploration order changed',
    'No centerline gate changed',
    'No directional readiness changed',
    'No fallback/terminal acceptance changed',
    'No autonomous exploration success claimed',
    'No exit success claimed',
]

FORBIDDEN_SUCCESS_TOKENS = [
    'autonomous exploration success achieved',
    'exit success achieved',
    'timeout accepted as success',
    'Phase92 entered',
    'fallback success',
    'terminal acceptance applied',
    'Nav2 tuned',
    'MPPI tuned',
    'inflation tuned',
    'branch scoring changed=true',
]


def read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase91 file: {path}'
    return path.read_text(encoding='utf-8')


def assert_tokens_in_order(text: str, tokens: list[str]) -> None:
    cursor = -1
    for token in tokens:
        idx = text.find(token)
        assert idx > cursor, f'token {token!r} missing or out of order'
        cursor = idx


def test_phase91_design_doc_exists_and_is_design_only():
    text = read(PROPOSAL)
    assert 'Phase91: Two-step corridor alignment staging goal design review' in text
    assert 'DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED' in text
    assert 'Phase90 classification: STAGING_ALIGNMENT_GOAL_NEEDED' in text
    assert 'Phase92 may consider implementation only after explicit approval' in text
    assert 'Do not implement in Phase91' in text
    assert PHASE90_JSON.exists(), f'missing Phase90 JSON evidence: {PHASE90_JSON}'


def test_phase91_trigger_conditions_and_staging_generation_are_specified():
    text = read(PROPOSAL)
    for token in TRIGGER_TOKENS:
        assert token in text
    for token in STAGING_GENERATION_TOKENS:
        assert token in text
    assert 'single-step forward refined goal is currently unexecutable' in text
    assert 'same-corridor and two-side-wall evidence remain required' in text


def test_phase91_second_step_requires_fresh_evidence_after_staging_success():
    text = read(PROPOSAL)
    assert_tokens_in_order(text, SECOND_STEP_TOKENS_IN_ORDER)
    assert 'do not reuse stale dispatch-time local-cost evidence for the second step' in text
    assert 'second-step forward exploration goal is generated only after fresh evidence passes safety gates' in text


def test_phase91_future_goal_events_contract_is_complete():
    text = read(PROPOSAL)
    for token in GOAL_EVENT_FIELDS:
        assert token in text
    assert 'two_step_staging_plan.enabled' in text
    assert 'staging_executability_check.hard_safety_pass' in text
    assert 'second_step_forward_goal.generated_after_fresh_evidence' in text


def test_phase91_report_records_guardrails_and_no_runtime_changes():
    text = read(REPORT)
    for token in GUARDRAIL_TOKENS:
        assert token in text
    assert 'Design review complete' in text
    assert 'No runtime strategy changed' in text
    assert 'No Phase92 implementation started' in text


def test_phase91_static_guardrails_forbidden_success_tokens_absent():
    proposal = read(PROPOSAL)
    report = read(REPORT)
    combined = proposal + '\n' + report
    for token in FORBIDDEN_SUCCESS_TOKENS:
        assert token not in combined
