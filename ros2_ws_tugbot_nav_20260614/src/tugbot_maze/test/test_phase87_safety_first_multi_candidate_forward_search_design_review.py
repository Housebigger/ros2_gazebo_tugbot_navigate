from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase87_safety_first_multi_candidate_forward_search_design_review.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase87_safety_first_multi_candidate_forward_search_design_review_report.md'

CANDIDATE_TOKENS = [
    'centerline projection',
    'small lateral offsets',
    'multiple forward offsets',
    'corridor heading variants',
]

PRIORITY_TOKENS = [
    'hard safety pass',
    'no footprint/front-wedge lethal regression',
    'safety_floor_ok',
    'forward_progress_ok',
    'clearance better',
    'balance error smaller',
]

GOAL_EVENT_FIELDS = [
    'multi_candidate_forward_search',
    'candidate_family',
    'candidate_count',
    'hard_safety_pass_candidate_count',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
    'selection_priority_trace',
    'rejected_candidate_summaries',
    'refinement_applied',
    'refinement_reject_reason',
    'original_target_preserved_on_reject',
    'branch_scoring_changed=false',
    'fallback_terminal_acceptance_used=false',
]

GUARDRAIL_TOKENS = [
    'No safety boundary is lowered',
    'No inflation/robot_radius/clearance_radius_m/MPPI/controller tuning',
    'No branch scoring changed',
    'No exploration order changed',
    'No centerline gate changed',
    'No directional readiness override changed',
    'No fallback/terminal acceptance changed',
    'No autonomous exploration success claimed',
    'No exit success claimed',
    'Phase88 not entered',
]


def read(path: Path) -> str:
    assert path.exists(), f'missing expected Phase87 file: {path}'
    return path.read_text(encoding='utf-8')


def assert_in_order(text: str, tokens: list[str]) -> None:
    cursor = -1
    for token in tokens:
        idx = text.find(token)
        assert idx > cursor, f'token {token!r} missing or out of order'
        cursor = idx


def test_phase87_design_doc_exists_and_is_design_only():
    text = read(PROPOSAL)
    assert 'Phase87' in text
    assert 'Safety-first multi-candidate forward search design review' in text
    assert 'DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED' in text
    assert 'Phase88 implementation phase' in text
    assert 'Do not implement in Phase87' in text
    assert 'reject keeps original target unchanged' in text


def test_phase87_candidate_family_and_safety_first_priority_order():
    text = read(PROPOSAL)
    for token in CANDIDATE_TOKENS:
        assert token in text
    assert_in_order(text, PRIORITY_TOKENS)


def test_phase87_goal_events_and_test_contract_are_specified():
    text = read(PROPOSAL)
    for token in GOAL_EVENT_FIELDS:
        assert token in text
    for token in [
        'test_candidate_family_generation_contract',
        'test_safety_first_priority_selects_hard_safe_candidate_before_best_balance',
        'test_reject_preserves_original_target_when_no_candidate_passes',
        'test_guardrails_no_strategy_or_parameter_tuning',
    ]:
        assert token in text


def test_phase87_report_records_guardrails_and_no_phase88():
    text = read(REPORT)
    for token in GUARDRAIL_TOKENS:
        assert token in text
    assert 'Design review complete' in text
    assert 'No runtime strategy changed' in text


def test_phase87_static_nav_strategy_guardrails():
    assert PROPOSAL.exists()
    assert REPORT.exists()
    proposal = PROPOSAL.read_text(encoding='utf-8')
    report = REPORT.read_text(encoding='utf-8')
    combined = proposal + '\n' + report
    forbidden = [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase88 entered',
        'MPPI tuned',
        'inflation tuned',
    ]
    for token in forbidden:
        assert token not in combined
