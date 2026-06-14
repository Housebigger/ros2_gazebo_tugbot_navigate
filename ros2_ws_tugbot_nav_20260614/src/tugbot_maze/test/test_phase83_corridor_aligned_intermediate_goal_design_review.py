from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / "doc" / "doc_proposal" / "phase83_corridor_aligned_intermediate_goal_design_review.md"
REPORT = ROOT / "doc" / "doc_report" / "phase83_corridor_aligned_intermediate_goal_design_review_report.md"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_phase83_design_doc_exists_and_is_design_only():
    assert PROPOSAL.exists(), "Phase83 design proposal is missing"
    text = read(PROPOSAL)
    required = [
        "DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED",
        "corridor-aligned intermediate goal refinement",
        "candidate point -> corridor centerline projection -> corridor heading -> forward executability check -> Nav2 goal",
        "No runtime integration is added in Phase83",
        "Phase84 not entered",
    ]
    for token in required:
        assert token in text


def test_phase83_design_contains_required_algorithm_contract():
    text = read(PROPOSAL)
    required = [
        "centerline projection",
        "corridor centerline projection",
        "corridor heading",
        "goal orientation follows corridor heading",
        "forward executability check",
        "near-goal lateral residual handling",
        "avoid lateral near-goal residual",
        "same-corridor evidence",
        "two-side-wall evidence",
        "insufficient evidence -> no refinement",
    ]
    for token in required:
        assert token in text


def test_phase83_guardrails_are_explicit():
    combined = read(PROPOSAL) + "\n" + read(REPORT)
    guardrails = [
        "Do not tune inflation",
        "Do not tune robot_radius",
        "Do not tune clearance_radius_m",
        "Do not tune MPPI",
        "Do not tune controller",
        "Do not change maze_explorer runtime strategy",
        "Do not change branch scoring",
        "Do not change centerline gate",
        "Do not change directional readiness",
        "Do not use fallback/terminal acceptance",
        "Do not treat timeout as success",
        "No autonomous exploration success claimed",
        "No exit success claimed",
    ]
    for token in guardrails:
        assert token in combined


def test_phase83_report_connects_phase80_81_82_evidence_to_design_decision():
    assert REPORT.exists(), "Phase83 report is missing"
    text = read(REPORT)
    required = [
        "NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR",
        "FORWARD_OPEN_CORRIDOR_BLOCKED",
        "FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED",
        "INFLATION_SPILLOVER_SUSPECTED",
        "DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE",
        "Phase84 not entered",
    ]
    for token in required:
        assert token in text


def test_phase83_acceptance_criteria_and_future_tests_are_defined():
    text = read(PROPOSAL)
    required = [
        "Acceptance criteria before any future implementation",
        "Focused test cases for a future implementation phase",
        "candidate already on centerline -> no lateral shift",
        "off-center candidate -> projected to centerline",
        "corridor heading yaw is used as Nav2 goal orientation",
        "blocked forward executability -> keep original target unchanged",
        "missing corridor evidence -> keep original target unchanged",
    ]
    for token in required:
        assert token in text
