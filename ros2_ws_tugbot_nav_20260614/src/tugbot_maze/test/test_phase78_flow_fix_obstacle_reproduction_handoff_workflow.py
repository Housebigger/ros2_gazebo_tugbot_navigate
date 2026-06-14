from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / 'doc' / 'doc_proposal' / 'obstacle_reproduction_handoff_workflow.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase78_flow_fix_obstacle_reproduction_handoff_report.md'
PHASE77_REPORT = ROOT / 'doc' / 'doc_report' / 'phase77_goal2_timeout_visual_root_cause_replay_first_application_report.md'

TRIGGERS = [
    'goal_timeout',
    'FAILED_EXHAUSTED',
    'no_candidate',
    'local_cost_risk',
    'recovery loop',
    'near-goal outside tolerance',
]

HANDOFF_TOKENS = [
    'automatic bounded reproduction',
    'run to the problem point',
    'keep Gazebo/RViz observable',
    'user screenshot',
    'initial judgment',
    'ChatGPT discussion',
    'Hermes executes the confirmed plan',
]

RUNBOOK_TOKENS = [
    'bounded reproduction runbook',
    'start simulation',
    'trigger problem segment',
    'pause or hold at problem point',
    'no long exploration',
    'max_goals',
    'timeout_sec',
]

GUARDRAILS = [
    'do not tune Nav2/MPPI/controller',
    'do not change exploration strategy',
    'do not claim autonomous success',
    'do not claim exit success',
]

FORBIDDEN_FLOW = [
    'lengthy human observation report required',
    'fill a long human observation report',
    '冗长 human observation report',
    '大量人工观察报告',
]


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase78-flow-fix file: {path}'
    return path.read_text(encoding='utf-8')


def test_phase78_handoff_workflow_defines_lightweight_closed_loop_and_triggers():
    text = _text(WORKFLOW)
    assert '# Obstacle Reproduction Handoff Workflow' in text
    assert 'Phase78-flow-fix' in text
    for token in TRIGGERS + HANDOFF_TOKENS:
        assert token in text
    assert 'minimal field summary' in text
    assert 'screenshot suggestions' in text
    assert 'no lengthy human report' in text
    for forbidden in FORBIDDEN_FLOW:
        assert forbidden not in text


def test_phase78_workflow_contains_bounded_reproduction_runbook_and_guardrails():
    text = _text(WORKFLOW)
    for token in RUNBOOK_TOKENS + GUARDRAILS:
        assert token in text
    assert 'Gazebo/RViz remains open for observation' in text
    assert 'do not modify branch scoring' in text
    assert 'do not modify centerline gate' in text
    assert 'do not modify directional readiness override' in text
    assert 'do not modify fallback/terminal acceptance' in text
    assert 'not an algorithm repair phase' in text


def test_phase78_report_supersedes_phase77_report_heavy_flow_and_stops_before_algorithm_fix():
    report = _text(REPORT)
    assert 'Phase78-flow-fix' in report
    assert 'Phase77 overlay/report-heavy workflow is superseded' in report
    assert 'automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan' in report
    assert 'No lengthy human observation report is required' in report
    assert 'No navigation strategy changed' in report
    assert 'No Nav2/MPPI/controller tuning' in report
    assert 'No autonomous exploration success claimed' in report
    assert 'No exit success claimed' in report
    assert 'algorithm repair phase not entered' in report
    assert 'Phase79 not entered' in report

    phase77 = _text(PHASE77_REPORT)
    assert 'Phase78-flow-fix supersession note' in phase77
    assert 'report-heavy handoff is superseded' in phase77
    assert 'use doc/doc_proposal/obstacle_reproduction_handoff_workflow.md' in phase77
