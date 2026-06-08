from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNBOOK = ROOT / 'tools' / 'run_phase79_goal2_timeout_bounded_reproduction_handoff.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase79_goal2_timeout_bounded_reproduction_handoff_report.md'
ARTIFACT_DIR = ROOT / 'log' / 'phase79_goal2_timeout_bounded_reproduction_handoff'
MIN_SUMMARY = ARTIFACT_DIR / 'phase79_goal2_timeout_bounded_reproduction_handoff_minimal_field_summary.md'
PHASE75 = ROOT / 'doc' / 'doc_report' / 'phase75_goal2_timeout_after_directional_redispatch_diagnosis_report.md'
PHASE78 = ROOT / 'doc' / 'doc_proposal' / 'obstacle_reproduction_handoff_workflow.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required file: {path}'
    return path.read_text(encoding='utf-8', errors='replace')


def test_phase79_runbook_exists_and_runs_visible_bounded_goal2_reproduction():
    text = _text(RUNBOOK)
    assert 'Phase79 Goal2 timeout bounded reproduction handoff run' in text
    assert 'RUN_ID="phase79_goal2_timeout_bounded_reproduction_handoff"' in text
    assert 'headless:=false' in text
    assert 'use_rviz:=true' in text
    assert 'max_goals:="$MAX_GOALS"' in text
    assert 'MAX_GOALS="${PHASE79_MAX_GOALS:-2}"' in text
    assert 'goal_timeout_sec:="$GOAL_TIMEOUT_SEC"' in text
    assert 'GOAL_TIMEOUT_SEC="${PHASE79_GOAL_TIMEOUT_SEC:-45.0}"' in text
    assert 'run to Goal2 problem point' in text
    assert 'hold scene for screenshot' in text
    assert 'SCENE_HELD_WAITING_FOR_USER_SCREENSHOT' in text
    assert 'PHASE79_CLEANUP_ON_EXIT' in text
    assert '--send-inner-ingress-goal' in text
    assert '--record-runtime' in text
    assert 'phase74_directional_local_costmap_readiness_gate_validation' in text
    assert 'phase77_goal2_timeout_visual_root_cause_event.json' in text
    assert 'human_observation_report_template' not in text
    assert 'fill out' not in text.lower()


def test_phase79_minimal_summary_exists_and_limits_screenshot_requests():
    text = _text(MIN_SUMMARY)
    assert 'Trigger: goal_timeout / local_cost_risk / recovery loop / near-goal outside tolerance' in text
    assert 'Run/artifacts:' in text
    assert 'Robot pose:' in text
    assert 'Target:' in text
    assert 'Key evidence:' in text
    assert 'Screenshot suggestions' in text
    assert 'Gazebo wide' in text
    assert 'RViz local costmap / footprint / front wedge' in text
    assert 'Goal tolerance' in text
    assert 'Recovery loop' in text
    assert text.count('- ') <= 16, 'summary must stay lightweight, not a long checklist'
    assert 'No lengthy human observation report is required' in text
    assert 'ChatGPT discussion' in text
    assert 'autonomous exploration success' not in text.lower().replace('no autonomous exploration success', '')


def test_phase79_report_documents_execution_or_blocker_and_guardrails_without_algorithm_fix():
    report = _text(REPORT)
    assert 'Phase79: Goal2 timeout bounded reproduction handoff run' in report
    assert 'automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan' in report
    assert 'Phase75/74 Goal2 timeout chain' in report
    assert 'visible mode' in report
    assert 'headless=false' in report
    assert 'use_rviz=true' in report
    assert 'SCENE_HELD_WAITING_FOR_USER_SCREENSHOT' in report or 'REPRODUCTION_START_BLOCKED' in report
    assert 'minimal field summary' in report
    assert 'No lengthy human observation report is required' in report
    assert 'No navigation strategy changed' in report
    assert 'No Nav2/MPPI/controller tuning' in report
    assert 'No autonomous exploration success claimed' in report
    assert 'No exit success claimed' in report
    assert 'algorithm repair phase not entered' in report
    assert 'Phase80 not entered' in report

    phase78 = _text(PHASE78)
    assert 'no lengthy human report is requested' in phase78
    phase75 = _text(PHASE75)
    assert 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP' in phase75
