from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase122_post_ingress_handoff_dry_start.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase122_post_ingress_handoff_dry_start.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase122_post_ingress_handoff_dry_start_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing required Phase122 file: {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _phase120_success_artifact() -> dict:
    return {
        'phase': 'Phase120',
        'mode': 'controlled_ingress_dispatch_with_managed_active_readiness_wait',
        'classification': 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER',
        'readiness_wait': {
            'enabled': True,
            'timed_out': False,
            'marker_found': True,
            'multi_source_ready': False,
            'action_server_ready': True,
        },
        'preflight': {
            'evaluated': True,
            'passed': True,
            'failed_gates': [],
            'ingress_preflight_reject_reason': None,
        },
        'dispatch': {
            'dispatch_attempted': True,
            'accepted': True,
            'rejected': False,
            'result_received': True,
            'result_status': 4,
            'result_status_label': 'SUCCEEDED',
            'cancel_requested': False,
            'cancel_result': None,
            'ingress_goal_sent': True,
            'maze_explorer_started': False,
            'goal_pose': {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
            'frame_id': 'map',
            'x': 2.0,
            'y': 0.0,
            'yaw': 0.0,
        },
    }


def _ready_handoff(mod) -> dict:
    handoff = mod.initial_handoff_record()
    handoff['robot_pose_after_ingress'].update({
        'pose_available': True,
        'frame_id': 'map',
        'x': 2.03,
        'y': 0.02,
        'yaw': 0.03,
        'stamp': {'sec': 12, 'nanosec': 0},
        'sample_wall_time_sec': 100.0,
        'source': 'tf_map_base_link',
    })
    handoff['distance_to_ingress_goal'].update({'meters': 0.036, 'within_tolerance': True})
    handoff['orientation_error'].update({'radians': 0.03, 'within_tolerance': True})
    handoff['costmap_freshness'].update({
        'global_costmap_available': True,
        'local_costmap_available': True,
        'global_costmap_age_sec': 0.4,
        'local_costmap_age_sec': 0.3,
        'fresh': True,
    })
    handoff['scan_freshness'].update({'scan_seen': True, 'scan_age_sec': 0.2, 'fresh': True})
    handoff['tf_freshness'].update({
        'map_odom_available': True,
        'odom_base_available': True,
        'map_base_available': True,
        'scan_to_base_available': True,
        'fresh': True,
    })
    handoff['nav2_action_idle_state'].update({
        'action_server_ready': True,
        'active_goal_count': 0,
        'pending_goal_count': 0,
        'executing_goal_count': 0,
        'canceling_goal_count': 0,
        'idle': True,
        'last_result_status_label': 'SUCCEEDED',
    })
    return handoff


def test_phase122_handoff_ready_dry_start_artifact_schema_and_classifier():
    mod = _load(RUNNER, 'phase122_runner_ready')
    handoff = _ready_handoff(mod)
    dry = mod.initial_dry_start_record(max_goals=0)
    dry.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'maze_explorer_max_goals': 0,
        'exploration_goal_dispatched': False,
        'dry_start_observed': True,
    })

    artifact = mod.build_phase122_artifact(
        run_id='unit_phase122_ready',
        phase120_artifact=_phase120_success_artifact(),
        handoff_artifact=handoff,
        dry_start=dry,
        classification=mod.classify_phase122(handoff, dry),
    )

    assert artifact['classification'] == 'INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP'
    assert artifact['handoff_allowed'] is True
    assert artifact['maze_explorer_start_allowed'] is True
    assert artifact['maze_explorer_started'] is True
    assert artifact['maze_explorer_max_goals'] == 0
    assert artifact['exploration_goal_dispatched'] is False
    for field in mod.REQUIRED_HANDOFF_FIELDS:
        assert field in artifact['handoff_artifact']


def test_phase122_fail_closed_specific_handoff_classifications():
    mod = _load(RUNNER, 'phase122_runner_failures')

    pose_bad = _ready_handoff(mod)
    pose_bad['distance_to_ingress_goal'].update({'meters': 0.9, 'within_tolerance': False})
    assert mod.classify_phase122(pose_bad, mod.initial_dry_start_record(max_goals=0)) == 'POSE_NOT_AT_INGRESS_GOAL'

    action_busy = _ready_handoff(mod)
    action_busy['nav2_action_idle_state'].update({'active_goal_count': 1, 'idle': False})
    assert mod.classify_phase122(action_busy, mod.initial_dry_start_record(max_goals=0)) == 'NAV2_ACTION_NOT_IDLE'

    tf_stale = _ready_handoff(mod)
    tf_stale['tf_freshness']['fresh'] = False
    assert mod.classify_phase122(tf_stale, mod.initial_dry_start_record(max_goals=0)) == 'TF_OR_SCAN_STALE'

    scan_stale = _ready_handoff(mod)
    scan_stale['scan_freshness']['fresh'] = False
    assert mod.classify_phase122(scan_stale, mod.initial_dry_start_record(max_goals=0)) == 'TF_OR_SCAN_STALE'

    costmap_missing = _ready_handoff(mod)
    costmap_missing['costmap_freshness']['fresh'] = False
    assert mod.classify_phase122(costmap_missing, mod.initial_dry_start_record(max_goals=0)) == 'COSTMAP_NOT_READY'


def test_phase122_dry_start_violation_overrides_ready_handoff():
    mod = _load(RUNNER, 'phase122_runner_violation')
    dry = mod.initial_dry_start_record(max_goals=0)
    dry.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'maze_explorer_max_goals': 0,
        'exploration_goal_dispatched': True,
    })
    assert mod.classify_phase122(_ready_handoff(mod), dry) == 'MAZE_EXPLORER_DRY_START_VIOLATION'


def test_phase122_analyzer_validates_guardrails_and_schema():
    runner = _load(RUNNER, 'phase122_runner_for_analyzer')
    analyzer = _load(ANALYZER, 'phase122_analyzer')
    handoff = _ready_handoff(runner)
    dry = runner.initial_dry_start_record(max_goals=0)
    dry.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'maze_explorer_max_goals': 0,
        'exploration_goal_dispatched': False,
        'dry_start_observed': True,
    })
    artifact = runner.build_phase122_artifact(
        run_id='unit_phase122_analysis',
        phase120_artifact=_phase120_success_artifact(),
        handoff_artifact=handoff,
        dry_start=dry,
        classification='INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP',
    )

    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is True
    assert analysis['classification_matches_evidence'] is True
    assert analysis['guardrails']['only_explicit_inner_ingress_goal_sent'] is True
    assert analysis['guardrails']['no_exploration_goal_dispatched'] is True
    assert analysis['guardrails']['dry_start_max_goals_zero'] is True
    assert analysis['guardrails']['no_autonomous_success_claim'] is True
    assert analysis['guardrails']['no_exit_success_claim'] is True


def test_phase122_source_preserves_scope_and_dry_start_only_guardrails():
    text = RUNNER.read_text(encoding='utf-8')
    required = [
        'Phase122',
        'post_ingress_handoff_dry_start',
        'INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP',
        'MAZE_EXPLORER_DRY_START_VIOLATION',
        'frame_id=map,x=2.0,y=0.0,yaw=0.0',
        'max_goals:=0',
        'exploration_goal_dispatched',
        'do not dispatch exploration goal',
        'not autonomous exploration success',
        'not exit success',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'max_goals:=1',
        'carry_over_applied = True',
        'fallback_terminal_acceptance_used = True',
        'branch_scoring_changed = True',
    ]
    for phrase in forbidden:
        assert phrase not in text


def test_phase122_report_records_runtime_result_and_cleanup():
    assert REPORT.exists(), f'missing required Phase122 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'Phase122 Post-ingress handoff smoke with maze_explorer dry-start',
        'log/phase122_post_ingress_handoff_dry_start/',
        'ingress_goal_result',
        'robot_pose_after_ingress',
        'distance_to_ingress_goal',
        'orientation_error',
        'costmap_freshness',
        'scan_freshness',
        'tf_freshness',
        'nav2_action_idle_state',
        'maze_explorer_max_goals=0',
        'exploration_goal_dispatched=false',
        'No autonomous exploration success or exit success is claimed',
        'Phase123 not entered',
    ]
    for phrase in required:
        assert phrase in text
