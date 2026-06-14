from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase119_nav2_lifecycle_activation_stability.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase119_nav2_lifecycle_activation_stability_report.md'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing required Phase119 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase119_analyzer', ANALYZER)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sample(elapsed: float, *, controller='active', bt='active', planner='active', behavior='active', action=True, marker=True):
    return {
        'elapsed_sec': elapsed,
        'wall_time_sec': 1000.0 + elapsed,
        'action_present': action,
        'launch_active_marker_present': marker,
        'nodes': {
            'lifecycle_manager_navigation': {'state': 'manager_present'},
            'controller_server': {'state': controller},
            'bt_navigator': {'state': bt},
            'planner_server': {'state': planner},
            'behavior_server': {'state': behavior},
            'map_server': {'state': 'not_expected_with_slam'},
            'slam_toolbox': {'state': 'active'},
        },
    }


def test_phase119_analyzer_detects_action_present_inactive_lifecycle_contradiction():
    mod = _load_analyzer()
    artifact = {
        'phase': 'Phase119',
        'run_id': 'unit_inactive_action',
        'launch_args': {
            'autostart': 'true',
            'use_sim_time': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        },
        'timeline_samples': [
            _sample(3.0, controller='inactive', bt='unconfigured', action=True, marker=False),
            _sample(10.0, controller='inactive', bt='unconfigured', action=True, marker=False),
        ],
        'preflight': {
            'start_wall_time_sec': 1011.0,
            'end_wall_time_sec': 1030.0,
            'passed': False,
            'failed_gates': ['ingress_lifecycle_ambiguous', 'ingress_preflight_timeout'],
            'reject_reason': 'ingress_preflight_timeout',
        },
        'guardrails': {'ingress_goal_sent': False, 'maze_explorer_started': False},
    }
    analysis = mod.analyze_artifact(artifact)
    assert analysis['classification'] == 'ACTION_PRESENT_WITH_INACTIVE_LIFECYCLE_WINDOW'
    assert 'ACTION_PRESENT_WHILE_BT_NOT_ACTIVE' in analysis['findings']
    assert 'LAUNCH_ACTIVE_MARKER_MISSING_BEFORE_PREFLIGHT' in analysis['findings']
    assert analysis['contradiction_windows']['action_present_bt_not_active_count'] == 2
    assert analysis['guardrails']['no_goal_guard_valid'] is True


def test_phase119_analyzer_detects_stable_activation_with_marker_before_preflight():
    mod = _load_analyzer()
    artifact = {
        'phase': 'Phase119',
        'run_id': 'unit_stable',
        'launch_args': {
            'autostart': 'true',
            'use_sim_time': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        },
        'launch_log_events': [
            {'elapsed_sec': 8.0, 'event': 'Managed nodes are active'},
        ],
        'timeline_samples': [
            _sample(9.0, action=True, marker=True),
            _sample(14.0, action=True, marker=True),
        ],
        'preflight': {
            'start_wall_time_sec': 1015.0,
            'end_wall_time_sec': 1030.0,
            'passed': True,
            'failed_gates': [],
            'reject_reason': None,
        },
        'guardrails': {'ingress_goal_sent': False, 'maze_explorer_started': False},
    }
    analysis = mod.analyze_artifact(artifact)
    assert analysis['classification'] == 'NAV2_LIFECYCLE_ACTIVATION_STABLE_PREFLIGHT_PASS'
    assert analysis['stable_active_window']['all_required_active_observed'] is True
    assert analysis['marker_timeline']['managed_nodes_active_seen'] is True
    assert analysis['preflight_window']['passed'] is True


def test_phase119_analyzer_identifies_activation_delay_or_manager_incomplete():
    mod = _load_analyzer()
    artifact = {
        'phase': 'Phase119',
        'run_id': 'unit_delayed',
        'launch_args': {
            'autostart': 'true',
            'use_sim_time': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        },
        'timeline_samples': [
            _sample(5.0, controller='unconfigured', bt='unconfigured', action=False, marker=False),
            _sample(15.0, controller='inactive', bt='unconfigured', action=True, marker=False),
            _sample(40.0, controller='active', bt='active', action=True, marker=True),
        ],
        'preflight': {
            'start_wall_time_sec': 1016.0,
            'end_wall_time_sec': 1030.0,
            'passed': False,
            'failed_gates': ['ingress_lifecycle_ambiguous'],
            'reject_reason': 'ingress_preflight_timeout',
        },
        'guardrails': {'ingress_goal_sent': False, 'maze_explorer_started': False},
    }
    analysis = mod.analyze_artifact(artifact)
    assert analysis['classification'] == 'NAV2_ACTIVATION_DELAY_OR_PREFLIGHT_TOO_EARLY'
    assert 'LATE_MANAGED_NODES_ACTIVE_AFTER_PREFLIGHT_START' in analysis['findings']
    assert analysis['stable_active_window']['first_all_active_elapsed_sec'] == 40.0


def test_phase119_launch_log_parser_extracts_transition_marker_and_failures():
    mod = _load_analyzer()
    log_text = '''
[controller_server-10] [INFO] [123.1] [controller_server]: Configuring controller interface
[lifecycle_manager-17] [INFO] [125.0] [lifecycle_manager_navigation]: Activating controller_server
[lifecycle_manager-17] [INFO] [126.0] [lifecycle_manager_navigation]: Managed nodes are active
[bt_navigator-12] [WARN] [127.0] [bt_navigator]: bond timeout, restarting lifecycle
'''
    events = mod.parse_launch_log_events(log_text, launch_start_wall_time_sec=100.0)
    event_names = [event['event'] for event in events]
    assert 'Activating controller_server' in event_names
    assert 'Managed nodes are active' in event_names
    assert 'bond timeout, restarting lifecycle' in event_names
    marker = [event for event in events if event['event'] == 'Managed nodes are active'][0]
    assert marker['ros_time_sec'] == 126.0


def test_phase119_report_records_no_goal_and_no_phase120():
    assert REPORT.exists(), f'missing Phase119 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE119_NAV2_LIFECYCLE_ACTIVATION_STABILITY_ANALYSIS_COMPLETE_MANUAL_PROCESS_CLEANUP_CONFIRMED_PYCACHE_GUARD_NOT_EMPTY_STOP_BEFORE_PHASE120',
        'Hermes cleanup attempts',
        'manual external cleanup',
        'Phase119 scoped pycache guard: not empty',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'No autonomous exploration success or exit success is claimed',
        'Phase120 not entered',
        'lifecycle_manager',
        'controller_server',
        'bt_navigator',
        'planner_server',
        'behavior_server',
        'slam_toolbox',
    ]
    for phrase in required:
        assert phrase in text
