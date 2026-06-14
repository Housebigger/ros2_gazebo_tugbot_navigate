from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase107_preflight_raw_capture_discrepancy_diagnosis.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase107_preflight_raw_capture_discrepancy_diagnosis_report.md'
PHASE106_RUN_ID = 'phase106_preflighted_carry_over_bounded_goal1_staging_validation'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase107_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def _artifact_dir(
    tmp_path: Path,
    *,
    preflight_start: float = 100.0,
    preflight_end: float = 120.0,
    raw_capture_wall: float = 130.0,
    raw_elapsed: float = 5.0,
    raw_scan_topic: str = '/scan',
    raw_scan_frame: str = 'tugbot/scan_omni/scan_omni',
    preflight_scan_topic_note: str = '/scan',
    preflight_scan_frame: str | None = None,
    controller_active: bool = False,
    bt_active: bool = False,
    ready_before_preflight: bool = False,
    managed_active_time: float | None = None,
    include_raw: bool = True,
) -> Path:
    artifact_dir = tmp_path / 'phase106'
    artifact_dir.mkdir(parents=True)
    sample_tail = [
        {
            'wall_time_sec': preflight_start,
            'elapsed_sec': 0.0,
            'map_base_tf_check': {'available': False, 'stable': False, 'latest_age_sec': None, 'sample_count': 1},
            'map_odom_tf_age_sec': None,
            'odom_base_tf_age_sec': None,
            'scan_transform_check': {
                'scan_available': preflight_scan_frame is not None,
                'scan_frame_id': preflight_scan_frame,
                'target_frame': 'map',
                'transform_available': False,
                'stable': False,
            },
            'controller_pose_check': {
                'controller_server_active': controller_active,
                'bt_navigator_active': bt_active,
                'navigate_to_pose_action_ready': True,
                'robot_pose_available': False,
            },
            'goal_pose_transform_check': {
                'goal_frame_id': 'map',
                'global_costmap_frame': 'map',
                'controller_frame': 'odom',
                'transform_to_global_costmap_available': True,
                'transform_to_controller_frame_available': False,
            },
            'failed_gates': ['ingress_map_base_tf_missing', 'ingress_controller_robot_pose_unavailable'],
        },
        {
            'wall_time_sec': preflight_end,
            'elapsed_sec': preflight_end - preflight_start,
            'force_timeout': True,
            'map_base_tf_check': {'available': False, 'stable': False, 'latest_age_sec': None, 'sample_count': 1},
            'map_odom_tf_age_sec': None,
            'odom_base_tf_age_sec': 999.0,
            'scan_transform_check': {
                'scan_available': preflight_scan_frame is not None,
                'scan_frame_id': preflight_scan_frame,
                'target_frame': 'map',
                'transform_available': False,
                'stable': False,
            },
            'controller_pose_check': {
                'controller_server_active': controller_active,
                'bt_navigator_active': bt_active,
                'navigate_to_pose_action_ready': True,
                'robot_pose_available': False,
            },
            'goal_pose_transform_check': {
                'goal_frame_id': 'map',
                'global_costmap_frame': 'map',
                'controller_frame': 'odom',
                'transform_to_global_costmap_available': True,
                'transform_to_controller_frame_available': False,
            },
            'failed_gates': ['ingress_preflight_timeout'],
        },
    ]
    _write_json(
        artifact_dir / f'{PHASE106_RUN_ID}_ingress_preflight.json',
        {
            'ingress_preflight': {
                'evaluated': True,
                'passed': False,
                'start_time_sec': preflight_start,
                'end_time_sec': preflight_end,
                'bounded_wait_elapsed_sec': preflight_end - preflight_start,
                'ingress_preflight_timeout_sec': 20.0,
                'tf_stability_window_sec': 2.0,
                'ingress_preflight_reject_reason': 'ingress_preflight_timeout',
                'last_specific_reject_reason': 'ingress_map_base_tf_missing',
                'failed_gates': ['ingress_map_base_tf_missing', 'ingress_controller_robot_pose_unavailable', 'ingress_preflight_timeout'],
                'ingress_goal_sent': False,
                'maze_explorer_started': False,
                'inner_ingress_goal_pose': {'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0},
                'sample_count': len(sample_tail),
                'sample_tail': sample_tail,
                'scan_transform_check': sample_tail[-1]['scan_transform_check'],
                'controller_pose_check': sample_tail[-1]['controller_pose_check'],
                'goal_pose_transform_check': sample_tail[-1]['goal_pose_transform_check'],
                'map_base_tf_check': sample_tail[-1]['map_base_tf_check'],
            }
        },
    )
    if include_raw:
        _write_json(
            artifact_dir / f'{PHASE106_RUN_ID}_raw_capture.json',
            {
                'capture_wall_time': raw_capture_wall,
                'elapsed_sec': raw_elapsed,
                'scan': {'topic': raw_scan_topic, 'frame_id': raw_scan_frame, 'stamp_sec': 42.0},
                'map': {'topic': '/map', 'frame_id': 'map', 'stamp_sec': 42.0},
                'local_costmap': {'topic': '/local_costmap/costmap', 'frame_id': 'odom', 'stamp_sec': 42.0},
                'odom': {'topic': '/odom', 'frame_id': 'odom', 'child_frame_id': 'base_link', 'stamp_sec': 42.0},
                'tf': {
                    'map->base_link': {'available': True, 'stamp_sec': 42.0},
                    'map->odom': {'available': True, 'stamp_sec': 42.0},
                    'odom->base_link': {'available': True, 'stamp_sec': 42.0},
                },
            },
        )
    _write_json(artifact_dir / f'{PHASE106_RUN_ID}_ingress_result.json', {'success': False, 'status': 'preflight_rejected', 'ingress_goal_sent': False})
    _write_json(artifact_dir / f'{PHASE106_RUN_ID}_initial_cleanup_summary.json', {'before_count': 3, 'after_count': 0, 'unrelated_processes_targeted': False})
    _write_json(artifact_dir / f'{PHASE106_RUN_ID}_post_cleanup_summary.json', {'before_count': 5, 'after_count': 0, 'unrelated_processes_targeted': False})
    (artifact_dir / f'{PHASE106_RUN_ID}_preflight.txt').write_text(f'ingress_preflight_topic={preflight_scan_topic_note}\nNo Phase88/92/101/105 logic changed\n')
    (artifact_dir / f'{PHASE106_RUN_ID}_ingress_preflight_stdout.json').write_text('{"passed": false, "reject_reason": "ingress_preflight_timeout"}\n')
    (artifact_dir / f'{PHASE106_RUN_ID}_ingress_preflight_stderr.log').write_text('')
    ready_text = ''
    if ready_before_preflight:
        ready_text = f'[INFO] [{preflight_start - 10.0}] [tf2_echo]: Translation: [0, 0, 0]\n/navigate_to_pose ready\n/map ready\n/scan ready\n/local_costmap/costmap ready\n'
    (artifact_dir / f'{PHASE106_RUN_ID}_ros_graph_ready.txt').write_text(ready_text)
    launch_lines = []
    if managed_active_time is not None:
        launch_lines.append(f'[lifecycle_manager-17] [INFO] [{managed_active_time}] [lifecycle_manager_navigation]: Managed nodes are active')
        launch_lines.append(f'[lifecycle_manager-17] [INFO] [{managed_active_time - 0.2}] [lifecycle_manager_navigation]: Server controller_server connected with bond.')
        launch_lines.append(f'[lifecycle_manager-17] [INFO] [{managed_active_time - 0.1}] [lifecycle_manager_navigation]: Server bt_navigator connected with bond.')
    (artifact_dir / f'{PHASE106_RUN_ID}_visible_launch.log').write_text('\n'.join(launch_lines) + '\n')
    (artifact_dir / f'{PHASE106_RUN_ID}_runtime_timeline.jsonl').write_text('')
    return artifact_dir


def test_phase107_classification_tokens_and_guardrails(tmp_path):
    analyzer = _load_analyzer()
    assert analyzer.ALLOWED_CLASSIFICATIONS == {
        'PREFLIGHT_SAMPLING_TOO_EARLY',
        'PREFLIGHT_STABILITY_WINDOW_TOO_SHORT',
        'PREFLIGHT_FRAME_QUERY_MISMATCH',
        'PREFLIGHT_TOPIC_QUERY_MISMATCH',
        'PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE',
        'PREFLIGHT_TOOL_FALSE_NEGATIVE',
        'RAW_CAPTURE_LATER_THAN_PREFLIGHT',
        'PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE',
    }
    result = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path, ready_before_preflight=True, managed_active_time=90.0))
    assert result['classification'] == 'PREFLIGHT_TOOL_FALSE_NEGATIVE'
    assert 'RAW_CAPTURE_LATER_THAN_PREFLIGHT' in result['contributing_classifications']
    assert result['guardrails']['phase105_preflight_changed'] is False
    assert result['guardrails']['phase101_carry_over_changed'] is False
    assert result['guardrails']['nav2_config_changed'] is False
    assert result['guardrails']['no_success_claimed'] is True
    assert result['guardrails']['phase108_entered'] is False


def test_phase107_timestamp_comparison_detects_raw_capture_later_than_preflight(tmp_path):
    analyzer = _load_analyzer()
    artifact_dir = _artifact_dir(tmp_path, preflight_start=100.0, preflight_end=120.0, raw_capture_wall=130.0, raw_elapsed=5.0)
    result = analyzer.analyze_phase106_artifacts(artifact_dir)
    timing = result['timestamp_comparison']
    assert timing['preflight_start_wall_time_sec'] == 100.0
    assert timing['preflight_end_wall_time_sec'] == 120.0
    assert timing['raw_capture_start_wall_time_sec'] == 125.0
    assert timing['raw_capture_end_wall_time_sec'] == 130.0
    assert timing['raw_capture_started_after_preflight_end'] is True
    assert timing['raw_capture_started_after_preflight_timeout_deadline'] is True
    assert result['classification'] == 'RAW_CAPTURE_LATER_THAN_PREFLIGHT'
    assert result['direct_answers']['raw_capture_available_after_preflight_timeout'] is True


def test_phase107_frame_and_topic_comparison(tmp_path):
    analyzer = _load_analyzer()
    same = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path / 'same', preflight_scan_frame='tugbot/scan_omni/scan_omni'))
    assert same['topic_comparison']['scan_topic_same'] is True
    assert same['frame_comparison']['core_tf_frames_same'] is True
    assert same['frame_comparison']['preflight_scan_frame_id'] == 'tugbot/scan_omni/scan_omni'
    assert same['frame_comparison']['raw_scan_frame_id'] == 'tugbot/scan_omni/scan_omni'

    topic_mismatch = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path / 'topic_mismatch', raw_scan_topic='/scan2'))
    assert topic_mismatch['classification'] == 'PREFLIGHT_TOPIC_QUERY_MISMATCH'
    assert topic_mismatch['topic_comparison']['scan_topic_same'] is False

    frame_mismatch = analyzer.analyze_phase106_artifacts(
        _artifact_dir(tmp_path / 'frame_mismatch', preflight_scan_frame='laser', raw_scan_frame='tugbot/scan_omni/scan_omni')
    )
    assert frame_mismatch['classification'] == 'PREFLIGHT_FRAME_QUERY_MISMATCH'
    assert frame_mismatch['frame_comparison']['scan_frame_same'] is False


def test_phase107_lifecycle_ordering_vs_tool_false_negative(tmp_path):
    analyzer = _load_analyzer()
    ordering = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path / 'ordering', managed_active_time=125.0))
    assert ordering['classification'] == 'PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE'
    assert ordering['lifecycle_comparison']['managed_nodes_active_after_preflight_end'] is True
    assert ordering['direct_answers']['bt_controller_inactive_is_startup_ordering_issue'] is True

    false_negative = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path / 'false_negative', managed_active_time=90.0, ready_before_preflight=True))
    assert false_negative['classification'] == 'PREFLIGHT_TOOL_FALSE_NEGATIVE'
    assert false_negative['lifecycle_comparison']['managed_nodes_active_before_preflight_start'] is True
    assert false_negative['direct_answers']['bt_controller_inactive_is_startup_ordering_issue'] is False


def test_phase107_insufficient_evidence_and_report_contract(tmp_path):
    analyzer = _load_analyzer()
    insufficient = analyzer.analyze_phase106_artifacts(_artifact_dir(tmp_path, include_raw=False))
    assert insufficient['classification'] == 'PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE'
    assert 'missing_raw_capture' in insufficient['evidence_gaps']

    # Report may not exist during the initial RED step, but once it exists it must preserve guardrails.
    if REPORT.exists():
        text = REPORT.read_text()
        for token in analyzer.ALLOWED_CLASSIFICATIONS:
            assert token in text
        for token in [
            'No Phase105 preflight changed',
            'No Phase101 carry-over changed',
            'No Phase88/92 logic changed',
            'No maze_explorer strategy changed',
            'No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning',
            'No autonomous exploration success claimed',
            'No exit success claimed',
            'Phase108 not entered',
        ]:
            assert token in text


def test_phase107_nav2_config_guard_clean():
    result = subprocess.run(['git', 'diff', '--', 'src/tugbot_navigation/config'], cwd=ROOT, text=True, capture_output=True, check=False)
    assert result.stdout == ''
