from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase103_inner_ingress_goal_failure_diagnosis.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase103_inner_ingress_goal_failure_diagnosis_report.md'


def _load_module():
    spec = importlib.util.spec_from_file_location('phase103_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, sort_keys=True) + '\n' for r in rows))


def _phase102_artifact(tmp_path: Path, *, ingress_status: int = 6, reuse_dirty: bool = True) -> Path:
    art = tmp_path / 'phase102'
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'
    _write_json(art / f'{run_id}_ingress_result.json', {
        'goal_sent': True,
        'goal_accepted': True,
        'result_received': True,
        'status': ingress_status,
        'status_text': str(ingress_status),
        'success': False,
        'error_code': 102,
        'goal_pose': {'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'feedback': [
            {'distance_remaining': 0.0, 'navigation_time_sec': 0.006, 'number_of_recoveries': 0},
            {'distance_remaining': 1.9926457405090332, 'navigation_time_sec': 0.015, 'number_of_recoveries': 0},
            {'distance_remaining': 1.9926457405090332, 'navigation_time_sec': 0.375, 'number_of_recoveries': 0},
        ],
    })
    _write_json(art / f'{run_id}_raw_capture.json', {
        'scan_available': True,
        'map_available': True,
        'local_costmap_available': True,
        'odom_available': True,
        'tf_available': True,
        'robot_pose_map': [0.0, 0.0, 0.0],
        'map_summary': {'known_count': 7102, 'free_count': 6747, 'occupied_count': 355, 'unknown_count': 14282},
        'local_costmap_summary': {'known_count': 3600, 'free_count': 1078, 'occupied_count': 2018, 'lethal_count': 1328},
    })
    _write_jsonl(art / f'{run_id}_runtime_timeline.jsonl', [
        {'navigate_to_pose_action_ready': True, 'map_available': True, 'scan_available': True, 'local_costmap_available': True, 'tf_ready': True,
         'nav2_lifecycle_active': True, 'planner_server_active': True, 'controller_server_active': True, 'bt_navigator_active': True,
         'mode': 'reuse_visible_stack'}
    ])
    _write_jsonl(art / f'{run_id}_nav2_feedback.jsonl', [
        {'distance_remaining': 1.9926457405090332, 'number_of_recoveries': 0, 'navigation_time_sec': 0.375}
    ])
    _write_jsonl(art / f'{run_id}_local_costmap_samples.jsonl', [
        {'target': [2.0, 0.0], 'target_footprint_lethal_count': 0, 'robot_footprint_lethal_count': 0, 'front_wedge_lethal_count': 0,
         'target_in_local_costmap_bounds': True, 'robot_in_local_costmap_bounds': True}
    ])
    preflight = [
        'reuse_visible_stack=true' if reuse_dirty else 'reuse_visible_stack=false',
        'inner_ingress_goal=(2.0,0.0,0.0)',
        'No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning',
    ]
    (art / f'{run_id}_preflight.txt').write_text('\n'.join(preflight) + '\n')
    _write_json(art / 'attempt1_ingress_failed_before_rerun' / f'{run_id}_ingress_result.json', {
        'goal_sent': True, 'goal_accepted': True, 'result_received': True, 'status': 6, 'success': False,
        'goal_pose': {'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'feedback': [{'distance_remaining': 1.9926457405090332}],
    })
    return art


def _success_ref(tmp_path: Path, name: str, *, success: bool = True) -> Path:
    art = tmp_path / name
    art.mkdir(parents=True, exist_ok=True)
    _write_json(art / f'{name}_ingress_result.json', {
        'success': success,
        'status': 'succeeded' if success else 6,
        'error_code': 0 if success else 102,
        'goal_pose': {'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'feedback': [{'distance_remaining': 1.992}, {'distance_remaining': 0.0}],
    })
    _write_json(art / f'{name}_raw_capture.json', {
        'scan_available': True,
        'map_available': True,
        'local_costmap_available': True,
        'odom_available': True,
        'tf_available': True,
        'robot_pose_map': [2.42, 1.01, 1.56],
    })
    return art


def test_phase103_classification_set_and_required_fields():
    module = _load_module()
    assert module.CLASSIFICATIONS == [
        'INNER_INGRESS_GOAL_POSE_BLOCKED',
        'INNER_INGRESS_START_POSE_OR_FRAME_MISMATCH',
        'INNER_INGRESS_NAV2_PLANNING_FAILED',
        'INNER_INGRESS_CONTROLLER_EXECUTION_FAILED',
        'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY',
        'INNER_INGRESS_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
    ]
    for field in [
        'ingress_goal_pose', 'start_pose', 'map_frame', 'tf_available',
        'global_plan', 'local_costmap', 'nav2_lifecycle', 'planner_controller_readiness',
        'costmap_obstruction', 'reuse_visible_stack', 'phase96_fix_comparison', 'phase97_comparison',
    ]:
        assert field in module.REQUIRED_DIAGNOSTIC_FIELDS


def test_phase103_reuse_visible_stack_dirty_classification(tmp_path: Path):
    module = _load_module()
    phase102 = _phase102_artifact(tmp_path, reuse_dirty=True)
    phase96 = _success_ref(tmp_path, 'phase96_fix_ingress_guided_startup_correction')
    phase97 = _success_ref(tmp_path, 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke')
    result = module.analyze_artifacts(phase102, phase96, phase97)
    assert result['classification'] == 'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY'
    assert result['phase102']['ingress']['status'] == 6
    assert result['diagnosis']['reuse_visible_stack']['dirty_reuse_detected'] is True
    assert result['diagnosis']['nav2_lifecycle']['ready'] is True
    assert result['comparison']['phase96_fix']['ingress_success'] is True
    assert result['comparison']['phase97']['ingress_success'] is True
    assert 'phase102_reused_visible_stack_after_failed_attempt' in result['evidence']


def test_phase103_goal_pose_blocked_when_costmap_target_lethal(tmp_path: Path):
    module = _load_module()
    phase102 = _phase102_artifact(tmp_path, reuse_dirty=False)
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'
    _write_jsonl(phase102 / f'{run_id}_local_costmap_samples.jsonl', [
        {'target': [2.0, 0.0], 'target_footprint_lethal_count': 7, 'robot_footprint_lethal_count': 0, 'front_wedge_lethal_count': 1,
         'target_in_local_costmap_bounds': True, 'robot_in_local_costmap_bounds': True}
    ])
    result = module.analyze_artifacts(phase102, _success_ref(tmp_path, 'p96'), _success_ref(tmp_path, 'p97'))
    assert result['classification'] == 'INNER_INGRESS_GOAL_POSE_BLOCKED'
    assert result['diagnosis']['costmap_obstruction']['target_pose_blocked'] is True


def test_phase103_start_pose_or_frame_mismatch_when_tf_or_frame_missing(tmp_path: Path):
    module = _load_module()
    phase102 = _phase102_artifact(tmp_path, reuse_dirty=False)
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'
    ingress = json.loads((phase102 / f'{run_id}_ingress_result.json').read_text())
    ingress['goal_pose']['frame_id'] = 'odom'
    _write_json(phase102 / f'{run_id}_ingress_result.json', ingress)
    raw = json.loads((phase102 / f'{run_id}_raw_capture.json').read_text())
    raw['tf_available'] = False
    _write_json(phase102 / f'{run_id}_raw_capture.json', raw)
    result = module.analyze_artifacts(phase102, _success_ref(tmp_path, 'p96'), _success_ref(tmp_path, 'p97'))
    assert result['classification'] == 'INNER_INGRESS_START_POSE_OR_FRAME_MISMATCH'
    assert result['diagnosis']['map_frame']['goal_frame_id'] == 'odom'
    assert result['diagnosis']['tf_available'] is False


def test_phase103_cli_writes_json_and_summary(tmp_path: Path):
    phase102 = _phase102_artifact(tmp_path, reuse_dirty=True)
    phase96 = _success_ref(tmp_path, 'phase96_fix_ingress_guided_startup_correction')
    phase97 = _success_ref(tmp_path, 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke')
    output_json = tmp_path / 'analysis.json'
    summary = tmp_path / 'summary.md'
    completed = subprocess.run([
        'python3', str(ANALYZER),
        '--phase102-artifact-dir', str(phase102),
        '--phase96-fix-artifact-dir', str(phase96),
        '--phase97-artifact-dir', str(phase97),
        '--output-json', str(output_json),
        '--minimal-summary-output', str(summary),
    ], cwd=ROOT, text=True, capture_output=True, check=True)
    payload = json.loads(output_json.read_text())
    assert payload['classification'] == 'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY'
    assert 'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY' in completed.stdout
    text = summary.read_text()
    assert 'Phase103 minimal summary' in text
    assert 'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY' in text
    assert 'No Phase101 carry-over or Nav2 parameter changes' in text


def test_phase103_report_records_cleanup_artifacts_and_guardrails():
    assert REPORT.exists()
    text = REPORT.read_text()
    assert 'phase103_initial_cleanup_summary.json' in text
    assert 'INNER_INGRESS_' in text
    assert 'Phase96-fix' in text
    assert 'Phase97' in text
    assert 'No Phase101 carry-over changes' in text
    assert 'No Nav2 parameter tuning' in text
    assert 'Phase104 not entered' in text
