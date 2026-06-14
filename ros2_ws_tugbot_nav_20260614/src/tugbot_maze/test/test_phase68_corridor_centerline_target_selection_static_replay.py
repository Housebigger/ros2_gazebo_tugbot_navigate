from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase68_corridor_centerline_target_selection_static_replay.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase68_corridor_centerline_target_selection_static_replay_report.md'
PHASE67_ARTIFACT = ROOT / 'log' / 'phase67_goal1_timeout_visual_replay' / 'phase67_goal1_timeout_visual_replay.json'
RUN_ID = 'phase68_corridor_centerline_target_selection_static_replay'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_module():
    spec = importlib.util.spec_from_file_location('phase68_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase68_analyzer_contract_static_only_and_guardrails():
    source = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in source
    assert 'Phase68 Corridor Centerline Target Selection Design / Static Replay' in source
    assert 'phase67_goal1_timeout_visual_replay' in source
    for guardrail in [
        'static/log replay only',
        'no runtime dispatch integration',
        'no Nav2/MPPI/controller parameter edits',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no corridor-following cmd_vel control',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
    ]:
        assert guardrail in source
    forbidden = ['rclpy', 'NavigateToPose', 'create_client', 'ros2 launch', '--send-inner-ingress-goal']
    for token in forbidden:
        assert token not in source


def test_phase68_required_replay_schema_and_classifications_present():
    source = _read(ANALYZER)
    for name in [
        'load_phase67_goal1_evidence',
        'wall_rectangles',
        'cast_wall_clearance',
        'sample_centerline_candidates',
        'evaluate_centerline_candidate',
        'classify_phase68',
        'analyze_phase68',
    ]:
        assert f'def {name}' in source
    for field in [
        'left_wall_clearance_m',
        'right_wall_clearance_m',
        'balance_error_m',
        'min_clearance_m',
        'occupancy',
        'static_local_cost',
        'footprint_cost_summary',
        'front_wedge_cost_summary',
        'forward_progress_m',
    ]:
        assert field in source
    for classification in [
        'CENTERLINE_REPLAY_IMPROVES_LOCAL_COST',
        'CENTERLINE_REPLAY_NO_SAFE_CANDIDATE',
        'TARGET_ALREADY_CENTERED',
        'INSUFFICIENT_EVIDENCE',
    ]:
        assert classification in source


def test_phase68_centerline_sampling_prefers_balanced_clearance_in_synthetic_corridor():
    module = _load_module()
    walls = [
        {'name': 'left_wall', 'xmin': -1.1, 'xmax': -1.0, 'ymin': -5.0, 'ymax': 5.0},
        {'name': 'right_wall', 'xmin': 1.0, 'xmax': 1.1, 'ymin': -5.0, 'ymax': 5.0},
    ]
    evidence = {
        'dispatch_pose': [0.0, 0.0, 1.57079632679],
        'target': [0.45, 1.0],
        'open_direction_rad': 1.57079632679,
    }
    candidates = module.sample_centerline_candidates(evidence, forward_offsets_m=[0.0], lateral_offsets_m=[-0.45, 0.0, 0.45])
    evaluated = [module.evaluate_centerline_candidate(c, walls, robot_radius_m=0.35, inflation_radius_m=0.70) for c in candidates]
    best = min(evaluated, key=lambda row: row['score'])
    assert abs(best['target'][0]) < 1e-6
    assert best['balance_error_m'] < 1e-6
    original = module.evaluate_centerline_candidate({'target': [0.45, 1.0], 'forward_progress_m': 1.0, 'lateral_offset_m': 0.0, 'forward_offset_m': 0.0, 'open_direction_rad': 1.57079632679}, walls, robot_radius_m=0.35, inflation_radius_m=0.70)
    assert best['static_local_cost'] < original['static_local_cost']


def test_phase68_analyzer_replays_phase67_artifact_and_emits_comparison(tmp_path):
    assert PHASE67_ARTIFACT.exists(), 'Phase68 must reuse accepted Phase67/Phase66 artifacts'
    module = _load_module()
    output = tmp_path / 'phase68.json'
    result = module.analyze_phase68(phase67_artifact=PHASE67_ARTIFACT, output=output)
    assert output.exists()
    assert result['run_id'] == RUN_ID
    assert result['runtime_dispatch_changed'] is False
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['classification'] in {
        'CENTERLINE_REPLAY_IMPROVES_LOCAL_COST',
        'CENTERLINE_REPLAY_NO_SAFE_CANDIDATE',
        'TARGET_ALREADY_CENTERED',
        'INSUFFICIENT_EVIDENCE',
    }
    assert result['original_goal1_target']['target']
    assert isinstance(result['centerline_candidates'], list)
    assert 'best_centerline_candidate' in result
    if result['classification'] == 'CENTERLINE_REPLAY_IMPROVES_LOCAL_COST':
        assert result['best_centerline_candidate']['improves_over_original'] is True
        assert result['best_centerline_candidate']['balance_error_m'] <= result['original_goal1_target']['balance_error_m']


def test_phase68_report_contract_stop_condition():
    report = _read(REPORT)
    assert 'Phase68' in report
    assert RUN_ID in report
    assert 'static/log replay only' in report or 'static replay' in report.lower()
    assert '不接 runtime' in report or 'no runtime dispatch integration' in report
    assert 'not autonomous exploration success' in report or '不是自主探索成功' in report
    assert 'not exit success' in report or '不是出口成功' in report
    assert '不进入 Phase69' in report or 'Do not enter Phase69' in report
