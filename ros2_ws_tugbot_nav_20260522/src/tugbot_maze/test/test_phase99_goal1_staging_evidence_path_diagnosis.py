from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase99_goal1_staging_evidence_path.py'
PHASE97_RUN_ID = 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'


def load_analyzer():
    spec = importlib.util.spec_from_file_location('phase99_analyzer', ANALYZER)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def make_phase97_artifacts(tmp_path: Path, *, staging_candidates: list[dict] | None = None, phase88_candidates: list[dict] | None = None) -> Path:
    ad = tmp_path / PHASE97_RUN_ID
    ad.mkdir()
    if phase88_candidates is None:
        phase88_candidates = [
            {
                'candidate_index': i,
                'target_xy': [2.017 + 0.1 * i, 0.924],
                'target_yaw': 1.5635,
                'same_corridor': True,
                'two_side_wall_evidence': True,
                'left_wall_hit': True,
                'right_wall_hit': True,
                'left_wall_clearance_m': 0.6,
                'right_wall_clearance_m': 0.6,
                'min_clearance_m': 0.35,
                'hard_safety_pass': False,
                'safety_floor_ok': False,
                'candidate_reject_reasons': ['safety_floor_ok'],
            }
            for i in range(3)
        ]
    if staging_candidates is None:
        staging_candidates = [
            {
                'candidate_index': 0,
                'target_xy': [1.9605, 0.0244],
                'target_yaw': 1.5635,
                'staging_distance_m': 0.10,
                'staging_forward_offset_m': 0.0,
                'staging_lateral_offset_m': -0.1,
                'same_corridor': True,
                'two_side_wall_evidence': False,
                'hard_safety_pass': False,
                'safety_floor_ok': True,
                'bounded_short_distance': True,
                'forward_progress_ok': True,
                'footprint_lethal_not_increased': True,
                'front_wedge_lethal_not_increased': True,
                'candidate_reject_reasons': ['missing_two_side_wall_evidence'],
            },
            {
                'candidate_index': 1,
                'target_xy': [2.0112, 0.1240],
                'target_yaw': 1.5635,
                'staging_distance_m': 0.18,
                'staging_forward_offset_m': 0.1,
                'staging_lateral_offset_m': -0.15,
                'same_corridor': True,
                'two_side_wall_evidence': False,
                'hard_safety_pass': False,
                'safety_floor_ok': True,
                'bounded_short_distance': True,
                'forward_progress_ok': True,
                'footprint_lethal_not_increased': True,
                'front_wedge_lethal_not_increased': True,
                'candidate_reject_reasons': ['missing_two_side_wall_evidence'],
            },
        ]
    dispatch = {
        'event': 'dispatch',
        'goal_sequence': 1,
        'dispatch_pose': [1.86054, 0.02512, -0.00721],
        'target': [2.01703, 0.92401],
        'original_target': [2.01703, 0.92401],
        'selected_candidate_target': [2.01703, 0.92401],
        'selected_candidate_yaw': 1.5635,
        'branch_angle': 1.5635,
        'staging_candidates': staging_candidates,
        'staging_goal_pose': None,
        'staging_applied': False,
        'staging_reject_reason': 'missing_two_side_wall_evidence',
        'staging_executability_check': {
            'checked': True,
            'reason': 'missing_two_side_wall_evidence',
            'same_corridor': False,
            'two_side_wall_evidence': False,
            'hard_safety_pass': False,
            'safety_floor_ok': False,
            'bounded_short_distance': False,
            'forward_progress_ok': False,
        },
        'two_step_staging_plan': {
            'enabled': True,
            'trigger_conditions': {
                'execution_time_footprint_front_wedge_risk': True,
                'near_goal_lateral_residual': True,
                'safety_floor_dominant_blocker': True,
                'single_step_forward_search_no_hard_safety_pass': True,
            },
            'source_single_step': {
                'candidate_count': len(phase88_candidates),
                'hard_safety_pass_candidate_count': 0,
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
            },
        },
        'centerline_target_refinement': {
            'candidate_count': len(phase88_candidates),
            'corridor_heading_yaw': 1.5635,
            'hard_safety_pass_candidate_count': 0,
            'two_side_wall_candidate_count': len(phase88_candidates),
            'refinement_applied': False,
            'refinement_reject_reason': 'lethal_cost_regression',
            'original_metrics': {
                'target': [2.01703, 0.92401],
                'same_corridor': True,
                'two_side_wall_evidence': True,
                'left_wall_hit': True,
                'right_wall_hit': True,
            },
            'forward_executability_check': {
                'checked': True,
                'same_corridor': True,
                'two_side_wall_evidence': True,
                'reason': 'lethal_cost_regression',
            },
            'candidates': phase88_candidates,
        },
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    write_jsonl(ad / f'{PHASE97_RUN_ID}_goal_events.jsonl', [dispatch, {'event': 'failure', 'goal_sequence': 1}])
    write_jsonl(ad / f'{PHASE97_RUN_ID}_local_costmap_samples.jsonl', [
        {'goal_sequence': 1, 'robot_pose': [2.35, 0.92, 1.58], 'front_wedge_lethal_count': 109}
    ])
    write_jsonl(ad / f'{PHASE97_RUN_ID}_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'distance_remaining': 0.347, 'number_of_recoveries': 15}
    ])
    (ad / f'{PHASE97_RUN_ID}_raw_capture.json').write_text(json.dumps({
        'scan': {'ranges_count': 900, 'ranges_min': 0.56},
        'map': {'summary': {'known_count': 7176}},
        'local_costmap': {'summary': {'known_count': 3600}},
        'odom': {'pose': {'x': 2.23, 'y': 1.52, 'yaw': 2.0}},
        'tf': {'map->base_link': {'available': True, 'yaw': 1.99}},
    }), encoding='utf-8')
    (ad / f'{PHASE97_RUN_ID}_analysis.json').write_text(json.dumps({'classification': 'INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT', 'evidence_gaps': []}), encoding='utf-8')
    return ad


def make_phase98_artifacts(tmp_path: Path) -> Path:
    ad = tmp_path / 'phase98_goal1_recovery_dominant_failure_root_cause_diagnosis'
    ad.mkdir()
    (ad / 'phase98_goal1_recovery_dominant_failure_root_cause_analysis.json').write_text(json.dumps({
        'classification': 'GOAL1_STAGING_REJECT_ROOT_CAUSE',
        'goal1': {
            'staging_reject_reason': 'missing_two_side_wall_evidence',
            'staging_executability_check': {'same_corridor': False, 'two_side_wall_evidence': False},
        },
        'evidence_gaps': [],
    }), encoding='utf-8')
    return ad


def test_phase99_classification_tokens_are_exact():
    module = load_analyzer()
    assert set(module.ALLOWED_CLASSIFICATIONS) == {
        'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE',
        'STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88',
        'STAGING_FRAME_OR_POSE_PROJECTION_MISMATCH',
        'ENTRANCE_GOAL1_SINGLE_SIDE_WALL_GEOMETRY',
        'STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN',
        'STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA',
    }


def test_extracts_goal1_staging_evidence_path_and_classifies_short_near_candidate(tmp_path: Path):
    module = load_analyzer()
    phase97_dir = make_phase97_artifacts(tmp_path)
    phase98_dir = make_phase98_artifacts(tmp_path)
    result = module.analyze_phase99(phase97_dir, phase98_dir)
    assert result['classification'] == 'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE'
    g1 = result['goal1_staging_evidence_path']
    assert g1['staging_plan']['enabled'] is True
    assert g1['staging_reject_reason'] == 'missing_two_side_wall_evidence'
    assert g1['dispatch_pose'] == [1.86054, 0.02512, -0.00721]
    assert g1['corridor_heading_yaw'] == 1.5635
    assert g1['staging_candidate_summary']['candidate_count'] == 2
    assert g1['staging_candidate_summary']['two_side_wall_true_count'] == 0
    assert g1['staging_candidate_summary']['same_corridor_true_count'] == 2
    assert g1['staging_candidate_summary']['staging_distance_max_m'] == 0.18
    assert result['root_cause_evidence']['staging_near_dispatch_pose'] is True


def test_compares_phase88_two_side_wall_evidence_against_phase92_staging_evidence(tmp_path: Path):
    module = load_analyzer()
    phase97_dir = make_phase97_artifacts(tmp_path)
    phase98_dir = make_phase98_artifacts(tmp_path)
    result = module.analyze_phase99(phase97_dir, phase98_dir)
    comparison = result['phase88_vs_phase92_comparison']
    assert comparison['phase88_candidate_count'] == 3
    assert comparison['phase88_two_side_wall_true_count'] == 3
    assert comparison['phase88_same_corridor_true_count'] == 3
    assert comparison['phase92_staging_candidate_count'] == 2
    assert comparison['phase92_two_side_wall_true_count'] == 0
    assert comparison['evidence_window_delta']['phase92_y_min'] < 0.2
    assert comparison['evidence_window_delta']['phase88_y_min'] > 0.8
    assert comparison['interpretation'].startswith('Phase88 evidence was available at the forward Goal1 target window')


def test_insufficient_when_required_staging_or_phase98_evidence_missing(tmp_path: Path):
    module = load_analyzer()
    phase97_dir = make_phase97_artifacts(tmp_path, staging_candidates=[])
    empty_phase98 = tmp_path / 'empty_phase98'
    empty_phase98.mkdir()
    result = module.analyze_phase99(phase97_dir, empty_phase98)
    assert result['classification'] == 'STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA'
    assert 'phase98_analysis_missing' in result['evidence_gaps']
    assert 'goal1_staging_candidates_missing' in result['evidence_gaps']


def test_guardrails_and_summary_preserve_no_algorithm_or_nav2_changes(tmp_path: Path):
    module = load_analyzer()
    phase97_dir = make_phase97_artifacts(tmp_path)
    phase98_dir = make_phase98_artifacts(tmp_path)
    result = module.analyze_phase99(phase97_dir, phase98_dir)
    assert result['guardrails'] == {
        'algorithm_changed': False,
        'phase88_92_logic_changed': False,
        'branch_scoring_or_order_changed': False,
        'nav2_config_changed': False,
        'autonomous_exploration_success_claimed': False,
        'exit_success_claimed': False,
        'phase100_entered': False,
    }
    summary = module.render_minimal_summary(result)
    assert 'STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE' in summary
    assert 'Phase88 corridor evidence carry-over / evidence reuse' in summary
