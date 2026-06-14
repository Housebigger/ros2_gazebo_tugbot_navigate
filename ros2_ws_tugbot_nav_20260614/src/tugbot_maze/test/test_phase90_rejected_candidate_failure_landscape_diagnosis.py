from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RUN_ID = 'phase90_rejected_candidate_failure_landscape_diagnosis'
PHASE89_RUN_ID = 'phase89_safety_first_refinement_bounded_goal2_validation'
ANALYZER = ROOT / 'tools' / 'analyze_phase90_rejected_candidate_failure_landscape.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase90_rejected_candidate_failure_landscape_diagnosis_report.md'
OUTPUT_JSON = ROOT / 'log' / RUN_ID / f'{RUN_ID}.json'
MIN_SUMMARY = ROOT / 'log' / RUN_ID / f'{RUN_ID}_minimal_summary.md'
CLEANUP_SUMMARY = ROOT / 'log' / RUN_ID / 'phase90_process_cleanup_summary.md'

ALLOWED_CLASSIFICATIONS = {
    'STAGING_ALIGNMENT_GOAL_NEEDED',
    'CANDIDATE_FAMILY_TOO_LOCAL',
    'FRONT_WEDGE_DOMINANT_BLOCKER',
    'FOOTPRINT_DOMINANT_BLOCKER',
    'SAFETY_FLOOR_DOMINANT_BLOCKER',
    'OFFSET_GRID_INSUFFICIENT',
    'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE',
}

REQUIRED_FAILURE_KEYS = {
    'safety_floor',
    'footprint_lethal_regression',
    'front_wedge_lethal_regression',
    'clearance_insufficient',
    'forward_progress',
    'occupancy',
    'same_corridor',
    'two_side_wall',
}


def load_analyzer():
    assert ANALYZER.exists(), f'missing Phase90 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase90_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps({'state': row}, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def candidate(**overrides):
    data = {
        'candidate_index': 0,
        'lateral_offset_m': 0.0,
        'forward_offset_m': 0.0,
        'heading_offset_rad': 0.0,
        'hard_safety_pass': False,
        'safety_floor_ok': True,
        'footprint_lethal_not_increased': True,
        'front_wedge_lethal_not_increased': True,
        'forward_progress_ok': True,
        'forward_progress_not_lowered': True,
        'target_has_clearance': True,
        'occupancy_free': True,
        'same_corridor': True,
        'two_side_wall_evidence': True,
        'min_clearance_m': 0.50,
        'safety_min_clearance_floor_m': 0.45,
        'front_wedge_lethal_count': 0,
        'footprint_lethal_count': 0,
        'front_wedge_high_cost_count': 0,
        'footprint_high_cost_count': 0,
        'local_cost_max_radius': 54,
        'left_wall_clearance_m': 0.55,
        'right_wall_clearance_m': 0.55,
        'target': [2.0, 1.0],
    }
    data.update(overrides)
    data['candidate_index'] = overrides.get('candidate_index', data['candidate_index'])
    return data


def make_phase89_artifact_dir(tmp_path: Path, candidates: list[dict] | None = None, *, include_analysis: bool = True) -> Path:
    artifact_dir = tmp_path / PHASE89_RUN_ID
    artifact_dir.mkdir(parents=True)
    cands = candidates if candidates is not None else [candidate(safety_floor_ok=False, min_clearance_m=0.35)]
    if include_analysis:
        analysis = {
            'classification': 'PHASE88_REFINEMENT_STILL_REJECTED',
            'validation_goal_sequence': 1,
            'used_target_match_fallback': True,
            'goal2_dispatch_context': {
                'candidate_count': len(cands),
                'hard_safety_pass_candidate_count': sum(1 for c in cands if c.get('hard_safety_pass') is True),
                'candidate_family': {
                    'bounded_local_search': True,
                    'centerline_projection': True,
                    'lateral_offsets_m': [-0.45, 0.0, 0.45],
                    'forward_offsets_m': [0.0, 0.1, 0.2],
                    'heading_offsets_rad': [-0.08726646259971647, 0.0, 0.08726646259971647],
                },
                'multi_candidate_forward_search': True,
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
                'original_target_preserved_on_reject': True,
                'branch_scoring_changed': False,
                'fallback_terminal_acceptance_used': False,
                'original_target': [2.076, 1.023],
                'target': [2.076, 1.023],
                'selection_priority_trace': ['hard safety pass', 'no footprint/front-wedge lethal regression'],
                'rejected_candidate_summaries': [{'candidate_index': c.get('candidate_index')} for c in cands],
                'centerline_target_refinement': {
                    'candidate_count': len(cands),
                    'hard_safety_pass_candidate_count': sum(1 for c in cands if c.get('hard_safety_pass') is True),
                    'multi_candidate_forward_search': True,
                    'refinement_applied': False,
                    'refinement_reject_reason': 'lethal_cost_regression',
                    'original_target_preserved_on_reject': True,
                    'candidate_family': {
                        'bounded_local_search': True,
                        'centerline_projection': True,
                        'lateral_offsets_m': [-0.45, 0.0, 0.45],
                        'forward_offsets_m': [0.0, 0.1, 0.2],
                        'heading_offsets_rad': [-0.08726646259971647, 0.0, 0.08726646259971647],
                    },
                    'original_metrics': {
                        'front_wedge_lethal_count': 0,
                        'footprint_lethal_count': 0,
                        'min_clearance_m': 0.3535533958616298,
                    },
                    'forward_executability_check': {'reason': 'lethal_cost_regression', 'passed': False},
                    'rejected_candidate_summaries': [{'candidate_index': c.get('candidate_index')} for c in cands],
                    'candidates': cands,
                },
            },
            'goal2_outcome': {'event': 'timeout', 'reason': 'goal_timeout', 'timed_out': True, 'succeeded': False},
            'guardrails': {'no_nav2_mppi_controller_tuning': True},
            'autonomous_exploration_success_claimed': False,
            'exit_success_claimed': False,
        }
        (artifact_dir / f'{PHASE89_RUN_ID}_analysis.json').write_text(json.dumps(analysis, indent=2, sort_keys=True), encoding='utf-8')
    write_jsonl(artifact_dir / f'{PHASE89_RUN_ID}_goal_events.jsonl', [{'event': 'dispatch', 'goal_sequence': 1}, {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}])
    write_jsonl(artifact_dir / f'{PHASE89_RUN_ID}_local_costmap_samples.jsonl', [{
        'goal_sequence': 1,
        'front_wedge_cost': {'lethal_count': 66, 'max': 99},
        'robot_footprint_cost': {'lethal_count': 52, 'max': 99},
        'target_footprint_cost': {'lethal_count': 0, 'max': 63},
        'robot_pose': [2.42, 1.02, 1.58],
    }])
    (artifact_dir / f'{PHASE89_RUN_ID}_raw_capture.json').write_text(json.dumps({'scan': {}, 'local_costmap': {}, 'odom': {}, 'tf': {'map->base_link': {}}, 'footprint': {'points': []}}), encoding='utf-8')
    return artifact_dir


def test_phase90_analyzer_contract_and_insufficient_evidence(tmp_path):
    module = load_analyzer()
    assert set(module.ALLOWED_CLASSIFICATIONS) == ALLOWED_CLASSIFICATIONS
    artifact_dir = make_phase89_artifact_dir(tmp_path, include_analysis=False)
    result = module.analyze_phase89_artifacts(artifact_dir)
    assert result['classification'] == 'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE'
    assert 'missing_phase89_analysis_json' in result['evidence_gaps']
    assert result['autonomous_exploration_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['phase91_entered'] is False


def test_phase90_counts_required_failure_reasons_and_offset_distributions(tmp_path):
    module = load_analyzer()
    cands = [
        candidate(candidate_index=0, lateral_offset_m=-0.45, forward_offset_m=0.0, heading_offset_rad=-0.087, safety_floor_ok=False, footprint_lethal_not_increased=False, front_wedge_lethal_not_increased=False, min_clearance_m=0.10, footprint_lethal_count=51, front_wedge_lethal_count=90, right_wall_clearance_m=0.10, left_wall_clearance_m=1.05),
        candidate(candidate_index=1, lateral_offset_m=0.0, forward_offset_m=0.0, heading_offset_rad=0.0, safety_floor_ok=False, min_clearance_m=0.35),
        candidate(candidate_index=2, lateral_offset_m=0.45, forward_offset_m=0.2, heading_offset_rad=0.087, safety_floor_ok=False, target_has_clearance=False, front_wedge_lethal_not_increased=False, min_clearance_m=0.05, front_wedge_lethal_count=40, left_wall_clearance_m=0.05, right_wall_clearance_m=1.0),
    ]
    result = module.analyze_phase89_artifacts(make_phase89_artifact_dir(tmp_path, cands))
    landscape = result['candidate_failure_landscape']
    assert REQUIRED_FAILURE_KEYS.issubset(set(landscape['failure_reason_counts']))
    assert landscape['failure_reason_counts']['safety_floor']['false_count'] == 3
    assert landscape['failure_reason_counts']['footprint_lethal_regression']['false_count'] == 1
    assert landscape['failure_reason_counts']['front_wedge_lethal_regression']['false_count'] == 2
    assert landscape['failure_reason_counts']['clearance_insufficient']['false_count'] == 1
    assert landscape['failure_reason_counts']['forward_progress']['false_count'] == 0
    assert landscape['failure_reason_counts']['occupancy']['false_count'] == 0
    assert landscape['failure_reason_counts']['same_corridor']['false_count'] == 0
    assert landscape['failure_reason_counts']['two_side_wall']['false_count'] == 0
    assert set(landscape['offset_distributions']) == {'lateral_offset_m', 'forward_offset_m', 'heading_offset_rad'}
    assert landscape['wall_side_concentration']['right_wall_nearer_count'] == 1
    assert landscape['wall_side_concentration']['left_wall_nearer_count'] == 1


def test_phase90_classifies_dominant_blockers_and_staging_support(tmp_path):
    module = load_analyzer()
    safety_floor_candidates = [
        candidate(candidate_index=i, safety_floor_ok=False, min_clearance_m=0.35, lateral_offset_m=0.0, forward_offset_m=0.0, footprint_lethal_not_increased=True, front_wedge_lethal_not_increased=True)
        for i in range(5)
    ]
    safety = module.analyze_phase89_artifacts(make_phase89_artifact_dir(tmp_path / 'safety', safety_floor_candidates))
    assert safety['classification'] == 'SAFETY_FLOOR_DOMINANT_BLOCKER'
    assert safety['candidate_failure_landscape']['all_local_candidate_families_unexecutable'] is True
    assert safety['two_step_intermediate_goal_assessment']['staging_alignment_goal_supported'] is True

    front_wedge_candidates = [
        candidate(candidate_index=i, front_wedge_lethal_not_increased=False, front_wedge_lethal_count=20, safety_floor_ok=True, min_clearance_m=0.50)
        for i in range(4)
    ] + [candidate(candidate_index=10, safety_floor_ok=True, min_clearance_m=0.50)]
    front = module.analyze_phase89_artifacts(make_phase89_artifact_dir(tmp_path / 'front', front_wedge_candidates))
    assert front['classification'] == 'FRONT_WEDGE_DOMINANT_BLOCKER'

    footprint_candidates = [
        candidate(candidate_index=i, footprint_lethal_not_increased=False, footprint_lethal_count=25, safety_floor_ok=True, min_clearance_m=0.50)
        for i in range(4)
    ] + [candidate(candidate_index=10, safety_floor_ok=True, min_clearance_m=0.50)]
    footprint = module.analyze_phase89_artifacts(make_phase89_artifact_dir(tmp_path / 'footprint', footprint_candidates))
    assert footprint['classification'] == 'FOOTPRINT_DOMINANT_BLOCKER'


def test_phase90_real_artifacts_report_and_guardrails_exist():
    assert CLEANUP_SUMMARY.exists(), f'missing Phase90 cleanup summary: {CLEANUP_SUMMARY}'
    assert OUTPUT_JSON.exists(), f'missing Phase90 JSON artifact: {OUTPUT_JSON}'
    assert MIN_SUMMARY.exists(), f'missing Phase90 minimal summary: {MIN_SUMMARY}'
    assert REPORT.exists(), f'missing Phase90 report: {REPORT}'
    result = json.loads(OUTPUT_JSON.read_text(encoding='utf-8'))
    assert result['classification'] in ALLOWED_CLASSIFICATIONS
    assert result['source_phase89_context']['candidate_count'] == 63
    assert result['source_phase89_context']['hard_safety_pass_candidate_count'] == 0
    assert result['source_phase89_context']['refinement_applied'] is False
    assert result['source_phase89_context']['refinement_reject_reason'] == 'lethal_cost_regression'
    assert result['guardrails']['no_nav2_mppi_controller_tuning'] is True
    assert result['guardrails']['no_phase88_refinement_changed'] is True
    assert result['autonomous_exploration_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['phase91_entered'] is False
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'Rejected candidate failure landscape diagnosis',
        'No Phase88 refinement changed',
        'No branch scoring changed',
        'No centerline gate changed',
        'No directional readiness changed',
        'No fallback/terminal acceptance changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase91 not entered',
    ]:
        assert phrase in text
