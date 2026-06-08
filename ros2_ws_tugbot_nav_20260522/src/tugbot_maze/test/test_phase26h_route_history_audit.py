import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26h_route_history_audit.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def candidate(angle: float, target: list[float], target_exit: float, delta: float, state='untried', rejection=None, rank=None, **extra):
    row = {
        'branch_angle': angle,
        'target': target,
        'target_exit_dist': target_exit,
        'exit_progress_delta_m': delta,
        'target_clearance_m': 0.8,
        'path_corridor_min_clearance_m': 0.7,
        'dispatch_path_local_cost_max': 0,
        'dispatch_path_local_cost_mean': 0.0,
        'target_local_cost': 0,
        'target_local_cost_max_radius': 0,
        'is_reverse_candidate': False,
        'is_backtrack_context': False,
        'is_near_exit_candidate': target_exit <= 1.0,
        'rejection_reason': rejection,
        'rank': rank,
        'state': state,
    }
    row.update(extra)
    return row


def dispatch(seq: int, pose: list[float], target: list[float], target_exit: float, robot_exit: float, candidates: list[dict], **extra):
    row = {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'dispatch_pose': pose,
        'target': target,
        'branch_angle': 0.0,
        'target_exit_dist': target_exit,
        'robot_exit_dist_at_dispatch': robot_exit,
        'chosen_branch_rank': 1,
        'candidate_branch_count': len(candidates),
        'candidate_branches': candidates,
        'selected_due_to_context': 'topology_exit_bias_score',
        'near_exit': target_exit <= 1.0,
        'dispatch_path_local_cost_max': 0,
        'dispatch_target_local_cost': 0,
    }
    row.update(extra)
    return row


def test_phase26h_route_history_audit_traces_explored_rejected_candidate_to_prior_success_and_joins_timeout_artifacts(tmp_path):
    log_dir = tmp_path / 'log'
    run_id = 'phase26h_synthetic'
    explored_target = [3.55, 3.67]
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [
        dispatch(
            1,
            pose=[2.8, 3.1, 0.0],
            target=explored_target,
            target_exit=0.80,
            robot_exit=1.20,
            candidates=[candidate(0.1, explored_target, 0.80, 0.40, rejection=None, rank=1, is_near_exit_candidate=True)],
        ),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            pose=[2.84, 3.13, 0.09],
            target=[2.77, 2.22],
            target_exit=1.45,
            robot_exit=1.16,
            candidates=[
                candidate(0.1, explored_target, 0.80, 0.36, state='explored', rejection='explored', rank=None,
                          is_near_exit_candidate=True, dispatch_path_local_cost_max=99, target_local_cost=99,
                          target_local_cost_max_radius=99),
                candidate(-1.48, [2.77, 2.22], 1.45, -0.29, rejection=None, rank=1,
                          dispatch_path_local_cost_max=73, target_local_cost=73, target_local_cost_max_radius=99),
            ],
        ),
        {'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    ])
    write_jsonl(log_dir / f'{run_id}_explorer_state.jsonl', [
        {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'goal_failure_count': 0, 'exit_distance_m': 1.1},
        {'mode': 'FAILED_EXHAUSTED', 'goal_count': 2, 'goal_success_count': 1, 'goal_failure_count': 1, 'exit_distance_m': 1.4},
    ])
    (log_dir / f'{run_id}_timeout_subtypes.json').write_text(json.dumps({
        'timeout_subtypes': [{
            'goal_sequence': 2,
            'controller_subtype': 'footprint_path_blocked_late_silent',
            'timing_subtype': 'cmd_silent_after_recovery_abort',
            'combined_subtype': 'footprint_path_blocked_late_silent+cmd_silent_after_recovery_abort',
            'reasons': {'footprint_path_blocked': True, 'path_ahead_1_0m_cost_max': 100},
        }]
    }), encoding='utf-8')
    (log_dir / f'{run_id}_post_recovery_enriched.json').write_text(json.dumps({
        'enriched_recovery_snapshots': [{
            'goal_sequence': 2,
            'snapshot_density_sufficient': True,
            'pre_recovery_path_ahead_1_0m_cost_max': 99,
            'post_recovery_path_ahead_1_0m_cost_max': 100,
            'near_zero_path_ahead_1_0m_cost_max': 100,
            'near_zero_robot_to_path_distance_m': 0.03,
            'path_update_count_after_recovery': 2,
        }]
    }), encoding='utf-8')
    output = tmp_path / 'phase26h.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--runs', run_id,
        '--focus-run', run_id,
        '--focus-goal-sequence', '2',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26H'
    assert data['analysis_only'] is True
    assert 'do_not_enter_phase27' in data['decision']['guardrails']

    run = data['runs'][run_id]
    assert run['summary']['rejection_reason_counts']['explored'] == 1
    assert run['summary']['rejection_reason_counts']['lower_rank_not_selected'] == 0
    assert run['summary']['near_exit_local_cost_explored_candidate_count'] == 1
    assert run['summary']['route_divergence_explored_case_count'] == 1

    focus = data['focus_audit']
    assert focus['run_id'] == run_id
    assert focus['goal_sequence'] == 2
    assert focus['route_divergence_case']['best_rejected_rejection_reason'] == 'explored'
    assert focus['explored_rejected_candidates'][0]['provenance']['source_goal_sequence'] == 1
    assert focus['explored_rejected_candidates'][0]['provenance']['source_outcome'] == 'success'
    assert focus['explored_rejected_candidates'][0]['provenance']['classification'] == 'prior_success_near_same_target'
    assert focus['joined_timeout_subtype']['controller_subtype'] == 'footprint_path_blocked_late_silent'
    assert focus['joined_post_recovery']['near_zero_path_ahead_1_0m_cost_max'] == 100
    assert focus['recommendation'] == 'do_not_intervene_audit_more_route_history'


def test_phase26h_route_history_audit_classifies_unknown_explored_provenance_when_no_prior_matching_event(tmp_path):
    log_dir = tmp_path / 'log'
    run_id = 'phase26h_unknown'
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [
        dispatch(
            5,
            pose=[2.0, 2.0, 0.0],
            target=[2.1, 1.1],
            target_exit=1.5,
            robot_exit=1.0,
            candidates=[
                candidate(0.0, [3.8, 3.2], 0.3, 0.7, state='explored', rejection='explored', rank=None,
                          is_near_exit_candidate=True, dispatch_path_local_cost_max=99, target_local_cost=99,
                          target_local_cost_max_radius=99),
                candidate(-1.0, [2.1, 1.1], 1.5, -0.5, rejection=None, rank=1),
            ],
        )
    ])
    output = tmp_path / 'phase26h_unknown.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--runs', run_id,
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['runs'][run_id]['explored_route_history_cases'][0]
    assert case['explored_rejected_candidates'][0]['provenance']['classification'] == 'unknown_no_prior_matching_dispatch'
    assert data['runs'][run_id]['summary']['unknown_explored_provenance_count'] == 1


def test_phase26h_route_history_audit_traces_explored_state_by_same_start_node_angle_match(tmp_path):
    log_dir = tmp_path / 'log'
    run_id = 'phase26h_angle_match'
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [
        dispatch(
            7,
            pose=[2.1, 3.0, 0.14],
            target=[3.0, 3.18],
            target_exit=1.02,
            robot_exit=1.89,
            candidates=[candidate(0.15, [3.0, 3.18], 1.02, 0.87, rejection=None, rank=1)],
            start_node_id=7,
            current_node_id=7,
            branch_angle=0.15,
        ),
        {'event': 'success', 'goal_sequence': 7, 'result_reason': 'succeeded'},
        dispatch(
            8,
            pose=[2.84, 3.13, 0.09],
            target=[2.77, 2.22],
            target_exit=1.45,
            robot_exit=1.16,
            candidates=[
                candidate(0.09, [3.55, 3.67], 0.81, 0.35, state='explored', rejection='explored', rank=None,
                          is_near_exit_candidate=True, dispatch_path_local_cost_max=99, target_local_cost=99,
                          target_local_cost_max_radius=99),
                candidate(-1.48, [2.77, 2.22], 1.45, -0.29, rejection=None, rank=1),
            ],
            start_node_id=7,
            current_node_id=7,
            branch_angle=-1.48,
        ),
    ])
    output = tmp_path / 'phase26h_angle_match.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--runs', run_id,
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    candidate_payload = data['runs'][run_id]['explored_route_history_cases'][0]['explored_rejected_candidates'][0]
    assert candidate_payload['provenance']['classification'] == 'prior_success_same_start_node_angle_match'
    assert candidate_payload['provenance']['source_goal_sequence'] == 7
    assert candidate_payload['provenance']['angle_delta_rad'] < 0.2
