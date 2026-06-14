from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase58_post_ingress_candidate_formation_stability_replay.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase58_post_ingress_candidate_formation_stability_replay.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase58_post_ingress_candidate_formation_stability_replay_report.md'
RUN_ID = 'phase58_post_ingress_candidate_formation_stability_replay'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase58_files_exist_and_use_required_run_id():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert f'log/${{RUN_ID}}' in wrapper
    assert 'phase58_post_ingress_candidate_formation_stability_replay/' in analyzer


def test_phase58_guardrails_and_replay_contract():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    joined = wrapper + '\n' + analyzer
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in joined
    assert 'maze_20260528_scaled_instance.yaml' in joined
    assert 'ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}' in wrapper
    assert 'INGRESS_X="1.0"' in wrapper
    assert 'INGRESS_Y="0.0"' in wrapper
    assert '-p max_goals:=1' in wrapper
    assert '-p near_exit_fallback_enabled:=false' in wrapper
    assert '-p startup_warmup_no_dispatch:=false' in wrapper
    assert 'no Nav2/MPPI/controller parameter edits' in joined
    assert 'no clearance_radius_m tuning' in joined
    assert 'no map sufficiency threshold tuning' in joined
    assert 'no maze_explorer strategy change' in joined
    assert 'no autonomous exploration success claim' in joined
    assert 'first dispatch is not exit success' in joined
    assert 'bounded replay only' in joined
    assert 'PHASE58_REPLAY_COUNT' in wrapper
    assert 'REPLAY_COUNT' in wrapper
    assert 'replay_$(printf' in wrapper or 'replay_${replay_index}' in wrapper


def test_phase58_analyzer_classifications_and_required_metrics():
    analyzer = _read(ANALYZER)
    for classification in [
        'CANDIDATE_FORMATION_STABLE_DISPATCH_REPRODUCED',
        'CANDIDATE_FORMATION_STABLE_NO_DISPATCH',
        'CANDIDATE_FORMATION_UNSTABLE_POSE_OR_TIMING_SENSITIVE',
        'CANDIDATE_FORMATION_UNSTABLE_COSTMAP_SENSITIVE',
        'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE',
        'CANDIDATE_FORMATION_STABILITY_INCONCLUSIVE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for metric in [
        'ingress_final_pose_map',
        'ingress_final_yaw_rad',
        'time_from_ingress_success_to_explorer_start_sec',
        'time_from_explorer_start_to_gate_ready_sec',
        'time_from_explorer_start_to_first_topology_sec',
        'robot_pose_at_first_topology_sampling',
        'map_evidence_at_topology',
        'scan_evidence_at_topology',
        'tf_evidence_at_topology',
        'local_costmap_evidence_at_topology',
        'global_costmap_evidence_at_topology',
        'raw_open_direction_count',
        'filtered_open_direction_count',
        'accepted_open_direction_angle_rad',
        'accepted_open_direction_vector',
        'candidate_before_filter_count',
        'candidate_after_filter_count',
        'branch_candidate_rejection_reason',
        'junction_or_dead_end_policy_filter',
        'duplicate_or_exhausted_filter',
        'candidate_local_costmap_cell_state',
        'max_radius_cost',
        'dispatch_observed',
        'compare_replays',
    ]:
        assert metric in analyzer


def test_phase58_report_contract():
    report = _read(REPORT)
    assert 'Phase58' in report
    assert 'Post-Ingress Candidate Formation Stability Replay' in report
    assert RUN_ID in report
    assert 'TIMEOUT_INCONCLUSIVE_DATA_GAP' in report
    assert '不得写成自主探索成功' in report or 'does not claim autonomous exploration success' in report
    assert 'does not attribute first-dispatch timeout' in report or '不得把 Phase57 当作 first-dispatch timeout 已归因' in report
