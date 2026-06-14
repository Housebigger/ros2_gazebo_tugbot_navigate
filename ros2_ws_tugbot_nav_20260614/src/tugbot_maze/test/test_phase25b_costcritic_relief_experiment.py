import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAV_BASE = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml'
NAV_PHASE25B = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_phase25b_costcritic_relief_params.yaml'
NAV_PHASE25A = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_phase25a_local_cost_relief_params.yaml'
LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
COMPARE = ROOT / 'tools' / 'compare_phase25b_metrics.py'


def test_phase25b_overlay_is_single_costcritic_family_change():
    assert NAV_PHASE25B.exists(), 'Phase25B CostCritic params must exist'
    base = NAV_BASE.read_text(encoding='utf-8')
    phase = NAV_PHASE25B.read_text(encoding='utf-8')
    phase25a = NAV_PHASE25A.read_text(encoding='utf-8')
    assert 'Phase 25B' in phase
    assert 'MPPI CostCritic narrow experiment' in phase
    assert 'CostCritic:' in phase
    assert 'cost_weight: 2.5' in phase
    assert 'cost_weight: 3.81' in base
    assert 'trajectory_point_step: 2' in phase
    assert 'inflation_radius: 0.46' in phase
    assert 'cost_scaling_factor: 3.0' in phase
    assert 'inflation_radius: 0.55' in phase25a
    forbidden = [
        'required_movement_radius: 0.3',
        'movement_time_allowance: 20.0',
        'branch_goal_step_m',
        'inflation_radius: 0.55',
        'cost_scaling_factor: 4.5',
    ]
    for marker in forbidden:
        assert marker not in phase, f'Phase25B must not mix intervention families: {marker}'


def test_phase25b_launch_and_wrapper_are_reversible():
    launch = LAUNCH.read_text(encoding='utf-8')
    wrapper = WRAPPER.read_text(encoding='utf-8')
    assert 'phase25b_costcritic_relief_profile' in launch
    assert 'nav2_slam_phase25b_costcritic_relief_params.yaml' in launch
    assert 'phase25b_runN' in wrapper
    assert 'PHASE25B_PROFILE=true' in wrapper
    assert 'phase25b_costcritic_relief_profile:="$PHASE25B_PROFILE"' in wrapper


def test_phase25b_metrics_comparator_acceptance_contract(tmp_path):
    baseline = tmp_path / 'baseline.json'
    experiment = tmp_path / 'experiment.json'
    output = tmp_path / 'decision.json'
    baseline.write_text(json.dumps({
        'run_summary': {
            'timeout_cancel_count': 4,
            'goal_success_count': 8,
            'blocked_branch_count': 0,
            'blacklisted_goal_count': 0,
            'final_mode': 'FAILED_EXHAUSTED',
        },
        'timeout_subtypes': {
            'summary': {
                'controller_subtype_counts': {'footprint_path_blocked_late_silent': 2},
            }
        }
    }), encoding='utf-8')
    experiment.write_text(json.dumps({
        'run_summary': {
            'timeout_cancel_count': 2,
            'goal_success_count': 9,
            'blocked_branch_count': 0,
            'blacklisted_goal_count': 0,
            'final_mode': 'EXIT_REACHED',
        },
        'timeout_subtypes': {
            'summary': {
                'controller_subtype_counts': {'footprint_path_blocked_late_silent': 1},
            }
        }
    }), encoding='utf-8')

    result = subprocess.run([
        sys.executable,
        str(COMPARE),
        '--baseline', str(baseline),
        '--experiment', str(experiment),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['accepted'] is True
    assert data['checks']['footprint_path_blocked_reduced']['passed'] is True
    assert data['checks']['timeouts_reduced']['passed'] is True
    assert data['checks']['blocked_branch_no_regression']['passed'] is True
    assert data['checks']['blacklisted_goal_no_regression']['passed'] is True
    assert data['checks']['success_no_regression']['passed'] is True
    assert data['checks']['exit_behavior_preserved_or_improved']['passed'] is True
