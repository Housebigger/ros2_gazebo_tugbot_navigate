from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAV_BASE = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml'
NAV_PHASE25E = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_phase25e_costcritic_compromise_params.yaml'
NAV_CANDIDATE = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_candidate_costcritic_275_params.yaml'
LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def test_phase25g_candidate_params_are_named_and_canonical_baseline_unchanged():
    assert NAV_CANDIDATE.exists(), 'candidate params file must exist'
    base = NAV_BASE.read_text(encoding='utf-8')
    phase25e = NAV_PHASE25E.read_text(encoding='utf-8')
    candidate = NAV_CANDIDATE.read_text(encoding='utf-8')
    assert 'cost_weight: 3.81' in base, 'canonical baseline must remain unchanged'
    assert 'cost_weight: 2.75' in phase25e
    assert 'Candidate baseline' in candidate
    assert 'cost_weight: 2.75' in candidate
    assert 'phase25e_costcritic_compromise_profile' not in candidate
    forbidden = [
        'required_movement_radius: 0.3',
        'movement_time_allowance: 20.0',
        'branch_goal_step_m',
        'inflation_radius: 0.55',
        'cost_scaling_factor: 4.5',
        'cost_weight: 2.5',
    ]
    for marker in forbidden:
        assert marker not in candidate, f'candidate baseline must stay CostCritic-only: {marker}'


def test_phase25g_launch_and_wrapper_support_candidate_baseline_run_id():
    launch = LAUNCH.read_text(encoding='utf-8')
    wrapper = WRAPPER.read_text(encoding='utf-8')
    assert 'candidate_costcritic_275_profile' in launch
    assert 'nav2_slam_candidate_costcritic_275_params.yaml' in launch
    assert 'candidate_baseline_runN' in wrapper
    assert 'CANDIDATE_COSTCRITIC_275_PROFILE=true' in wrapper
    assert 'candidate_costcritic_275_profile:="$CANDIDATE_COSTCRITIC_275_PROFILE"' in wrapper
