from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def test_phase26l_real_run_ids_are_supported_by_controlled_smoke_wrapper():
    source = WRAPPER.read_text(encoding='utf-8')
    pattern_block = source[source.index('RUN_ID_PATTERN'):source.index('ROOT_DIR=')]
    assert 'phase26l_baseline_run[0-9]+' in pattern_block
    assert 'phase26l_candidate_run[0-9]+' in pattern_block
    assert 'phase26l_baseline_runN' in source
    assert 'phase26l_candidate_runN' in source


def test_phase26l_real_run_wrapper_captures_spatial_contract_artifacts_and_logs():
    source = WRAPPER.read_text(encoding='utf-8')
    assert '--max-high-cost-points 40' in source
    assert 'PHASE26L_BASELINE_PROFILE' in source
    assert 'PHASE26L_CANDIDATE_PROFILE' in source
    assert 'phase26l_candidate_run[0-9]+' in source
    assert 'phase26l_baseline_run[0-9]+' in source
    assert 'POST_RECOVERY_SNAPSHOTS' in source
    assert 'CONTROLLER_DYNAMICS' in source
    assert 'LAUNCH_LOG' in source
    assert 'GOAL_EVENTS' in source
    assert 'record_post_recovery_snapshots.py' in source
