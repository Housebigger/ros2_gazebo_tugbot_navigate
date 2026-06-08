from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase26f_branch_choice_diagnostics.py'
COVERAGE = ROOT / 'tools' / 'check_phase26e_branch_diagnostics_coverage.py'


def test_phase26g_smoke_wrapper_accepts_matched_baseline_and_candidate_run_ids():
    source = WRAPPER.read_text(encoding='utf-8')
    pattern_block = source[source.index('RUN_ID_PATTERN'):source.index('ROOT_DIR=')]
    assert 'phase26g_baseline_run[0-9]+' in pattern_block
    assert 'phase26g_candidate_run[0-9]+' in pattern_block
    assert 'phase26g_baseline_runN' in source
    assert 'phase26g_candidate_runN' in source


def test_phase26g_candidate_runs_use_existing_candidate_profile_without_changing_baseline():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'PHASE26G_CANDIDATE_PROFILE=false' in source
    assert 'phase26g_candidate_run[0-9]+' in source
    candidate_block = source[source.index('PHASE26G_CANDIDATE_PROFILE=false'):source.index('NAV2_PARAMS_FILE=')]
    assert 'CANDIDATE_COSTCRITIC_275_PROFILE=true' in candidate_block
    assert 'phase26g_baseline_run' not in candidate_block


def test_phase26g_cleanup_patterns_cover_new_run_ids():
    source = WRAPPER.read_text(encoding='utf-8')
    cleanup_block = source[source.index('cleanup()'):source.index('trap cleanup EXIT')]
    assert '26g_baseline' in cleanup_block
    assert '26g_candidate' in cleanup_block
    assert 'phase26g_baseline_run[0-9]+' in source
    assert 'phase26g_candidate_run[0-9]+' in source


def test_phase26g_analysis_tools_are_analysis_only_and_accept_matched_runs():
    analyzer_source = ANALYZER.read_text(encoding='utf-8')
    coverage_source = COVERAGE.read_text(encoding='utf-8')
    assert '--baseline-runs' in analyzer_source
    assert '--candidate-runs' in analyzer_source
    assert 'analysis_only' in analyzer_source
    assert 'do_not_enter_phase27' in analyzer_source
    assert 'coverage check only' in coverage_source
