import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = ROOT / 'src' / 'tugbot_navigation' / 'config'
NAV_BASE = CONFIG_DIR / 'nav2_slam_params.yaml'
NAV_PHASE25E = CONFIG_DIR / 'nav2_slam_phase25e_costcritic_compromise_params.yaml'
NAV_CANDIDATE = CONFIG_DIR / 'nav2_slam_candidate_costcritic_275_params.yaml'
FINGERPRINT = ROOT / 'tools' / 'fingerprint_nav2_params.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def _load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _controller_params(data):
    return data['controller_server']['ros__parameters']


def _costcritic(data):
    return _controller_params(data)['FollowPath']['CostCritic']


def test_phase25e_and_candidate_275_are_semantically_equivalent_except_comments():
    phase25e = _load_yaml(NAV_PHASE25E)
    candidate = _load_yaml(NAV_CANDIDATE)
    assert phase25e == candidate


def test_phase26a_costcritic_weights_are_locked_for_baseline_and_candidate_profiles():
    base = _load_yaml(NAV_BASE)
    phase25e = _load_yaml(NAV_PHASE25E)
    candidate = _load_yaml(NAV_CANDIDATE)
    assert _costcritic(base)['cost_weight'] == 3.81
    assert _costcritic(phase25e)['cost_weight'] == 2.75
    assert _costcritic(candidate)['cost_weight'] == 2.75
    assert _costcritic(phase25e) == _costcritic(candidate)


def test_phase26a_semantic_diff_reports_no_delta_between_phase25e_and_candidate():
    result = subprocess.run([
        sys.executable,
        str(FINGERPRINT),
        '--params-file', str(NAV_PHASE25E),
        '--compare-params-file', str(NAV_CANDIDATE),
        '--run-id', 'phase26a_unit',
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data['run_id'] == 'phase26a_unit'
    assert data['params_file']['costcritic_cost_weight'] == 2.75
    assert data['compare']['equivalent'] is True
    assert data['compare']['semantic_differences'] == []


def test_phase26a_fingerprint_reports_candidate_vs_canonical_delta(tmp_path):
    output = tmp_path / 'fingerprint.json'
    result = subprocess.run([
        sys.executable,
        str(FINGERPRINT),
        '--params-file', str(NAV_CANDIDATE),
        '--baseline-params-file', str(NAV_BASE),
        '--run-id', 'candidate_baseline_run99',
        '--selected-profile', 'candidate_costcritic_275',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['run_id'] == 'candidate_baseline_run99'
    assert data['selected_profile'] == 'candidate_costcritic_275'
    assert data['params_file']['sha256']
    assert data['params_file']['costcritic_cost_weight'] == 2.75
    assert data['baseline']['costcritic_cost_weight'] == 3.81
    assert data['baseline_delta']['costcritic_cost_weight'] == -1.06


def test_phase26a_wrapper_creates_fingerprint_and_runtime_param_artifacts():
    wrapper = WRAPPER.read_text(encoding='utf-8')
    assert 'PARAMS_FINGERPRINT="$LOG_DIR/${RUN_ID}_params_fingerprint.json"' in wrapper
    assert 'RUNTIME_PARAMS_DIR="$LOG_DIR/${RUN_ID}_runtime_params"' in wrapper
    assert 'tools/fingerprint_nav2_params.py' in wrapper
    assert 'tools/dump_controller_runtime_params.py' in wrapper
    assert '--output "$RUNTIME_PARAMS_DIR/controller_server.yaml"' in wrapper
    assert 'phase26a_fingerprint_smoke' in wrapper
