import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = ROOT / 'src' / 'tugbot_navigation' / 'config'
NAV_CANDIDATE = CONFIG_DIR / 'nav2_slam_candidate_costcritic_275_params.yaml'
RUNTIME_DUMP = ROOT / 'tools' / 'dump_controller_runtime_params.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def test_phase26a1_runtime_dump_extractor_reads_costcritic_weight(tmp_path):
    runtime_yaml = tmp_path / 'controller_server.yaml'
    runtime_yaml.write_text(yaml.safe_dump({
        'controller_server': {
            'ros__parameters': {
                'FollowPath': {
                    'CostCritic': {
                        'cost_weight': 2.75,
                        'critical_cost': 300.0,
                    }
                }
            }
        }
    }), encoding='utf-8')
    result = subprocess.run([
        sys.executable,
        str(RUNTIME_DUMP),
        '--extract-cost-weight', str(runtime_yaml),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data['controller_server']['FollowPath']['CostCritic']['cost_weight'] == 2.75


def test_phase26a1_runtime_dump_extractor_accepts_ros2_slash_node_key(tmp_path):
    runtime_yaml = tmp_path / 'controller_server.yaml'
    runtime_yaml.write_text(yaml.safe_dump({
        '/controller_server': {
            'ros__parameters': {
                'FollowPath': {
                    'CostCritic': {
                        'cost_weight': 2.75,
                    }
                }
            }
        }
    }), encoding='utf-8')
    result = subprocess.run([
        sys.executable,
        str(RUNTIME_DUMP),
        '--extract-cost-weight', str(runtime_yaml),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data['controller_server']['FollowPath']['CostCritic']['cost_weight'] == 2.75


def test_phase26a1_runtime_dump_extractor_rejects_empty_or_wrong_yaml(tmp_path):
    runtime_yaml = tmp_path / 'controller_server.yaml'
    runtime_yaml.write_text('controller_server: {}\n', encoding='utf-8')
    result = subprocess.run([
        sys.executable,
        str(RUNTIME_DUMP),
        '--extract-cost-weight', str(runtime_yaml),
    ], text=True, capture_output=True, check=False)
    assert result.returncode != 0
    assert 'FollowPath.CostCritic.cost_weight' in result.stderr


def test_phase26a1_wrapper_uses_retrying_runtime_dump_helper_and_candidate_smoke_id():
    wrapper = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26a_candidate_fingerprint_smoke' in wrapper
    assert 'PHASE26A_CANDIDATE_FINGERPRINT_PROFILE=true' in wrapper
    assert 'SELECTED_PROFILE="candidate_costcritic_275"' in wrapper
    assert 'tools/dump_controller_runtime_params.py' in wrapper
    assert '--node /controller_server' in wrapper
    assert '--output "$RUNTIME_PARAMS_DIR/controller_server.yaml"' in wrapper
    assert '--summary-json "$RUNTIME_PARAMS_DIR/controller_server_summary.json"' in wrapper
    assert '--expected-cost-weight "$RUNTIME_EXPECTED_COST_WEIGHT"' in wrapper
    assert 'RUNTIME_EXPECTED_COST_WEIGHT="3.81"' in wrapper
    assert 'RUNTIME_EXPECTED_COST_WEIGHT="2.5"' in wrapper
    assert 'RUNTIME_EXPECTED_COST_WEIGHT="3.0"' in wrapper
    assert 'RUNTIME_EXPECTED_COST_WEIGHT="2.75"' in wrapper
    assert 'RUNTIME_DUMP_PID' in wrapper


def test_phase26a1_candidate_params_still_costcritic_275():
    data = yaml.safe_load(NAV_CANDIDATE.read_text(encoding='utf-8'))
    costcritic = data['controller_server']['ros__parameters']['FollowPath']['CostCritic']
    assert costcritic['cost_weight'] == 2.75
