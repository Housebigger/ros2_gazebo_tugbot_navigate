from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase38_initial_local_topology.py'
PHASE37 = ROOT / 'log' / 'phase37_scaled_clean_world_maze_explorer_bounded_smoke'
OUT = ROOT / 'log' / 'phase38_initial_local_topology' / 'phase38_initial_topology_analysis.json'


def test_phase38_analyzer_script_exists_and_is_read_only_contract():
    assert ANALYZER.exists(), 'Phase38 analyzer must exist'
    source = ANALYZER.read_text(encoding='utf-8')
    assert 'PHASE38_ALLOWED_CONCLUSIONS' in source
    assert 'INSUFFICIENT_MAP_AT_START' in source
    assert 'START_POSE_OUTSIDE_MAZE_OR_TOO_CLOSE_TO_BOUNDARY' in source
    assert 'TOPOLOGY_SAMPLING_SCALE_MISMATCH' in source
    assert 'UNKNOWN_CELL_POLICY_TOO_STRICT' in source
    assert 'TF_OR_POSE_ALIGNMENT_ISSUE' in source
    assert 'INSUFFICIENT_EVIDENCE' in source
    forbidden_runtime_tokens = [
        'ros2 launch',
        'ros2 topic pub',
        'NavigateToPose',
        'send_goal',
        'pkill',
        'controller_server',
        'nav2_slam_params.yaml',
    ]
    for token in forbidden_runtime_tokens:
        assert token not in source


def test_phase38_analyzer_generates_expected_json_and_overlays_from_phase37_artifacts():
    assert PHASE37.exists(), 'Phase37 artifact dir is required for Phase38 offline diagnostics'
    OUT.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [
            sys.executable,
            str(ANALYZER),
            '--workspace-root',
            str(ROOT),
            '--phase37-artifacts',
            str(PHASE37),
            '--output-dir',
            str(OUT.parent),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    data = json.loads(result.stdout)
    assert data['phase'] == 'Phase38 Initial Local Topology Diagnostics on Scaled Clean World'
    assert data['source_artifacts']['phase37_dir'].endswith('phase37_scaled_clean_world_maze_explorer_bounded_smoke')
    assert data['phase37_failure_signature']['mode'] == 'FAILED_EXHAUSTED'
    assert data['phase37_failure_signature']['goal_count'] == 0
    assert data['phase37_failure_signature']['last_local_topology_kind'] == 'unknown'
    assert data['phase37_failure_signature']['last_open_direction_count'] == 0
    assert data['phase37_failure_signature']['last_candidate_count'] == 0
    assert data['active_truth']['entrance']['x_m'] == -11.011281
    assert data['active_truth']['exit']['radius_m'] == 1.2
    assert data['map_analysis']['sample_exists'] is True
    assert data['scan_analysis']['sample_exists'] is True
    assert data['topology_sampling']['attempted_from_artifact_map'] is True
    assert data['conclusion'] in data['allowed_conclusions']
    assert data['conclusion'] in {
        'INSUFFICIENT_MAP_AT_START',
        'START_POSE_OUTSIDE_MAZE_OR_TOO_CLOSE_TO_BOUNDARY',
        'TOPOLOGY_SAMPLING_SCALE_MISMATCH',
        'UNKNOWN_CELL_POLICY_TOO_STRICT',
        'TF_OR_POSE_ALIGNMENT_ISSUE',
        'INSUFFICIENT_EVIDENCE',
    }
    for name in [
        'initial_pose_map_overlay.png',
        'local_topology_sampling_overlay.png',
        'entrance_alignment_overlay.png',
        'phase38_initial_topology_analysis.json',
    ]:
        path = OUT.parent / name
        assert path.exists() and path.stat().st_size > 0


def test_phase38_analysis_preserves_phase37_non_success_semantics():
    data = json.loads(OUT.read_text(encoding='utf-8'))
    assert data['phase37_conclusion'] == 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH'
    assert data['autonomous_success_claimed'] is False
    assert data['dispatch_analysis']['dispatch_events'] == 0
    assert data['dispatch_analysis']['goal_events_samples'] == 0
