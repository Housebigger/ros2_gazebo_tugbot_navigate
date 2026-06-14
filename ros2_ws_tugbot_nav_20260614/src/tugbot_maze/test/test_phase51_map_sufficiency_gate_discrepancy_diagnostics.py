from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE50_ANALYZER = ROOT / 'tools' / 'analyze_phase50_dispatch_entry_readiness_gate_runtime.py'
PHASE51_ANALYZER = ROOT / 'tools' / 'analyze_phase51_map_sufficiency_gate_discrepancy.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase51_map_sufficiency_gate_discrepancy_diagnostics_report.md'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase51_gate_payload_records_map_sampling_window_and_counts():
    source = _read(MAZE_EXPLORER)
    for token in [
        'map_stamp',
        'grid_frame_id',
        'grid_width',
        'grid_height',
        'grid_resolution',
        'grid_origin',
        'robot_cell',
        'sample_window',
        'bbox_world',
        'in_bounds_count',
        'out_of_bounds_count',
        'known_count',
        'free_count',
        'occupied_count',
        'unknown_count',
        'out_of_bounds_as_unknown',
        'phase51_map_sufficiency_discrepancy_diagnostics',
    ]:
        assert token in source


def test_phase51_gate_keeps_thresholds_and_strategy_unchanged():
    source = _read(MAZE_EXPLORER)
    assert "declare_parameter('dispatch_readiness_min_map_known_ratio', 0.70)" in source
    assert "declare_parameter('dispatch_readiness_min_map_free_ratio', 0.50)" in source
    assert "declare_parameter('dispatch_readiness_near_robot_radius_m', 1.0)" in source
    assert "declare_parameter('clearance_radius_m', 0.35)" in source
    gate_body = source[source.index('def _dispatch_entry_readiness_gate'):source.index('def _nav2_lifecycle_sufficiency')]
    assert 'clearance_radius_m' not in gate_body


def test_phase50_runtime_recorder_records_same_inclusive_and_in_bounds_map_ratios():
    source = _read(PHASE50_ANALYZER)
    for token in [
        '_grid_near_robot_window',
        'inclusive_near_robot',
        'in_bounds_near_robot',
        'out_of_bounds_as_unknown',
        'sample_window',
        'out_of_bounds_count',
        'robot_cell',
        'map_stamp',
    ]:
        assert token in source


def test_phase51_analyzer_classifies_out_of_bounds_denominator_discrepancy(tmp_path):
    runtime = tmp_path / 'runtime.json'
    explorer = tmp_path / 'explorer_state.jsonl'
    output = tmp_path / 'phase51.json'
    runtime.write_text(json.dumps({
        'snapshots': [{
            'elapsed_sec': 150.0,
            'map': {
                'available': True,
                'near_robot': {
                    'known_ratio': 0.7189119170984456,
                    'free_ratio': 0.7189119170984456,
                    'total': 772,
                    'center_cell': [3, 21],
                },
                'inclusive_near_robot': {
                    'known_ratio': 0.4429369513168396,
                    'free_ratio': 0.4429369513168396,
                    'sample_count': 1253,
                    'in_bounds_count': 772,
                    'out_of_bounds_count': 481,
                    'unknown_count': 698,
                    'robot_cell': [3, 21],
                    'sample_window': {'min_x': -17, 'max_x': 23, 'min_y': 1, 'max_y': 41},
                },
            },
        }],
    }), encoding='utf-8')
    explorer.write_text(json.dumps({
        'elapsed_sec': 179.7,
        'state': {
            'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS',
            'dispatch_readiness_gate': {
                'passed': False,
                'blocking_reasons': ['map_sufficient'],
                'map': {
                    'known_ratio': 0.4429369513168396,
                    'free_ratio': 0.4429369513168396,
                    'sample_count': 1253,
                    'robot_cell': [3, 21],
                    'out_of_bounds_count': 481,
                    'in_bounds_count': 772,
                    'sample_window': {'min_x': -17, 'max_x': 23, 'min_y': 1, 'max_y': 41},
                },
            },
        },
    }) + '\n', encoding='utf-8')

    result = subprocess.run([
        sys.executable, str(PHASE51_ANALYZER),
        '--runtime-evidence', str(runtime),
        '--explorer-state', str(explorer),
        '--output', str(output),
    ], text=True, capture_output=True, check=True)
    assert 'RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH' in result.stdout
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['classification'] == 'RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH'
    assert data['gate_vs_runtime']['sample_count_match'] is True
    assert data['gate_vs_runtime']['gate_matches_runtime_inclusive'] is True
    assert data['gate_vs_runtime']['legacy_recorder_excluded_out_of_bounds'] is True
    assert data['next_step'] == 'keep_gate_waiting_logic; use inclusive diagnostics before considering threshold_or_rotation'


def test_phase51_report_exists_with_guardrails_and_classification_after_phase_completion():
    report = _read(REPORT)
    for token in [
        'Phase51: Map Sufficiency Gate Discrepancy Diagnostics',
        'RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH',
        'out_of_bounds_count',
        'sample_window',
        'No clearance strategy change',
        'No Nav2 / MPPI / controller parameter tuning',
        'No autonomous exploration success claim',
    ]:
        assert token in report
