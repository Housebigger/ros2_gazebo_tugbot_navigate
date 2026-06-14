from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase81_goal2_forward_open_machine_evidence.py'
RECORDER = ROOT / 'tools' / 'record_phase81_goal2_forward_open_machine_evidence.py'
RUNBOOK = ROOT / 'tools' / 'run_phase81_goal2_forward_open_machine_evidence_capture.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase81_goal2_forward_open_machine_evidence_capture_report.md'
OUTPUT_JSON = ROOT / 'log' / 'phase81_goal2_forward_open_machine_evidence_capture' / 'phase81_goal2_forward_open_machine_evidence_capture_analysis.json'
MIN_SUMMARY = ROOT / 'log' / 'phase81_goal2_forward_open_machine_evidence_capture' / 'phase81_goal2_forward_open_machine_evidence_capture_minimal_field_summary.md'


def load_analyzer():
    assert ANALYZER.exists(), f'missing Phase81 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase81_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_scan(front_range_m: float = 2.0, *, blocked_angle_deg: float | None = None, blocked_range_m: float = 0.25) -> dict:
    angle_min = -math.pi
    angle_increment = math.radians(1.0)
    ranges = [front_range_m for _ in range(361)]
    if blocked_angle_deg is not None:
        idx = int(round((math.radians(blocked_angle_deg) - angle_min) / angle_increment))
        ranges[idx] = blocked_range_m
    return {
        'topic': '/scan',
        'frame_id': 'scan_omni',
        'angle_min': angle_min,
        'angle_max': math.pi,
        'angle_increment': angle_increment,
        'range_min': 0.05,
        'range_max': 10.0,
        'ranges': ranges,
    }


def make_costmap(value: int = 0, *, high_front: bool = False, unknown: bool = False) -> dict:
    width = 80
    height = 80
    resolution = 0.05
    origin_x = -1.0
    origin_y = -2.0
    data = [value for _ in range(width * height)]
    if unknown:
        data = [-1 for _ in range(width * height)]
    if high_front:
        # Fill a rectangular corridor in front of pose (0, 0, yaw 0) with lethal cost.
        for y in range(height):
            cy = origin_y + (y + 0.5) * resolution
            for x in range(width):
                cx = origin_x + (x + 0.5) * resolution
                if 0.0 <= cx <= 1.0 and abs(cy) <= 0.30:
                    data[y * width + x] = 100
    return {
        'topic': '/local_costmap/costmap',
        'frame_id': 'odom',
        'info': {
            'resolution': resolution,
            'width': width,
            'height': height,
            'origin': {'position': {'x': origin_x, 'y': origin_y}, 'orientation_yaw': 0.0},
        },
        'data': data,
    }


def make_artifact(tmp_path: Path, *, scan: dict | None, costmap: dict | None, pose: list[float] | None = None) -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    artifact = {
        'run_id': 'synthetic_phase81',
        'goal_sequence': 2,
        'target': [0.0, 0.4],
        'terminal_pose': pose,
        'robot_pose_by_frame': {'odom': pose, 'map': pose} if pose else {},
        'scan': scan,
        'local_costmap': costmap,
        'footprint': {'frame_id': 'base_link', 'points': [[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]},
        'source_phase80_classification': 'NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR',
        'source_phase80_forward_open_status': 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT',
    }
    path = tmp_path / 'capture.json'
    path.write_text(json.dumps(artifact), encoding='utf-8')
    return path


def test_phase81_analyzer_confirms_forward_open_with_scan_and_low_local_cost(tmp_path):
    module = load_analyzer()
    path = make_artifact(tmp_path, scan=make_scan(2.5), costmap=make_costmap(0), pose=[0.0, 0.0, 0.0])

    result = module.analyze_capture(path)

    assert result['forward_open_classification'] == 'FORWARD_OPEN_CORRIDOR_CONFIRMED'
    assert result['front_corridor_scan']['min_clearance_m'] >= 1.0
    assert result['front_corridor_local_cost']['lethal_ratio'] == 0.0
    assert result['front_corridor_local_cost']['high_cost_ratio'] <= 0.05
    assert result['evidence_sufficient'] is True
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False


def test_phase81_analyzer_blocks_when_scan_or_costmap_obstructs_front_corridor(tmp_path):
    module = load_analyzer()
    scan_blocked = make_artifact(tmp_path / 'scan_blocked', scan=make_scan(2.5, blocked_angle_deg=0.0, blocked_range_m=0.22), costmap=make_costmap(0), pose=[0.0, 0.0, 0.0])
    cost_blocked = make_artifact(tmp_path / 'cost_blocked', scan=make_scan(2.5), costmap=make_costmap(0, high_front=True), pose=[0.0, 0.0, 0.0])

    scan_result = module.analyze_capture(scan_blocked)
    cost_result = module.analyze_capture(cost_blocked)

    assert scan_result['forward_open_classification'] == 'FORWARD_OPEN_CORRIDOR_BLOCKED'
    assert scan_result['front_corridor_scan']['min_clearance_m'] < 0.5
    assert cost_result['forward_open_classification'] == 'FORWARD_OPEN_CORRIDOR_BLOCKED'
    assert cost_result['front_corridor_local_cost']['lethal_ratio'] >= 0.50


def test_phase81_analyzer_keeps_insufficient_when_raw_evidence_or_pose_missing(tmp_path):
    module = load_analyzer()
    no_scan = make_artifact(tmp_path / 'no_scan', scan=None, costmap=make_costmap(0), pose=[0.0, 0.0, 0.0])
    no_costmap = make_artifact(tmp_path / 'no_costmap', scan=make_scan(2.5), costmap=None, pose=[0.0, 0.0, 0.0])
    no_pose = make_artifact(tmp_path / 'no_pose', scan=make_scan(2.5), costmap=make_costmap(0), pose=None)

    assert module.analyze_capture(no_scan)['forward_open_classification'] == 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'
    assert 'missing_raw_scan' in module.analyze_capture(no_scan)['evidence_gaps']
    assert module.analyze_capture(no_costmap)['forward_open_classification'] == 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'
    assert 'missing_raw_local_costmap' in module.analyze_capture(no_costmap)['evidence_gaps']
    assert module.analyze_capture(no_pose)['forward_open_classification'] == 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'
    assert 'missing_robot_pose_for_costmap_frame' in module.analyze_capture(no_pose)['evidence_gaps']


def test_phase81_static_outputs_and_report_contract():
    assert RECORDER.exists(), 'Phase81 live raw evidence recorder is missing'
    assert RUNBOOK.exists(), 'Phase81 bounded capture runbook is missing'
    assert OUTPUT_JSON.exists(), 'Phase81 analysis JSON artifact is missing'
    assert MIN_SUMMARY.exists(), 'Phase81 minimal field summary is missing'
    assert REPORT.exists(), 'Phase81 report is missing'

    data = json.loads(OUTPUT_JSON.read_text(encoding='utf-8'))
    assert data['forward_open_classification'] in {
        'FORWARD_OPEN_CORRIDOR_CONFIRMED',
        'FORWARD_OPEN_CORRIDOR_BLOCKED',
        'FORWARD_OPEN_EVIDENCE_INSUFFICIENT',
    }
    assert data['phase82_entered'] is False
    assert data['complete_autonomous_success_claimed'] is False
    assert data['exit_success_claimed'] is False
    assert data['raw_capture']['scan_available'] is True
    assert data['raw_capture']['local_costmap_available'] is True

    report = REPORT.read_text(encoding='utf-8')
    for token in [
        'Phase81',
        'FORWARD_OPEN_CORRIDOR_CONFIRMED',
        'FORWARD_OPEN_CORRIDOR_BLOCKED',
        'FORWARD_OPEN_EVIDENCE_INSUFFICIENT',
        'No maze_explorer strategy changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase82 not entered',
    ]:
        assert token in report
