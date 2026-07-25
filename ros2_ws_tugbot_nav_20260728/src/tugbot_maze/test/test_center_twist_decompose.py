"""Unit tests for the POSEDIAG bucketing analysis: parsing, world->map
compensation, residual math, and center-box bucketing."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from center_twist_decompose import parse_posediag_line, bucket_stats  # noqa: E402


LINE = ('[flood_fill_solver-18] [INFO] [1785000000.5] [flood_fill_solver]: '
        'POSEDIAG gt=(-0.471, -0.025, 0.010) odom=(10.540, 9.000, 0.000) '
        'solver=(10.640, 9.000, 0.030)')


def test_parse_extracts_three_triples_and_residuals():
    row = parse_posediag_line(LINE)
    assert row is not None
    # gt WORLD -> MAP: x -0.471+11.011281=10.540281, y -0.025+9.02507=9.00007
    assert row['gt_map'][0] == pytest.approx(10.540281, abs=1e-6)
    assert row['gt_map'][1] == pytest.approx(9.00007, abs=1e-6)
    assert row['yaw_res_solver'] == pytest.approx(0.020, abs=1e-9)   # 0.030-0.010
    assert row['yaw_res_odom'] == pytest.approx(-0.010, abs=1e-9)    # 0.000-0.010
    assert row['pos_res_solver'] == pytest.approx(0.0997, abs=1e-3)
    assert row['t_s'] == pytest.approx(1785000000.5)


def test_parse_rejects_non_posediag_lines():
    assert parse_posediag_line('DIAG pose=(1,2,3)') is None


def test_bucket_stats_splits_center_vs_rest():
    rows = []
    for i in range(10):
        rows.append({'t_s': float(i), 'gt_map': (10.5, 9.0),
                     'solver_map': (10.5, 9.0),
                     'yaw_res_solver': 0.03, 'yaw_res_odom': 0.0,
                     'pos_res_solver': 0.05})
    for i in range(10, 20):
        rows.append({'t_s': float(i), 'gt_map': (2.0, 2.0),
                     'solver_map': (2.0, 2.0),
                     'yaw_res_solver': 0.001, 'yaw_res_odom': 0.0,
                     'pos_res_solver': 0.01})
    b = bucket_stats(rows, center=(10.54, 9.04), half=3.2)
    assert b['center']['n'] == 10 and b['rest']['n'] == 10
    assert b['center']['yaw_p95'] > b['rest']['yaw_p95'] * 5
