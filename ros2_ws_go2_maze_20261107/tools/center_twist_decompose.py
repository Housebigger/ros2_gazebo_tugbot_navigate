#!/usr/bin/env python3
"""POSEDIAG residual bucketing for the center-twist forensics.

Parses POSEDIAG lines from one or more launch.logs and reports, per run and
per region bucket (center box vs rest):
  yaw_res_solver = solver_yaw - gt_yaw    (the residual that paints fan twist)
  yaw_res_odom   = odom_yaw   - gt_yaw    (sanity: ~0 on the drift-free rig)
  pos_res_solver = |solver_xy - gt_map_xy|
plus the top-10 worst |yaw_res_solver| windows with timestamps and positions.
Because odom is drift-free, (solver - odom) == ICP-injected error; the
decomposition prints that too (yaw_res_solver - yaw_res_odom).

Usage: python3 tools/center_twist_decompose.py LOG [LOG...] \
           [--center 10.54 9.04] [--half 3.2]
"""
import argparse
import math
import re
import sys

import numpy as np

WORLD_TO_MAP_X = 11.011281
WORLD_TO_MAP_Y = 9.02507

_RE = re.compile(
    r'\[(\d+\.\d+)\].*POSEDIAG '
    r'gt=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) '
    r'odom=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) '
    r'solver=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)')


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_posediag_line(line):
    m = _RE.search(line)
    if not m:
        return None
    v = [float(g) for g in m.groups()]
    t_s = v[0]
    gt = (v[1] + WORLD_TO_MAP_X, v[2] + WORLD_TO_MAP_Y, v[3])
    odom = (v[4], v[5], v[6])
    sol = (v[7], v[8], v[9])
    return {
        't_s': t_s,
        'gt_map': gt[:2], 'solver_map': sol[:2],
        'yaw_res_solver': _wrap(sol[2] - gt[2]),
        'yaw_res_odom': _wrap(odom[2] - gt[2]),
        'pos_res_solver': math.hypot(sol[0] - gt[0], sol[1] - gt[1]),
    }


def _stats(rows):
    if not rows:
        return {'n': 0}
    yaw = np.abs([r['yaw_res_solver'] for r in rows])
    yod = np.abs([r['yaw_res_odom'] for r in rows])
    pos = np.array([r['pos_res_solver'] for r in rows])
    return {'n': len(rows),
            'yaw_p50': float(np.percentile(yaw, 50)),
            'yaw_p95': float(np.percentile(yaw, 95)),
            'yaw_max': float(yaw.max()),
            'yaw_odom_p95': float(np.percentile(yod, 95)),
            'pos_p50': float(np.percentile(pos, 50)),
            'pos_p95': float(np.percentile(pos, 95))}


def bucket_stats(rows, center, half):
    cx, cy = center
    inc, out = [], []
    for r in rows:
        x, y = r['gt_map']
        (inc if (abs(x - cx) <= half and abs(y - cy) <= half) else out).append(r)
    return {'center': _stats(inc), 'rest': _stats(out)}


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--center', nargs=2, type=float, default=[10.54, 9.04])
    ap.add_argument('--half', type=float, default=3.2)
    args = ap.parse_args(argv)
    for path in args.logs:
        rows = []
        with open(path, errors='replace') as f:
            for line in f:
                r = parse_posediag_line(line)
                if r:
                    rows.append(r)
        print('==== %s: %d POSEDIAG rows ====' % (path, len(rows)))
        if not rows:
            print('  (was the run launched with pose_diag:=true?)')
            continue
        b = bucket_stats(rows, tuple(args.center), args.half)
        for name in ('center', 'rest'):
            s = b[name]
            if s['n'] == 0:
                print('  %-6s n=0' % name)
                continue
            print('  %-6s n=%4d yaw_p50=%.4f yaw_p95=%.4f yaw_max=%.4f '
                  'yaw_odom_p95=%.4f pos_p50=%.3f pos_p95=%.3f'
                  % (name, s['n'], s['yaw_p50'], s['yaw_p95'], s['yaw_max'],
                     s['yaw_odom_p95'], s['pos_p50'], s['pos_p95']))
        worst = sorted(rows, key=lambda r: -abs(r['yaw_res_solver']))[:10]
        print('  worst |yaw_res_solver| windows:')
        for r in worst:
            print('    t=%.1f pos=(%.2f, %.2f) yaw_res=%+.4f pos_res=%.3f'
                  % (r['t_s'], r['gt_map'][0], r['gt_map'][1],
                     r['yaw_res_solver'], r['pos_res_solver']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
