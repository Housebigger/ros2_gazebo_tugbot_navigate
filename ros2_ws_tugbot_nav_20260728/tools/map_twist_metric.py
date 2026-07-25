#!/usr/bin/env python3
"""Center-region wall-twist metric (center-twist-root-cause).

Loads one or two dumped voxel sets (.npy, N x 3, map frame) and, for every
ground-truth wall segment intersecting the configured center box, reports the
wall-band statistics: voxel count, perpendicular spread (p50/p95 of |distance
from the segment line|), and the fitted principal-axis angle vs the segment
axis (degrees). With two files (production + gt) it prints a side-by-side
table -- the phase's improvement metric.

Wall segments come from tugbot_maze.maze_sim.load_segments() (map frame).
Usage:
  python3 tools/map_twist_metric.py PROD.npy [GT.npy] \
      [--center 10.54 9.04] [--half 3.2]
"""
import argparse
import math
import sys

import numpy as np

Z_BAND = (0.15, 1.5)          # wall band: excludes floor and lidar-mast returns
PERP_BAND_M = 0.35            # voxels within this distance of the segment line
END_TRIM_M = 0.15             # trim segment ends to avoid corner cross-talk


def wall_points(pts, seg, ground_z=0.0):
    """Voxels in the wall band of segment (x0,y0,x1,y1): perpendicular
    distance <= PERP_BAND_M, longitudinally inside the (end-trimmed) segment,
    z inside Z_BAND RELATIVE to ground_z. Datum-invariance contract (Task 1
    quality review I2): the gt and production maps may sit on different z
    datums (spawn-anchored odom vs world z), so the band is selected per
    cloud relative to its OWN ground level -- never absolute z."""
    x0, y0, x1, y1 = seg
    p = pts[(pts[:, 2] >= ground_z + Z_BAND[0])
            & (pts[:, 2] <= ground_z + Z_BAND[1])]
    d = np.array([x1 - x0, y1 - y0])
    L = float(np.hypot(*d))
    if L < 1e-9 or len(p) == 0:
        return p[:0]
    u = d / L
    rel = p[:, :2] - np.array([x0, y0])
    lon = rel @ u
    perp = rel @ np.array([-u[1], u[0]])
    keep = ((np.abs(perp) <= PERP_BAND_M)
            & (lon >= END_TRIM_M) & (lon <= L - END_TRIM_M))
    return p[keep]


def segment_band_stats(pts, seg, ground_z=0.0):
    """count / spread_p50 / spread_p95 (perpendicular, median-centered) and
    fitted principal-axis angle vs the segment axis (deg).

    Interpretation contract (Task 2 quality review, measured on synthetics):
    - spread_p95 DELTA (production minus gt, per segment) is the PRIMARY
      discriminator; absolute levels carry the common 0.24 m wall-thickness
      baseline (both faces in-band floor a clean interior wall at ~0.10).
    - angle is CORROBORATING-ONLY: parallel faces and the clean+twisted union
      attenuate the fitted angle ~2x (a 0.09 m fan reads ~-1.6 deg).
    - p50 is descriptive, NON-monotone in twist (fan fill can lower it).
    - sub-voxel twists read clean: a POSITIVE detection is trustworthy, a
      null is NOT proof of no twist."""
    sel = wall_points(pts, seg, ground_z)
    x0, y0, x1, y1 = seg
    d = np.array([x1 - x0, y1 - y0])
    u = d / np.hypot(*d)
    out = {'count': int(len(sel)), 'spread_p50': float('nan'),
           'spread_p95': float('nan'), 'angle_deg': float('nan')}
    if len(sel) < 30:
        return out
    perp = (sel[:, :2] - np.array([x0, y0])) @ np.array([-u[1], u[0]])
    centered = np.abs(perp - np.median(perp))
    out['spread_p50'] = float(np.percentile(centered, 50))
    out['spread_p95'] = float(np.percentile(centered, 95))
    xy = sel[:, :2] - sel[:, :2].mean(axis=0)
    cov = np.cov(xy.T)
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, int(np.argmax(evals))]
    ang = math.degrees(math.atan2(axis[1], axis[0])
                       - math.atan2(u[1], u[0]))
    while ang > 90.0:
        ang -= 180.0
    while ang < -90.0:
        ang += 180.0
    out['angle_deg'] = float(ang)
    return out


def center_segments(center, half):
    from tugbot_maze.maze_sim import load_segments
    cx, cy = center
    segs = []
    for seg in load_segments():
        x0, y0, x1, y1 = seg
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        if abs(mx - cx) <= half and abs(my - cy) <= half:
            segs.append(tuple(seg))
    return segs


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+', help='1-2 .npy voxel dumps (prod [gt])')
    ap.add_argument('--center', nargs=2, type=float, default=[10.54, 9.04])
    ap.add_argument('--half', type=float, default=3.2)
    args = ap.parse_args(argv)
    clouds = [np.load(f) for f in args.files]
    # Per-cloud ground datum (1st percentile of z): the two maps may sit on
    # different z datums (Task 1 quality review I2) -- bands are selected
    # relative to each cloud's own ground, and the datum delta is reported.
    grounds = [float(np.percentile(c[:, 2], 1.0)) for c in clouds]
    for f, g in zip(args.files, grounds):
        print('DATUM %s ground_z=%.3f' % (f, g))
        if not (-1.0 <= g <= 0.5):
            print('DATUM IMPLAUSIBLE %s: ground_z outside [-1.0, 0.5] -- '
                  'floor coverage sparse? Bands may be mis-selected.' % f)
    if len(grounds) == 2 and abs(grounds[0] - grounds[1]) > 0.1:
        print('DATUM WARNING: z datums differ by %.3f m (expected if odom is '
              'spawn-anchored; bands are per-cloud relative, comparison valid)'
              % abs(grounds[0] - grounds[1]))
    segs = center_segments(args.center, args.half)
    print('TWISTMETRIC center=(%.2f, %.2f) half=%.1f segments=%d'
          % (args.center[0], args.center[1], args.half, len(segs)))
    hdr = 'segment'.ljust(34) + ''.join(
        ('  [%d] n/p50/p95/ang' % i).ljust(28) for i in range(len(clouds)))
    print(hdr)
    agg = [[] for _ in clouds]
    deltas = []
    for seg in segs:
        row = ('(%.2f,%.2f)-(%.2f,%.2f)' % seg).ljust(34)
        stats = []
        for i, c in enumerate(clouds):
            s = segment_band_stats(c, seg, grounds[i])
            stats.append(s)
            row += ('  %5d/%.3f/%.3f/%+.2f' % (
                s['count'], s['spread_p50'], s['spread_p95'],
                s['angle_deg'])).ljust(28)
            if s['count'] >= 30:
                agg[i].append(s)
        print(row)
        # Paired per-segment delta (file0 - file1, i.e. production - gt): THE
        # discriminating statistic (Task 2 quality review I1). Absolute p95
        # levels carry the common wall-thickness baseline; the delta cancels
        # it. Only segments where BOTH clouds have count >= 30 pair up.
        if len(stats) == 2 and all(s['count'] >= 30 for s in stats):
            deltas.append((stats[0]['spread_p95'] - stats[1]['spread_p95'],
                           abs(stats[0]['angle_deg']) - abs(stats[1]['angle_deg'])))
    for i, rows in enumerate(agg):
        if rows:
            print('AGG[%d] bands=%d spread_p95_mean=%.4f |angle|_mean=%.3f'
                  % (i, len(rows),
                     float(np.mean([r['spread_p95'] for r in rows])),
                     float(np.mean([abs(r['angle_deg']) for r in rows]))))
    if deltas:
        dp = np.array([d[0] for d in deltas])
        da = np.array([d[1] for d in deltas])
        print('AGG_DELTA bands=%d dp95_mean=%.4f dp95_p95=%.4f dangle_mean=%.3f'
              % (len(deltas), float(dp.mean()),
                 float(np.percentile(dp, 95)), float(da.mean())))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
