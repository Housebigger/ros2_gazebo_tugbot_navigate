#!/usr/bin/env python3
"""CLI: decompose Task A POSEDIAG lines in a run's launch.log into per-segment / per-axis error.

Usage: python3 tools/pose_decompose.py <artifact_dir> [<artifact_dir> ...]

Reports, per run: how many POSEDIAG lines; and three segment decompositions -- (gt - odom)
(does /odom itself drift?), (odom - solver) (ICP quality), and (gt - solver) (total error) --
each split into along / lateral / yaw with p10/p50/p90 and along-sign consistency. The verdict
predicts (gt - odom) along ~ 0 and (odom - solver) along = +2..+6 m."""
import os
import sys
from tugbot_maze.pose_decompose import parse_posediag_line, decompose_error


def _pctl(xs, q):
    if not xs:
        return float("nan")
    ys = sorted(xs)
    return ys[min(len(ys) - 1, int(q * len(ys)))]


def analyze(path):
    rows = []
    try:
        with open(os.path.join(path, "launch.log"), errors="replace") as f:
            for line in f:
                r = parse_posediag_line(line)
                if r:
                    rows.append(r)
    except FileNotFoundError:
        print("==== %s: no launch.log ====" % path)
        return
    print("==== %s: %d POSEDIAG lines ====" % (path, len(rows)))
    if not rows:
        print("  (no POSEDIAG -- was the run launched with pose_diag:=true?)")
        return
    for seg_name, a, b in (("gt-odom", "gt", "odom"), ("odom-solver", "odom", "solver"),
                           ("gt-solver", "gt", "solver")):
        ds = [decompose_error(r[a], r[b]) for r in rows]
        al = [d["along"] for d in ds]
        la = [abs(d["lateral"]) for d in ds]
        yw = [abs(d["yaw"]) for d in ds]
        pos = sum(1 for v in al if v > 0)
        print("  %-11s along p10/p50/p90 = %+.2f/%+.2f/%+.2f  |lat| p50=%.2f  |yaw| p50=%.2f  "
              "along>0: %d/%d" % (seg_name, _pctl(al, 0.1), _pctl(al, 0.5), _pctl(al, 0.9),
                                  _pctl(la, 0.5), _pctl(yw, 0.5), pos, len(al)))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
