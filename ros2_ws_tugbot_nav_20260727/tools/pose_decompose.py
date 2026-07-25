#!/usr/bin/env python3
"""CLI: decompose Task A POSEDIAG lines in a run's launch.log into per-segment / per-axis error.

Usage: python3 tools/pose_decompose.py <artifact_dir> [<artifact_dir> ...]

POSEDIAG carries gt in the maze WORLD frame but odom/solver in the entrance-anchored MAP frame.
We first derive the fixed world->map offset from the data (odom - gt on the FIRST row), sanity-check
it against the LAST row (odom is world-anchored ground truth, so the offset should be near-constant
-- if it drifts, that is a finding: /odom itself drifts), then move gt into the map frame.

Reports, per run: how many POSEDIAG lines; the frame-offset consistency (first vs last row); and
three same-frame segment decompositions -- (gt - odom) (does /odom drift after the fixed offset is
removed? expect ~0), (odom - solver) (ICP quality), and (gt - solver) (total belief error) -- each
split into along / lateral / yaw with p10/p50/p90 and along-sign consistency."""
import os
import sys
from tugbot_maze.pose_decompose import (align_gt_to_map, decompose_error,
                                        offset_consistency, parse_posediag_line)

# offset should be near-constant since odom is world-anchored ground truth up to a fixed transform.
_DPOS_TOL = 0.2   # m
_DYAW_TOL = 0.05  # rad


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

    # Frame-offset consistency check (first vs last row) BEFORE reconciling.
    oc = offset_consistency(rows)
    fx, fy, fw = oc["off_first"]
    lx, ly, lw = oc["off_last"]
    flag = "" if (oc["dpos"] <= _DPOS_TOL and oc["dyaw"] <= _DYAW_TOL) \
        else "  <-- ODOM DRIFTS (offset not constant!)"
    print("  frame off (odom-gt) first=(%+.3f, %+.3f, %+.3f) last=(%+.3f, %+.3f, %+.3f)  "
          "dpos=%.3fm dyaw=%.3frad%s" % (fx, fy, fw, lx, ly, lw, oc["dpos"], oc["dyaw"], flag))

    # Reconcile gt (world) -> gt_map (entrance-anchored map frame), then decompose same-frame.
    arows = align_gt_to_map(rows)
    for seg_name, a, b in (("gt-odom", "gt", "odom"), ("odom-solver", "odom", "solver"),
                           ("gt-solver", "gt", "solver")):
        ds = [decompose_error(r[a], r[b]) for r in arows]
        al = [d["along"] for d in ds]
        la = [abs(d["lateral"]) for d in ds]
        yw = [abs(d["yaw"]) for d in ds]
        pos = sum(1 for v in al if v > 0)
        print("  %-11s along p10/p50/p90 = %+.2f/%+.2f/%+.2f  |lat| p10/p50/p90 = %.2f/%.2f/%.2f  "
              "|yaw| p10/p50/p90 = %.2f/%.2f/%.2f  along>0: %d/%d"
              % (seg_name, _pctl(al, 0.1), _pctl(al, 0.5), _pctl(al, 0.9),
                 _pctl(la, 0.1), _pctl(la, 0.5), _pctl(la, 0.9),
                 _pctl(yw, 0.1), _pctl(yw, 0.5), _pctl(yw, 0.9), pos, len(al)))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
