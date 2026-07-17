#!/usr/bin/env python3
"""Offline true-footprint collision-oracle replay over Gazebo DIAG samples.

Replays the (x,y,yaw) DIAG poses logged by flood_fill_solver through the true asymmetric
footprint oracle (maze_sim.collides) and reports the collision rate plus the geometry of
every grazing sample. Gazebo does NOT log physical collisions; this is how the collision
rate is measured. Usage:

    python3 tools/replay_collision_oracle.py <run_dir_or_batch_dir> [more ...]

A batch dir is one containing manifest.txt (one run dir per line); a run dir is one
containing launch.log. Reproduces the documented (3,9) 0.33% baseline on the clean batches.

Verified baseline: `replay_collision_oracle.py log/batch_diag_20260628_232253
log/batch_diag_20260629_071328` -> runs=16 samples=1808 collide=6 rate=0.332%, by cell {(3,9):6}.
"""
import argparse
import math
import os
import re
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "tugbot_maze"))
from tugbot_maze.maze_sim import MazeSim, load_segments          # noqa: E402
from tugbot_maze.flood_fill_brain import pose_to_cell, cell_center  # noqa: E402

DIAG_RE = re.compile(
    r"DIAG pose=\(([-\d.eE+]+), ([-\d.eE+]+), ([-\d.eE+]+)\) "
    r"dcell=\((\d+), (\d+)\) odomcell=\((\d+), (\d+)\) "
    r"dist_to_exit=([-\d.]+) phase=(\S+)"
)


def run_logs_from(path):
    """Expand a batch dir (has manifest.txt) or a run dir (has launch.log) to launch.log paths."""
    mani = os.path.join(path, "manifest.txt")
    if os.path.exists(mani):
        out = []
        with open(mani) as f:
            for line in f:
                d = line.strip()
                if d and os.path.exists(os.path.join(d, "launch.log")):
                    out.append(os.path.join(d, "launch.log"))
        return out
    lg = os.path.join(path, "launch.log")
    return [lg] if os.path.exists(lg) else []


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="+", help="run dirs and/or batch dirs")
    args = ap.parse_args(argv)

    sim = MazeSim(load_segments(), (0.0, 0.0), 0.0)
    logs = []
    for p in args.paths:
        logs.extend(run_logs_from(p))
    if not logs:
        print("no launch.log found under the given paths", file=sys.stderr)
        return 2

    total = coll = 0
    grazes = []
    for lg in logs:
        with open(lg, errors="ignore") as f:
            for line in f:
                m = DIAG_RE.search(line)
                if not m:
                    continue
                x, y, yaw = float(m.group(1)), float(m.group(2)), float(m.group(3))
                phase = m.group(9)
                total += 1
                if sim.collides(x, y, yaw):
                    coll += 1
                    cell = pose_to_cell(x, y)
                    cx, cy = cell_center(cell)
                    grazes.append((cell, x, y, math.degrees(yaw), phase, x - cx, y - cy))

    if total == 0:
        print("warning: 0 DIAG samples matched under the given paths -- rate is UNDEFINED, "
              "not a clean run (check the log path / DIAG format)", file=sys.stderr)
    rate = (100.0 * coll / total) if total else 0.0
    print(f"runs={len(logs)} samples={total} collide={coll} rate={rate:.3f}%")
    for cell, x, y, yd, phase, offx, offy in sorted(grazes):
        print(f"  cell={cell} pose=({x:.2f},{y:.2f}) yaw={yd:+.0f}deg "
              f"phase={phase} off=(x{offx:+.2f},y{offy:+.2f})")
    print("by cell :", dict(Counter(g[0] for g in grazes)))
    print("by phase:", dict(Counter(g[4] for g in grazes)))
    return 0 if total > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
