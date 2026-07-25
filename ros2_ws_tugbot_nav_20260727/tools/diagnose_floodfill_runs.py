#!/usr/bin/env python3
"""Failure-mode classifier for flood-fill Gazebo runs.

Parses the artifacts under log/flood_fill_run_*/ and produces a failure-mode
distribution so decisions rest on the run distribution, not single-run anecdotes.

Observable signals only (Gazebo runs do NOT log physical collisions; collision
rate comes from the offline oracle, not from these logs):
  - outcome:        result.txt / run_meta.txt  (EXIT_REACHED / TIMEOUT / LAUNCH_DIED / empty)
  - best_dist:      min dist_to_exit over all DIAG lines  (how close it ever got)
  - final cell:     last DIAG dcell + phase
  - desync:         max |dcell - odomcell| (manhattan) + RECONCILE count
  - sense mismatch: fraction of SENSE lines with good=False
  - thrash:         STALL / UNSTICK / ESCAPE line counts
  - duration:       last DIAG sim-time - first DIAG sim-time (seconds)

Usage:
  python3 tools/diagnose_floodfill_runs.py [GLOB ...]
  (default GLOB: log/flood_fill_run_*)
"""
from __future__ import annotations

import glob
import os
import re
import sys
from dataclasses import dataclass, field

# Older logs print pose=(x, y) (2-tuple); yaw was added later -> pose=(x, y, yaw).
# Make the yaw component optional so both eras parse.
DIAG_RE = re.compile(
    r"DIAG pose=\([-\d.]+, [-\d.]+(?:, [-\d.eE]+)?\) "
    r"dcell=\((\d+), (\d+)\) odomcell=\((\d+), (\d+)\) "
    r"dist_to_exit=([-\d.]+) phase=(\S+)"
)
TS_RE = re.compile(r"\[(\d{10}\.\d+)\]")

# Distance-to-exit buckets (metres). NEAR ~= within ~2 cells of the exit.
NEAR_M = 4.0
MID_M = 12.0
# Signal thresholds for "heavy" flags.
DESYNC_CELLS = 2          # max |dcell-odomcell| manhattan >= this == localization broke
SENSE_BAD_FRAC = 0.20     # >= 20% good=False SENSE reads == sensing unreliable


@dataclass
class RunStats:
    name: str
    outcome: str = "EMPTY"
    n_diag: int = 0
    best_dist: float = float("inf")
    best_cell: tuple[int, int] | None = None
    final_cell: tuple[int, int] | None = None
    final_phase: str = "?"
    max_desync: int = 0
    desync_frac: float = 0.0
    sense_total: int = 0
    sense_bad: int = 0
    stall: int = 0
    unstick: int = 0
    escape: int = 0
    reconcile: int = 0
    duration_s: float = 0.0
    label: str = "?"

    @property
    def sense_bad_frac(self) -> float:
        return self.sense_bad / self.sense_total if self.sense_total else 0.0


def _read(path: str) -> str:
    try:
        with open(path, "r", errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""


def parse_run(run_dir: str) -> RunStats:
    name = os.path.basename(run_dir.rstrip("/"))
    st = RunStats(name=name)

    # Outcome: prefer result.txt, fall back to run_meta.txt "result=...".
    outcome = _read(os.path.join(run_dir, "result.txt")).strip()
    if not outcome:
        m = re.search(r"result=(\S+)", _read(os.path.join(run_dir, "run_meta.txt")))
        outcome = m.group(1) if m else "EMPTY"
    st.outcome = outcome or "EMPTY"

    log = _read(os.path.join(run_dir, "launch.log"))
    if not log:  # some runs only kept the tail
        log = _read(os.path.join(run_dir, "flood_fill_tail.txt"))

    first_ts = last_ts = None
    desync_lines = 0
    for line in log.splitlines():
        m = DIAG_RE.search(line)
        if m:
            st.n_diag += 1
            dx, dy = int(m.group(1)), int(m.group(2))
            ox, oy = int(m.group(3)), int(m.group(4))
            dist = float(m.group(5))
            phase = m.group(6)
            if dist < st.best_dist:
                st.best_dist = dist
                st.best_cell = (dx, dy)
            st.final_cell = (dx, dy)
            st.final_phase = phase
            desync = abs(dx - ox) + abs(dy - oy)
            st.max_desync = max(st.max_desync, desync)
            if desync > 0:
                desync_lines += 1
            ts = TS_RE.search(line)
            if ts:
                t = float(ts.group(1))
                first_ts = t if first_ts is None else first_ts
                last_ts = t
        elif "SENSE" in line and "good=" in line:
            st.sense_total += 1
            if "good=False" in line:
                st.sense_bad += 1
        # event keyword line counts (relative thrash signals)
        if "STALL" in line:
            st.stall += 1
        if "UNSTICK" in line:
            st.unstick += 1
        if "ESCAPE" in line:
            st.escape += 1
        if "RECONCILE" in line:
            st.reconcile += 1
        if "EXIT_REACHED" in line and st.outcome in ("EMPTY", "TIMEOUT"):
            st.outcome = "EXIT_REACHED"

    if st.n_diag:
        st.desync_frac = desync_lines / st.n_diag
    if first_ts is not None and last_ts is not None:
        st.duration_s = last_ts - first_ts
    if st.best_dist == float("inf"):
        st.best_dist = float("nan")

    st.label = classify(st)
    return st


def classify(st: RunStats) -> str:
    if st.outcome == "EXIT_REACHED":
        return "SUCCESS"
    if st.outcome in ("LAUNCH_DIED", "EMPTY"):
        return "INFRA_FAIL"
    # TIMEOUT (or anything else) — assign primary failure mode by priority.
    if st.best_dist == st.best_dist and st.best_dist <= NEAR_M:  # not NaN and near
        return "FINAL_APPROACH"      # reached the doorstep, oscillated, never finished
    if st.max_desync >= DESYNC_CELLS:
        return "DESYNC"              # planner cell vs odom cell diverged (localization broke)
    if st.sense_bad_frac >= SENSE_BAD_FRAC:
        return "SENSE_HEAVY"         # pervasive mis-sensing
    return "EARLY_STUCK"             # wedged/looped far from exit with clean-ish signals


def _hist(stats: list[RunStats]) -> None:
    from collections import Counter

    n = len(stats)
    print(f"\n{'='*72}\n  FLOOD-FILL RUN DIAGNOSTIC  —  {n} runs\n{'='*72}")

    print("\nOUTCOME distribution:")
    for k, c in Counter(s.outcome for s in stats).most_common():
        print(f"  {k:<14} {c:>3}  ({100*c/n:4.0f}%)")

    print("\nPRIMARY FAILURE-MODE distribution:")
    order = ["SUCCESS", "FINAL_APPROACH", "DESYNC", "SENSE_HEAVY", "EARLY_STUCK", "INFRA_FAIL"]
    lc = Counter(s.label for s in stats)
    for k in order:
        if lc.get(k):
            print(f"  {k:<14} {lc[k]:>3}  ({100*lc[k]/n:4.0f}%)")

    timeouts = [s for s in stats if s.outcome == "TIMEOUT"]
    if timeouts:
        near = sum(1 for s in timeouts if s.best_dist <= NEAR_M)
        mid = sum(1 for s in timeouts if NEAR_M < s.best_dist <= MID_M)
        far = sum(1 for s in timeouts if s.best_dist > MID_M)
        print(f"\nHOW-CLOSE (best dist_to_exit, {len(timeouts)} TIMEOUT runs):")
        print(f"  NEAR (<= {NEAR_M:g}m)   {near:>3}")
        print(f"  MID  ({NEAR_M:g}-{MID_M:g}m) {mid:>3}")
        print(f"  FAR  (> {MID_M:g}m)    {far:>3}")
        parsed = [s for s in timeouts if s.best_dist == s.best_dist]  # drop NaN
        if parsed:
            best = min(parsed, key=lambda s: s.best_dist)
            print(f"  best-ever: {best.best_dist:.2f}m at cell {best.best_cell}  ({best.name})")

    heavy_desync = sum(1 for s in stats if s.max_desync >= DESYNC_CELLS)
    heavy_sense = sum(1 for s in stats if s.sense_bad_frac >= SENSE_BAD_FRAC)
    any_reconcile = sum(1 for s in stats if s.reconcile > 0)
    print("\nSIGNAL PREVALENCE (independent flags, runs may overlap):")
    print(f"  heavy desync (max>= {DESYNC_CELLS} cells)  {heavy_desync:>3}  ({100*heavy_desync/n:4.0f}%)")
    print(f"  heavy mis-sense (>= {int(100*SENSE_BAD_FRAC)}% bad)   {heavy_sense:>3}  ({100*heavy_sense/n:4.0f}%)")
    print(f"  any RECONCILE event           {any_reconcile:>3}  ({100*any_reconcile/n:4.0f}%)")


def _table(stats: list[RunStats]) -> None:
    print(f"\n{'run':<28}{'label':<15}{'out':<12}{'best':>7}{'cell':>9}"
          f"{'desyn':>6}{'bad%':>6}{'st/un/es':>10}{'dur_s':>8}")
    for s in sorted(stats, key=lambda r: (r.label, r.best_dist)):
        cell = f"{s.best_cell}" if s.best_cell else "-"
        print(f"{s.name:<28}{s.label:<15}{s.outcome:<12}{s.best_dist:>7.2f}{cell:>9}"
              f"{s.max_desync:>6}{100*s.sense_bad_frac:>5.0f}%"
              f"{s.stall:>4}/{s.unstick}/{s.escape}{s.duration_s:>8.0f}")


def main(argv: list[str]) -> int:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    patterns = argv[1:] or [os.path.join(here, "log", "flood_fill_run_*")]
    run_dirs = sorted(d for p in patterns for d in glob.glob(p) if os.path.isdir(d))
    if not run_dirs:
        print(f"no run dirs match: {patterns}", file=sys.stderr)
        return 1
    stats = [parse_run(d) for d in run_dirs]
    _table(stats)
    _hist(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
