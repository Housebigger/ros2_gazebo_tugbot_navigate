#!/usr/bin/env python3
"""P3 forensics: cadence + timeline analysis of ESCAPE/UNSTICK events in a run artifact.

Usage: python3 tools/p3_forensics.py <artifact_dir> [<artifact_dir> ...]

Parses launch.log. Reports, per run:
  - every ESCAPE/UNSTICK event with its ROS stamp (printed relative to the first
    parsed stamp t0);
  - inter-escape intervals (the no-progress cadence: expect ~90s clusters);
  - per-window distinct-cell footprint and phase mix (from DIAG lines between escapes);
  - each UNSTICK's reopen count n (old format) and the recovery span until the next
    escape/exit (pins NO_PROGRESS_FAST_S: the fast window must cover one full
    reopen -> re-sense -> re-route cycle);
  - dexit (geometric cell distance to EXIT_CELL=(10,9)) of every event cell, to sanity-check
    the exit-directed selection metric against where the run actually got locked.

Regex notes (verified against the real 20260719 artifacts):
  - solver DIAG lines carry the discrete cell as `dcell=(x, y)` (NOT `cell=`; a bare
    `cell=` regex would also latch onto `odomcell=`), so CELL pins `dcell=`;
  - solver phase is slash-compound (`phase=entering/center`), so PHASE uses \\S+;
    locomotion LOCO lines have a numeric `phase=` but no dcell, so the
    (cell AND phase) filter excludes them;
  - ESCAPE/UNSTICK regexes match maze_motion's event strings verbatim.
"""
import math
import re
import sys

EXIT_CELL = (10, 9)
STAMP = re.compile(r'\[(\d+)\.(\d+)\]')
ESC = re.compile(r'ESCAPE tier=(\d+) count=(\d+) cell=\((\d+), (\d+)\) prev=(\S+ ?\S*?) '
                 r'can_reverse=(\w+) gave_up_edge=(\w+)')
UNS = re.compile(r'UNSTICK reopen cell=\((\d+), (\d+)\) n=(\d+)')
CELL = re.compile(r'dcell=\((\d+), (\d+)\)')
PHASE = re.compile(r'phase=(\S+)')


def _t(line):
    m = STAMP.search(line)
    return float(m.group(1)) + float('0.' + m.group(2)) if m else None


def _dexit(cx, cy):
    return math.hypot(cx - EXIT_CELL[0], cy - EXIT_CELL[1])


def analyze(path):
    events = []          # (t, 'ESC'|'UNS', cell, raw)
    ticks = []           # (t, cell, phase) from solver DIAG lines
    t0 = None
    t_last = None
    with open(path + '/launch.log', errors='replace') as f:
        for line in f:
            t = _t(line)
            if t is None:
                continue
            if t0 is None:
                t0 = t
            t_last = t
            me, mu = ESC.search(line), UNS.search(line)
            if me:
                events.append((t, 'ESC', (int(me.group(3)), int(me.group(4))), me.group(0)))
                continue
            if mu:
                events.append((t, 'UNS', (int(mu.group(1)), int(mu.group(2))), mu.group(0)))
                continue
            mc, mp = CELL.search(line), PHASE.search(line)
            if mc and mp:
                ticks.append((t, (int(mc.group(1)), int(mc.group(2))), mp.group(1)))
    print('==== %s: %d events, %d DIAG ticks ====' % (path, len(events), len(ticks)))
    print('t0=%.1f run_span=%.1fs' % (t0, t_last - t0))
    esc = [(t, cell, raw) for (t, k, cell, raw) in events if k == 'ESC']
    esc_ts = [t for (t, _, _) in esc]
    gaps = [b - a for a, b in zip(esc_ts, esc_ts[1:])]
    if gaps:
        gaps_s = sorted(gaps)
        print('inter-escape gaps: n=%d min=%.1f p50=%.1f p90=%.1f max=%.1f' %
              (len(gaps), gaps_s[0], gaps_s[len(gaps_s) // 2],
               gaps_s[int(len(gaps_s) * 0.9)], gaps_s[-1]))
    if esc_ts:
        regime = t_last - esc_ts[0]
        print('escape regime: first_esc=+%.1fs -> log end = %.1fs (%.0f%% of run)' %
              (esc_ts[0] - t0, regime, 100.0 * regime / (t_last - t0)))
        tiers = {}
        for (_, _, raw) in esc:
            m = ESC.search(raw)
            key = 'tier=%s gave_up=%s' % (m.group(1), m.group(7))
            tiers[key] = tiers.get(key, 0) + 1
        print('escape mix: %s' % sorted(tiers.items(), key=lambda kv: -kv[1]))
    for i, (t, k, cell, raw) in enumerate(events):
        nxt = events[i + 1][0] if i + 1 < len(events) else None
        span = ('%.1f' % (nxt - t)) if nxt is not None else 'END'
        win = [(tt, c, p) for (tt, c, p) in ticks if t <= tt and (nxt is None or tt < nxt)]
        cells = {c for (_, c, _) in win}
        phases = {}
        for (_, _, p) in win:
            phases[p] = phases.get(p, 0) + 1
        print('%+9.1f %s cell=%s dexit=%.2f span=%s cells=%d phases=%s' %
              (t - t0, k, cell, _dexit(*cell), span, len(cells),
               sorted(phases.items(), key=lambda kv: -kv[1])[:4]))
        print('          %s' % raw)


if __name__ == '__main__':
    for p in sys.argv[1:]:
        analyze(p)
