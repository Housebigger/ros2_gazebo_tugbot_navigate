#!/usr/bin/env python3
"""Draw the tugbot's ground-truth path as a red LINE_STRIP marker in the Gazebo scene.

Polls the model pose via the gz CLI (~2 Hz), min-distance filters the samples, and
redraws a growing /marker LINE_STRIP. Pure gz-side: no ROS, no extra dependencies.
Started automatically by tools/run_flood_fill_maze.sh when HEADLESS=false; also
standalone-runnable:  python3 tools/gz_trail.py [--model tugbot] [--period 0.5]
[--min-dist 0.10].
See docs/superpowers/specs/2026-07-08-gazebo-truth-trail-design.md.
"""
from __future__ import annotations
import argparse
import re
import signal
import subprocess
import sys
import time
from typing import List, Optional, Tuple

Point = Tuple[float, float, float]

# One bracketed numeric triple; separators are whitespace and/or a pipe (both styles
# of `gz model -p` output exist across gz versions).
# NOTE: the regex cannot distinguish a 2-element bracket [A | B] from a 3-element
# one -- it backtracks and splits B into two tokens. Harmless in practice because
# gz never emits 2-element brackets.
_TRIPLE_RE = re.compile(
    r"\[\s*([-+\d.eE]+)\s*(?:\|\s*)?([-+\d.eE]+)\s*(?:\|\s*)?([-+\d.eE]+)\s*\]")


def parse_model_pose(text: str) -> Optional[Point]:
    """First bracketed [x y z] triple after a line containing 'Pose'; None if absent."""
    if not text:
        return None
    idx = text.find('Pose')
    if idx < 0:
        return None
    m = _TRIPLE_RE.search(text, idx)
    if not m:
        return None
    try:
        return (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    except ValueError:
        return None


def should_record(new_point: Point, last_point: Optional[Point], min_dist: float) -> bool:
    """Record a sample only when it moved >= min_dist (xy-plane) from the last one."""
    if last_point is None:
        return True
    dx = new_point[0] - last_point[0]
    dy = new_point[1] - last_point[1]
    return (dx * dx + dy * dy) >= min_dist * min_dist


def marker_request(points: List[Point], ns: str = 'tugbot_trail', marker_id: int = 1,
                   z: float = 0.05) -> str:
    """Protobuf-text gz.msgs.Marker request: a red LINE_STRIP through the points.
    Each input point's own z is IGNORED and replaced by the z parameter (lifted off
    the floor). Re-sending with the same ns/id replaces the marker, so the line grows."""
    parts = [
        'ns: "%s"' % ns,
        'id: %d' % marker_id,
        'action: ADD_MODIFY',
        'type: LINE_STRIP',
        'material { ambient { r: 1 a: 1 } diffuse { r: 1 a: 1 } emissive { r: 1 a: 1 } }',
    ]
    for p in points:
        parts.append('point { x: %.3f y: %.3f z: %.3f }' % (p[0], p[1], z))
    return ', '.join(parts)


def _poll_pose(model: str) -> Optional[Point]:
    """One ground-truth pose sample via the gz CLI; None while the sim isn't up."""
    try:
        r = subprocess.run(['gz', 'model', '-m', model, '-p'],
                           capture_output=True, text=True, timeout=5)
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    return parse_model_pose(r.stdout)


def _redraw(points: List[Point], ns: str, marker_id: int,
            timeout_ms: int = 2000) -> bool:
    """Replace the trail marker with the full point list. False on failure."""
    req = marker_request(points, ns, marker_id)
    try:
        r = subprocess.run(
            ['gz', 'service', '-s', '/marker',
             '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Boolean',
             '--timeout', str(timeout_ms), '--req', req],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 3.0)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return r.returncode == 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Red ground-truth trail in the Gazebo scene (poll + /marker).')
    ap.add_argument('--model', default='tugbot', help='Gazebo model name')
    ap.add_argument('--period', type=float, default=0.5, help='poll period, seconds')
    ap.add_argument('--min-dist', type=float, default=0.10,
                    help='min xy motion (m) before a new trail point is recorded')
    args = ap.parse_args(argv)

    stop = {'flag': False}

    def _sig(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    points: List[Point] = []
    fails = 0
    while not stop['flag']:
        p = _poll_pose(args.model)
        if p is not None and should_record(p, points[-1] if points else None,
                                           args.min_dist):
            points.append(p)
            # Each redraw resends the FULL list (same ns/id replaces the marker).
            # Growth is intentional and bounded by run length: a maze run is a few
            # hundred 0.10m-filtered points -> a few KB per request. Keep it simple.
            if len(points) >= 2:                       # a strip needs 2+ points
                if _redraw(points, 'tugbot_trail', 1):
                    fails = 0
                else:
                    fails += 1
                    print('gz_trail: marker redraw failed (n=%d, consecutive=%d)'
                          % (len(points), fails), file=sys.stderr, flush=True)
        time.sleep(args.period)
    return 0


if __name__ == '__main__':
    sys.exit(main())
