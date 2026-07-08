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
