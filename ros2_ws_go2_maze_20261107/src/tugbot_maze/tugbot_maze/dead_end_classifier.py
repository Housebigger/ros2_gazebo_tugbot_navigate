"""Distinguish a geometry-confirmed dead end from a transient navigation failure.

Pure Python so it is unit-testable without ROS. A Nav2 goal timeout is NEVER a
dead-end signal on its own; only perception saying DEAD_END together with a scan
that confirms a wall within ~corridor length counts.
"""
from __future__ import annotations

from typing import Optional

DEAD_END = 'dead_end'


def is_true_dead_end(
    perception_kind: str,
    forward_min_range_m: Optional[float],
    corridor_length_m: float = 1.76,
    wall_margin_m: float = 0.4,
) -> bool:
    if perception_kind != DEAD_END:
        return False
    if forward_min_range_m is None:
        return False  # no scan corroboration -> do not condemn the corridor
    return forward_min_range_m <= (corridor_length_m - wall_margin_m)
