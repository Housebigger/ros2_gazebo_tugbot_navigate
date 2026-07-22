"""N-point (K-turn) maneuver planning for the Ackermann chassis.

The gz AckermannSteering plugin cannot rotate in place (v=0, w!=0 stalls the
car; min turn radius = wheel_base/tan(steering_limit) ~= 0.41 m), so heading
changes are executed as alternating forward/reverse arcs at maximum curvature:
forward-left then reverse-right both advance the heading the same way while
their displacements largely cancel, keeping the excursion bounded inside the
2 m cell. Pure math, offline-testable; MazeMotion drives the runner
tick-by-tick and stops early once its own yaw tolerance is met."""
from __future__ import annotations
import math
from typing import List, Tuple

Segment = Tuple[int, float, float]     # (v_sign +-1, signed curvature 1/m, arc length m)

MAX_CURVATURE = 2.4      # tan(0.5)/0.2255 = 2.42; 1% margin under the steering limit
SEG_LEN_M = 0.45         # ~1.08 rad heading change per segment at max curvature
EXCURSION_LIMIT_M = 0.5  # max distance from the turn's start point (2 m cell, walls >=0.88)
TURN_V_MAG = 0.15        # slow maneuver speed: tame smoother ramps + contact transients
PAUSE_S = 0.6            # zero-speed gap between segments: steering rack swings (vel limit 1.0)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def simulate_segments(segs: List[Segment], yaw0: float) -> Tuple[float, float]:
    """(final_yaw, max_excursion): integrate the arc chain from the origin, sampling
    each segment at quarter points (the mid-arc bulge exceeds the chord endpoints)."""
    x = y = 0.0
    yaw = yaw0
    exc = 0.0
    for v_sign, k, L in segs:
        for _ in range(4):
            s = v_sign * (L / 4.0)
            new_yaw = yaw + s * k
            x += (math.sin(new_yaw) - math.sin(yaw)) / k
            y += -(math.cos(new_yaw) - math.cos(yaw)) / k
            yaw = new_yaw
            exc = max(exc, math.hypot(x, y))
    return yaw, exc


def plan_n_point_turn(yaw_now: float, yaw_target: float, *,
                      max_curvature: float = MAX_CURVATURE,
                      seg_len: float = SEG_LEN_M,
                      excursion_limit: float = EXCURSION_LIMIT_M) -> List[Segment]:
    """Alternating forward/reverse max-curvature arcs closing wrap(target-now).
    Shrinks the segment length until the simulated excursion fits the limit."""
    err = _wrap(yaw_target - yaw_now)
    if abs(err) < 1e-9:
        return []
    length = seg_len
    while True:
        segs: List[Segment] = []
        remaining, v_sign = err, 1
        while abs(remaining) > 1e-9:
            dpsi = max(-length * max_curvature, min(length * max_curvature, remaining))
            k = math.copysign(max_curvature, dpsi * v_sign)   # yaw rate v*k must carry sign(dpsi)
            segs.append((v_sign, k, abs(dpsi) / max_curvature))
            remaining -= dpsi
            v_sign = -v_sign
        _, exc = simulate_segments(segs, 0.0)
        if exc <= excursion_limit:
            return segs
        length *= 0.75
        if length < 0.10:      # unreachable for maze-scale turns; guards a bad param set
            raise ValueError('cannot satisfy excursion limit')


class NPointTurnRunner:
    """Executes a planned turn tick-by-tick on wall-clock time: each segment runs
    v = v_sign * v_mag, w = v * curvature for arc_len / v_mag seconds, with a
    zero-speed pause between segments for the steering rack to swing."""

    def __init__(self, yaw_now: float, yaw_target: float, t_now: float, *,
                 v_mag: float = TURN_V_MAG, pause_s: float = PAUSE_S,
                 max_curvature: float = MAX_CURVATURE, seg_len: float = SEG_LEN_M,
                 excursion_limit: float = EXCURSION_LIMIT_M) -> None:
        self.target = yaw_target
        self.v_mag = float(v_mag)
        self.pause_s = float(pause_s)
        self.segs = plan_n_point_turn(yaw_now, yaw_target, max_curvature=max_curvature,
                                      seg_len=seg_len, excursion_limit=excursion_limit)
        self.i = 0
        self.seg_start_t = t_now
        self.pause_until = t_now       # no leading pause

    def command(self, t: float) -> Tuple[float, float, bool]:
        """(v, w, exhausted). exhausted=True once every segment (and trailing pause)
        has run; the caller re-checks its own yaw tolerance and may replan."""
        if t < self.pause_until:
            return (0.0, 0.0, False)
        while self.i < len(self.segs):
            v_sign, k, L = self.segs[self.i]
            dur = L / self.v_mag
            if t - self.seg_start_t < dur:
                v = v_sign * self.v_mag
                return (v, v * k, False)
            self.i += 1
            self.seg_start_t = t + self.pause_s
            self.pause_until = t + self.pause_s
            return (0.0, 0.0, False)
        return (0.0, 0.0, True)
