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
from typing import List, Optional, Tuple

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
    each segment at quarter points (the mid-arc bulge exceeds the chord endpoints).
    A segment with curvature ~0 (a straight leg, e.g. the reverse leg of
    back-and-arc) integrates as a straight line instead -- the arc formula divides
    by k and blows up at k=0."""
    x = y = 0.0
    yaw = yaw0
    exc = 0.0
    for v_sign, k, L in segs:
        if abs(k) < 1e-9:
            for _ in range(4):
                s = v_sign * (L / 4.0)
                x += s * math.cos(yaw)
                y += s * math.sin(yaw)
                exc = max(exc, math.hypot(x, y))
            continue
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


def plan_back_and_arc(rel_dir: int, d_back: float) -> List[Segment]:
    """Two segments for a 90-degree junction turn: straight reverse d_back along the
    current heading, then a forward quarter-arc of radius R = d_back turning toward
    rel_dir (+1 = left/CCW, -1 = right/CW). The arc is tangent to both corridor
    centerlines: backing d_back from the junction center puts the car at the arc's
    entry point, and the arc exits ON the new centerline R past the center -- the
    drive phase picks up from there. Requires d_back >= 0.45 (curvature 1/0.45 =
    2.22 < the 2.4 steering limit); raises ValueError below."""
    if d_back < 0.45:
        raise ValueError('insufficient reverse room for back-and-arc')
    k = rel_dir / d_back
    return [(-1, 0.0, d_back), (1, k, (math.pi / 2.0) * d_back)]


class NPointTurnRunner:
    """Executes a planned turn tick-by-tick on wall-clock time: each segment runs
    v = v_sign * v_mag, w = v * curvature for arc_len / v_mag seconds, with a
    zero-speed pause between segments for the steering rack to swing."""

    def __init__(self, yaw_now: float, yaw_target: float, t_now: float, *,
                 v_mag: float = TURN_V_MAG, pause_s: float = PAUSE_S,
                 max_curvature: float = MAX_CURVATURE, seg_len: float = SEG_LEN_M,
                 excursion_limit: float = EXCURSION_LIMIT_M,
                 segments: Optional[List[Segment]] = None) -> None:
        self.target = yaw_target
        self.v_mag = float(v_mag)
        self.pause_s = float(pause_s)
        if segments is not None:
            # Pre-planned segments (e.g. plan_back_and_arc) override the N-point plan;
            # `target` is still stored above so the caller's replan-key comparison
            # (target changed -> replan) treats this program like any other.
            self.segs = list(segments)
        else:
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


def clamp_to_ackermann(v: float, w: float, *,
                       max_curvature: float = MAX_CURVATURE,
                       v_floor: float = 0.08) -> Tuple[float, float]:
    """Project a (v, w) command into the Ackermann-feasible set: the car cannot
    pivot, so a meaningful steering demand requires wheel speed. Floors |v| when
    |w| > ~0 (a small nonzero v keeps its sign; a pure rotation becomes a slow
    REVERSE arc -- retreating into the just-traversed clear corridor), then caps
    |w| at |v| * max_curvature (the steering-limit curvature). At cruise (v=0.4)
    the cap is 0.96 rad/s > every steering law's w_max, so normal driving is
    untouched; only the pivot-shaped collapse regime changes (the DRIVE-phase
    near-wall keep-out law emitted v=0, w=-0.5 -- the third in-place source the
    full-solve invariant test caught)."""
    if abs(w) <= 1e-3:
        return v, w
    if abs(v) < v_floor:
        # A pure pivot intent (v exactly 0) retreats while steering: the stop-and-
        # reorient law fires precisely where front_block/wedge gating is suppressed
        # (large heading error), so a FORWARD creep would head into the obstacle
        # that caused the stop; the just-traversed corridor behind is clear (the
        # reverse-to-center/backout precedent). A small nonzero v keeps its sign.
        v = math.copysign(v_floor, v) if abs(v) > 1e-9 else -v_floor
    w_cap = abs(v) * max_curvature
    return v, max(-w_cap, min(w_cap, w))
