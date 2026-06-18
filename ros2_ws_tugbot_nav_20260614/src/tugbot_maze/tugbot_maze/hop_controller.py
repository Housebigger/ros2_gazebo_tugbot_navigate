"""ROS-free drive commands for the cell-grid solver, within the real motion envelope
(v,w <= 0.5). P-control on heading; forward speed is throttled by heading error so the
robot turns toward its goal before driving (no overshoot).

Three commands:
  - hop_command:      point-to-point drive to an (x,y) target (turn-then-drive).
  - centering_command: cardinal-aligned 1-axis re-centering -- face the worse axis's map
    cardinal, then drive forward to null that offset. No diagonal chase, so the robot
    stays cardinal-aligned and converges (the chase oscillated and ate the hop deadline).
  - hop_drive_command: straight cell hop that HOLDS the cardinal heading with continuous
    steering, so a small start mis-alignment / diff-drive bias can't accumulate into the
    large lateral drift that desynced the discrete cell tracker and triggered false walls.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def centering_command(pose, ox: Optional[float], oy: Optional[float], *,
                      tol: float = 0.12, yaw_tol: float = 0.10,
                      v_max: float = 0.4, w_max: float = 0.5,
                      kp_ang: float = 1.5, kp_lin: float = 0.8,
                      v_min: float = 0.06) -> Tuple[float, float, bool]:
    """Re-center the robot to the current cell centre one cardinal axis at a time.

    (ox, oy) is the robot's offset from the cell centre in MAP axes (+x=E, +y=N), as
    returned by cell_center_offset (a component is None for an open axis with no wall to
    reference). Returns (v, w, done). Corrects the larger out-of-tol axis by facing the
    map cardinal that reduces it, then driving forward (speed proportional to the
    remaining offset). done=True when every referenced axis is within tol."""
    cands = []
    if ox is not None and abs(ox) > tol:
        cands.append(('x', ox))
    if oy is not None and abs(oy) > tol:
        cands.append(('y', oy))
    if not cands:
        return (0.0, 0.0, True)
    axis, off = max(cands, key=lambda c: abs(c[1]))
    if axis == 'x':
        want = math.pi if off > 0 else 0.0           # east of centre -> face west, drive
    else:
        want = -math.pi / 2 if off > 0 else math.pi / 2   # north of centre -> face south
    dyaw = _norm(want - pose[2])
    if abs(dyaw) > yaw_tol:
        return (0.0, max(-w_max, min(w_max, kp_ang * dyaw)), False)   # face the axis first
    v = min(v_max, max(v_min, kp_lin * abs(off)))    # drive forward to null the offset
    return (v, 0.0, False)


def hop_drive_command(pose, target_yaw: float, *, v_max: float = 0.3, w_max: float = 0.5,
                      kp_ang: float = 1.5, slow_angle: float = 0.6) -> Tuple[float, float]:
    """Forward drive for one cell hop that holds `target_yaw` (a map cardinal). Steers
    continuously to correct heading drift while moving; forward speed is throttled toward
    0 when badly mis-aligned so the robot straightens before it makes lateral distance."""
    dyaw = _norm(target_yaw - pose[2])
    w = max(-w_max, min(w_max, kp_ang * dyaw))
    throttle = max(0.0, 1.0 - abs(dyaw) / slow_angle)
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w)


def hop_command(pose, target_xy, *, v_max: float = 0.5, w_max: float = 0.5,
                kp_ang: float = 1.5, slow_angle: float = 0.6,
                arrive_m: float = 0.25) -> Tuple[float, float, bool]:
    px, py, yaw = pose
    dx, dy = target_xy[0] - px, target_xy[1] - py
    dist = math.hypot(dx, dy)
    if dist <= arrive_m:
        return (0.0, 0.0, True)
    err = _norm(math.atan2(dy, dx) - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)   # 0 when |err|>=slow_angle
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w, False)
