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
from typing import Dict, Optional, Tuple


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def centering_command(pose, ox: Optional[float], oy: Optional[float], *,
                      tol: float = 0.12, yaw_tol: float = 0.10,
                      v_max: float = 0.4, w_max: float = 0.5,
                      kp_ang: float = 1.5, kp_lin: float = 0.8,
                      v_min: float = 0.06, kd_ang: float = 0.0,
                      yaw_rate: float = 0.0) -> Tuple[float, float, bool]:
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
        w = kp_ang * dyaw - kd_ang * yaw_rate                         # PD: damp latency overshoot
        return (0.0, max(-w_max, min(w_max, w)), False)               # face the axis first
    v = min(v_max, max(v_min, kp_lin * abs(off)))    # drive forward to null the offset
    return (v, 0.0, False)


def cross_track_offset(ox: Optional[float], oy: Optional[float], hop_dir) -> float:
    """Signed lateral offset of the robot from the corridor centerline, measured to the
    LEFT of the hop direction (+ = robot is left of centre), from the map-axis offsets
    (ox=+E, oy=+N) returned by cell_center_offset. For N/S travel the lateral axis is x;
    for E/W it is y. Returns 0.0 if that perpendicular axis is open (None) -- no side wall
    to reference, so hold heading only."""
    dx, dy = hop_dir
    if dy != 0:                                  # N/S travel -> lateral is the x axis
        if ox is None:
            return 0.0
        return -ox if dy > 0 else ox             # N: east(+ox)=right=>-; S: east=left=>+
    if oy is None:                               # E/W travel -> lateral is the y axis
        return 0.0
    return oy if dx > 0 else -oy                 # E: north(+oy)=left=>+; W: north=right=>-


def side_distances(perp: Dict[str, float], hop_dir) -> Tuple[float, float]:
    """Map the per-cardinal perpendicular-distance dict (keys 'E','W','N','S', from
    cell_wall_perp_dist) to the robot's (left, right) side-wall distances for travel along
    hop_dir. Robot-frame left is +90 deg from heading."""
    dx, dy = hop_dir
    if dy > 0:                       # N: left=W, right=E
        return perp['W'], perp['E']
    if dy < 0:                       # S: left=E, right=W
        return perp['E'], perp['W']
    if dx > 0:                       # E: left=N, right=S
        return perp['N'], perp['S']
    return perp['S'], perp['N']      # W: left=S, right=N


def centerline_cross(d_left: float, d_right: float, *, fallback_cross: float = 0.0,
                     wall_seen_m: float = 1.3, half_corridor_m: float = 0.88) -> float:
    """Signed lateral offset of the robot to the LEFT of the corridor centerline (+ = left),
    from the left/right side-wall distances. A side counts as a seen wall when < wall_seen_m.
      both seen   -> balance the two (the drift-immune true centerline -- THE key change),
      one seen    -> hold half_corridor_m from that wall,
      neither     -> the caller's fallback (cell-grid/odom estimate), else 0 (hold heading)."""
    left_seen = d_left < wall_seen_m
    right_seen = d_right < wall_seen_m
    if left_seen and right_seen:
        return (d_right - d_left) / 2.0
    if right_seen:
        return d_right - half_corridor_m
    if left_seen:
        return half_corridor_m - d_left
    return fallback_cross


def hop_drive_command(pose, target_yaw: float, cross_track: float = 0.0, *,
                      v_max: float = 0.3, w_max: float = 0.5, kp_ang: float = 1.5,
                      kp_cross: float = 1.2, cross_w_max: float = 0.25,
                      slow_angle: float = 0.6) -> Tuple[float, float]:
    """Straight forward drive for one cell hop, holding `target_yaw` (a map cardinal) AND
    the corridor centerline. Steering corrects both the heading error and the cross-track
    (steer right when left of centre). The cross-track contribution is capped at
    cross_w_max so a large/bogus lateral estimate can only NUDGE the heading -- it can
    never swing the robot far off the cardinal, which would throttle forward speed to zero
    and stall the hop. Forward speed is throttled only by the (small) heading error, so the
    caller's turn-in-place leaves the robot moving at near-full speed."""
    dyaw = _norm(target_yaw - pose[2])
    w_cross = max(-cross_w_max, min(cross_w_max, kp_cross * cross_track))
    w = max(-w_max, min(w_max, kp_ang * dyaw - w_cross))
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


def corridor_drive_command(yaw: float, cardinal_yaw: float, cross_track: float,
                           near_wall_m: Optional[float] = None, *, v_max: float = 0.3,
                           w_max: float = 0.5, kp_ang: float = 1.5, lookahead_m: float = 0.7,
                           slow_angle: float = 0.6, wedge_slow_m: float = 0.60,
                           wedge_stop_m: float = 0.40, wedge_v_floor: float = 0.10,
                           max_cross_steer: float = 0.35,
                           safety_radius: float = 0.60, keepout_max_cross_steer: float = 0.8
                           ) -> Tuple[float, float]:
    """Drive forward along `cardinal_yaw` while converging onto the corridor centerline.

    cross_track = signed lateral offset (+ = robot LEFT of centre). The robot aims at the
    centerline point `lookahead_m` ahead, so it CURVES back toward centre as it nears a side
    wall (inherently anti-wedge). Forward speed is throttled to ~0 only by a large HEADING
    error (turn first). If `near_wall_m` (min perpendicular distance to the two side walls)
    is given, speed is additionally slowed toward `wedge_v_floor` as the nearer wall
    approaches `wedge_stop_m` -- a NEVER-zero floor (the heading term can still zero v), so
    the robot keeps creeping while steering away rather than freezing or wedging."""
    cap = keepout_max_cross_steer if (near_wall_m is not None and near_wall_m < safety_radius) else max_cross_steer
    cross_steer = max(-cap, min(cap, math.atan2(-cross_track, lookahead_m)))   # keep-out overrides the cap near a wall
    setpoint = cardinal_yaw + cross_steer        # cross-track heading authority (capped; uncapped within safety_radius)
    err = _norm(setpoint - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)              # heading: -> 0 when misaligned
    if near_wall_m is not None:                                  # wedge: slow near a wall, never 0
        wedge = max(0.0, min(1.0, (near_wall_m - wedge_stop_m) / (wedge_slow_m - wedge_stop_m)))
        throttle *= max(wedge_v_floor / v_max, wedge)
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w)


def corridor_follow_command(yaw: float, cardinal_yaw: float, d_left: float, d_right: float,
                            near_wall_m: Optional[float] = None, *, fallback_cross: float = 0.0,
                            wall_seen_m: float = 1.3, half_corridor_m: float = 0.88,
                            max_cross_track_m: float = 0.6, v_max: float = 0.3,
                            w_max: float = 0.5, kp_ang: float = 1.5, lookahead_m: float = 0.7,
                            slow_angle: float = 0.6, wedge_slow_m: float = 0.60,
                            wedge_stop_m: float = 0.40, wedge_v_floor: float = 0.10,
                            max_cross_steer: float = 0.35,
                            safety_radius: float = 0.60, keepout_max_cross_steer: float = 0.8
                            ) -> Tuple[float, float]:
    """Symmetric wall-following straight drive. Derives the lateral centerline offset from the
    two side-wall distances (centerline_cross), clamps it, then steers/throttles with the proven
    pure-pursuit + never-zero wedge-floor law (corridor_drive_command). Keeps the 0.35 m robot on
    the PHYSICAL centerline using both walls (drift-immune) instead of the cell-grid offset that
    vanished at openings -- closing the off-center-entry -> wedge path."""
    cross = centerline_cross(d_left, d_right, fallback_cross=fallback_cross,
                             wall_seen_m=wall_seen_m, half_corridor_m=half_corridor_m)
    cross = max(-max_cross_track_m, min(max_cross_track_m, cross))
    return corridor_drive_command(yaw, cardinal_yaw, cross, near_wall_m, v_max=v_max,
                                  w_max=w_max, kp_ang=kp_ang, lookahead_m=lookahead_m,
                                  slow_angle=slow_angle, wedge_slow_m=wedge_slow_m,
                                  wedge_stop_m=wedge_stop_m, wedge_v_floor=wedge_v_floor,
                                  max_cross_steer=max_cross_steer,
                                  safety_radius=safety_radius, keepout_max_cross_steer=keepout_max_cross_steer)


def profiled_turn_command(yaw: float, target_cardinal: float, yaw_rate: float = 0.0, *,
                          ang_decel: float = 1.2, turn_w_max: float = 0.35,
                          kd: float = 0.0) -> float:
    """Rotate-in-place toward target_cardinal with a DECEL-LIMITED angular profile:
    |w| = min(turn_w_max, sqrt(2*ang_decel*|err|)) ramps to 0 as the heading error closes, so a
    delayed (command-latency) command lands while the robot is already decelerating -> no
    overshoot (the PD law's failure mode). Optional kd*yaw_rate adds light damping. Returns w
    only (the caller drives v=0)."""
    err = _norm(target_cardinal - yaw)
    w_mag = min(turn_w_max, math.sqrt(2.0 * ang_decel * abs(err)))
    w = math.copysign(w_mag, err) - kd * yaw_rate
    return max(-turn_w_max, min(turn_w_max, w))     # clamp guards a kd*yaw_rate overshoot (kd>0)


def backout_command(yaw: float, hold_cardinal: float, *, backout_v: float = 0.30,
                    w_max: float = 0.5, kp_ang: float = 1.5) -> Tuple[float, float]:
    """Firm straight reverse out of a dead-end while holding `hold_cardinal`. Returns (v, w) with
    v = -backout_v (decisive, ~cruise speed -- NOT the 0.15 un-wedge crawl). Proportional steering
    on the heading error keeps the robot backing straight along the cardinal; the steering sign is
    correct for reverse travel because w rotates the body toward hold_cardinal regardless of the
    sign of v. No cross-track term -- the reverse is a single ~2 m cell retracing a path just
    driven centered, so heading-hold suffices."""
    err = _norm(hold_cardinal - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    return (-backout_v, w)


def grid_cross_track(pose_x: float, pose_y: float, cell, hop_dir, *,
                     cell_size_m: float = 2.0) -> float:
    """Signed lateral offset of the robot to the LEFT of the hop direction (+ = robot left of
    centre), measured from the GRID corridor centerline (the cell-centre line) using the odom pose.
    Unlike cross_track_offset (wall-derived, 0 when no side wall is visible), this is ALWAYS valid,
    so it gives the corridor follower a centerline reference through an open junction where both side
    walls vanish. Sign convention matches cross_track_offset. `cell` is the hop's SOURCE cell; it
    shares the perpendicular coordinate with the target (the hop is along a cardinal), so it defines
    the corridor centerline for the whole hop. The convergence target is the ODOM centerline (off the
    true centre by the bounded drift residual); the caller clamps the result, so a small drift error
    can only nudge, never swing."""
    dx, dy = hop_dir
    if dy != 0:                                       # N/S travel -> lateral is the x axis
        off = pose_x - cell_size_m * cell[0]          # +E
        return -off if dy > 0 else off                # N: east=right=>-; S: east=left=>+
    off = pose_y - cell_size_m * cell[1]              # +N
    return off if dx > 0 else -off                    # E: north=left=>+; W: north=right=>-
