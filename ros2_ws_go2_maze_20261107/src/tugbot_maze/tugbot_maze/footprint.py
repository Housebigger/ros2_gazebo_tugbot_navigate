"""Conservative Go2 body and moving-foot envelope for maze safety checks."""
from __future__ import annotations
import math

FOOT_X_FRONT = 0.37     # head and front-foot sweep
FOOT_X_REAR  = -0.42    # rear body and foot sweep
FOOT_HALF_W  = 0.25     # lateral leg and foot sweep
# The geometry certificate samples all official collision primitives over the
# gait domain with independent body roll/pitch ±0.1 rad. Its measured extrema
# are x=[-0.395255,0.344527], |y|<=0.218049; these bounds add >2 cm.
FOOTPRINT_MAX_TILT_RAD = 0.1
SCAN_OFFSET_X = 0.0      # /scan projected from the centered 3D lidar


def beam_endpoint(r, bearing, scan_offset_x: float = SCAN_OFFSET_X):
    """Endpoint of a scan beam (range r, robot-frame bearing) in BASE_LINK coords, accounting for
    the sensor's fore-aft mount offset. bearing 0 = forward."""
    return (scan_offset_x + r * math.cos(bearing), r * math.sin(bearing))


def front_gap(ranges, angle_min, angle_inc, *, half_w: float = FOOT_HALF_W,
              side_pad: float = 0.05, scan_offset_x: float = SCAN_OFFSET_X) -> float:
    """Min clearance AHEAD of the front face (FOOT_X_FRONT) over beams whose endpoint is within the
    body width (|y| <= half_w+side_pad) AND ahead of the front face. inf if none. Gates forward v."""
    g = math.inf
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.05:
            continue
        ex, ey = beam_endpoint(r, angle_min + i * angle_inc, scan_offset_x)
        if abs(ey) <= half_w + side_pad and ex > FOOT_X_FRONT:
            g = min(g, ex - FOOT_X_FRONT)
    return g


def rear_gap(ranges, angle_min, angle_inc, *, half_w: float = FOOT_HALF_W,
             side_pad: float = 0.05, scan_offset_x: float = SCAN_OFFSET_X) -> float:
    """Min clearance BEHIND the rear face (FOOT_X_REAR) over beams within the body width AND behind
    the rear face. inf if none. Gates reverse v, protecting the rear face. (Note: front_gap and
    rear_gap currently have no runtime caller in this workspace -- the live consumer of the
    footprint rectangle is the offline true-collision oracle, maze_sim.collides.)"""
    g = math.inf
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.05:
            continue
        ex, ey = beam_endpoint(r, angle_min + i * angle_inc, scan_offset_x)
        if abs(ey) <= half_w + side_pad and ex < FOOT_X_REAR:
            g = min(g, FOOT_X_REAR - ex)
    return g


def inside_footprint(px, py, margin: float = 0.0) -> bool:
    """True if base_link point (px,py) is inside the footprint rectangle inflated by margin."""
    return (FOOT_X_REAR - margin) <= px <= (FOOT_X_FRONT + margin) and abs(py) <= (FOOT_HALF_W + margin)
