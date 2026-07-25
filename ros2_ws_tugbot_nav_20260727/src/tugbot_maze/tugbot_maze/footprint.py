"""MR-Buggy3 kinematic chassis collision footprint + sensor placement (see
tugbot_description/models/mr_buggy3/model.sdf). All in the base_link frame,
+x=forward, +y=left. Symmetric rectangle = the full wheel-inclusive envelope
with NO padding (tugbot-era convention: safety margins live in the
gates/thresholds, not in the footprint): wheel centers at x=+0.112/-0.1135
(radius 0.0365 -> extents ~+-0.15), wheel outer faces at |y|=0.13
(0.10 + 0.015 hub offset + 0.015 half-width); the 0.3 x 0.09 body box sits
inside. The 16-beam lidar sits at the body centre (SCAN_OFFSET_X=0)."""
from __future__ import annotations
import math

FOOT_X_FRONT = 0.15      # front wheel center 0.112 + wheel radius 0.0365, rounded up
FOOT_X_REAR  = -0.15     # rear wheel center -0.1135 - 0.0365, rounded (symmetric)
FOOT_HALF_W  = 0.13      # wheel outer face: 0.10 + 0.015 + 0.015
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
