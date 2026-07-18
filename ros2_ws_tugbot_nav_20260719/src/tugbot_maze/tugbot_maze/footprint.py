"""ANYmal C dog collision footprint + sensor placement (see
tugbot_description/models/anymal_c/model.sdf). All in the base_link frame,
+x=forward, +y=left. Symmetric rectangle = the TRUE dynamic foot envelope of
the legged gait with NO padding (tugbot-era convention: safety margins live in
the gates/thresholds, not in the footprint): neutral stance feet at x=+-0.34 /
y=+-0.3012, plus worst-case trot stride reach and the 0.03 foot ball — see
legged/params.foot_envelope(), enforced by test_footprint_covers_gait_envelope.
The 2D omni lidar sits at the body centre."""
from __future__ import annotations
import math

FOOT_X_FRONT = 0.49      # neutral stance 0.34 + max stride reach + ball, rounded up
FOOT_X_REAR  = -0.49     # symmetric
FOOT_HALF_W  = 0.37      # lateral stance 0.3012 + yaw-stride reach + ball, rounded up
SCAN_OFFSET_X = 0.0      # /scan (scan_omni) at body centre


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
    the rear face. inf if none. Gates reverse v -- this is what finally protects the gripper."""
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
