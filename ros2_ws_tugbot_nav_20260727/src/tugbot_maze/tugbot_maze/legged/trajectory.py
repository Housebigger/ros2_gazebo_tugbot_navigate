"""Per-leg foot trajectory around the neutral stance.

Stance (phase fraction < duty): foot pinned to ground (z=0 offset), drifting
backward relative to the body from +half-stride to -half-stride — this is what
propels the body. Swing: mirror return with a sinusoidal lift. The half-stride
is the body-twist velocity AT the neutral foot point times half the stance
time, so stance feet track the ground without slip by construction.
"""
import math

from tugbot_maze.legged.params import NEUTRAL_FOOT


def foot_displacement(leg, leg_phase, vx, wz, p):
    """Displacement (dx, dy, dz) of the foot target from NEUTRAL_FOOT[leg].
    leg_phase in radians (the caller applies the per-leg trot offset)."""
    nx, ny, _nz = NEUTRAL_FOOT[leg]
    tvx = vx - wz * ny            # body twist at the foot point
    tvy = wz * nx
    half_t = p.duty / (2.0 * p.f_trot)   # half the stance duration (s)
    s = (leg_phase % (2.0 * math.pi)) / (2.0 * math.pi)
    if s < p.duty:                # stance: k 1 -> -1
        k = 1.0 - 2.0 * (s / p.duty)
        return (tvx * half_t * k, tvy * half_t * k, 0.0)
    u = (s - p.duty) / (1.0 - p.duty)    # swing: k -1 -> 1, lifted
    k = 2.0 * u - 1.0
    return (tvx * half_t * k, tvy * half_t * k, p.swing_lift * math.sin(math.pi * u))
