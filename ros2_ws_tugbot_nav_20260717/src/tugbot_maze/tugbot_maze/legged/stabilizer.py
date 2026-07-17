"""Attitude righting: with stance feet on the ground, body attitude is set by
relative leg lengths. +roll (left up) means the left legs are effectively too
long -> raise the left foot targets; +pitch (nose down) means the front legs
are too short -> extend the front. dz_i = kp * (roll * y_i - pitch * x_i),
clamped. Gain 1.0 would level exactly in one shot; 0.8 leaves damping."""
from tugbot_maze.legged.params import NEUTRAL_FOOT


def height_offsets(roll, pitch, p):
    out = {}
    for leg, (nx, ny, _nz) in NEUTRAL_FOOT.items():
        dz = p.kp_att * (roll * ny - pitch * nx)
        out[leg] = max(-p.dz_max, min(p.dz_max, dz))
    return out
