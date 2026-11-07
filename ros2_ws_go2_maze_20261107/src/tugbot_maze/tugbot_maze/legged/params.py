"""Go2 gait parameters and its centered neutral stance."""
from dataclasses import dataclass
import math

from tugbot_maze.legged.kinematics import FOOT_BALL_R, LEGS, leg_ik

NEUTRAL_FOOT = {
    'LF': (0.1934, 0.142, -0.32), 'RF': (0.1934, -0.142, -0.32),
    'LH': (-0.1934, 0.142, -0.32), 'RH': (-0.1934, -0.142, -0.32),
}
STAND_POSE = {}
for leg in LEGS:
    q,_ = leg_ik(leg,NEUTRAL_FOOT[leg])
    STAND_POSE.update(zip((f'{leg}_{axis}' for axis in ('HAA','HFE','KFE')),q))


@dataclass(frozen=True)
class LeggedParams:
    stand_height: float = 0.32
    f_trot: float = 1.6666666666666667
    duty: float = 0.7333333333333333
    swing_lift: float = 0.035
    vx_max: float = 0.25
    wz_max: float = 0.5
    ax_max: float = 0.5
    aw_max: float = 1.5
    kp_att: float = 0.7
    dz_max: float = 0.025
    trot_enter_v: float = 0.025
    trot_enter_w: float = 0.04
    stand_dwell_s: float = 0.5
    init_s: float = 2.0


def foot_envelope(p: LeggedParams = LeggedParams()):
    xs = max(abs(v[0]) for v in NEUTRAL_FOOT.values())
    ys = max(abs(v[1]) for v in NEUTRAL_FOOT.values())
    half_stride_x = (p.vx_max+p.wz_max*ys)*p.duty/(2*p.f_trot)
    half_stride_y = p.wz_max*xs*p.duty/(2*p.f_trot)
    return xs+half_stride_x+FOOT_BALL_R,ys+half_stride_y+FOOT_BALL_R
