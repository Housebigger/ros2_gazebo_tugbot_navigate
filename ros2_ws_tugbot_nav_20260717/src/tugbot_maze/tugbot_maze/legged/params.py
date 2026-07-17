"""Single home of every legged-gait number. Tune HERE (and gz-side PID gains
in model.sdf) — nothing else hides magic constants. footprint.py must cover
foot_envelope(); test_footprint_covers_gait_envelope enforces it."""
from dataclasses import dataclass

from tugbot_maze.legged.kinematics import FOOT_BALL_R

# Neutral stance foot centres (base frame). Deliberately narrower than the
# X-stance (foot x 0.4527) so the honest dynamic footprint stays maze-sized.
NEUTRAL_FOOT = {'LF': (0.34, 0.3012, -0.50), 'RF': (0.34, -0.3012, -0.50),
                'LH': (-0.34, 0.3012, -0.50), 'RH': (-0.34, -0.3012, -0.50)}

# IK solution of NEUTRAL_FOOT (locked by test_stand_pose_is_ik_of_neutral_foot).
# The model-surgery step of this phase mirrors these into the 12
# <initial_position> values in model.sdf.
STAND_POSE = {
    'LF_HAA': 0.0, 'LF_HFE': 0.705, 'LF_KFE': -0.9608,
    'RF_HAA': 0.0, 'RF_HFE': 0.705, 'RF_KFE': -0.9608,
    'LH_HAA': 0.0, 'LH_HFE': -0.705, 'LH_KFE': 0.9608,
    'RH_HAA': 0.0, 'RH_HFE': -0.705, 'RH_KFE': 0.9608,
}


@dataclass(frozen=True)
class LeggedParams:
    stand_height: float = -NEUTRAL_FOOT['LF'][2]   # derived: |neutral foot z|; standing base height = this + ball
    f_trot: float = 2.0            # stride cycle Hz
    duty: float = 0.5              # stance fraction of the cycle
    swing_lift: float = 0.07       # swing apex height (m)
    vx_max: float = 0.4            # cmd clamp (m/s) — solver hop v_max aligned to this
    wz_max: float = 0.5            # cmd clamp (rad/s) — matches solver w_max default
    ax_max: float = 0.8            # cmd slew limit (m/s^2)
    aw_max: float = 2.5            # cmd slew limit (rad/s^2)
    kp_att: float = 0.8            # roll/pitch righting gain -> foot dz
    dz_max: float = 0.04           # stabilizer dz clamp (m)
    trot_enter_v: float = 0.03     # |vx| above this enters TROT
    trot_enter_w: float = 0.05     # |wz| above this enters TROT
    stand_dwell_s: float = 0.5     # quiet time below half-thresholds before TROT->STAND
    init_s: float = 2.0            # settle time holding STAND_POSE after startup


def foot_envelope(p: LeggedParams = LeggedParams()):
    """Worst-case |x|,|y| any foot CENTRE+ball reaches over the command domain.
    footprint.FOOT_X_FRONT / FOOT_HALF_W must dominate these."""
    xs = max(abs(v[0]) for v in NEUTRAL_FOOT.values())
    ys = max(abs(v[1]) for v in NEUTRAL_FOOT.values())
    half_stride_x = (p.vx_max + p.wz_max * ys) * p.duty / (2.0 * p.f_trot)
    half_stride_y = (p.wz_max * xs) * p.duty / (2.0 * p.f_trot)
    return xs + half_stride_x + FOOT_BALL_R, ys + half_stride_y + FOOT_BALL_R
