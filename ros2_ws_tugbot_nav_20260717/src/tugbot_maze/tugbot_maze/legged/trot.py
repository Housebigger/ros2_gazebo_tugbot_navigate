"""Trot phase machine + mode FSM + 12-joint target assembly.

Modes: INIT (hold STAND_POSE for init_s while the model settles onto its
feet) -> STAND (quiet stance with attitude feedback) <-> TROT (diagonal-pair
stepping). Commands are slew-limited and clamped here so a step change from
the solver can never teleport a foot target between ticks.
"""
import math

from tugbot_maze.legged.kinematics import HAA_LIMITS, LEGS, leg_ik
from tugbot_maze.legged.params import LeggedParams, NEUTRAL_FOOT, STAND_POSE
from tugbot_maze.legged.stabilizer import height_offsets
from tugbot_maze.legged.trajectory import foot_displacement

PAIR_PHASE = {'LF': 0.0, 'RH': 0.0, 'RF': math.pi, 'LH': math.pi}


def _toward(cur, target, rate, dt):
    lo, hi = cur - rate * dt, cur + rate * dt
    return max(lo, min(hi, target))


class LocomotionFSM:
    INIT, STAND, TROT = 'INIT', 'STAND', 'TROT'

    def __init__(self, params=None):
        self.p = params or LeggedParams()
        self.mode = self.INIT
        self.t = 0.0
        self.phase = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self._quiet_since = None
        self._seed = {leg: [STAND_POSE[f'{leg}_HAA'], STAND_POSE[f'{leg}_HFE'],
                            STAND_POSE[f'{leg}_KFE']] for leg in LEGS}

    def step(self, dt, vx_cmd, wz_cmd, roll, pitch, force_trot=False):
        """Advance dt seconds; returns {joint_name: target_angle} for all 12."""
        p = self.p
        self.t += dt
        self.vx = _toward(self.vx, max(-p.vx_max, min(p.vx_max, vx_cmd)), p.ax_max, dt)
        self.wz = _toward(self.wz, max(-p.wz_max, min(p.wz_max, wz_cmd)), p.aw_max, dt)

        if self.mode == self.INIT:
            if self.t >= p.init_s:
                self.mode = self.STAND
            return dict(STAND_POSE)

        active = force_trot or abs(self.vx) > p.trot_enter_v or abs(self.wz) > p.trot_enter_w
        if self.mode == self.STAND and active:
            self.mode = self.TROT
            self._quiet_since = None
        elif self.mode == self.TROT:
            quiet = (not force_trot and abs(self.vx) <= 0.5 * p.trot_enter_v
                     and abs(self.wz) <= 0.5 * p.trot_enter_w)
            if quiet:
                self._quiet_since = self.t if self._quiet_since is None else self._quiet_since
                if self.t - self._quiet_since >= p.stand_dwell_s:
                    self.mode = self.STAND
            else:
                self._quiet_since = None

        if self.mode == self.TROT:
            self.phase = (self.phase + 2.0 * math.pi * p.f_trot * dt) % (2.0 * math.pi)

        dz = height_offsets(roll, pitch, p)
        out = {}
        for leg in LEGS:
            if self.mode == self.TROT:
                d = foot_displacement(leg, self.phase + PAIR_PHASE[leg], self.vx, self.wz, p)
            else:
                d = (0.0, 0.0, 0.0)
            n = NEUTRAL_FOOT[leg]
            target = (n[0] + d[0], n[1] + d[1], n[2] + d[2] + dz[leg])
            q, _err = leg_ik(leg, target, seed=self._seed[leg])
            lo, hi = HAA_LIMITS[leg]
            q[0] = max(lo, min(hi, q[0]))
            self._seed[leg] = q
            out[f'{leg}_HAA'], out[f'{leg}_HFE'], out[f'{leg}_KFE'] = q[0], q[1], q[2]
        return out
