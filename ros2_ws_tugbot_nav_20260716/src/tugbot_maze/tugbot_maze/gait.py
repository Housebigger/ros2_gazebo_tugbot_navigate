"""Open-loop trot gait for the ANYmal C dog — VISUAL ANIMATION ONLY.

Pure module (no ROS imports). The kinematic base is moved by the Gazebo
VelocityControl plugin; these joint targets exist so the dog visibly walks.
Diagonal pairs LF+RH and RF+LH swing in anti-phase (classic trot); amplitude
and stride frequency scale with commanded speed and vanish at standstill, so
the dog settles into STAND_POSE when the solver stops.

Sign conventions come from the model SDF: the HFE/KFE joint axes are mirrored
left/right (left `+1 0 0`, right `-1 0 0`), so the SAME value commanded to a
left and right HFE/KFE joint produces mirror-symmetric motion. HAA axes are
instead mirrored front/hind (LF_HAA=RF_HAA=`1 0 0`, LH_HAA=RH_HAA=`-1 0 0`),
which matters if HAA sway is ever animated. Signs are eyeballed in the Gazebo
GUI during acceptance; the tests here pin structure, not looks.
"""
from __future__ import annotations

import math

JOINTS = [f'{leg}_{j}' for leg in ('LF', 'RF', 'LH', 'RH') for j in ('HAA', 'HFE', 'KFE')]

# X-stance (knees inward). Same value left/right thanks to mirrored axes.
STAND_POSE = {
    'LF_HAA': 0.0, 'LF_HFE': 0.4, 'LF_KFE': -0.8,
    'RF_HAA': 0.0, 'RF_HFE': 0.4, 'RF_KFE': -0.8,
    'LH_HAA': 0.0, 'LH_HFE': -0.4, 'LH_KFE': 0.8,
    'RH_HAA': 0.0, 'RH_HFE': -0.4, 'RH_KFE': 0.8,
}

# Hardware limits from model.sdf (HFE/KFE are +-9.42 rad, effectively free).
HAA_LIMITS = {
    'LF_HAA': (-0.72, 0.49), 'RF_HAA': (-0.49, 0.72),
    'LH_HAA': (-0.72, 0.49), 'RH_HAA': (-0.49, 0.72),
}

V_REF = 0.4        # m/s at which swing amplitude saturates
OMEGA_WEIGHT = 0.5  # unitless: converts |omega| [rad/s] into an equivalent linear-speed contribution
HFE_SWING = 0.25   # rad fore-aft hip swing at full amplitude
KFE_LIFT = 0.35    # rad extra knee flex during swing at full amplitude
F_MIN, F_MAX = 0.8, 2.2  # Hz stride frequency range while moving

_PAIR_PHASE = {'LF': 0.0, 'RH': 0.0, 'RF': math.pi, 'LH': math.pi}


def amplitude_scale(v: float, omega: float) -> float:
    """0..1 swing amplitude from commanded planar speed."""
    return min(1.0, (abs(v) + OMEGA_WEIGHT * abs(omega)) / V_REF)


def stride_frequency(v: float, omega: float) -> float:
    """Stride frequency in Hz; 0 at standstill so the phase freezes."""
    s = amplitude_scale(v, omega)
    if s <= 1e-3:
        return 0.0
    return F_MIN + (F_MAX - F_MIN) * s


def clamp_haa(pose: dict[str, float]) -> dict[str, float]:
    """Clamp HAA joints to hardware limits in place; returns the same dict."""
    for j, (lo, hi) in HAA_LIMITS.items():
        pose[j] = min(hi, max(lo, pose[j]))
    return pose


def trot_pose(phase: float, v: float, omega: float) -> dict[str, float]:
    """12 joint targets for gait phase [rad] at speed (v, omega).

    Swing legs (sin(leg_phase) > 0) lift by flexing the knee further from
    stand while the hip swings fore-aft; the lift term is >= 0 so feet only
    ever rise from the stand pose (they never dig below ground).
    """
    s = amplitude_scale(v, omega)
    pose: dict[str, float] = {}
    for leg in ('LF', 'RF', 'LH', 'RH'):
        lp = phase + _PAIR_PHASE[leg]
        kfe0 = STAND_POSE[f'{leg}_KFE']
        hind = leg in ('LH', 'RH')
        swing_sign = -1.0 if hind else 1.0
        lift = max(0.0, math.sin(lp))
        pose[f'{leg}_HAA'] = STAND_POSE[f'{leg}_HAA']
        pose[f'{leg}_HFE'] = STAND_POSE[f'{leg}_HFE'] + swing_sign * HFE_SWING * s * math.sin(lp)
        pose[f'{leg}_KFE'] = kfe0 + math.copysign(1.0, kfe0) * KFE_LIFT * s * lift
    return clamp_haa(pose)
