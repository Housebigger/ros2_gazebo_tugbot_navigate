"""Analytic Go2 kinematics in the base frame, from the official Unitree URDF."""
import math

LEGS = ('LF', 'RF', 'LH', 'RH')
JOINTS = [f'{leg}_{axis}' for leg in LEGS for axis in ('HAA', 'HFE', 'KFE')]
HAA_LIMITS = {leg: (-1.0472, 1.0472) for leg in LEGS}
FOOT_BALL_R = 0.022
HIP_X, HIP_Y, THIGH_OFFSET, LENGTH = 0.1934, 0.0465, 0.0955, 0.213


def _hip(leg):
    return (HIP_X if leg in ('LF', 'RF') else -HIP_X,
            HIP_Y if leg in ('LF', 'LH') else -HIP_Y, 0.0)


def _offset(leg):
    return THIGH_OFFSET if leg in ('LF', 'LH') else -THIGH_OFFSET


def leg_fk(leg, q):
    roll, thigh, knee = q
    hx, hy, hz = _hip(leg)
    lateral = _offset(leg)
    x = -LENGTH * (math.sin(thigh) + math.sin(thigh+knee))
    z = -LENGTH * (math.cos(thigh) + math.cos(thigh+knee))
    return (hx+x, hy+lateral*math.cos(roll)-z*math.sin(roll),
            hz+lateral*math.sin(roll)+z*math.cos(roll))


def leg_ik(leg, target, seed=None):
    """Use the Go2's negative-knee branch and report any unreachable residual."""
    x,y,z = (p-h for p,h in zip(target,_hip(leg)))
    lateral = _offset(leg)
    sagittal_z = -math.sqrt(max(1e-9,y*y+z*z-lateral*lateral))
    extension = min(1.0-1e-9,math.hypot(x,sagittal_z)/(2*LENGTH))
    roll = math.remainder(math.atan2(z,y)-math.atan2(sagittal_z,lateral),math.tau)
    knee = -2*math.acos(extension)
    thigh = math.atan2(-x,-sagittal_z)-knee/2
    q = [roll,thigh,knee]
    return q,math.dist(leg_fk(leg,q),target)
