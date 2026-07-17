"""Analytic-chain FK + damped Gauss-Newton IK for the four ANYmal C legs.

Pure python, no ROS/numpy. The chain anchors are the zero-config link poses
copied verbatim from model.sdf (locked by test_chain_matches_sdf). FK composes
T = A_hip * R(haa) * inv(A_hip)*A_thigh * R(hfe) * inv(A_thigh)*A_shank * R(kfe)
and maps the SHANK-frame foot-centre offset to the base frame. IK is a damped
Gauss-Newton on the 3x3 finite-difference Jacobian with warm start — no
hand-derived closed form, so no sign-convention bugs; correctness is anchored
to FK, which is anchored to the SDF.
"""
import math

LEGS = ('LF', 'RF', 'LH', 'RH')
JOINTS = [f'{leg}_{j}' for leg in LEGS for j in ('HAA', 'HFE', 'KFE')]

HAA_LIMITS = {'LF': (-0.72, 0.49), 'RF': (-0.49, 0.72),
              'LH': (-0.72, 0.49), 'RH': (-0.49, 0.72)}
FOOT_BALL_R = 0.03

# (hip pose, thigh pose, shank pose, (haa,hfe,kfe) x-axis signs, foot centre in shank frame)
_CHAIN = {
    'LF': ((0.2999, 0.104, 0.0, 2.61799, 0.0, 0.0),
           (0.3598, 0.18781, 0.0, 0.0, 0.0, 1.5708),
           (0.3598, 0.28811, -0.285, 0.0, 0.0, 1.5708),
           (1, 1, 1), (0.01305, -0.08795, -0.31547)),
    'RF': ((0.2999, -0.104, 0.0, -2.61799, 0.0, 0.0),
           (0.3598, -0.18781, 0.0, 0.0, 0.0, -1.5708),
           (0.3598, -0.28811, -0.285, 0.0, 0.0, -1.5708),
           (1, -1, -1), (0.01305, 0.08795, -0.31547)),
    'LH': ((-0.2999, 0.104, 0.0, -2.61799, 0.0, 3.14159),
           (-0.3598, 0.18781, 0.0, 0.0, 0.0, 1.5708),
           (-0.3598, 0.28811, -0.285, 0.0, 0.0, 1.5708),
           (-1, 1, 1), (0.01305, 0.08795, -0.31547)),
    'RH': ((-0.2999, -0.104, 0.0, 2.61799, 0.0, 3.14159),
           (-0.3598, -0.18781, 0.0, 0.0, 0.0, -1.5708),
           (-0.3598, -0.28811, -0.285, 0.0, 0.0, -1.5708),
           (-1, -1, -1), (0.01305, -0.08795, -0.31547)),
}


def _rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mat4(R, t):
    return [[R[0][0], R[0][1], R[0][2], t[0]],
            [R[1][0], R[1][1], R[1][2], t[1]],
            [R[2][0], R[2][1], R[2][2], t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _mmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _minv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [T[i][3] for i in range(3)]
    ti = [-(R[i][0] * t[0] + R[i][1] * t[1] + R[i][2] * t[2]) for i in range(3)]
    return _mat4(R, ti)


def _rotx4(sign, q):
    a = sign * q
    c, s = math.cos(a), math.sin(a)
    return _mat4([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], [0.0, 0.0, 0.0])


_PRE = {}
for _leg, (_hip, _thigh, _shank, _axes, _foot) in _CHAIN.items():
    _A1 = _mat4(_rpy_mat(*_hip[3:]), _hip[:3])
    _A2 = _mat4(_rpy_mat(*_thigh[3:]), _thigh[:3])
    _A3 = _mat4(_rpy_mat(*_shank[3:]), _shank[:3])
    _PRE[_leg] = (_A1, _mmul(_minv(_A1), _A2), _mmul(_minv(_A2), _A3), _axes, _foot)


def leg_fk(leg, q):
    """Foot-centre position (x, y, z) in the base frame for joint angles q=(haa, hfe, kfe)."""
    A1, L12, L23, axes, foot = _PRE[leg]
    M = _mmul(_mmul(_mmul(_mmul(A1, _rotx4(axes[0], q[0])), L12), _rotx4(axes[1], q[1])),
              _mmul(L23, _rotx4(axes[2], q[2])))
    return (M[0][0] * foot[0] + M[0][1] * foot[1] + M[0][2] * foot[2] + M[0][3],
            M[1][0] * foot[0] + M[1][1] * foot[1] + M[1][2] * foot[2] + M[1][3],
            M[2][0] * foot[0] + M[2][1] * foot[1] + M[2][2] * foot[2] + M[2][3])


def _solve3(J, e, lam=1e-9):
    A = [[sum(J[k][i] * J[k][j] for k in range(3)) + (lam if i == j else 0.0)
          for j in range(3)] for i in range(3)]
    b = [sum(J[k][i] * e[k] for k in range(3)) for i in range(3)]

    def det(M):
        return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))

    d = det(A)
    if abs(d) < 1e-14:
        return [0.0, 0.0, 0.0]
    out = []
    for c in range(3):
        B = [row[:] for row in A]
        for i in range(3):
            B[i][c] = b[i]
        out.append(det(B) / d)
    return out


def leg_ik(leg, target, seed, max_iters=30, tol=1e-8):
    """Damped Gauss-Newton IK. Returns (q, residual_m). Never raises; an
    unreachable target converges to the closest reachable point and reports
    the leftover residual — the caller decides what residual is acceptable."""
    q = list(seed)
    err = float('inf')
    for _ in range(max_iters):
        p = leg_fk(leg, q)
        e = [target[i] - p[i] for i in range(3)]
        err = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        if err < tol:
            break
        h = 1e-6
        J = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            dq = q[:]
            dq[i] += h
            pd = leg_fk(leg, dq)
            for k in range(3):
                J[k][i] = (pd[k] - p[k]) / h
        step = _solve3(J, e)
        for i in range(3):
            q[i] += step[i]
    p = leg_fk(leg, q)
    err = math.sqrt(sum((target[i] - p[i]) ** 2 for i in range(3)))
    return q, err
