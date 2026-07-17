# Legged Locomotion (真实四腿物理行走) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The ANYmal C dog walks the maze on genuinely physical legs — gravity on, all 54 collisions restored, VelocityControl deleted, 12 torque-mode PID joint controllers driven by an in-repo CPG trot + numeric IK + attitude feedback — and completes the maze autonomously (online_slam, EXIT_REACHED, oracle 0.000%, zero falls).

**Architecture:** gz-side: force-PID JointPositionControllers track joint position targets at physics rate (1 ms). ROS-side: a pure-python `legged/` package (kinematics / trajectory / trot FSM / stabilizer / params — zero ROS deps, fully unit-testable) plus a thin `locomotion_controller` node (100 Hz) that maps /cmd_vel → 12 joint targets over the existing two-sided-named bridge topics. Nav stack frozen; /odom stays simulator ground truth (upgraded to dimensions=3 for attitude feedback).

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim 8.11 (ODE), pure-python math (no numpy dependency in the new modules), pytest, existing tugbot_maze/tugbot_gazebo/tugbot_bringup packages.

**Spec:** `docs/superpowers/specs/2026-07-17-legged-locomotion-design.md`

---

## Global context for every task (read first)

- **Repo root** = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate` (git repo). The workspace for THIS phase is `ros2_ws_tugbot_nav_20260717/` inside it (created in Task 1 as an rsync clone of `ros2_ws_tugbot_nav_20260716/`). All relative paths below are inside the new workspace unless prefixed `REPO/`.
- **Branch**: all work on branch `legged-locomotion` (created in Task 1). Never commit to main.
- **Build**: `colcon build --symlink-install` is MANDATORY (plain build breaks the maze_sim data path). Source with `set +u` in effect — ROS setup.bash references unset vars.
- **Pure-module tests**: `cd ros2_ws_tugbot_nav_20260717 && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q` (the `:$PYTHONPATH` suffix is required). Environment has `python3` only.
- **Commits**: message via heredoc `git commit -F - <<'EOF' ... EOF`; NO backticks inside commit messages; end with the trailer line `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- **No foreground `sleep`** in your own tool calls (sleep inside executed script files is fine). Never launch GUI Gazebo (`gz sim` without `-s`); headless `gz sim -s` is fine.
- **Baseline suite state** (inherited from 20260716): 397 passed / 7 pre-existing failed / 1 xfailed. The 7 failures are pre-existing phase tests, not yours. `test_gait.py` (10 of the 397) is retired in Task 6.
- **Hard-won gz traps that apply here** (from memory, all verified last phase):
  1. JointPositionController has NO `<topic>` key; its command topic is auto-derived as `/model/anymal_c/joint/<JOINT>/0/cmd_pos` (note the `/0`). This is independent of force-vs-velocity mode — the existing bridge yaml (ROS side without `/0`, gz side with `/0`) keeps working unchanged.
  2. ROS 2 rejects digit-leading topic tokens — never create a ROS-side topic containing `/0/`.
  3. OdometryPublisher is WORLD-anchored (odom = absolute world pose, not spawn-zeroed). The solver's entrance anchor (launch `entrance_x/y` = 11.011/9.025) already handles frame registration. Do NOT add `<xyz_offset>` (it composes body-frame post-multiplied and generates teleports).
  4. (Retired this phase, for awareness): gravity-off + collisions accumulated attitude impulses. Gravity is ON now, so this trap no longer applies; z/roll/pitch have restoring forces (legs + ground).

### Locked numeric facts (derived from the SDF, verified numerically — do not re-derive)

Kinematic chain anchors (model-frame zero-config link poses, from `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf`, identical in the workspace `model.sdf` which kept all link poses):

| leg | HIP pose (xyz rpy) | THIGH pose | SHANK pose | joint axes (HAA,HFE,KFE) x-signs | foot centre in SHANK frame |
|---|---|---|---|---|---|
| LF | 0.2999 0.104 0 / 2.61799 0 0 | 0.3598 0.18781 0 / 0 0 1.5708 | 0.3598 0.28811 -0.285 / 0 0 1.5708 | +1, +1, +1 | 0.01305 -0.08795 -0.31547 |
| RF | 0.2999 -0.104 0 / -2.61799 0 0 | 0.3598 -0.18781 0 / 0 0 -1.5708 | 0.3598 -0.28811 -0.285 / 0 0 -1.5708 | +1, -1, -1 | 0.01305 0.08795 -0.31547 |
| LH | -0.2999 0.104 0 / -2.61799 0 3.14159 | -0.3598 0.18781 0 / 0 0 1.5708 | -0.3598 0.28811 -0.285 / 0 0 1.5708 | -1, +1, +1 | 0.01305 0.08795 -0.31547 |
| RH | -0.2999 -0.104 0 / 2.61799 0 3.14159 | -0.3598 -0.18781 0 / 0 0 -1.5708 | -0.3598 -0.28811 -0.285 / 0 0 -1.5708 | -1, -1, -1 | 0.01305 -0.08795 -0.31547 |

- Foot ball radius 0.03. HAA limits: LF/LH (-0.72, 0.49); RF/RH (-0.49, 0.72). Effort limit 80 Nm, velocity 7.5 rad/s (all 12).
- X-stance (old initial_position 0/±0.4/∓0.8) foot centres: (±0.4527, ±0.3012, −0.5188) — foot-bottom drop 0.5488. **Note: the 20260716 footprint's assumed stance x 0.3598 was actually the HFE pivot; the real X-stance foot x is 0.4527.** This phase therefore uses a deliberately NARROWER neutral stance (below) so the honest dynamic footprint stays maze-compatible.
- **Neutral stance** (this phase's stand pose): foot centres (±0.34, ±0.3012, −0.50). IK-exact joint values: front legs (HAA 0, HFE 0.705, KFE −0.9608); hind legs (HAA 0, HFE −0.705, KFE 0.9608). Standing base height ≈ 0.53 (0.50 + 0.03 ball).
- **Gait numbers**: f_trot 2.0 Hz, duty 0.5, swing lift 0.07, vx_max 0.4, wz_max 0.5. Foot-envelope bound: x = 0.34 + (0.4 + 0.5·0.3012)·0.5/(2·2.0) + 0.03 = 0.439; y = 0.3012 + 0.5·0.34·0.5/(2·2.0) + 0.03 = 0.353. Footprint constants become **FOOT_X_FRONT 0.49 / FOOT_X_REAR −0.49 / FOOT_HALF_W 0.37** (margin over the bound; 0.37 also covers the true static lateral extremity 0.3012+0.03=0.3312 which the old 0.32 understated).
- **Spec deviations locked in this plan** (record in the spec addendum in Task 10): (a) OdometryPublisher `dimensions` 2→3 (spec said "不动"; needed because dimensions=2 zeroes z/roll/pitch which the stabilizer + fall detection require); (b) FOOT_HALF_W 0.32→0.37 (spec's "unchanged 0.32" missed the yaw-stride lateral excursion and the true 0.3312 static stance); (c) initial_position/STAND_POSE = neutral stance (0, ±0.705, ∓0.9608) replacing X-stance (narrower honest envelope); (d) `hop_command` default v_max 0.5→0.4 (spec §6 velocity alignment).

---

### Task 1: New workspace + branch + baseline

**Files:**
- Create: `REPO/ros2_ws_tugbot_nav_20260717/` (rsync clone)

- [ ] **Step 1: Create branch (at repo root)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b legged-locomotion
```

- [ ] **Step 2: Clone the workspace**

```bash
rsync -a --exclude build --exclude install --exclude log \
  ros2_ws_tugbot_nav_20260716/ ros2_ws_tugbot_nav_20260717/
```

- [ ] **Step 3: Build**

```bash
cd ros2_ws_tugbot_nav_20260717
bash -c 'set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install' 2>&1 | tail -5
```
Expected: `Summary: X packages finished` with 0 failures.

- [ ] **Step 4: Baseline test run**

```bash
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
```
Expected: `7 failed, 397 passed, 1 xfailed` (the 7 are pre-existing phase tests — record their names for the final gate).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717
git commit -F - <<'EOF'
chore: clone workspace 20260716 -> 20260717 for the legged-locomotion phase

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: `legged/kinematics.py` — pure-python FK + damped Gauss-Newton IK

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/legged/__init__.py` (empty)
- Create: `src/tugbot_maze/tugbot_maze/legged/kinematics.py`
- Test: `src/tugbot_maze/test/test_legged_kinematics.py`

- [ ] **Step 1: Write the failing tests**

```python
"""Tests for the ANYmal C leg FK/IK. The FK chain anchors are locked to the
SDF by test_chain_matches_sdf, so any drift between code constants and the
model is caught here, not in Gazebo."""
import math
import re
from pathlib import Path

import pytest

from tugbot_maze.legged.kinematics import (
    LEGS, JOINTS, HAA_LIMITS, FOOT_BALL_R, _CHAIN, leg_fk, leg_ik,
)


def _model_sdf_text():
    # workspace layout: <ws>/src/tugbot_maze/test/  ->  <ws>/src/tugbot_description/...
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def test_chain_matches_sdf():
    """Chain anchor constants must equal the model.sdf zero-config link poses."""
    sdf = _model_sdf_text()
    for leg in LEGS:
        hip, thigh, shank, _axes, _foot = _CHAIN[leg]
        for link, expect in ((f'{leg}_HIP', hip), (f'{leg}_THIGH', thigh), (f'{leg}_SHANK', shank)):
            m = re.search(rf'<link name="{link}">\s*<pose[^>]*>([^<]+)</pose>', sdf)
            got = [float(v) for v in m.group(1).split()]
            assert got == pytest.approx(list(expect), abs=1e-5), link


def test_fk_x_stance_matches_measured():
    """X-stance FK verified against the numeric derivation of 2026-07-17."""
    xst = {'LF': (0, 0.4, -0.8), 'RF': (0, 0.4, -0.8),
           'LH': (0, -0.4, 0.8), 'RH': (0, -0.4, 0.8)}
    expect = {'LF': (0.4527, 0.3012, -0.5188), 'RF': (0.4527, -0.3012, -0.5188),
              'LH': (-0.4527, 0.3012, -0.5188), 'RH': (-0.4527, -0.3012, -0.5188)}
    for leg in LEGS:
        assert leg_fk(leg, xst[leg]) == pytest.approx(expect[leg], abs=2e-4)


def test_ik_fk_roundtrip():
    for leg in LEGS:
        for q in [(0.1, 0.5, -0.9), (-0.2, 0.7, -1.2), (0.0, 0.3, -0.6)]:
            # mirror the sagittal signs for hind legs (their HFE/KFE bend the other way)
            qq = q if leg in ('LF', 'RF') else (q[0], -q[1], -q[2])
            target = leg_fk(leg, qq)
            sol, err = leg_ik(leg, target, seed=qq)
            assert err < 1e-6
            assert leg_fk(leg, sol) == pytest.approx(target, abs=1e-5)


def test_ik_warm_start_converges_fast():
    near = (0.36, 0.3012, -0.48)   # a step-sized hop from the stand pose
    _sol, err = leg_ik('LF', near, seed=(0.0, 0.705, -0.9608))
    assert err < 1e-6


def test_ik_unreachable_reports_error():
    _sol, err = leg_ik('LF', (1.5, 0.3, -0.5), seed=(0.0, 0.705, -0.9608))
    assert err > 0.5   # far outside the ~0.6 m leg reach: residual reported, no exception


def test_mirror_symmetry():
    p = leg_fk('LF', (0.1, 0.6, -1.0))
    pr = leg_fk('RF', (-0.1, 0.6, -1.0))
    assert pr == pytest.approx((p[0], -p[1], p[2]), abs=1e-9)


def test_joints_order_and_limits():
    assert JOINTS[:3] == ['LF_HAA', 'LF_HFE', 'LF_KFE'] and len(JOINTS) == 12
    assert HAA_LIMITS['LF'] == (-0.72, 0.49) and HAA_LIMITS['RF'] == (-0.49, 0.72)
```

Remove the stray `SDF = ...` placeholder line when writing the file (only `_model_sdf_text` is used).

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_kinematics.py -q`
Expected: FAIL / collection error with `ModuleNotFoundError: tugbot_maze.legged`

- [ ] **Step 3: Implement `legged/kinematics.py`**

```python
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
```

Also create the empty `src/tugbot_maze/tugbot_maze/legged/__init__.py`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_kinematics.py -q`
Expected: `7 passed`

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717/src/tugbot_maze
git commit -F - <<'EOF'
feat: legged/kinematics — SDF-anchored FK + damped GN IK for the 4 legs

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: `legged/params.py` + `legged/trajectory.py`

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/legged/params.py`
- Create: `src/tugbot_maze/tugbot_maze/legged/trajectory.py`
- Test: `src/tugbot_maze/test/test_legged_trajectory.py`

- [ ] **Step 1: Write the failing tests**

```python
import math

import pytest

from tugbot_maze.legged.params import (
    LeggedParams, NEUTRAL_FOOT, STAND_POSE, foot_envelope,
)
from tugbot_maze.legged.kinematics import LEGS, leg_ik
from tugbot_maze.legged.trajectory import foot_displacement

P = LeggedParams()


def test_stand_pose_is_ik_of_neutral_foot():
    for leg in LEGS:
        seed = (STAND_POSE[f'{leg}_HAA'], STAND_POSE[f'{leg}_HFE'], STAND_POSE[f'{leg}_KFE'])
        q, err = leg_ik(leg, NEUTRAL_FOOT[leg], seed=seed)
        assert err < 1e-6
        assert q == pytest.approx(list(seed), abs=1e-3)


def test_zero_cmd_steps_in_place():
    """Zero command: never any xy stride; the swing z-lift still applies so
    that force_trot steps in place (ladder rung 2). The FSM's STAND mode is
    what holds the feet fully still — not the trajectory."""
    for leg in LEGS:
        for ph in (0.0, 1.0, 2.0, 4.0):
            d = foot_displacement(leg, ph, 0.0, 0.0, P)
            assert d[0] == 0.0 and d[1] == 0.0
            assert 0.0 <= d[2] <= P.swing_lift + 1e-12


def test_stance_swing_continuity():
    """Displacement is continuous at the stance->swing and swing->stance boundaries."""
    for ph_lo, ph_hi in ((P.duty * 2 * math.pi - 1e-6, P.duty * 2 * math.pi + 1e-6),
                         (2 * math.pi - 1e-6, 1e-6)):
        a = foot_displacement('LF', ph_lo, 0.3, 0.2, P)
        b = foot_displacement('LF', ph_hi, 0.3, 0.2, P)
        assert a == pytest.approx(b, abs=1e-3)


def test_swing_lifts_stance_stays_down():
    mid_stance = foot_displacement('LF', P.duty * math.pi, 0.3, 0.0, P)
    mid_swing = foot_displacement('LF', (1 + P.duty) * math.pi, 0.3, 0.0, P)
    assert mid_stance[2] == 0.0
    assert mid_swing[2] == pytest.approx(P.swing_lift, abs=1e-9)


def test_stance_moves_backward_under_forward_cmd():
    early = foot_displacement('LF', 0.1, 0.4, 0.0, P)
    late = foot_displacement('LF', P.duty * 2 * math.pi - 0.1, 0.4, 0.0, P)
    assert early[0] > late[0]          # foot drifts backward relative to body
    assert early[1] == late[1] == 0.0  # no lateral component for pure vx


def test_yaw_cmd_gives_tangential_strides():
    dlf = foot_displacement('LF', 0.0, 0.0, 0.5, P)
    drh = foot_displacement('RH', 0.0, 0.0, 0.5, P)
    # LF at (+0.34,+0.3012): tangent (-wz*ny, wz*nx) = (-0.1506, +0.17); RH mirrored
    assert dlf[0] < 0 and dlf[1] > 0
    assert drh[0] > 0 and drh[1] < 0


def test_envelope_bound_values():
    ex, ey = foot_envelope()
    assert ex == pytest.approx(0.439, abs=5e-3)
    assert ey == pytest.approx(0.353, abs=5e-3)


def test_all_targets_reachable_across_cycle():
    """Every commanded foot target over a full cycle at max cmd is IK-reachable."""
    for leg in LEGS:
        seed = (STAND_POSE[f'{leg}_HAA'], STAND_POSE[f'{leg}_HFE'], STAND_POSE[f'{leg}_KFE'])
        for i in range(24):
            ph = 2 * math.pi * i / 24
            d = foot_displacement(leg, ph, P.vx_max, P.wz_max, P)
            n = NEUTRAL_FOOT[leg]
            _q, err = leg_ik(leg, (n[0] + d[0], n[1] + d[1], n[2] + d[2]), seed=seed)
            assert err < 1e-4, (leg, ph)
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_trajectory.py -q`
Expected: `ModuleNotFoundError: tugbot_maze.legged.params`

- [ ] **Step 3: Implement `legged/params.py`**

```python
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
# Also mirrored into the 12 <initial_position> values in model.sdf.
STAND_POSE = {
    'LF_HAA': 0.0, 'LF_HFE': 0.705, 'LF_KFE': -0.9608,
    'RF_HAA': 0.0, 'RF_HFE': 0.705, 'RF_KFE': -0.9608,
    'LH_HAA': 0.0, 'LH_HFE': -0.705, 'LH_KFE': 0.9608,
    'RH_HAA': 0.0, 'RH_HFE': -0.705, 'RH_KFE': 0.9608,
}


@dataclass(frozen=True)
class LeggedParams:
    stand_height: float = 0.50     # |neutral foot z|; standing base height = this + ball
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
```

- [ ] **Step 4: Implement `legged/trajectory.py`**

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_trajectory.py -q`
Expected: `8 passed`

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717/src/tugbot_maze
git commit -F - <<'EOF'
feat: legged params + stance/swing foot trajectory (envelope-bounded)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: `legged/stabilizer.py` + `legged/trot.py` (FSM + 12-joint assembly)

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/legged/stabilizer.py`
- Create: `src/tugbot_maze/tugbot_maze/legged/trot.py`
- Test: `src/tugbot_maze/test/test_legged_trot.py`

- [ ] **Step 1: Write the failing tests**

```python
import math

import pytest

from tugbot_maze.legged.params import LeggedParams, STAND_POSE
from tugbot_maze.legged.stabilizer import height_offsets
from tugbot_maze.legged.trot import PAIR_PHASE, LocomotionFSM

P = LeggedParams()


def test_stabilizer_signs():
    """+roll = left side up = left legs too long -> raise left foot targets (dz>0)."""
    dz = height_offsets(0.1, 0.0, P)
    assert dz['LF'] > 0 and dz['LH'] > 0 and dz['RF'] < 0 and dz['RH'] < 0
    # +pitch = nose down = front legs too short -> extend front (dz<0)
    dz = height_offsets(0.0, 0.1, P)
    assert dz['LF'] < 0 and dz['RF'] < 0 and dz['LH'] > 0 and dz['RH'] > 0


def test_stabilizer_clamped():
    dz = height_offsets(1.0, -1.0, P)
    assert all(abs(v) <= P.dz_max + 1e-12 for v in dz.values())


def test_pair_phase_diagonal():
    assert PAIR_PHASE['LF'] == PAIR_PHASE['RH'] == 0.0
    assert PAIR_PHASE['RF'] == PAIR_PHASE['LH'] == pytest.approx(math.pi)


def _run(fsm, seconds, vx, wz, dt=0.01, roll=0.0, pitch=0.0, force_trot=False):
    out = None
    for _ in range(int(seconds / dt)):
        out = fsm.step(dt, vx, wz, roll, pitch, force_trot=force_trot)
    return out


def test_init_holds_stand_pose_then_stand():
    fsm = LocomotionFSM()
    out = fsm.step(0.01, 0.4, 0.0, 0.0, 0.0)
    assert fsm.mode == fsm.INIT
    assert out == pytest.approx(STAND_POSE, abs=1e-9)
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    assert fsm.mode == fsm.STAND


def test_stand_trot_hysteresis():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    _run(fsm, 1.0, 0.3, 0.0)
    assert fsm.mode == fsm.TROT
    # window = slew-down time (0.3/ax_max) + quiet dwell + margin: the dwell
    # clock only starts once the slewed cmd decays below the half-threshold
    _run(fsm, 0.3 / P.ax_max + P.stand_dwell_s + 0.3, 0.0, 0.0)
    assert fsm.mode == fsm.STAND


def test_force_trot_diagnostic():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    _run(fsm, 0.2, 0.0, 0.0, force_trot=True)
    assert fsm.mode == fsm.TROT


def test_trot_outputs_all_joints_within_limits():
    from tugbot_maze.legged.kinematics import JOINTS, HAA_LIMITS
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    for _ in range(200):
        out = fsm.step(0.01, P.vx_max, P.wz_max, 0.02, -0.02)
        assert sorted(out) == sorted(JOINTS)
        for leg, (lo, hi) in HAA_LIMITS.items():
            assert lo - 1e-9 <= out[f'{leg}_HAA'] <= hi + 1e-9
        assert all(math.isfinite(v) for v in out.values())


def test_slew_limits_cmd():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    fsm.step(0.01, 10.0, 10.0, 0.0, 0.0)   # absurd cmd
    assert abs(fsm.vx) <= P.ax_max * 0.011
    assert abs(fsm.wz) <= P.aw_max * 0.011
    for _ in range(500):
        fsm.step(0.01, 10.0, 10.0, 0.0, 0.0)
    assert abs(fsm.vx) <= P.vx_max + 1e-9 and abs(fsm.wz) <= P.wz_max + 1e-9


def test_stand_applies_attitude_feedback():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    flat = dict(fsm.step(0.01, 0.0, 0.0, 0.0, 0.0))
    rolled = dict(fsm.step(0.01, 0.0, 0.0, 0.2, 0.0))
    assert rolled != pytest.approx(flat)   # feedback changes the pose
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_trot.py -q`
Expected: `ModuleNotFoundError: tugbot_maze.legged.stabilizer`

- [ ] **Step 3: Implement `legged/stabilizer.py`**

```python
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
```

- [ ] **Step 4: Implement `legged/trot.py`**

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_legged_trot.py src/tugbot_maze/test/test_legged_kinematics.py src/tugbot_maze/test/test_legged_trajectory.py -q`
Expected: all pass (9 + 7 + 8 = 24)

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717/src/tugbot_maze
git commit -F - <<'EOF'
feat: legged trot FSM + attitude stabilizer (12-joint target assembly)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 5: Physical model — collisions, gravity, torque-PID controllers, 1 ms worlds

**Files:**
- Create: `tools/make_legged_model.py` (one-shot, auditable surgery script)
- Modify: `src/tugbot_description/models/anymal_c/model.sdf` (script output)
- Modify: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf` (physics 1 ms, spawn z)
- Modify: `src/tugbot_gazebo/worlds/anymal_test_world.sdf` (physics 1 ms, spawn z)

- [ ] **Step 1: Write the surgery script `tools/make_legged_model.py`**

```python
#!/usr/bin/env python3
"""One-shot surgery: turn the kinematic-ghost anymal_c model.sdf into the
physical one. Reads the CURRENT workspace model.sdf + the ORIGINAL CERBERUS
model.sdf, writes the workspace file in place. Asserts every count so a
partial application cannot slip through. Run from the workspace root:

    python3 tools/make_legged_model.py
"""
import re
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
CUR = WS / 'src/tugbot_description/models/anymal_c/model.sdf'
ORIG = WS.parent / 'tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf'

# initial_position = STAND_POSE (legged/params.py); order matches the 12 plugin blocks
STAND = {
    'LF_HAA': 0.0, 'LF_HFE': 0.705, 'LF_KFE': -0.9608,
    'RF_HAA': 0.0, 'RF_HFE': 0.705, 'RF_KFE': -0.9608,
    'LH_HAA': 0.0, 'LH_HFE': -0.705, 'LH_KFE': 0.9608,
    'RH_HAA': 0.0, 'RH_HFE': -0.705, 'RH_KFE': 0.9608,
}
LINKS = ['base'] + [f'{l}_{part}' for l in ('LF', 'RF', 'LH', 'RH')
                    for part in ('HIP', 'THIGH', 'SHANK')]
FRICTION = ('<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
            '</surface>')


def link_block(text, name):
    m = re.search(rf'(<link name="{name}">.*?</link>)', text, re.S)
    assert m, f'link {name} not found'
    return m.group(1)


def main():
    cur = CUR.read_text()
    orig = ORIG.read_text()

    # 1. collisions: copy every <collision> block of each link from the original
    total = 0
    for name in LINKS:
        ob = link_block(orig, name)
        cols = re.findall(r'<collision name=.*?</collision>', ob, re.S)
        assert cols, f'no collisions in original link {name}'
        total += len(cols)
        block = '\n      '.join(cols)
        cb = link_block(cur, name)
        assert '<collision' not in cb, f'link {name} already has collisions'
        cur = cur.replace(cb, cb.replace('</link>', f'  {block}\n    </link>'))
    assert total == 54, f'expected 54 collisions, copied {total}'

    # 2. foot friction: FOOT sphere + cylinder collisions get explicit mu
    n_friction = 0
    def add_friction(m):
        nonlocal n_friction
        n_friction += 1
        return m.group(0).replace('</collision>', f'  {FRICTION}\n        </collision>')
    cur = re.sub(r'<collision name="[^"]*FOOT[^"]*">.*?</collision>', add_friction, cur, flags=re.S)
    assert n_friction == 8, f'expected 8 FOOT collisions to get friction, got {n_friction}'

    # 3. gravity on
    cur, n = re.subn(r'\s*<gravity>false</gravity>', '', cur)
    assert n == 13, f'expected 13 gravity tags, removed {n}'

    # 4. delete VelocityControl plugin
    cur, n = re.subn(
        r'\s*<plugin filename="gz-sim-velocity-control-system".*?</plugin>', '', cur, flags=re.S)
    assert n == 1, 'VelocityControl block not found'

    # 5. OdometryPublisher dimensions 2 -> 3 (stabilizer + fall detection need z/roll/pitch)
    cur, n = re.subn(r'<dimensions>2</dimensions>', '<dimensions>3</dimensions>', cur)
    assert n == 1, 'odometry dimensions tag not found'

    # 6. controllers: velocity-command mode -> torque PID with gains
    def controller(joint):
        return (
            '<plugin filename="gz-sim-joint-position-controller-system" '
            'name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>{joint}</joint_name>\n'
            '      <p_gain>250.0</p_gain>\n'
            '      <i_gain>0.0</i_gain>\n'
            '      <d_gain>5.0</d_gain>\n'
            '      <cmd_max>80.0</cmd_max>\n'
            '      <cmd_min>-80.0</cmd_min>\n'
            f'      <initial_position>{STAND[joint]}</initial_position>\n'
            '    </plugin>')
    n_ctl = 0
    def replace_ctl(m):
        nonlocal n_ctl
        n_ctl += 1
        joint = re.search(r'<joint_name>([^<]+)</joint_name>', m.group(0)).group(1)
        return controller(joint)
    cur = re.sub(
        r'<plugin filename="gz-sim-joint-position-controller-system".*?</plugin>',
        replace_ctl, cur, flags=re.S)
    assert n_ctl == 12, f'expected 12 controller blocks, rewrote {n_ctl}'
    for joint, val in STAND.items():
        assert f'<joint_name>{joint}</joint_name>' in cur, joint

    # final sanity (friction was added INSIDE existing blocks, so still 54 tags)
    assert cur.count('<collision') == 54
    assert 'use_velocity_commands' not in cur
    assert 'VelocityControl' not in cur
    CUR.write_text(cur)
    print(f'OK: 54 collisions, 8 foot frictions, gravity on, 12 torque-PID controllers, dims=3')


if __name__ == '__main__':
    sys.exit(main())
```

Note on the final sanity count: `cur.count('<collision')` counts opening tags only — friction sits inside existing blocks, so 54 is the right expectation. Keep the assert as `assert cur.count('<collision') == 54`.

- [ ] **Step 2: Run the script**

Run: `python3 tools/make_legged_model.py`
Expected: `OK: 54 collisions, 8 foot frictions, gravity on, 12 torque-PID controllers, dims=3`

Verify by hand:
```bash
grep -c '<collision' src/tugbot_description/models/anymal_c/model.sdf   # 54
grep -c 'gravity' src/tugbot_description/models/anymal_c/model.sdf      # 0
grep -c 'p_gain' src/tugbot_description/models/anymal_c/model.sdf       # 12
grep -c 'mu>1.0' src/tugbot_description/models/anymal_c/model.sdf       # 16 (mu+mu2 x8)
grep 'dimensions' src/tugbot_description/models/anymal_c/model.sdf      # <dimensions>3</dimensions>
```

- [ ] **Step 3: Worlds — 1 ms physics + spawn z**

In BOTH `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf` and `src/tugbot_gazebo/worlds/anymal_test_world.sdf`:

- `<max_step_size>0.003</max_step_size>` → `<max_step_size>0.001</max_step_size>` (maze world block is named `3ms` — rename to `1ms`)
- anymal_c include pose z: maze world `-11.011 -9.025 0.580 ...` → `-11.011 -9.025 0.62 0 0 0`; test world `0 0 0.58 0 0 0` → `0 0 0.62 0 0 0`.

Rationale: joints spawn at 0 (legs near-straight, feet at z −0.6005); z=0.62 puts the feet ~0.02 above ground, the PID folds the legs to STAND_POSE while the body drops ~7 cm onto its feet — the INIT settle absorbs this. If Step 4 shows bounce or tip-over, tune z in [0.56, 0.66] (this is the spec's "实测微调").

- [ ] **Step 4: Rebuild + headless stand smoke (ladder rung 1)**

```bash
bash -c 'set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install' 2>&1 | tail -3
```

Write and run a small smoke script `tools/smoke_stand.sh`:

```bash
#!/usr/bin/env bash
# Rung-1 smoke: spawn in the test world, settle 8 sim-s, assert standing.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f "gz sim" 2>/dev/null; sleep 1
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > /tmp/smoke_stand_gz.log 2>&1 &
GZ_PID=$!
sleep 12   # ~8 sim-s at RTF<=1 plus startup
POSE=$(timeout 10 gz topic -e -n 1 -t /model/anymal_c/odometry)
kill -9 $GZ_PID 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null
echo "$POSE"
python3 - "$POSE" <<'EOF'
import math, re, sys
t = sys.argv[1]
z = float(re.search(r'position\s*{[^}]*z:\s*([-\d.e]+)', t, re.S).group(1))
m = re.search(r'orientation\s*{\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)\s*z:\s*([-\d.e]+)\s*w:\s*([-\d.e]+)', t, re.S)
x, y, zz, w = map(float, m.groups())
roll = math.atan2(2*(w*x + y*zz), 1 - 2*(x*x + y*y))
pitch = math.asin(max(-1, min(1, 2*(w*y - zz*x))))
print(f'z={z:.3f} roll={math.degrees(roll):.1f}deg pitch={math.degrees(pitch):.1f}deg')
assert 0.42 <= z <= 0.60, f'not standing: z={z}'
assert abs(math.degrees(roll)) < 5 and abs(math.degrees(pitch)) < 5, 'tilted'
print('STAND SMOKE PASS')
EOF
```

Run: `bash tools/smoke_stand.sh`
Expected: `STAND SMOKE PASS` with z ≈ 0.53, |roll|,|pitch| < 5°.
Note: the odometry gz topic name — confirm with `gz topic -l | grep odom` if `/model/anymal_c/odometry` differs. If the model bounces or tips, tune p_gain/d_gain (250/5 start; try 300/8, then 200/4) and spawn z per Step 3 before escalating. Also record RTF here: `timeout 10 gz topic -e -t /stats 2>/dev/null | grep -m 3 real_time_factor` (or the world-scoped stats topic from `gz topic -l | grep stats`) — write the observed RTF into the commit message body.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717
git commit -F - <<'EOF'
feat: physical anymal_c — 54 collisions, gravity on, torque-PID joints, 1ms worlds

VelocityControl deleted; OdometryPublisher dimensions=3; foot friction mu=1.0;
initial_position = neutral stand pose; stand smoke passes (z~0.53, RTF noted).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: `locomotion_controller` node; retire gait_animator

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/locomotion_controller.py`
- Delete: `src/tugbot_maze/tugbot_maze/gait.py`, `src/tugbot_maze/tugbot_maze/gait_animator.py`, `src/tugbot_maze/test/test_gait.py`
- Modify: `src/tugbot_maze/setup.py` (console_scripts: replace gait_animator entry)
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` (Node swap)
- Modify: `tools/run_flood_fill_maze.sh` (kill list only — rest of wrapper changes in Task 9)

- [ ] **Step 1: Write `locomotion_controller.py`**

```python
"""Real legged locomotion: /cmd_vel -> trot FSM -> 12 joint position targets.

Replaces the 20260716 gait_animator (which was a visual-only animation over a
magic-force base). This node IS the base motion now: if it stops stepping the
dog stops. Still guarded per-tick — an exception holds the last targets and
logs, it never crashes the node.

Fall detection lives here too (odom is dimensions=3 now): |roll| or |pitch|
> fall_attitude_rad, or base z < fall_z_m, sustained fall_hold_s, logs one
FALL_DETECTED line (the run wrapper greps it) and freezes the gait in STAND.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from tugbot_maze.legged.kinematics import JOINTS
from tugbot_maze.legged.trot import LocomotionFSM


def _roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        rate_hz = self.declare_parameter('rate_hz', 100.0).value
        model = self.declare_parameter('model_name', 'anymal_c').value
        self.declare_parameter('force_trot', False)
        self._fall_att = self.declare_parameter('fall_attitude_rad', 0.6).value
        self._fall_z = self.declare_parameter('fall_z_m', 0.25).value
        self._fall_hold = self.declare_parameter('fall_hold_s', 1.0).value
        # ROS-side topics have no /0 joint-index token (illegal in ROS names);
        # the bridge maps them onto gz .../joint/<J>/0/cmd_pos.
        self._pubs = {
            j: self.create_publisher(Float64, f'/model/{model}/joint/{j}/cmd_pos', 10)
            for j in JOINTS
        }
        self._fsm = LocomotionFSM()
        self._cmd = (0.0, 0.0)
        self._att = (0.0, 0.0)
        self._z = None
        self._odom_t = None          # sim time (s) of last odom
        self._viol_since = None
        self._fallen = False
        self._last_out = None
        self._dt = 1.0 / max(float(rate_hz), 1.0)
        self._diag_count = 0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'locomotion_controller started (model={model}, rate={1.0 / self._dt:.0f} Hz).')

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_odom(self, msg):
        self._att = _roll_pitch(msg.pose.pose.orientation)
        self._z = msg.pose.pose.position.z
        self._odom_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._check_fall()

    def _check_fall(self):
        if self._fallen or self._odom_t is None:
            return
        roll, pitch = self._att
        bad = abs(roll) > self._fall_att or abs(pitch) > self._fall_att or self._z < self._fall_z
        if not bad:
            self._viol_since = None
            return
        if self._viol_since is None:
            self._viol_since = self._odom_t
        elif self._odom_t - self._viol_since >= self._fall_hold:
            self._fallen = True
            self.get_logger().error(
                f'FALL_DETECTED roll={roll:.2f} pitch={pitch:.2f} z={self._z:.2f} '
                f't={self._odom_t:.1f}')

    def _tick(self):
        try:
            if self._fallen:
                vx, wz = 0.0, 0.0
            else:
                vx, wz = self._cmd
            force_trot = bool(self.get_parameter('force_trot').value)
            roll, pitch = self._att
            out = self._fsm.step(self._dt, vx, wz, roll, pitch,
                                 force_trot=force_trot and not self._fallen)
            self._last_out = out
        except Exception as exc:   # never crash the control loop
            self.get_logger().warning(f'locomotion tick failed: {exc}')
            out = self._last_out
        if out is None:
            return
        for joint, angle in out.items():
            msg = Float64()
            msg.data = float(angle)
            self._pubs[joint].publish(msg)
        self._diag_count += 1
        if self._diag_count >= int(1.0 / self._dt):   # ~1 Hz
            self._diag_count = 0
            roll, pitch = self._att
            self.get_logger().info(
                f'LOCO mode={self._fsm.mode} phase={self._fsm.phase:.2f} '
                f'vx={self._fsm.vx:.2f} wz={self._fsm.wz:.2f} '
                f'roll={roll:.3f} pitch={pitch:.3f} z={self._z if self._z is None else round(self._z, 3)}')


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Retire the animation layer**

```bash
git rm ros2_ws_tugbot_nav_20260717/src/tugbot_maze/tugbot_maze/gait.py \
       ros2_ws_tugbot_nav_20260717/src/tugbot_maze/tugbot_maze/gait_animator.py \
       ros2_ws_tugbot_nav_20260717/src/tugbot_maze/test/test_gait.py
```

In `src/tugbot_maze/setup.py` console_scripts replace:
```python
            'gait_animator = tugbot_maze.gait_animator:main',
```
with:
```python
            'locomotion_controller = tugbot_maze.locomotion_controller:main',
```

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` replace the gait_animator Node block (and its entry in the returned actions list — keep it unconditional, started at launch t=0 so INIT settle finishes before the solver's 13 s timer):
```python
    locomotion_controller = Node(
        package='tugbot_maze', executable='locomotion_controller', name='locomotion_controller',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'model_name': 'anymal_c',
        }],
        output='screen',
    )
```
and change `gait_animator,` to `locomotion_controller,` in the LaunchDescription list.

In `tools/run_flood_fill_maze.sh` kill list change `flood_fill_solver gait_animator \` to `flood_fill_solver locomotion_controller \`.

- [ ] **Step 3: Rebuild + full pure-module suite**

```bash
bash -c 'set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install' 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
```
Expected: build clean; suite = baseline minus the 10 retired gait tests plus 24 new legged tests → `7 failed, 411 passed, 1 xfailed`.

- [ ] **Step 4: Headless closed-loop smoke (node + sim)**

Write `tools/smoke_walk.sh` (temporary — superseded by the Task 8 ladder; still commit it, the ladder reuses its skeleton):

```bash
#!/usr/bin/env bash
# Closed-loop smoke: gz server + bridge + locomotion_controller; drive 0.3 m/s
# for 10 sim-s; assert forward displacement > 1.5 m and no fall.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f locomotion_controller 2>/dev/null; sleep 1
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > /tmp/smoke_walk_gz.log 2>&1 &
sleep 6
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$WS/install/tugbot_gazebo/share/tugbot_gazebo/config/tugbot_bridge.yaml \
  > /tmp/smoke_walk_bridge.log 2>&1 &
ros2 run tugbot_maze locomotion_controller --ros-args -p use_sim_time:=true \
  > /tmp/smoke_walk_loco.log 2>&1 &
sleep 6    # INIT settle
python3 - <<'EOF'
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = Node('smoke_walk_check', parameter_overrides=[])
poses = []
n.create_subscription(Odometry, '/odom', lambda m: poses.append(
    (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time()
tw = Twist(); tw.linear.x = 0.3
while time.time() - t0 < 25 and rclpy.ok():   # ~10+ sim-s even at RTF~0.5
    pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(Twist())
assert poses, 'no odom received'
dx = poses[-1][0] - poses[0][0]
zs = [p[2] for p in poses[len(poses)//2:]]
print(f'dx={dx:.2f} z_range=({min(zs):.2f},{max(zs):.2f}) n={len(poses)}')
assert dx > 1.5, f'walked only {dx:.2f} m'
assert min(zs) > 0.30, 'body collapsed'
print('WALK SMOKE PASS')
EOF
RC=$?
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f locomotion_controller
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
exit $RC
```

Run: `bash tools/smoke_walk.sh`
Expected: `WALK SMOKE PASS` (dx > 1.5 m). **This is the first genuinely walking milestone.** If it fails, tune in this order and document each attempt: (1) gz PID gains in model.sdf (p 200–350, d 4–10); (2) `LeggedParams.f_trot` 1.6–2.5 / `swing_lift` 0.05–0.09; (3) `kp_att` 0.5–1.0. If dx > 0 but < 1.5, note the tracking ratio and continue tuning; if the dog falls repeatedly, STOP and escalate with the smoke logs (per BLOCKED protocol).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add -A ros2_ws_tugbot_nav_20260717
git commit -F - <<'EOF'
feat: locomotion_controller node (cmd_vel -> trot -> joints); retire gait_animator

Closed-loop walk smoke passes: dog walks on its legs in the test world.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: Honest footprint 0.49/0.37 + suite re-derivation + solver speed alignment

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/footprint.py`
- Modify: `src/tugbot_maze/tugbot_maze/hop_controller.py` (hop_command v_max default 0.5 → 0.4)
- Modify: `src/tugbot_maze/test/test_footprint.py` (re-derive + new consistency test)
- Modify: `src/tugbot_maze/test/test_maze_sim.py`, `src/tugbot_maze/test/test_maze_motion_sim.py` (re-derive)

- [ ] **Step 1: Add the failing consistency test to `test_footprint.py`**

```python
def test_footprint_covers_gait_envelope():
    """The static footprint rectangle must dominate the worst-case dynamic
    foot reach (stance + stride + ball) — this ties footprint.py to
    legged/params.py so gait retuning cannot silently break oracle honesty."""
    from tugbot_maze.legged.params import foot_envelope
    ex, ey = foot_envelope()
    assert footprint.FOOT_X_FRONT >= ex
    assert -footprint.FOOT_X_REAR >= ex
    assert footprint.FOOT_HALF_W >= ey
```

Run: `PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_footprint.py -q` — the new test FAILS (0.39 < 0.439).

- [ ] **Step 2: Update `footprint.py` constants + docstring**

```python
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
```
(functions below the constants stay byte-identical)

- [ ] **Step 3: hop_command speed alignment**

In `src/tugbot_maze/tugbot_maze/hop_controller.py` line ~142, change `hop_command(pose, target_xy, *, v_max: float = 0.5, ...)` to `v_max: float = 0.4`. (This is the only solver-side change; corridor/centering defaults 0.3/0.4 are already within the gait's 0.4 clamp; w_max 0.5 matches wz_max exactly.)

- [ ] **Step 4: Re-derive footprint-dependent test expectations**

Process (the 20260716 0.35→0.39 re-derivation precedent): grep the test tree for every literal derived from 0.39/0.32:

```bash
grep -rn '0\.39\|0\.32\|0\.6245\|0\.71\|hypot' src/tugbot_maze/test/test_footprint.py src/tugbot_maze/test/test_maze_sim.py | head -40
```

For each: recompute from 0.49/0.37 with the derivation written next to the assert. Known anchors: bounding circle `hypot(0.49, 0.37) = 0.6140`, so old `hypot(0.39,0.32)+0.12 = 0.6245` becomes `0.6140+0.12 = 0.7340`; corner cases move from (±0.39, ±0.32) to (±0.49, ±0.37). Do NOT weaken any assertion logic — only re-derive the numbers.

- [ ] **Step 5: Run the full suite; triage the offline motion sim**

```bash
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -15
```

`test_maze_motion_sim.py` cases may change behaviour (bigger stop envelope + hop v_max 0.4). Triage rules:
- A case that still solves but grazes ≤ 1 cm in the drift+latency stress tiers, where the equivalent tugbot-era case grazed: `xfail(strict=False)` with the 2026-06-26 precedent citation (the existing `[0.05-3]` xfail stays).
- A case that FAILS TO SOLVE (stall/timeout, not graze): STOP — do not xfail, do not weaken. Escalate with the case name and its event trace (this decides gate-vs-envelope retuning, a controller decision).
- Target end state: `7 failed (pre-existing), 412+ passed, xfails each individually justified`.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717/src/tugbot_maze
git commit -F - <<'EOF'
feat: honest dynamic footprint 0.49/0.37 + envelope consistency lock + hop v_max 0.4

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---### Task 8: Verification ladder (rungs 1–4) as one harness

**Files:**
- Create: `tools/verify_legged_walk.sh` (boots sim+bridge+node once, runs all rungs)
- Create: `tools/legged_walk_check.py` (rclpy checker, one rung per invocation)
- Delete: `tools/smoke_walk.sh` (superseded; keep `smoke_stand.sh`)

- [ ] **Step 1: Write `tools/legged_walk_check.py`**

```python
#!/usr/bin/env python3
"""One verification-ladder rung against a live sim (booted by
verify_legged_walk.sh). Usage: legged_walk_check.py --rung {stand|step|walk|turn}
Exit 0 = PASS. Asserts also that attitude never exceeds fall thresholds."""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node


class Rung(Node):
    def __init__(self):
        super().__init__('legged_walk_check')
        self.samples = []           # (sim_t, x, y, z, roll, pitch, yaw)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.samples.append((t, p.x, p.y, p.z, roll, pitch, yaw))

    def sim_span(self):
        return self.samples[-1][0] - self.samples[0][0] if len(self.samples) > 1 else 0.0

    def drive(self, vx, wz, sim_seconds, wall_cap=120.0):
        """Publish cmd until sim_seconds of sim time elapse (RTF-independent)."""
        self.samples.clear()
        tw = Twist(); tw.linear.x = float(vx); tw.angular.z = float(wz)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < wall_cap:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.sim_span() >= sim_seconds:
                break
        self.pub.publish(Twist())
        assert self.sim_span() >= sim_seconds, \
            f'sim advanced only {self.sim_span():.1f}s in {wall_cap}s wall'

    def set_force_trot(self, on):
        cli = self.create_client(SetParameters, '/locomotion_controller/set_parameters')
        assert cli.wait_for_service(timeout_sec=20.0), 'locomotion param service missing'
        req = SetParameters.Request()
        req.parameters = [Parameter(name='force_trot', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(on)))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        assert fut.result() is not None


def _attitude_ok(samples, limit=0.6):
    return all(abs(s[4]) < limit and abs(s[5]) < limit for s in samples)


def _unwrap(yaws):
    out = [yaws[0]]
    for y in yaws[1:]:
        d = y - out[-1]
        while d > math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rung', required=True, choices=['stand', 'step', 'walk', 'turn'])
    args = ap.parse_args()
    rclpy.init()
    n = Rung()

    if args.rung == 'stand':
        n.drive(0.0, 0.0, 5.0)
        zs = [s[3] for s in n.samples]
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert 0.42 <= min(zs) and max(zs) <= 0.60, f'z out of band ({min(zs):.2f},{max(zs):.2f})'
        assert _attitude_ok(n.samples, 0.09), 'attitude > 5deg while standing'
        assert drift < 0.05, f'stand drift {drift:.3f} m'
        print(f'RUNG stand PASS z=({min(zs):.3f},{max(zs):.3f}) drift={drift:.3f}')

    elif args.rung == 'step':
        n.set_force_trot(True)
        n.drive(0.0, 0.0, 10.0)
        n.set_force_trot(False)
        zs = [s[3] for s in n.samples]
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert _attitude_ok(n.samples), 'fell while stepping in place'
        assert max(zs) - min(zs) < 0.06, f'z oscillation {max(zs)-min(zs):.3f}'
        assert drift < 0.30, f'step-in-place drifted {drift:.2f} m'
        print(f'RUNG step PASS z_osc={max(zs)-min(zs):.3f} drift={drift:.2f}')

    elif args.rung == 'walk':
        n.drive(0.3, 0.0, 10.0)
        d = math.hypot(n.samples[-1][1] - n.samples[0][1], n.samples[-1][2] - n.samples[0][2])
        assert _attitude_ok(n.samples), 'fell while walking'
        assert d >= 2.4, f'tracked only {d:.2f} m of 3.0 commanded'
        print(f'RUNG walk PASS dist={d:.2f} ({d/3.0*100:.0f}% tracking)')

    elif args.rung == 'turn':
        n.drive(0.0, 0.5, 8.0)
        yaw = _unwrap([s[6] for s in n.samples])
        turned = abs(yaw[-1] - yaw[0])
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert _attitude_ok(n.samples), 'fell while turning'
        assert turned >= 3.2, f'turned only {turned:.2f} rad of 4.0 commanded'
        assert drift < 0.40, f'turn-in-place drifted {drift:.2f} m'
        print(f'RUNG turn PASS yaw={turned:.2f} rad drift={drift:.2f}')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Write `tools/verify_legged_walk.sh`**

```bash
#!/usr/bin/env bash
# Verification ladder rungs 1-4 (spec section 8). Boots gz server + bridge +
# locomotion_controller once, then runs stand -> step -> walk -> turn,
# aborting on the first failure. Maze (rung 5) is run via run_flood_fill_maze.sh.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))

cleanup() {
  pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f locomotion_controller 2>/dev/null
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
cleanup; sleep 1

LOG="$(mktemp -d /tmp/legged_ladder.XXXXXX)"
echo "ladder logs: $LOG"
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > "$LOG/gz.log" 2>&1 &
sleep 6
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$WS/install/tugbot_gazebo/share/tugbot_gazebo/config/tugbot_bridge.yaml" \
  > "$LOG/bridge.log" 2>&1 &
ros2 run tugbot_maze locomotion_controller --ros-args -p use_sim_time:=true \
  > "$LOG/loco.log" 2>&1 &
sleep 8   # INIT settle done

for rung in stand step walk turn; do
  echo "=== RUNG $rung ==="
  if ! python3 tools/legged_walk_check.py --rung "$rung"; then
    echo "LADDER FAIL at rung $rung (logs: $LOG)"; cleanup; exit 1
  fi
done
echo "LADDER PASS (all 4 rungs)"
cleanup
```

- [ ] **Step 3: Run the ladder; tune until all 4 rungs pass**

Run: `bash tools/verify_legged_walk.sh`
Expected: `LADDER PASS (all 4 rungs)`.

Tuning discipline (same order as Task 6 Step 4, one variable per experiment, log each attempt in the commit body): gz PID gains → f_trot/swing_lift → kp_att/dz_max → spawn z. If ODE contact chatter is visible in gz.log or z oscillates > 0.06 while standing, raise ODE solver iterations in BOTH world files: inside `<physics>` add `<ode><solver><iters>150</iters></solver></ode>` as the escalation step. If a rung still fails after a documented sweep, STOP and escalate (BLOCKED) with `$LOG`.

- [ ] **Step 4: Delete the superseded smoke and commit**

```bash
git rm ros2_ws_tugbot_nav_20260717/tools/smoke_walk.sh
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add -A ros2_ws_tugbot_nav_20260717
git commit -F - <<'EOF'
feat: 4-rung legged verification ladder (stand/step/walk/turn) all passing

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 9: Maze integration — wrapper upgrades + 2 headless full runs

**Files:**
- Modify: `tools/run_flood_fill_maze.sh` (FALL_DETECTED result, LOCO in tail grep)
- Test: two full headless maze runs + oracle + teleport + MATCH health

- [ ] **Step 1: Wrapper upgrades**

In `tools/run_flood_fill_maze.sh`:

1. In the RESULT loop after the EXIT_REACHED check, add:
```bash
    if grep -qa "FALL_DETECTED" "$ART/launch.log" 2>/dev/null; then RESULT="FALL_DETECTED"; break; fi
```
2. Extend the tail grep pattern `EXIT_REACHED|HOP_BACKUP|JUNCTION|DIAG|SENSE|flood_fill_solver` with `|LOCO|FALL_DETECTED`.
3. Header comment: note the legged default budget (walking ~0.35 m/s + RTF < 1 → invoke with 3600).

- [ ] **Step 2: First full maze run**

```bash
cd ros2_ws_tugbot_nav_20260717
bash tools/run_flood_fill_maze.sh 3600 true false online_slam
```
Expected: `result=EXIT_REACHED` (wall-clock budget 3600 s; the wrapper prints ARTIFACT_DIR).

- [ ] **Step 3: Post-run validation (repeat for every run)**

```bash
ART=<ARTIFACT_DIR from the run>
# 1. official collision oracle
python3 tools/replay_collision_oracle.py "$ART"          # expect rate=0.000%
# 2. teleport check (DIAG jumps > 2 m within 5 s)
python3 - "$ART/launch.log" <<'EOF'
import math, re, sys
poses = [(float(m.group(1)), float(m.group(2))) for m in
         re.finditer(r'DIAG pose=\(([-\d.]+), ([-\d.]+)', open(sys.argv[1]).read())]
jumps = [(i, math.hypot(b[0]-a[0], b[1]-a[1])) for i, (a, b) in enumerate(zip(poses, poses[1:]))
         if math.hypot(b[0]-a[0], b[1]-a[1]) > 2.0]
print(f'{len(poses)} DIAG poses, teleports: {jumps[:5]}')
assert not jumps, 'TELEPORT DETECTED'
EOF
# 3. ICP health (healthy: n~600, rms~0.02, fb=-)
grep -a 'MATCH' "$ART/launch.log" | tail -5
# 4. fall/stability + events
grep -ac 'FALL_DETECTED' "$ART/launch.log" || true       # expect 0
grep -aE 'ESCAPE|UNSTICK' "$ART/flood_fill_tail.txt" || true
# 5. RTF + attitude spot-check from LOCO lines
grep -a 'LOCO' "$ART/launch.log" | tail -3
```
Pass bar per spec §8: EXIT_REACHED, oracle 0.000%, 0 falls, 0 teleports, MATCH n in the healthy hundreds.

- [ ] **Step 4: Debug loop authorization**

This is the phase's integration campaign (the 20260716 Task-8 equivalent). If the run fails: use systematic debugging — instrument, form ONE hypothesis, test it decisively, document in the spec addendum. Known-likely failure classes and first probes:
- **Fell in a corridor**: check LOCO roll/pitch trend before the fall + whether a wall contact preceded it (real collision = oracle + scan); consider kp_att/dz_max, then gains.
- **ICP degrades (MATCH n collapses) while walking**: body pitch tilts the lidar plane; check LOCO pitch amplitude; reduce swing_lift/f_trot before touching the sensor mount (z 0.35→0.25 is the last resort — it changes launch TF too).
- **Solver stalls (no progress, hop retries)**: tracking ratio too low for the motion layer's tolerances; measure achieved v from DIAG poses; consider raising motion-layer arrival tolerances (config constants only — the one sanctioned nav change class).
- **TIMEOUT at 3600 wall**: check RTF (LOCO lines vs wall clock); rerun with 5400 rather than tuning anything, and note the RTF.

- [ ] **Step 5: Second full run (repeatability)**

Repeat Steps 2–3. Expected: same pass bar. Two consecutive passing runs = rung 5 complete.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260717
git commit -F - <<'EOF'
feat: legged maze integration — wrapper fall detection; 2 headless EXIT_REACHED runs

oracle 0.000 percent, 0 falls, 0 teleports, ICP healthy on both runs.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 10: Docs + spec addendum + final gate

**Files:**
- Modify: `README.md` (workspace `ros2_ws_tugbot_nav_20260717/README.md`)
- Modify: `REPO/docs/superpowers/specs/2026-07-17-legged-locomotion-design.md` (addendum)

- [ ] **Step 1: Spec addendum**

Append an `## 附记(实现期决策,2026-07-17)` section to the spec recording, with one line of rationale each: (a) OdometryPublisher dimensions 2→3; (b) FOOT_HALF_W 0.32→0.37 and why the spec's "不变" was wrong (yaw-stride lateral + true static 0.3312); (c) neutral stance (±0.34, ±0.3012, −0.50) with STAND_POSE (0, ±0.705, ∓0.9608) replacing X-stance as initial_position, and the 0.4527-vs-0.3598 X-stance discovery that motivated it; (d) hop_command v_max 0.5→0.4; (e) any tuning deltas from Tasks 6/8/9 (final PID gains, params, ODE iters, spawn z) and the measured RTF; (f) anything the debug loop (Task 9 Step 4) changed.

- [ ] **Step 2: README update**

Update the workspace README's robot section: the dog now physically walks (gravity on, 54 collisions, torque-PID joints, trot controller `legged/` + `locomotion_controller`); footprint 0.49/0.37; ladder + wrapper usage (`bash tools/verify_legged_walk.sh`; maze budget 3600); FALL_DETECTED semantics.

- [ ] **Step 3: Final suite + build gate**

```bash
cd ros2_ws_tugbot_nav_20260717
bash -c 'set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install' 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -5
```
Expected: build clean; `7 failed (identical pre-existing list from Task 1), 412+ passed, xfails individually justified`. Compare the failed-test NAMES against the Task 1 baseline — any new name = regression, fix before proceeding.

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add -A
git commit -F - <<'EOF'
docs: legged-locomotion README + spec addendum (implementation decisions)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

## After all tasks

Process (not plan tasks): final whole-change review (most capable model), then GUI acceptance — wait for the user's explicit "set up the run" before any GUI launch (`export DISPLAY=:1 && bash tools/run_flood_fill_maze.sh 3600 false true online_slam` from the workspace). Merge/push/cleanup via superpowers:finishing-a-development-branch after the user's acceptance.
