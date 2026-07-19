# Yaw-Only Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新 workspace `ros2_ws_tugbot_nav_20260721` 给 `OnlineScanMatchLocalizer` 加 1-DOF yaw-only 回退:全 ICP 被门控/拒配时仅解转角(位置保持 odom 先验),抽走中央路口陷阱的"站立 yaw 偏差"燃料;headless ×8 零坏跑统计门。

**Architecture:** 单文件改动(`online_scan_match_localizer.py`):模块级纯函数 `yaw_only_correct` 复用底层 ICP 的 `_beams_to_points`/`_associate` 残差机制,±0.2 rad 网格搜索(0.01 步)最优 dθ,三重守门(inliers≥60、rms 改善≥20%、单 tick 钳位 0.1 rad);`correct()` 全部拒配路径统一先试 yaw-only。观测通道零 solver 改动:info 带 `'fell_back': {'yaw_only'}`,既有 MATCH 日志行自动显示 `fb=yaw_only`。

**Tech Stack:** 纯 NumPy(既有依赖)、pytest + MazeSim 合成射线、Gazebo headless 统计门(PRIME 自带)。

**Spec:** `docs/superpowers/specs/2026-07-19-yaw-only-fallback-design.md`

---

## 全局约束(每任务适用)

- `$REPO` = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`;`$WS` = `$REPO/ros2_ws_tugbot_nav_20260721`(克隆源 **20260719**;20260720 名字属停放分支勿用)。
- `colcon build --symlink-install`;显式 `cd`;`set +u` 后 source;只有 `python3`;直接调用禁前台 `sleep`;禁自动 GUI。
- ⚠️ pkill/pgrep 含 "gz sim"/"ros2 launch" 模式:脚本文件 + 写/执行分两次调用。
- git:heredoc、无反引号、trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`;显式 add 路径。
- 套件(在 `$WS`):`PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`;基线 **7 failed / 424 passed / 3 xfailed**,名单存档对照。
- **解冻仅限 `online_scan_match_localizer.py`**:`tugbot_maze/tugbot_maze/` 下任何其他文件被改动即缺陷(scan_match_localizer.py、flood_fill_solver.py、maze_motion.py 一律不动)。既有测试破坏 → BLOCKED,不许改既有测试(若遇被取代合同,先例是上报后由控制器授权)。

## 已核实的现状事实(实现前必读)

- `OnlineScanMatchLocalizer.correct()`(行 ~171-193):Gate 1 `sparse_interior_gate`、Gate 2 `no_local_interior_walls` 各自 `return prior_pose, {'rejected': True, ...}`;Gate 3 `masked = self._mask_far_beams(...)` 后 `return self._icp.correct(prior_pose, masked, ...)`——内层 ICP 自己可能拒配(under_inliers 等 reason)。
- 底层 `ScanMatchLocalizer`(scan_match_localizer.py)可复用件:`_beams_to_points(pose, ranges, angle_min, angle_inc) -> (N,2)` 与 `_associate(pts) -> (foot, n, dist)`(dist 为各点到最近段距离;空参考/空点时返回 None——实现前读 75-95 行确认返回约定)。对应距离阈值:ctor kwarg `max_corr_dist_m=0.30`——实现前确认它存储在哪个属性(读 24-58 行与 correct() 的用法),yaw-only 复用同一阈值。
- solver 对 info 只 `.get` 打日志(flood_fill_solver.py 384-390),`rejected` 无决策语义;MATCH 行 `fb=` 打印 `','.join(sorted(i.get('fell_back', set())))`——info 放 `'fell_back': {'yaw_only'}` 即可零改动观测。
- 测试惯例:`MazeSim(segments, (x, y), yaw)` + `sim.scan(n_beams=360, fov_rad=2*math.pi)` 返回 `(ranges, amin, ainc)`(test_maze_motion.py 用法);`load_segments()` 可得全迷宫段,测试也可手造周界矩形段列表。
- 陷阱统计基线:修前 2/5 坏、双信号门后 2/6 坏(~30% 触发率);8 连清概率在 30% 率下 <6%。

---

### Task 1: 新 workspace 20260721 + 分支 + 基线

**Files:** Create: `ros2_ws_tugbot_nav_20260721/`(克隆 20260719)

- [ ] **Step 1-5**(与前四阶段 Task 1 完全同构,替换名字):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b yaw-only-fallback
cp -a ros2_ws_tugbot_nav_20260719 ros2_ws_tugbot_nav_20260721
rm -rf ros2_ws_tugbot_nav_20260721/build ros2_ws_tugbot_nav_20260721/install ros2_ws_tugbot_nav_20260721/log
find ros2_ws_tugbot_nav_20260721 -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
find ros2_ws_tugbot_nav_20260721 -name '.pytest_cache' -type d -exec rm -rf {} + 2>/dev/null
```

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort > /tmp/yawonly_baseline_failures.txt
wc -l /tmp/yawonly_baseline_failures.txt
```

Expected: 6 packages 成功;`7 failed, 424 passed, 3 xfailed`;7 行。不符 BLOCKED。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260721
git status --short | grep -v '^A' | head -3
git commit -F - <<'EOF'
chore: clone workspace 20260719 -> 20260721 for the yaw-only-fallback phase

Byte-identical copy (build artifacts and caches stripped; 20260720 is
the parked yaw-freshness-gate branch's workspace, skipped). Baseline
suite: 7 pre-existing failures / 424 passed / 3 xfailed, names archived.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: yaw_only_correct 纯函数(TDD)

**Files:**
- Test: `ros2_ws_tugbot_nav_20260721/src/tugbot_maze/test/test_yaw_only_fallback.py`(新)
- Modify: `ros2_ws_tugbot_nav_20260721/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py`

- [ ] **Step 1: 写失败测试**

新建 `test_yaw_only_fallback.py`:

```python
"""1-DOF yaw-only fallback: when full ICP is gated (junction cells have
almost no walls -> deterministic dead reckoning), wall DIRECTION is still
observable at range - even from the perimeter alone. The fallback corrects
yaw only; x,y must be bit-identical to the prior on EVERY path (a 1-DOF
fit structurally cannot reproduce the 2026-06 two-axis position corruption)."""
import math

import pytest

from tugbot_maze.maze_sim import MazeSim
from tugbot_maze.online_scan_match_localizer import (
    YAW_MIN_IMPROVE, YAW_MIN_INLIERS, YAW_STEP_CLAMP, YAW_WINDOW_RAD,
    OnlineScanMatchLocalizer, yaw_only_correct,
)

# 20x20 m perimeter rectangle (matches the maze scale); interior EMPTY so
# OnlineScanMatchLocalizer's sparse-interior gate always fires -> fallback path.
PERIM = [(0.0, 0.0, 20.0, 0.0), (20.0, 0.0, 20.0, 20.0),
         (20.0, 20.0, 0.0, 20.0), (0.0, 20.0, 0.0, 0.0)]
POSE_TRUE = (7.0, 9.0, 0.9)     # off-center, arbitrary heading


def _scan_at(pose):
    sim = MazeSim(PERIM, (pose[0], pose[1]), pose[2])
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def _localizer():
    return OnlineScanMatchLocalizer(PERIM)


def _icp_of(loc):
    return loc._icp     # the underlying ScanMatchLocalizer holding the reference


@pytest.mark.parametrize('bias', [0.05, 0.15])
def test_recovers_in_window_bias_and_freezes_xy(bias):
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    prior = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + bias)
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is False and 'yaw_only' in info['fell_back']
    assert est[0] == prior[0] and est[1] == prior[1]          # x,y bit-identical
    resid = math.atan2(math.sin(est[2] - POSE_TRUE[2]), math.cos(est[2] - POSE_TRUE[2]))
    assert abs(resid) < 0.02, f'bias {bias} -> residual {resid}'


def test_overwindow_bias_steps_clamped_toward_truth():
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    prior = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + 0.3)   # beyond the 0.2 window
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is False
    step = est[2] - prior[2]
    assert step == pytest.approx(-YAW_STEP_CLAMP, abs=1e-6)    # full clamp, correct sign
    assert est[0] == prior[0] and est[1] == prior[1]


def test_declines_when_scan_unrelated_to_reference():
    # Beams all shorter than any wall distance (robot "sees" phantom close
    # clutter the reference cannot explain) -> too few inliers -> decline.
    prior = (10.0, 10.0, 0.0)
    ranges = [0.6] * 360
    amin, ainc = -math.pi, 2 * math.pi / 360
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is True and 'yaw_only_declined' in info['reason']
    assert est == prior


def test_declines_when_no_improvement_available():
    # Prior yaw already exact: best-case rms improvement ~0 < 20% -> decline
    # (guards against noise-driven idle corrections).
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    loc = _localizer()
    est, info = yaw_only_correct(POSE_TRUE, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is True and 'yaw_only_declined' in info['reason']
    assert est == POSE_TRUE


def test_repeated_ticks_converge_without_oscillation():
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    pose = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + 0.28)
    loc = _localizer()
    resids = []
    for _ in range(6):
        est, info = yaw_only_correct(pose, ranges, amin, ainc, _icp_of(loc))
        pose = est
        resids.append(abs(math.atan2(math.sin(pose[2] - POSE_TRUE[2]),
                                     math.cos(pose[2] - POSE_TRUE[2]))))
    assert resids[-1] < 0.02
    assert all(resids[i + 1] <= resids[i] + 1e-9 for i in range(len(resids) - 1)), resids


def test_constants_exported_with_spec_values():
    assert YAW_WINDOW_RAD == pytest.approx(0.2)
    assert YAW_MIN_INLIERS == 60
    assert YAW_MIN_IMPROVE == pytest.approx(0.20)
    assert YAW_STEP_CLAMP == pytest.approx(0.1)
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_yaw_only_fallback.py -v
```

Expected: ImportError(yaw_only_correct/常量不存在)。若 MazeSim 构造或 scan 返回约定与假设不符(读 maze_sim.py 核实),修 harness 不修断言,记附记。

- [ ] **Step 3: 实现(online_scan_match_localizer.py 模块级)**

在 import 区之后加常量与函数(**复用底层 ICP 的对应阈值属性——实现前读 scan_match_localizer.py 24-58 行确认 max_corr_dist_m 存储属性名,下面以 `icp._max_corr` 占位表述,以实际为准**):

```python
# --- yaw-only fallback (2026-07-19 spec) -------------------------------------
# Junction cells have almost no walls, so the gates below deterministically
# reject full ICP there and yaw bias stands uncorrected through dead-reckoning
# windows (the central-junction trap's fuel). Wall DIRECTION is observable at
# range even where 2-axis position is not (the 2026-06 fundamental wall was
# position, never yaw), so when full ICP is unavailable we fit rotation ONLY.
# A 1-DOF fit cannot drag position; x,y pass through bit-identical.
YAW_WINDOW_RAD = 0.2      # search window for the residual scan
YAW_GRID_STEP = 0.01      # 0.57deg resolution; convergence target is 0.02 rad
YAW_MIN_INLIERS = 60      # beams that must land near reference at the optimum
YAW_MIN_IMPROVE = 0.20    # required relative rms improvement vs d_theta = 0
YAW_STEP_CLAMP = 0.1      # max applied per-tick correction (rad)


def yaw_only_correct(prior_pose, ranges, angle_min, angle_inc, icp, *,
                     window=YAW_WINDOW_RAD, grid=YAW_GRID_STEP,
                     min_inliers=YAW_MIN_INLIERS, min_improve=YAW_MIN_IMPROVE,
                     step_clamp=YAW_STEP_CLAMP):
    """1-DOF fallback: grid-search d_theta minimizing point-to-segment rms of
    the (premasked) beams against icp's reference. Position NEVER changes.
    Accept only if inliers >= min_inliers at the optimum AND rms improves by
    >= min_improve relative to d_theta=0 (improvement is 1.0 when the prior
    has no inliers at all). Applied step is clamped to +/-step_clamp."""
    x, y, yaw = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
    max_corr = icp._max_corr          # SAME correspondence threshold as full ICP

    def eval_at(dth):
        pts = icp._beams_to_points((x, y, yaw + dth), ranges, angle_min, angle_inc)
        if pts is None or len(pts) == 0:
            return math.inf, 0
        foot, n, dist = icp._associate(pts)
        if dist is None:
            return math.inf, 0
        mask = dist <= max_corr
        n_in = int(mask.sum())
        if n_in == 0:
            return math.inf, 0
        return float(np.sqrt(np.mean(dist[mask] ** 2))), n_in

    rms0, _n0 = eval_at(0.0)
    best_dth, best_rms, best_n = 0.0, rms0, _n0
    steps = int(round(window / grid))
    for k in range(-steps, steps + 1):
        dth = k * grid
        rms, n_in = eval_at(dth)
        if rms < best_rms:
            best_dth, best_rms, best_n = dth, rms, n_in
    improve = 1.0 if not math.isfinite(rms0) else (
        0.0 if rms0 <= 0.0 else 1.0 - (best_rms / rms0))
    if (not math.isfinite(best_rms) or best_n < min_inliers
            or improve < min_improve):
        return tuple(prior_pose), {'rejected': True,
                                   'reason': 'yaw_only_declined',
                                   'residual_rms': best_rms,
                                   'n_inliers': best_n}
    applied = max(-step_clamp, min(step_clamp, best_dth))
    return ((x, y, yaw + applied),
            {'rejected': False, 'reason': 'yaw_only',
             'fell_back': {'yaw_only'}, 'yaw_step': applied,
             'residual_rms': best_rms, 'n_inliers': best_n})
```

(`math`/`np` 已在文件 import。注意:测试 4"无改善拒绝"要求先验已精确时 decline——improve≈0 <0.20 自动满足。测试 3 要求 reason 含 `yaw_only_declined`,上面 reason 即是;Task 3 接线时会把门控原 reason 拼上。)

- [ ] **Step 4: 跑测试**——6 个用例中 test_declines_when_scan_unrelated / constants / recovers / overwindow / converge 应 PASS;涉及 correct() 接线的没有(本任务纯函数)。Expected: `6 passed`(测试全走 yaw_only_correct 直调)。若 `icp._max_corr` 属性名不对,按实际改并记附记。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_yaw_only_fallback.py -v
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
```

Expected: 6 passed;全套件 `7 failed, 430 passed, 3 xfailed`(+6)。

- [ ] **Step 5: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260721/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py ros2_ws_tugbot_nav_20260721/src/tugbot_maze/test/test_yaw_only_fallback.py
git commit -F - <<'EOF'
feat: yaw_only_correct - 1-DOF grid-search rotation fit (pure function)

Wall direction is observable at range even where 2-axis position is not;
this fits rotation only against the full reference (perimeter included),
triple-guarded (inliers, 20 percent rms improvement, 0.1 rad clamp), and
passes x,y through bit-identical on every path.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: correct() 接线(全部拒配路径)+ 既有测试无破坏

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260721/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py`(correct())
- Test: `test_yaw_only_fallback.py` 追加接线测试

- [ ] **Step 1: 追加失败测试**(接线级,走 OnlineScanMatchLocalizer.correct):

```python
def test_correct_falls_back_on_sparse_interior_gate():
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    prior = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + 0.15)
    loc = _localizer()                                    # interior EMPTY -> gate 1
    est, info = loc.correct(prior, ranges, amin, ainc, [])
    assert info['rejected'] is False and 'yaw_only' in info['fell_back']
    assert info['gate_reason'] == 'sparse_interior_gate'
    assert est[0] == prior[0] and est[1] == prior[1]
    resid = math.atan2(math.sin(est[2] - POSE_TRUE[2]), math.cos(est[2] - POSE_TRUE[2]))
    assert abs(resid) < 0.05


def test_correct_declined_fallback_reports_both_reasons():
    prior = (10.0, 10.0, 0.0)
    ranges = [0.6] * 360
    amin, ainc = -math.pi, 2 * math.pi / 360
    loc = _localizer()
    est, info = loc.correct(prior, ranges, amin, ainc, [])
    assert info['rejected'] is True
    assert 'sparse_interior_gate' in info['reason'] and 'yaw_only_declined' in info['reason']
    assert est == prior
```

- [ ] **Step 2: 确认红**(gate 1 路径现在直接返回纯先验,两个测试都 FAIL)。

- [ ] **Step 3: 实现接线**——`correct()` 改为(结构;门控 reason 保存进 `gate_reason`,decline 时拼接 reason):

```python
    def _yaw_fallback(self, prior_pose, ranges, angle_min, angle_inc, gate_reason):
        masked = self._mask_far_beams(prior_pose, ranges, angle_min, angle_inc)
        est, info = yaw_only_correct(prior_pose, masked, angle_min, angle_inc, self._icp)
        if info['rejected']:
            info['reason'] = gate_reason + '+yaw_only_declined'
        else:
            info['gate_reason'] = gate_reason
        return est, info

    def correct(self, prior_pose, ranges, angle_min, angle_inc, interior_segments):
        ...(签名/缓存重建不变)
        # Gate 1
        if len(interior_segments) < self._min_interior:
            return self._yaw_fallback(prior_pose, ranges, angle_min, angle_inc,
                                      'sparse_interior_gate')
        # Gate 2
        if not self._has_local_interior(prior_pose, interior_segments):
            return self._yaw_fallback(prior_pose, ranges, angle_min, angle_inc,
                                      'no_local_interior_walls')
        # Gate 3 + ICP
        masked = self._mask_far_beams(prior_pose, ranges, angle_min, angle_inc)
        est, info = self._icp.correct(prior_pose, masked, angle_min, angle_inc)
        if info.get('rejected'):
            inner = info.get('reason', 'icp_rejected')
            est2, info2 = yaw_only_correct(prior_pose, masked, angle_min,
                                           angle_inc, self._icp)
            if not info2['rejected']:
                info2['gate_reason'] = inner
                return est2, info2
            info['reason'] = str(inner) + '+yaw_only_declined'
        return est, info
```

(under_inliers 路径复用已 masked 的光束,不二次掩膜;`_yaw_fallback` 对 gate 1/2 路径先掩膜——spec 要求。内层 ICP 的 info 其余字段(eig_min、fell_back 等)原样透传。)

- [ ] **Step 4: 全绿 + 既有定位器测试 + 全套件对照**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_yaw_only_fallback.py src/tugbot_maze/test/test_online_scan_match_localizer.py src/tugbot_maze/test/test_scan_match_localizer.py src/tugbot_maze/test/test_online_slam_sim.py -v 2>&1 | tail -5
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort | diff /tmp/yawonly_baseline_failures.txt -
```

Expected: 8 个新测试全 PASS;既有定位器/online_slam 离线测试零破坏(⚠️ 它们若断言"gate 触发时 pose == prior"即被取代合同——BLOCKED 上报待授权,先例见 yaw-freshness-gate 附记);全套件 `7 failed, 432 passed, 3 xfailed`;diff 空。

- [ ] **Step 5: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260721/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py ros2_ws_tugbot_nav_20260721/src/tugbot_maze/test/test_yaw_only_fallback.py
git commit -F - <<'EOF'
feat: wire yaw-only fallback into every ICP rejection path

Gates 1/2 premask then try the 1-DOF fit; the inner-ICP rejection path
reuses its already-masked beams. Accepted fallbacks surface as
fb=yaw_only in the existing solver MATCH log line (fell_back set - zero
solver changes); declines append yaw_only_declined to the gate reason.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: headless 统计门 ×8(分批 3+3+2)

**Files:** 无代码;结果记 spec 附记

- [ ] **Step 1: 分批跑**(run 脚本自带 PRIME;每批后判定,任一坏跑立即停批取证——方法学惯例):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
bash tools/run_flood_fill_maze.sh 3600 true false online_slam
```

- [ ] **Step 2: 每跑判定**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
python3 tools/replay_collision_oracle.py log/flood_fill_run_<stamp> | grep 'rate='
L=log/flood_fill_run_<stamp>/launch.log
echo "STALL=$(grep -c 'STALL' $L) FALL=$(grep -c 'FALL_DETECTED' $L) yaw_only=$(grep -c 'fb=yaw_only' $L) nan=$(grep -c 'rejected=True' $L)"
grep -o 'MATCH rms=[0-9.]\+' $L | cut -d= -f2 | sort -n | awk '{a[NR]=$1} END {print "rms n=" NR, "median=" a[int((NR+1)/2)]}'
```

门槛:8/8 EXIT_REACHED、**零 TIMEOUT**、oracle ≤0.719%/跑(期望 0.000%)、rms 基线域、FALL 0。
行为证据:`fb=yaw_only` 计数应显著 >0(回退在工作);拒配计数应低于历史 55-60% 水平。
预案(spec):振荡/劣化 → 钳位 0.1→0.05 或改善阈 20→35%;最后手段 kwarg 停用;陷阱仍触发 → 停批取证,不叠加其他方案。

- [ ] **Step 3: 附记 + 提交**(8 跑逐条数字 + yaw_only 触发率/步长分布证据)

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-19-yaw-only-fallback-design.md
git commit -F - <<'EOF'
docs: record the 8-run statistical gate for the yaw-only fallback

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 5: GUI + RViz 验收(用户发起,禁止自动启动)

- [ ] 等用户明确说开跑:`bash tools/run_flood_fill_maze.sh 3600 false true online_slam`(在 `$WS`);判定同 Task 4;用户确认;记附记。

---

### Task 6: 文档 + 终审 + 合并

- [ ] README(workspace 名 20260719→20260721;定位节补 yaw-only 回退一段:机理一句 + 常量 + fb=yaw_only 观测法);附记收全。
- [ ] 全分支终审(最强模型):diff 映射、**解冻边界审计(tugbot_maze 内仅 online_scan_match_localizer.py 变化)**、附记数字重算、套件门。
- [ ] finishing:套件门 → `git merge --no-ff yaw-only-fallback -F <tempfile>` → main 复跑 → push → 删分支 → memory(注意与停放分支 yaw-freshness-gate 的 memory 交叉链接)。
