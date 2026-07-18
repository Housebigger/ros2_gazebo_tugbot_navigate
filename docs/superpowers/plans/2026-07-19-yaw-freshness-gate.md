# Yaw Freshness Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新 workspace `ros2_ws_tugbot_nav_20260720` 给 `MazeMotion._front_blocked` 加双信号门(连续 2 tick 确认 + 转弯活动 0.5s 冷却),消灭"转弯瞬间 yaw 陈旧 → 真墙误分桶 → 假 front_block → 逃逸活锁"陷阱;离线红→绿铁证 + headless ×6 零 TIMEOUT 统计门。

**Architecture:** 首次解冻导航链,改动收敛为 `maze_motion.py` 单文件:`__init__` 两个新 kwargs、`step()` 里用既有 `yaw_rate` + 'turn' phase 维护 `_last_turn_t`、`_front_blocked` 增 streak/cooldown 门(原三条件保留)、STALL DIAG 行尾追加 `fb_streak`/`cooldown` 字段。行为红测试(旧代码单 tick 闪报即 STALL)先行。

**Tech Stack:** 纯 Python(无新依赖)、pytest + MazeSim 离线射线仿真、Gazebo headless 统计门(run 脚本自带 PRIME)。

**Spec:** `docs/superpowers/specs/2026-07-19-yaw-freshness-gate-design.md`

---

## 全局约束(每任务适用)

- `$REPO` = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`;`$WS` = `$REPO/ros2_ws_tugbot_nav_20260720`。
- `colcon build --symlink-install`;显式 `cd`;`set +u` 后 source;只有 `python3`;直接调用禁前台 `sleep`;禁自动 GUI。
- ⚠️ pkill/pgrep 含 "gz sim"/"ros2 launch" 模式必须走脚本文件,且写与执行分两次 Bash 调用。
- git:heredoc 提交、无反引号、结尾 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`;`git add` 显式路径。
- 套件命令(在 `$WS`):`PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`;基线 **7 failed / 424 passed / 3 xfailed**,名单存档对照。
- **导航链解冻仅限 `maze_motion.py`**:任何其他 `tugbot_maze/tugbot_maze/` 文件被改动都是缺陷。若任何既有测试因本改动破坏:STOP 报 BLOCKED,不许改既有测试。

---

### Task 1: 新 workspace 20260720 + 分支 + 基线

**Files:** Create: `ros2_ws_tugbot_nav_20260720/`(克隆 20260719)

- [ ] **Step 1: 分支**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b yaw-freshness-gate
```

- [ ] **Step 2: 克隆+清产物**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
cp -a ros2_ws_tugbot_nav_20260719 ros2_ws_tugbot_nav_20260720
rm -rf ros2_ws_tugbot_nav_20260720/build ros2_ws_tugbot_nav_20260720/install ros2_ws_tugbot_nav_20260720/log
find ros2_ws_tugbot_nav_20260720 -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
find ros2_ws_tugbot_nav_20260720 -name '.pytest_cache' -type d -exec rm -rf {} + 2>/dev/null
```

- [ ] **Step 3: 构建**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Expected: 6 packages 成功。

- [ ] **Step 4: 基线 + 存档**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort > /tmp/yawgate_baseline_failures.txt
wc -l /tmp/yawgate_baseline_failures.txt
```

Expected: `7 failed, 424 passed, 3 xfailed`;7 行。不符 BLOCKED。

- [ ] **Step 5: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260720
git status --short | grep -v '^A' | head -3
git commit -F - <<'EOF'
chore: clone workspace 20260719 -> 20260720 for the yaw-freshness-gate phase

Byte-identical copy (build artifacts and caches stripped). Baseline
suite: 7 pre-existing failures / 424 passed / 3 xfailed, names archived.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: 双信号门(TDD:行为红 → 实现 → 绿)

**Files:**
- Test: `ros2_ws_tugbot_nav_20260720/src/tugbot_maze/test/test_front_block_gate.py`(新)
- Modify: `ros2_ws_tugbot_nav_20260720/src/tugbot_maze/tugbot_maze/maze_motion.py`

**实现前必读**(维护者视角的现状,均已核实):
- `step(pose, scan, t)` 是 tick 入口(行 ~144),顶部已维护 `self.yaw_rate = _norm(yaw−prev_yaw)/(t−prev_t)` 与 `prev_yaw/prev_t`(行 145-148)。phase 集合:center/turn/drive/recover/backout/stuck/done。
- `_front_blocked(perp, dirn, moved, yaw)`(行 ~518)现为纯函数:`perp[dirn] < front_block_m(0.7) and moved > 0.3 and |_norm(target_cardinal−yaw)| < front_block_max_yaw(0.30)`。唯一调用点在 `_drive`(行 ~578);行 ~592 的 deadline 分支复用其返回值。
- `_stall_event(reason, perp, moved, marked, x, y, yaw)`(行 ~485)格式化 STALL DIAG 行(行 ~499)。
- 测试惯例(test_maze_motion.py):直接 `MazeMotion()`、强置 `m.phase/hop_dir/target_cardinal/hop_start`、`MazeSim(load_segments(), pos, yaw)` 供真实射线扫描、dt=0.1 tick。
- spec 的"wz 指令>0.1"信号在实现上采用 **phase=='turn'**(命令旋转的 phase;last-wz 未被存储)+ `|yaw_rate|>0.3`(覆盖一切实际旋转来源,含 ICP yaw 跳变——后者恰是陈旧窗口本尊)。此实现映射记入附记。

- [ ] **Step 1: 写行为测试(先红)**

新建 `src/tugbot_maze/test/test_front_block_gate.py`:

```python
"""Dual-signal front_block gate: a single-tick short front reading right
after turn activity is the stale-yaw mis-bucketing signature (lidar-3d-swap
forensics: 15/15 STALL flickers were single-tick) and must NOT stall; a
sustained short reading in steady state is a REAL wall and must still stall
(one extra confirm tick of delay only)."""
import math

from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, cell_center


N_BEAMS = 360
AMIN = -math.pi
AINC = 2 * math.pi / N_BEAMS


def _scan(front_m, base_m=5.0, yaw=math.pi / 2, cardinal=math.pi / 2):
    """Synthetic scan: all beams base_m except a +/-20deg sector around the
    world-frame `cardinal` direction (converted to body bearing via yaw)."""
    ranges = [base_m] * N_BEAMS
    for i in range(N_BEAMS):
        bearing = AMIN + i * AINC
        world = yaw + bearing
        d = math.atan2(math.sin(world - cardinal), math.cos(world - cardinal))
        if abs(d) <= math.radians(20):
            ranges[i] = front_m
    return (ranges, AMIN, AINC)


def _driving_motion(t0=0.0):
    """MazeMotion forced into a mid-hop 'drive' state heading N (+y), already
    0.45m past hop_start (moved>0.3), aligned. Attribute names verified
    against _drive()'s reads; adapt here (not in the assertions) if the
    implementation renames state."""
    m = MazeMotion()
    m.phase = 'drive'
    m.hop_dir = (0, 1)
    m.target_cardinal = math.pi / 2
    cx, cy = cell_center(ENTRANCE_CELL)
    m.hop_start = (cx, cy)
    m.hop_deadline = t0 + m.hop_timeout_s
    m.progress_pose = (cx, cy + 0.45)
    m.progress_t = t0
    return m, (cx, cy + 0.45, math.pi / 2)


def _stall_reasons(m):
    return [e for e in m.events if 'STALL' in e and 'front_block' in e]


def test_single_tick_short_read_after_turn_activity_does_not_stall():
    m, (x, y, yaw) = _driving_motion()
    t = 0.0
    # tick 1: clear front, but yaw JUMPS 0.4 rad between ticks (yaw_rate=4rad/s
    # = turn activity / ICP yaw snap - the stale-yaw window opener)
    m.step((x, y, yaw - 0.4), _scan(front_m=5.0, yaw=yaw - 0.4), t)
    t += 0.1
    # tick 2: aligned again, ONE short front read (the mis-bucketing flicker)
    m.step((x, y, yaw), _scan(front_m=0.5, yaw=yaw), t)
    t += 0.1
    # tick 3: reading recovered
    m.step((x, y, yaw), _scan(front_m=5.0, yaw=yaw), t)
    assert not _stall_reasons(m), f'flicker stalled: {m.events}'
    assert m.phase == 'drive'


def test_true_front_wall_still_stalls_with_one_confirm_tick():
    m, (x, y, yaw) = _driving_motion()
    t = 0.0
    for _ in range(8):                       # steady straight driving, no turn activity
        m.step((x, y, yaw), _scan(front_m=5.0, yaw=yaw), t)
        t += 0.1
    m.step((x, y, yaw), _scan(front_m=0.5, yaw=yaw), t)   # short read #1: no stall yet
    assert not _stall_reasons(m)
    t += 0.1
    m.step((x, y, yaw), _scan(front_m=0.5, yaw=yaw), t)   # short read #2: confirmed -> stall
    assert _stall_reasons(m), f'real wall not stalled: {m.events}'


def test_streak_resets_on_clear_read():
    m, (x, y, yaw) = _driving_motion()
    t = 0.0
    for _ in range(8):
        m.step((x, y, yaw), _scan(front_m=5.0, yaw=yaw), t)
        t += 0.1
    for front in (0.5, 5.0, 0.5):            # short, clear, short: never 2 consecutive
        m.step((x, y, yaw), _scan(front_m=front, yaw=yaw), t)
        t += 0.1
    assert not _stall_reasons(m)


def test_cooldown_defers_then_confirmed_stall_fires():
    m, (x, y, yaw) = _driving_motion()
    t = 0.0
    m.step((x, y, yaw - 0.4), _scan(front_m=5.0, yaw=yaw - 0.4), t)   # turn activity
    t += 0.1
    fired_at = None
    for k in range(10):                      # sustained short read from here on
        m.step((x, y, yaw), _scan(front_m=0.5, yaw=yaw), t)
        if _stall_reasons(m):
            fired_at = t
            break
        t += 0.1
    assert fired_at is not None, 'sustained wall never stalled after cooldown'
    assert fired_at >= 0.1 + m.front_block_turn_cooldown_s - 1e-9


def test_gate_params_exposed_with_defaults():
    m = MazeMotion()
    assert m.front_block_confirm_ticks == 2
    assert m.front_block_turn_cooldown_s == 0.5


def test_streak_resets_on_direction_change():
    # white-box unit: two short reads on DIFFERENT dirns must not accumulate
    m, (x, y, yaw) = _driving_motion()
    m._last_turn_t = float('-inf')            # cooldown open
    perp_short_n = {'N': 0.5, 'S': 5.0, 'E': 5.0, 'W': 5.0}
    perp_short_e = {'N': 5.0, 'S': 5.0, 'E': 0.5, 'W': 5.0}
    assert m._front_blocked(perp_short_n, 'N', 0.45, math.pi / 2, 10.0) is False
    m.target_cardinal = 0.0                   # now heading E
    assert m._front_blocked(perp_short_e, 'E', 0.45, 0.0, 10.1) is False
    assert m._fb_streak == 1                  # restarted, not accumulated


def test_turn_phase_refreshes_cooldown():
    # white-box: a tick spent in phase 'turn' must stamp _last_turn_t even
    # with zero yaw_rate (commanded rotation not yet moving).
    m, (x, y, yaw) = _driving_motion()
    m.phase = 'turn'
    m.step((x, y, yaw), _scan(front_m=5.0, yaw=yaw), 7.0)
    assert m._last_turn_t == 7.0
```

- [ ] **Step 2: 跑测试确认红(行为红是本阶段的 bug 铁证)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_front_block_gate.py -v
```

Expected: `test_single_tick_short_read_after_turn_activity_does_not_stall` **FAIL**(旧代码单 tick 即 STALL——这就是取证的 bug)、`test_gate_params_exposed_with_defaults` FAIL(AttributeError)、`test_cooldown_defers...` FAIL;`test_true_front_wall...` 可能旧代码下在第 1 短读就 stall(断言 `not _stall_reasons` 失败)= 同样红。**记录每个测试的红理由**;若 `_driving_motion` 的强置状态与 `_drive` 实际读取不符(AttributeError 之类),先读 `_drive` 修 harness 到能复现,再确认红——harness 修正记附记。

- [ ] **Step 3: 实现(仅 maze_motion.py)**

三处改动:

(1) `__init__` 签名加 kwargs(插在 `front_block_m: float = 0.7,` 之后):

```python
                 front_block_confirm_ticks: int = 2,
                 front_block_turn_cooldown_s: float = 0.5,
```

赋值区(`self.front_block_max_yaw = 0.30` 行之后):

```python
        self.front_block_confirm_ticks = front_block_confirm_ticks
        self.front_block_turn_cooldown_s = front_block_turn_cooldown_s
        # Dual-signal gate state: the alignment gate above shares the possibly-
        # stale yaw with the cardinal bucketing, so it cannot see yaw staleness;
        # these two signals (consecutive-tick confirmation + post-turn-activity
        # cooldown) are independent of that yaw's absolute value.
        self._fb_streak = 0
        self._fb_streak_dirn = None
        self._fb_cooldown_ok = True
        self._last_turn_t = float('-inf')
```

(2) `step()` 中 `self.prev_yaw = yaw; self.prev_t = t` 行之后插入:

```python
        if self.phase == 'turn' or abs(self.yaw_rate) > 0.3:
            self._last_turn_t = t   # turn activity (incl. ICP yaw snaps): distrust cardinal bucketing briefly
```

(3) `_front_blocked` 整体替换(签名加 `t`;唯一调用点 `_drive` 行 ~578 同步加 `t`):

```python
    def _front_blocked(self, perp, dirn, moved, yaw, t):
        """A hop is front-blocked only when ALIGNED (a short front reading while
        mis-aligned is an angled beam - false), AND confirmed across
        front_block_confirm_ticks consecutive ticks, AND outside the post-turn
        cooldown. The stale-yaw mis-bucketing flicker (lidar-3d-swap forensics)
        is single-tick and turn-adjacent; a real wall ahead is sustained and
        steady-state. The streak keeps counting during cooldown so a real wall
        approached mid-cooldown stalls at the first eligible tick."""
        raw = (perp[dirn] < self.front_block_m and moved > 0.3
               and abs(_norm(self.target_cardinal - yaw)) < self.front_block_max_yaw)
        if raw and self._fb_streak_dirn == dirn:
            self._fb_streak += 1
        elif raw:
            self._fb_streak = 1
            self._fb_streak_dirn = dirn
        else:
            self._fb_streak = 0
            self._fb_streak_dirn = None
        self._fb_cooldown_ok = (t - self._last_turn_t) >= self.front_block_turn_cooldown_s
        return self._fb_streak >= self.front_block_confirm_ticks and self._fb_cooldown_ok
```

(4) `_stall_event` 的格式串行尾追加两个字段(保持既有字段不动,行尾加
` fb_streak=%d cooldown=%s`,实参 `self._fb_streak, 'ok' if self._fb_cooldown_ok else 'wait'`)。

- [ ] **Step 4: 绿 + 全套件 + 离线 e2e 无破坏**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_front_block_gate.py -v
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_maze_motion.py src/tugbot_maze/test/test_maze_motion_sim.py src/tugbot_maze/test/test_flood_fill_maze_sim.py -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort | diff /tmp/yawgate_baseline_failures.txt -
```

Expected: 新测试 7 PASSED;既有 motion/e2e 测试零新增失败;全套件 `7 failed, 431 passed, 3 xfailed`;diff 空。**任何既有测试被破坏 → BLOCKED,不许改既有测试。**

- [ ] **Step 5: 确认改动面**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git diff --stat
```

Expected: 恰好 2 个文件(maze_motion.py + 新测试文件)。

- [ ] **Step 6: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260720/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260720/src/tugbot_maze/test/test_front_block_gate.py
git commit -F - <<'EOF'
feat: dual-signal front_block gate (confirm ticks + post-turn cooldown)

The existing alignment gate shares the possibly-stale yaw with the
cardinal bucketing, so single-tick mis-bucketing flickers right after
turn activity stalled hops and fed the central-junction escape livelock
(lidar-3d-swap forensics). front_block now needs 2 consecutive short
reads AND 0.5s since the last turn activity (phase=turn or
|yaw_rate|>0.3, which also catches ICP yaw snaps). A real wall still
stalls, one confirm tick later; the streak keeps counting during
cooldown. Behavioral red test reproduced the old single-tick stall
before the fix.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: headless 统计门 ×6

**Files:** 无代码;结果记 spec 附记

- [ ] **Step 1: 6 次 headless(run 脚本自带 PRIME;后台串行,分批 3+3 以便中途判定)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
bash tools/run_flood_fill_maze.sh 3600 true false online_slam
```

(×6;每跑判定后再续,任一 TIMEOUT 立即停批取证——按 20260718 方法学,不盲调。)

- [ ] **Step 2: 每跑判定**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
python3 tools/replay_collision_oracle.py log/flood_fill_run_<stamp> | grep 'rate='
L=log/flood_fill_run_<stamp>/launch.log
grep -c 'MATCH rms=nan' $L; grep -c 'STALL' $L; grep -c 'FALL_DETECTED' $L
grep -o 'MATCH rms=[0-9.]\+' $L | cut -d= -f2 | sort -n | awk '{a[NR]=$1} END {print "n=" NR, "median=" a[int((NR+1)/2)]}'
```

门槛:6/6 EXIT_REACHED、**零 TIMEOUT**、oracle 每跑 ≤0.719%(期望 0.000%)、rms median ≤0.05、FALL 0。对照(修复前):3×0.000% + 1×0.719% + 1×TIMEOUT(22.3%)。

- [ ] **Step 3: 附记 + 提交**

spec 附记记录:6 跑逐条数字、STALL 计数对比(门生效的行为证据:假 front_block STALL 应显著减少)、(如有)任何 `cooldown=wait` DIAG 观测。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-19-yaw-freshness-gate-design.md
git commit -F - <<'EOF'
docs: record the 6-run headless statistical gate for the front_block gate

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: GUI + RViz 验收(用户发起,禁止自动启动)

- [ ] **Step 1: 等用户明确说开跑**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260720
bash tools/run_flood_fill_maze.sh 3600 false true online_slam
```

- [ ] **Step 2: 判定同 Task 3;用户人工确认;结果记附记(可与 Task 5 合并提交)**

---

### Task 5: 文档收尾 + 终审 + 合并

**Files:** `ros2_ws_tugbot_nav_20260720/README.md`、spec 附记

- [ ] **Step 1: README**:workspace 自述名 20260719→20260720;在描述 solver/STALL 行为处补一句双信号门(参数名 + 默认值 + 一句机制)。
- [ ] **Step 2: 附记收全(实现映射、harness 修正、GUI 结果、全部偏差)+ 提交**
- [ ] **Step 3: 全分支终审(最强模型):diff 映射、**导航链解冻边界审计(tugbot_maze 内仅 maze_motion.py 变化)**、附记数字重算、套件门。
- [ ] **Step 4: finishing:套件门 → `git merge --no-ff yaw-freshness-gate -F <tempfile>` → main 复跑 → push → 删分支 → memory。

---

## 完成后

同前三阶段惯例:merge 信息完整阶段总结;memory 新文件 + MEMORY.md 索引。
