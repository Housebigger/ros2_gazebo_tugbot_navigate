# Dog Front Camera Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新 workspace `ros2_ws_tugbot_nav_20260718` 给 ANYmal C 装 1 个前向观察相机(Gazebo 渲染 → ROS 话题 → RViz 第一视角),导航链零改动,迷宫全回归不退化。

**Architecture:** 相机传感器加进 `base` link(规格照抄 CERBERUS camera_4,姿态全新、朝 +x 行进方向);现有 `tugbot_bridge.yaml` 加 Image+CameraInfo 两条 GZ→ROS 映射;照 scan_omni 先例加静态 TF;RViz 默认配置加 Image 面板。验收 = 程序化图像检查 + SDF 守卫测试 + 完整迷宫回归(headless + GUI/PRIME)。

**Tech Stack:** ROS 2 Jazzy、Gazebo Sim 8(ogre2 渲染,与 gpu_lidar 同引擎)、ros_gz_bridge(YAML parameter_bridge)、pytest(纯模块,无 rclpy 依赖的守卫测试)、rclpy(仅活体检查脚本)。

**Spec:** `docs/superpowers/specs/2026-07-17-dog-front-camera-design.md`(含"关键几何事实"节——实现前必读)

---

## 全局约束(每个任务都适用)

- 仓库根:`/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`,下文 `$REPO`。新 workspace:`$REPO/ros2_ws_tugbot_nav_20260718`,下文 `$WS`。
- 构建**必须** `colcon build --symlink-install`(plain build 破坏 maze_sim 数据路径)。
- 每个 shell 命令块前显式 `cd`(后台命令的 cwd 会漂移)。source ROS 前先 `set +u`。
- 只有 `python3`(无 `python`)。**禁止**在直接工具调用里前台 `sleep`(脚本文件内的 sleep 没问题)。
- **禁止自动启动 GUI Gazebo**(Task 6 由用户发起);headless `gz sim -s` / `headless:=true` 随意。
- git 提交信息**不带反引号**,用 heredoc:`git commit -F - <<'EOF' ... EOF`,结尾 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。`git add` 用显式路径,**永远不用 `git add -A`**。
- 纯模块测试命令(在 `$WS` 下):`PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`
- ⚠️ `tools/make_legged_model.py` 是**一次性历史手术脚本**(已执行过),不要重跑;本阶段直接手改 model.sdf。

---

### Task 1: 新 workspace 20260718 + 分支 + 基线

**Files:**
- Create: `ros2_ws_tugbot_nav_20260718/`(整体克隆 20260717)

- [ ] **Step 1: 建分支(从 main,含 spec 提交)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b dog-front-camera
```

- [ ] **Step 2: 克隆 workspace,清掉构建产物与缓存**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
cp -a ros2_ws_tugbot_nav_20260717 ros2_ws_tugbot_nav_20260718
rm -rf ros2_ws_tugbot_nav_20260718/build ros2_ws_tugbot_nav_20260718/install ros2_ws_tugbot_nav_20260718/log
find ros2_ws_tugbot_nav_20260718 -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
find ros2_ws_tugbot_nav_20260718 -name '.pytest_cache' -type d -exec rm -rf {} + 2>/dev/null
```

- [ ] **Step 3: 构建**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Expected: 全部包 build 成功,0 abort。

- [ ] **Step 4: 跑基线测试套件,把失败名单存档**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -20
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort > /tmp/camera_baseline_failures.txt
cat /tmp/camera_baseline_failures.txt
```

Expected: `7 failed, 411 passed, 3 xfailed`(7 个失败为历史既有:1 个 maze asset + 6 个 wall_follow;名单以实际输出为准,后续任务用 `/tmp/camera_baseline_failures.txt` 对照,**不允许新增失败**)。

- [ ] **Step 5: 提交(显式路径)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260718
git status --short | head -5
git commit -F - <<'EOF'
chore: clone workspace 20260717 -> 20260718 for the dog-front-camera phase

Byte-identical copy (build/install/log and caches stripped). Baseline
suite on the clone: 7 pre-existing failures / 411 passed / 3 xfailed,
failure names archived for regression comparison.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: SDF 守卫测试(TDD)+ camera_front 传感器

**Files:**
- Test: `ros2_ws_tugbot_nav_20260718/src/tugbot_maze/test/test_front_camera.py`(新建)
- Modify: `ros2_ws_tugbot_nav_20260718/src/tugbot_description/models/anymal_c/model.sdf`(scan_omni 传感器之后、`<!-- Lights -->` 之前插入)

**背景(为什么守卫这么写):** 本模型行进方向 = **+x**(`legged/trajectory.py` 支撑相蹬地实锤);原版 CERBERUS 的 7 个相机全朝 −x(CERBERUS 约定的"前"),照搬任何一个姿态都会朝后拍。守卫测试就是防这个回归。

- [ ] **Step 1: 写失败测试**

新建 `src/tugbot_maze/test/test_front_camera.py`:

```python
"""Guard: the forward observation camera must exist and face +x (the travel
direction). The 7 original CERBERUS cameras all face -x (CERBERUS calls -x
"front"); copying any of their poses points the camera backward — that exact
mistake is what this file pins down. See the 2026-07-17 dog-front-camera spec."""
import math
import re
from pathlib import Path


def _model_sdf_text():
    # workspace layout: <ws>/src/tugbot_maze/test/  ->  <ws>/src/tugbot_description/...
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _camera_block():
    sdf = _model_sdf_text()
    m = re.search(r'<sensor name="camera_front" type="camera">(.*?)</sensor>', sdf, re.S)
    assert m is not None, 'camera_front sensor missing from model.sdf'
    return m.group(1)


def test_camera_front_faces_travel_direction():
    block = _camera_block()
    m = re.search(r'<pose>([^<]+)</pose>', block)
    assert m is not None, 'camera_front has no pose'
    x, y, z, roll, pitch, yaw = (float(v) for v in m.group(1).split())
    assert x > 0.3, f'camera x={x}: must sit at the +x (travel) end, not the CERBERUS head end'
    assert abs(yaw) < 0.3, f'camera yaw={yaw}: must face +x; yaw~pi means the original -x pose was copied'
    assert abs(pitch) < 0.3 and abs(roll) < 0.1, f'camera tilted: roll={roll} pitch={pitch}'
    assert 0.05 < z < 0.4, f'camera z={z}: expected just above the shell'


def test_camera_front_spec_matches_cerberus_camera_4():
    block = _camera_block()
    assert '<update_rate>20</update_rate>' in block
    assert '<width>720</width>' in block
    assert '<height>540</height>' in block
    hfov = float(re.search(r'<horizontal_fov>([^<]+)</horizontal_fov>', block).group(1))
    assert math.isclose(hfov, 2.19911, abs_tol=1e-4)
    assert '<topic>/camera/front/image</topic>' in block
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_front_camera.py -v
```

Expected: 2 FAILED,信息 `camera_front sensor missing from model.sdf`。

- [ ] **Step 3: 在 model.sdf 加传感器**

编辑 `src/tugbot_description/models/anymal_c/model.sdf`,在 `base` link 内、`scan_omni` 的 `</sensor>` 之后(即 `<!-- Lights -->` 注释之前)插入:

```xml
      <sensor name="camera_front" type="camera">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <!-- Faces +x = the actual travel direction (stance feet push -x).
             The 7 original CERBERUS cameras all face -x; none of their
             poses may be copied here. -->
        <pose>0.45 0 0.20 0 0 0</pose>
        <topic>/camera/front/image</topic>
        <camera name="camera_front">
          <horizontal_fov>2.19911</horizontal_fov>
          <camera_info_topic>/camera/front/camera_info</camera_info_topic>
          <lens>
            <intrinsics>
              <!-- fx = fy = width / ( 2 * tan (hfov / 2 ) ) -->
              <fx>183.43</fx>
              <fy>183.43</fy>
              <cx>360.5</cx>
              <cy>270.5</cy>
              <s>0</s>
            </intrinsics>
          </lens>
          <image>
            <width>720</width>
            <height>540</height>
          </image>
          <clip>
            <near>0.02</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
      </sensor>
```

(规格逐值来自 `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf` 的 camera_4,行 780–812;只有 pose、name、topic 是新的。`<camera_info_topic>` 若 Gazebo 不认,Task 3 的实测步骤会暴露并给出回退。)

- [ ] **Step 4: 跑测试确认通过 + 全套件无新增失败**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_front_camera.py -v
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort | diff /tmp/camera_baseline_failures.txt -
```

Expected: 新测试 2 PASSED;全套件 `7 failed, 413 passed, 3 xfailed`;diff 无输出(失败名单与基线逐字一致)。

- [ ] **Step 5: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260718/src/tugbot_maze/test/test_front_camera.py ros2_ws_tugbot_nav_20260718/src/tugbot_description/models/anymal_c/model.sdf
git commit -F - <<'EOF'
feat: forward observation camera camera_front on the base link

CERBERUS camera_4 spec verbatim (126deg hfov, 720x540, 20Hz, intrinsics,
gaussian noise) with a NEW pose (0.45 0 0.20, yaw 0) facing +x = the
actual travel direction. All 7 original CERBERUS camera poses face -x
and must not be copied; a guard test pins pose x>0 / yaw~0 plus the
camera_4 spec values.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: 桥接 + 静态 TF + 活体验证(verify-first)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260718/src/tugbot_gazebo/config/tugbot_bridge.yaml`(文件末尾追加)
- Modify: `ros2_ws_tugbot_nav_20260718/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`(scan_tf 之后)
- Create: `ros2_ws_tugbot_nav_20260718/tools/front_camera_check.py`
- Create: `ros2_ws_tugbot_nav_20260718/tools/verify_front_camera.sh`

- [ ] **Step 1: bridge YAML 追加两条映射**

在 `src/tugbot_gazebo/config/tugbot_bridge.yaml` 末尾追加:

```yaml
- ros_topic_name: "/camera/front/image"
  gz_topic_name: "/camera/front/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: "GZ_TO_ROS"

- ros_topic_name: "/camera/front/camera_info"
  gz_topic_name: "/camera/front/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: "GZ_TO_ROS"
```

- [ ] **Step 2: 实测 gz 话题名(20260716 的 /0 话题陷阱教训:不猜,先看)**

重建后 headless 起 sim,列话题(全部写进一个脚本文件跑,避免前台 sleep 限制):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_gazebo tugbot_description
cat > /tmp/list_cam_topics.sh <<'SH'
#!/usr/bin/env bash
set +u
WS="$(cd "$(dirname "$0")" && pwd)"
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f "gz sim" 2>/dev/null; sleep 1
gz sim -s -r src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf > /tmp/cam_topics_gz.log 2>&1 &
sleep 15
gz topic -l | grep -i 'camera\|image' | tee /tmp/cam_topics.txt
pkill -9 -f "gz sim" 2>/dev/null
SH
bash /tmp/list_cam_topics.sh
```

Expected: `/camera/front/image` 与 `/camera/front/camera_info` 都在列表里。
**回退分支**:若 camera_info 出现在别的名字下(例如 scoped 默认名 `/world/.../camera_info`),把 bridge YAML 第二条的 `gz_topic_name` 改成实测名(ros 侧名字保持 `/camera/front/camera_info`),并从 model.sdf 删掉无效的 `<camera_info_topic>` 行;把实测名记进 spec 附记。

- [ ] **Step 3: launch 加静态 TF(先用预期 frame 名,Step 5 用实测值校正)**

在 `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py` 的 `scan_tf` 节点定义之后加:

```python
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_front_static_tf',
        arguments=[
            '--x', '0.45', '--y', '0.0', '--z', '0.20',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'anymal_c/base/camera_front',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
```

并把 `camera_tf` 加进文件末尾 `LaunchDescription([...])` 列表的 `scan_tf,` 之后。

- [ ] **Step 4: 写活体检查脚本**

新建 `tools/front_camera_check.py`:

```python
#!/usr/bin/env python3
"""Live check: /camera/front/image must publish ~20Hz 720x540 frames whose
content is an actual rendered scene (not a constant fill). Dumps the last
frame to /tmp/front_camera_frame.ppm for eyeballing."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Check(Node):
    def __init__(self):
        super().__init__('front_camera_check')
        self.msgs = []
        self.create_subscription(Image, '/camera/front/image', self.msgs_cb, 10)

    def msgs_cb(self, msg):
        self.msgs.append((time.monotonic(), msg))


def main():
    rclpy.init()
    node = Check()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    msgs = node.msgs
    assert len(msgs) >= 2, f'only {len(msgs)} image msgs in 8s - bridge or sensor dead'
    span = msgs[-1][0] - msgs[0][0]
    hz = (len(msgs) - 1) / span if span > 0 else 0.0
    m = msgs[-1][1]
    print(f'frames={len(msgs)} hz={hz:.1f} {m.width}x{m.height} '
          f'encoding={m.encoding} frame_id={m.header.frame_id}')
    assert m.width == 720 and m.height == 540, 'wrong resolution'
    assert 10.0 <= hz <= 30.0, f'rate {hz:.1f}Hz outside 10-30 (nominal 20)'
    data = bytes(m.data)
    assert len(set(data[:30000])) > 8, 'frame nearly constant - camera not rendering the scene?'
    if m.encoding in ('rgb8', 'bgr8'):
        with open('/tmp/front_camera_frame.ppm', 'wb') as f:
            f.write(b'P6\n%d %d\n255\n' % (m.width, m.height))
            if m.encoding == 'rgb8':
                f.write(data)
            else:
                swapped = bytearray(data)
                swapped[0::3], swapped[2::3] = data[2::3], data[0::3]
                f.write(bytes(swapped))
        print('frame saved: /tmp/front_camera_frame.ppm')
    rclpy.shutdown()
    print('FRONT CAMERA CHECK PASS')


if __name__ == '__main__':
    main()
```

新建 `tools/verify_front_camera.sh`:

```bash
#!/usr/bin/env bash
# Front-camera live gate: boot the maze sim headless (gz + bridge + TFs via
# tugbot_gazebo.launch.py - no solver), then assert /camera/front/image
# publishes real frames. The dog just stands (JointPositionController holds
# STAND_POSE), which is all an observation camera needs.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100))
echo "DOMAIN=$ROS_DOMAIN_ID"
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f ros_gz_bridge 2>/dev/null; sleep 1
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true > /tmp/front_camera_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 18   # gz boot + sensors + bridge up
python3 tools/front_camera_check.py
RC=$?
kill -INT $LAUNCH_PID 2>/dev/null; sleep 2
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f ros_gz_bridge 2>/dev/null
exit $RC
```

```bash
chmod +x /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718/tools/verify_front_camera.sh
```

- [ ] **Step 5: 跑活体检查;frame_id 实测校正 TF;眼球确认帧内容**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
bash tools/verify_front_camera.sh
```

Expected: `FRONT CAMERA CHECK PASS`,hz≈20,720x540,输出里有 `frame_id=...`。

1. 若实测 `frame_id` ≠ `anymal_c/base/camera_front`:把 launch 里 `--child-frame-id` 改成实测值,重跑本步骤确认;实测值记进 spec 附记。
2. 眼球确认视野干净(帧里不应是机壳大特写):若装了 PIL 则 `python3 -c "from PIL import Image; Image.open('/tmp/front_camera_frame.ppm').save('/tmp/front_camera_frame.png')"` 后用 Read 工具看 PNG;PIL 不在就把 PPM 交给图像分析工具或让评审者看。若被机体/提手遮挡,z 在 0.20→0.28 内上调,重跑 Task 2 Step 4 + 本步骤(守卫测试允许 z<0.4),最终 z 记进 spec 附记。

- [ ] **Step 6: 全套件回归 + 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260718/src/tugbot_gazebo/config/tugbot_bridge.yaml ros2_ws_tugbot_nav_20260718/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py ros2_ws_tugbot_nav_20260718/tools/front_camera_check.py ros2_ws_tugbot_nav_20260718/tools/verify_front_camera.sh
git commit -F - <<'EOF'
feat: bridge + static TF + live gate for the front camera

Two GZ_TO_ROS entries (Image, CameraInfo) in tugbot_bridge.yaml; static
TF base_link -> camera frame mirroring the scan_omni precedent; and
tools/verify_front_camera.sh boots the maze sim headless and asserts
~20Hz 720x540 non-constant frames, dumping one for eyeballing.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: RViz Image 面板

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260718/src/tugbot_bringup/rviz/tugbot_nav.rviz`

- [ ] **Step 1: 在 Displays 列表末尾加 Image 显示**

`tugbot_nav.rviz` 的 Displays 列表最后一项是 `Name: ScatterCloud` 的 PointCloud2 块(以 `      Value: true` 结尾,后面紧跟 2 空格缩进的 `  Enabled: true`)。在 ScatterCloud 块的最后一行 `      Value: true` 之后、`  Enabled: true` 之前插入(注意 4 空格列表项缩进,与其他 display 一致):

```yaml
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: FrontCamera
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/front/image
      Value: true
```

- [ ] **Step 2: 校验 YAML 仍可解析且新块挂对了位置**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
python3 - <<'EOF'
import yaml
cfg = yaml.safe_load(open('src/tugbot_bringup/rviz/tugbot_nav.rviz'))
displays = cfg['Visualization Manager']['Displays']
names = [d.get('Name') for d in displays]
print(names)
img = [d for d in displays if d.get('Class') == 'rviz_default_plugins/Image']
assert len(img) == 1, 'expected exactly one Image display'
assert img[0]['Topic']['Value'] == '/camera/front/image'
print('RVIZ CONFIG OK')
EOF
```

Expected: 名单含既有 displays + `FrontCamera`,输出 `RVIZ CONFIG OK`。

- [ ] **Step 3: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260718/src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: RViz Image panel FrontCamera on /camera/front/image

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 5: headless 完整迷宫回归

**Files:** 无代码改动;产物 = 运行 artifact + spec 附记结果记录

- [ ] **Step 1: 后台跑完整迷宫(headless,预算 3600s,实际 ~700 sim-s)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
bash tools/run_flood_fill_maze.sh 3600 true false online_slam
```

(用 run_in_background;脚本自己管 DOMAIN/进程卫生/artifact 目录,结束时打印 `result=` 与 `ARTIFACT_DIR=`。轮询其 result.txt/输出,别干等通知。)

- [ ] **Step 2: 判定结果 + oracle + ICP 质量**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
ART=log/flood_fill_run_<stamp>   # 用上一步实际打印的 ARTIFACT_DIR
grep 'result=' <运行输出>        # 必须 EXIT_REACHED(FALL_DETECTED/TIMEOUT = 失败,先查 launch.log)
python3 tools/replay_collision_oracle.py "$ART"
grep -o 'MATCH rms=[0-9.]*' "$ART"/launch.log | cut -d= -f2 | sort -n | awk '{a[NR]=$1} END {print "n=" NR, "median=" a[int((NR+1)/2)]}'
```

Expected: `EXIT_REACHED`;oracle 碰撞率 **0.000%**;MATCH rms 中位数与基线同量级(~0.02–0.05,腿式阶段 4 跑中位 0.026–0.034)。若 oracle >0% 或 rms 明显劣化:按 spec 预案,相机 update_rate 20→10(改 model.sdf + 守卫测试),重跑本任务;**不动导航参数**;决策记 spec 附记。

- [ ] **Step 3: 结果记入 spec 附记并提交**

在 `docs/superpowers/specs/2026-07-17-dog-front-camera-design.md` 附记节记录:run stamp、result、oracle 数字、rms 中位数、(如有)实测话题名/frame_id/z 微调。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-17-dog-front-camera-design.md
git commit -F - <<'EOF'
docs: record headless full-maze regression with the front camera

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: GUI + RViz 验收跑(用户发起,禁止自动启动)

**Files:** 无代码改动

- [ ] **Step 1: 等用户明确说开跑;届时命令(PRIME offload 是本机硬要求)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260718
export DISPLAY=:1 __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
bash tools/run_flood_fill_maze.sh 3600 false true online_slam
```

- [ ] **Step 2: 用户人工确认 RViz Image 面板第一视角;程序判定同 Task 5 Step 2**

Expected: EXIT_REACHED + oracle 0.000% + rms 同量级 + 用户确认画面。结果记 spec 附记(可与 Task 7 文档提交合并)。

---

### Task 7: 文档收尾

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260718/README.md`
- Modify: `docs/superpowers/specs/2026-07-17-dog-front-camera-design.md`(附记)

- [ ] **Step 1: README 更新**

在 README 的传感器/运行说明处补:前向相机 `/camera/front/image` + `/camera/front/camera_info`(720×540@20Hz,126° 广角,朝 +x 行进方向);RViz 含 FrontCamera Image 面板;`tools/verify_front_camera.sh` 活体门;GUI 命令保持既有 PRIME offload 写法不变。同时把 README 里的 workspace 自述名从 20260717 改为 20260718(如有出现)。

- [ ] **Step 2: spec 附记补全(所有实现期偏差:实测话题名、frame_id、z 值、帧率决策、GUI 验收结果)+ 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260718/README.md docs/superpowers/specs/2026-07-17-dog-front-camera-design.md
git commit -F - <<'EOF'
docs: README camera section + spec addendum for dog-front-camera

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

## 完成后

全部任务完成 + 用户 GUI 验收通过后,走 superpowers:finishing-a-development-branch:套件门(失败名单对照基线)→ `--no-ff` 合并 main(`git merge --no-ff dog-front-camera -F <tempfile>`,merge 不读 stdin)→ push origin → 删分支 → 更新 memory。
