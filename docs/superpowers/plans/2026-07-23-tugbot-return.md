# tugbot-return Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Swap MR-Buggy3 back to the original Tugbot (real DiffDrive physics, full collision set) with the modern perception stack grafted on, and measure the accumulated-3D-map stability against the 450k-voxel buggy baseline.

**Architecture:** New workspace `ros2_ws_tugbot_nav_20260727` (clone of 20260726), branch `tugbot-return`. Model-layer surgery on the stock tugbot SDF (sensor pruning + lidar_3d/camera_front graft + world-anchored OdometryPublisher with DiffDrive odom silenced), a one-line motion-mode flip (`ackermann_drive` default False), an honest footprint restore (0.262/−0.468/0.292), and world/launch swap. Navigation chain, localizers, projector, and cloud accumulator are byte-untouched.

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic (gz-sim DiffDrive / OdometryPublisher / gpu_lidar), pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-07-23-tugbot-return-design.md` (read it first; user adjudications: classic diff mode, honest-comparison acceptance).

---

## Standing process constraints (bind every task)

- Every command block starts with an explicit `cd` to an absolute path (cwd drifts between calls).
- `set +u` before sourcing any ROS `setup.bash`; only `python3` exists on this machine.
- Build ONLY from inside the workspace: `cd <ws> && colcon build --symlink-install` (repo-root builds fail on duplicate packages).
- Suite accounting is authoritative ONLY from a main-session workspace-root run over BOTH test dirs: `cd <ws> && python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q` (544 collected; tugbot_maze alone under-collects by the 5 bringup-contract tests — Task 0 measured exactly this trap). Subagent pytest counts have twice been collection-scope artifacts — report them, but the controller re-verifies.
- `git add` explicit paths only, never `-A` (tmp_resources holds untracked junk).
- Commit messages: NO backticks; use heredoc `git commit -F - <<'EOF'`; end with the trailer lines used project-wide (Co-Authored-By: Claude Fable 5 <noreply@anthropic.com> and the Claude-Session URL).
- Subagents run ALL commands FOREGROUND. Long sim runs (Task 6 headless acceptance, any GUI run) happen in the MAIN session only. GUI runs launch ONLY on the user's explicit go-ahead.
- RED LINE: `/gt/dynamic_pose` is eval-only (oracle + the Task 5 anchor check readout). It must never feed a control loop.

**Workspace under edit:** `ros2_ws_tugbot_nav_20260727` (created in Task 0). ALL `src/` paths below are relative to `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727/`. The repo root is `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`.

## File map

| File | Action | Responsibility |
|---|---|---|
| `ros2_ws_tugbot_nav_20260727/` | create (Task 0) | phase workspace, clone of 20260726 |
| `src/tugbot_description/models/tugbot/model.sdf` | modify (Task 1) | sensor pruning, lidar_3d/camera_front graft, OdometryPublisher, DiffDrive odom redirect |
| `src/tugbot_maze/test/test_tugbot_return_contract.py` | create (Task 1) | pins every model/world/launch decision of this phase |
| `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf` | modify (Task 2) | spawn tugbot, not buggy |
| `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py` | modify (Task 2) | static TFs → tugbot sensor frames |
| `src/tugbot_maze/test/test_mr_buggy3_contract.py` | modify (Task 2) | re-freeze the two world/launch-coupled tests |
| `src/tugbot_maze/tugbot_maze/footprint.py` | modify (Task 3) | honest tugbot rectangle |
| `src/tugbot_maze/test/test_footprint.py` | modify (Task 3) | envelope test + comments follow the constants |
| `src/tugbot_maze/test/test_maze_sim.py` | modify (Task 3) | 6 fixture geometries recomputed for the new rectangle |
| `src/tugbot_maze/test/test_wall_follow_maze_sim.py` | modify (Task 3) | era-4 adjudication of the footprint-coupled records |
| `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` | modify (Task 4) | ackermann_drive default False |
| `tools/verify_tugbot_model.sh` + `tools/tugbot_odom_anchor_check.py` | create (Task 5) | smoke: plugin semantics, /scan single-publisher, world-anchored odom |
| `docs/superpowers/specs/2026-07-23-tugbot-return-design.md` | append (Task 7) | results addendum |

---

### Task 0: Workspace clone, branch, baseline

**Files:** Create `ros2_ws_tugbot_nav_20260727/` (whole tree).

- [ ] **Step 1: Branch off main at the repo root**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b tugbot-return
```

- [ ] **Step 2: Clone the workspace (source only)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
rsync -a --exclude build --exclude install --exclude log --exclude __pycache__ --exclude .pytest_cache \
  ros2_ws_tugbot_nav_20260726/ ros2_ws_tugbot_nav_20260727/
```

- [ ] **Step 3: Build (from INSIDE the workspace, symlink mandatory)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install
```

Expected: all packages build, no errors.

- [ ] **Step 4: Baseline suite run — record the frozen quartet by name**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -15
```

Expected: `4 failed, 540 passed` (0 xfailed). Copy the 4 failed test names verbatim into the Task 0 commit message — every later suite run is adjudicated against this list.

- [ ] **Step 5: Commit the clone**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727
git commit -F - <<'EOF'
chore: new workspace ros2_ws_tugbot_nav_20260727 (clone of 20260726)

tugbot-return phase workspace. Baseline suite on the fresh clone:
4 failed / 540 passed / 0 xfailed. Frozen quartet:
<paste the 4 test names here>

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 1: Tugbot contract test + model surgery

**Files:**
- Create: `src/tugbot_maze/test/test_tugbot_return_contract.py`
- Modify: `src/tugbot_description/models/tugbot/model.sdf`

Reference geometry of the stock SDF (before surgery): links/sensors at
`camera_link`+`color` (~138–163), `scan_front` (~165–188), `scan_back`
(~190–213), `scan_omni` (~215–304, its sensor publishes directly on `/scan`),
`sensor_contact` gpu_lidar inside the `gripper` link (~354–372), DiffDrive
plugin at ~794–809 currently publishing `<odom_topic>/odom` + `<tf_topic>/tf`.
Delete by ANCHOR STRINGS, not line numbers (they shift as you edit).

- [ ] **Step 1: Write the failing contract test**

Create `src/tugbot_maze/test/test_tugbot_return_contract.py`:

```python
"""Contract: the tugbot-return phase — the ORIGINAL Tugbot (real DiffDrive
physics, full collision set incl. rear gripper) carrying the modern perception
stack. Pins: stock 2D sensors REMOVED (scan_omni published directly on /scan
and would collide with the projection chain — the only legitimate /scan
source), lidar_3d centered (SCAN_OFFSET_X=0 contract) with the dog-identical
<lidar> block, camera_front forward, world-anchored OdometryPublisher owning
/odom + /tf while DiffDrive's wheel-integrated odom (drifts under slip — would
break both aliasing gates' drift-free premise) is redirected to inert topics."""
import re
from pathlib import Path


def _ws_src():
    return Path(__file__).resolve().parents[2]


def _tugbot_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf').read_text()


def _dog_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _block(text, pattern):
    m = re.search(pattern, text, re.S)
    assert m is not None, f'pattern not found: {pattern}'
    return m.group(1)


def test_model_is_dynamic_tugbot():
    sdf = _tugbot_sdf()
    assert '<model name="tugbot">' in sdf
    assert '<static>' not in sdf or '<static>false</static>' in sdf
    assert '<gravity>false</gravity>' not in sdf     # real physics everywhere


def test_stock_2d_sensors_removed():
    sdf = _tugbot_sdf()
    # scan_omni published DIRECTLY on /scan -> would collide with the
    # slice-projector-regenerated /scan. Removed wholesale, not muted.
    assert 'scan_omni' not in sdf
    assert 'scan_front' not in sdf
    assert 'scan_back' not in sdf
    assert 'sensor_contact' not in sdf
    assert '<sensor name="color"' not in sdf         # stock camera gone
    assert '/camera/image_raw' not in sdf
    # NOTHING in the model may publish on /scan
    assert '<topic>/scan</topic>' not in sdf


def test_diff_drive_kept_with_odom_silenced():
    p = _block(_tugbot_sdf(), r'name="gz::sim::systems::DiffDrive">(.*?)</plugin>')
    assert '<left_joint>wheel_left_joint</left_joint>' in p
    assert '<right_joint>wheel_right_joint</right_joint>' in p
    assert '<wheel_separation>0.514</wheel_separation>' in p
    assert '<wheel_radius>0.195</wheel_radius>' in p
    assert '<topic>/cmd_vel</topic>' in p
    # Wheel-integrated odom DRIFTS under slip; both aliasing gates snap to
    # /odom on the drift-free premise. DiffDrive keeps only its drive role.
    assert '<odom_topic>/diffdrive/odom_unused</odom_topic>' in p
    assert '<tf_topic>/diffdrive/tf_unused</tf_topic>' in p
    assert '<odom_topic>/odom</odom_topic>' not in p
    assert '<tf_topic>/tf</tf_topic>' not in p


def test_world_anchored_odometry_publisher():
    p = _block(_tugbot_sdf(), r'name="gz::sim::systems::OdometryPublisher">(.*?)</plugin>')
    assert '<odom_frame>odom</odom_frame>' in p
    assert '<robot_base_frame>base_link</robot_base_frame>' in p
    assert '<odom_topic>/odom</odom_topic>' in p
    assert '<tf_topic>/tf</tf_topic>' in p
    assert '<odom_publish_frequency>30</odom_publish_frequency>' in p
    assert '<dimensions>3</dimensions>' in p
    assert 'xyz_offset' not in p                     # archived trap: body-frame post-multiply teleports


def test_lidar_3d_centered_specs_match_dog():
    b = _block(_tugbot_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    d = _block(_dog_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) == 0.0 and float(pose[1]) == 0.0   # SCAN_OFFSET_X=0 contract
    assert float(pose[2]) == 0.55                            # above the stock pole top (0.5318)
    assert '<topic>/lidar/points</topic>' in b
    bb = re.sub(r'\s+', '', _block(b, r'<lidar>(.*?)</lidar>'))
    dd = re.sub(r'\s+', '', _block(d, r'<lidar>(.*?)</lidar>'))
    assert bb == dd                                          # field-identical to the dog


def test_camera_front_forward_and_spec():
    b = _block(_tugbot_sdf(), r'<sensor name="camera_front" type="camera">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) > 0.0 and abs(float(pose[5])) < 0.3    # faces +x travel direction
    assert '<topic>/camera/front/image</topic>' in b
    assert '<width>720</width>' in b and '<height>540</height>' in b


def test_gripper_and_collisions_survive():
    sdf = _tugbot_sdf()
    assert '<link name="gripper">' in sdf            # honest rear overhang stays
    assert '<link name="gripper_hand">' in sdf
    assert sdf.count('<collision') >= 11             # base+gripper+hand set (incl. side pads)+4 wheels


def test_world_spawns_tugbot():
    world = (_ws_src() / 'tugbot_gazebo' / 'worlds'
             / 'tugbot_maze_world_20260528_clean_scaled2x.sdf').read_text()
    assert 'model://tugbot' in world
    assert 'model://mr_buggy3' not in world          # buggy assets stay in repo, not spawned
    m = re.search(r'<uri>model://tugbot</uri>\s*<name>tugbot</name>\s*<pose>([^<]+)</pose>', world)
    assert m is not None
    x, y, z = (float(v) for v in m.group(1).split()[:3])
    assert (x, y) == (-11.011, -9.025)               # entrance spawn, historic original pose
    assert z == 0.0


def test_static_tfs_point_at_tugbot_frames():
    launch = (_ws_src() / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'tugbot/base_link/lidar_3d' in launch and 'mr_buggy3/base/lidar_3d' not in launch
    assert 'tugbot/base_link/camera_front' in launch and 'mr_buggy3/base/camera_front' not in launch
    # The TRANSLATIONS must match the SDF mounts (lidar z=0.55, camera 0.20/0.40):
    # a renamed frame with a stale buggy offset would project the cloud ~0.25 m low
    # while every name-only assertion stays green (Task 1 quality-review finding).
    lidar = _block(launch, r"name='lidar_3d_static_tf'.*?arguments=\[(.*?)\]")
    assert "'--z', '0.55'" in lidar
    cam = _block(launch, r"name='camera_front_static_tf'.*?arguments=\[(.*?)\]")
    assert "'--x', '0.20'" in cam and "'--z', '0.40'" in cam


def test_bridge_unchanged_platform_agnostic():
    bridge = (_ws_src() / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    assert '/cmd_vel' in bridge
    assert '/lidar/points/points' in bridge          # cloud split-topic trap entry stays
    assert '/camera/front/image' in bridge
    assert 'diffdrive' not in bridge                 # inert topics are NOT bridged
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_tugbot_return_contract.py -q
```

Expected: FAIL — `test_stock_2d_sensors_removed`, `test_diff_drive_kept_with_odom_silenced`, `test_world_anchored_odometry_publisher`, `test_lidar_3d_centered_specs_match_dog`, `test_camera_front_forward_and_spec` fail (world/launch tests also fail; they turn green in Task 2 — that is EXPECTED here; only model-level tests must go green in this task).

- [ ] **Step 3: Model surgery on `src/tugbot_description/models/tugbot/model.sdf`**

Perform these edits (anchor-based, top to bottom):

1. **Delete the stock camera:** the whole `<link name="camera_link">` … `</link>` block AND the `<joint name="camera_link_joint" type="fixed">` … `</joint>` block.
2. **Delete scan_front:** `<link name="scan_front">` … `</link>` and `<joint name="scan_front_joint" ...>` … `</joint>`.
3. **Delete scan_back:** `<link name="scan_back">` … `</link>` and `<joint name="scan_back_joint" ...>` … `</joint>`.
4. **Delete scan_omni:** `<link name="scan_omni">` … `</link>` (this link holds many `velodyne_*` visuals — delete through ITS closing `</link>`, the one right before `<joint name="scan_omni_joint"`) and `<joint name="scan_omni_joint" ...>` … `</joint>`.
5. **Delete sensor_contact only (keep the gripper link):** inside `<link name="gripper">`, delete `<sensor name="sensor_contact" type="gpu_lidar">` … `</sensor>`.
6. **Keep:** `imu_link` + its sensor/joint, `warnign_light` (sic — stock typo, keep as-is), gripper/gripper_hand with all collisions, all four wheels.
7. **Graft sensors into `base_link`** — insert immediately before `base_link`'s closing `</link>`:

```xml
      <!-- tugbot-return graft: 16-beam 3D lidar, centered (SCAN_OFFSET_X=0
           contract), z=0.55 above the stock sensor-pole top (0.5318) so the
           structure does not occlude the rings. <lidar> block field-identical
           to the dog/buggy contract. -->
      <sensor name="lidar_3d" type="gpu_lidar">
        <pose>0 0 0.55 0 0 0</pose>
        <topic>/lidar/points</topic>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>1800</samples>
              <resolution>1.0</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.261799</min_angle>
              <max_angle>0.261799</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.05</min>
            <max>100</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
      <!-- tugbot-return graft: forward observation camera, same spec as the
           dog/buggy camera_front. Body front face is x=0.262. -->
      <sensor name="camera_front" type="camera">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <pose>0.20 0 0.40 0 0 0</pose>
        <topic>/camera/front/image</topic>
        <camera name="camera_front">
          <horizontal_fov>2.19911</horizontal_fov>
          <camera_info_topic>/camera/front/camera_info</camera_info_topic>
          <lens>
            <intrinsics>
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

8. **Silence DiffDrive odom** — in the DiffDrive plugin block replace:

```xml
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
```

with:

```xml
      <!-- Wheel-integrated odom drifts under slip; the aliasing gates snap to
           /odom on a drift-free premise. Drive role only — odom redirected to
           inert topics (world-anchored OdometryPublisher below owns /odom+/tf). -->
      <odom_topic>/diffdrive/odom_unused</odom_topic>
      <tf_topic>/diffdrive/tf_unused</tf_topic>
```

9. **Add the world-anchored OdometryPublisher** — insert immediately after the DiffDrive plugin's closing `</plugin>`:

```xml
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
```

- [ ] **Step 4: Run the model-level contract tests — green**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_tugbot_return_contract.py -q -k "not spawns and not static_tfs"
```

(`not world` would also exclude `test_world_anchored_odometry_publisher` — use `not spawns`.)

Expected: PASS (8 tests). `test_world_spawns_tugbot` / `test_static_tfs_point_at_tugbot_frames` stay red until Task 2.

- [ ] **Step 5: Verify the frozen dog-asset tests are untouched**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_lidar_3d.py src/tugbot_maze/test/test_front_camera.py -q
```

Expected: PASS (they read the anymal_c SDF, which this task must not touch).

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727/src/tugbot_description/models/tugbot/model.sdf \
        ros2_ws_tugbot_nav_20260727/src/tugbot_maze/test/test_tugbot_return_contract.py
git commit -F - <<'EOF'
feat: tugbot model surgery - modern perception on the original chassis

Stock 2D sensors removed wholesale (scan_omni published directly on
/scan and would collide with the projection chain; scan_front/back,
sensor_contact and the stock camera had no consumers). Grafted the
dog-identical 16-beam lidar_3d centered at z=0.55 and camera_front.
DiffDrive keeps its drive role only: wheel-integrated odom redirected
to inert topics; the world-anchored OdometryPublisher owns /odom+/tf
(the aliasing gates' drift-free premise). Contract test pins it all;
world/launch assertions intentionally stay red until the swap task.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 2: World/launch swap + mr_buggy3 contract re-freeze

**Files:**
- Modify: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Modify: `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py` (lines ~79, ~92)
- Modify: `src/tugbot_maze/test/test_mr_buggy3_contract.py` (two tests)

- [ ] **Step 1: Swap the world include**

In the world file, replace:

```xml
    <include>
      <uri>model://mr_buggy3</uri>
      <name>mr_buggy3</name>
      <pose>-11.011 -9.025 0.04 0 0 0</pose>
    </include>
```

with:

```xml
    <include>
      <uri>model://tugbot</uri>
      <name>tugbot</name>
      <pose>-11.011 -9.025 0.000 0.000 0.000 0.000</pose>
    </include>
```

- [ ] **Step 2: Point the static TFs at tugbot sensor frames — NAMES AND TRANSLATIONS**

In `tugbot_gazebo.launch.py`, the two static-TF nodes still carry the BUGGY
mount offsets (lidar `--z 0.30`, camera `--x 0.12 --z 0.15`). A name-only
rename would project the cloud ~0.25 m low with every frame assertion green
(Task 1 quality-review finding). Replace the full argument lists:

`lidar_3d_static_tf` arguments:

```python
            '--x', '0.0', '--y', '0.0', '--z', '0.55',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'tugbot/base_link/lidar_3d',
```

`camera_front_static_tf` arguments:

```python
            '--x', '0.20', '--y', '0.0', '--z', '0.40',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'tugbot/base_link/camera_front',
```

(gz scoped sensor frames follow `<model>/<link>/<sensor>`; tugbot's link is
`base_link`, the buggy's was `base`. Task 5's smoke verifies the live frame
string from the actual cloud header. The translations mirror the SDF mounts
pinned in Task 1: lidar `0 0 0.55`, camera `0.20 0 0.40`.)

- [ ] **Step 3: Re-freeze the two coupled buggy-contract tests**

In `test_mr_buggy3_contract.py`, replace `test_world_spawns_buggy_not_dog` with:

```python
def test_world_no_longer_spawns_buggy():
    # Re-frozen at tugbot-return: the buggy ASSET stays in the repo (pure-SDF
    # assertions above still pin it) but the world spawns the original tugbot.
    world = (_ws_src() / 'tugbot_gazebo' / 'worlds'
             / 'tugbot_maze_world_20260528_clean_scaled2x.sdf').read_text()
    assert 'model://mr_buggy3' not in world
    assert 'model://anymal_c' not in world
    assert 'model://tugbot' in world
```

and replace `test_static_tfs_point_at_buggy_frames` with:

```python
def test_static_tfs_no_longer_point_at_buggy_frames():
    # Re-frozen at tugbot-return (same convention as the dog re-freeze at
    # buggy-swap): launch TFs now belong to the tugbot contract test.
    launch = (_ws_src() / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'mr_buggy3/base/lidar_3d' not in launch
    assert 'mr_buggy3/base/camera_front' not in launch
```

- [ ] **Step 4: Run both contract files — all green now**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_tugbot_return_contract.py \
                  src/tugbot_maze/test/test_mr_buggy3_contract.py -q
```

Expected: PASS (all — tugbot contract 10/10 including world/TF; buggy contract with the two re-frozen tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
        ros2_ws_tugbot_nav_20260727/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py \
        ros2_ws_tugbot_nav_20260727/src/tugbot_maze/test/test_mr_buggy3_contract.py
git commit -F - <<'EOF'
feat: world and launch swap to tugbot, buggy contract re-frozen

World spawns model://tugbot at the historic entrance pose (z=0); static
TFs point at tugbot/base_link sensor frames. The two world/launch-coupled
buggy-contract tests re-frozen per the platform-swap convention (asset
stays pinned in-repo, spawn assertions inverted).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 3: Footprint restore + coupled-test recompute

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/footprint.py` (constants + module docstring)
- Modify: `src/tugbot_maze/test/test_footprint.py`
- Modify: `src/tugbot_maze/test/test_maze_sim.py` (6 fixtures)
- Modify: `src/tugbot_maze/test/test_wall_follow_maze_sim.py` (empirical-record adjudication)

Derived geometry used throughout this task: rectangle front 0.262, rear
−0.468, half-width 0.292. `MazeSim.collides` no-yaw bounding radius is
computed from the constants: `max(0.468, 0.262, 0.292, hypot(0.468, 0.292)=0.5516) = 0.5516`;
+ `wall_half_thickness_m 0.12` → **0.6716**.

- [ ] **Step 1: Restore the constants**

Replace the docstring and constants block at the top of `footprint.py` with:

```python
"""Original Tugbot collision footprint + sensor placement (tugbot-return; see
tugbot_description/models/tugbot/model.sdf). All in the base_link frame,
+x=forward, +y=left. Asymmetric rectangle = the honest full-body envelope
recovered verbatim from the pre-dog workspaces (20260705/20260712), rear
dominated by the gripper hand. NO padding (safety margins live in the
gates/thresholds, not in the footprint). The 16-beam lidar is grafted at the
body centre (SCAN_OFFSET_X=0) — the historic -0.1855 belonged to the removed
2D scan_omni and must NOT return."""
from __future__ import annotations
import math

FOOT_X_FRONT = 0.262     # body front (base mesh)
FOOT_X_REAR  = -0.468    # rear gripper hand
FOOT_HALF_W  = 0.292     # half-width (base mesh / wheels at 0.257)
SCAN_OFFSET_X = 0.0      # /scan projected from the centered 3D lidar
```

Everything below the constants is untouched.

- [ ] **Step 2: Update `test_footprint.py`**

Replace `test_inside_footprint` with (assertions mostly survive; comments must
tell the truth):

```python
def test_inside_footprint():
    assert inside_footprint(0.1, 0.0) is True        # mid-body
    assert inside_footprint(-0.4, 0.0) is True       # gripper overhang region (rear face -0.468)
    # exactly on the rear face (FOOT_X_REAR = -0.468): inside_footprint's rectangle test uses <=
    # on both bounds, so a boundary point is inclusive (True) -- assert that documented behavior
    # explicitly.
    assert inside_footprint(FOOT_X_REAR, 0.0) is True
    assert inside_footprint(0.5, 0.0) is False       # beyond front (front face 0.262)
    assert inside_footprint(0.0, 0.4) is False       # beyond side (half-width 0.292)
    assert inside_footprint(0.4, 0.0, margin=0.3) is True   # inflated: front+margin = 0.562 >= 0.4
```

Replace `test_footprint_covers_buggy_envelope` with:

```python
def test_footprint_covers_tugbot_envelope():
    """The footprint rectangle must dominate the original Tugbot body: base
    mesh front 0.262, rear gripper hand at -0.468, drive wheels out to
    |y|=0.257. Recovered verbatim from the pre-dog workspaces — the honest
    envelope that exposed the real ~43-61% collision rate in 2026-06."""
    assert FOOT_X_FRONT >= 0.26
    assert -FOOT_X_REAR >= 0.46
    assert FOOT_HALF_W >= 0.257
```

`test_beam_endpoint_applies_offset`, `test_front_gap_*`, `test_rear_gap_*` are
constant-relative and survive unchanged.

- [ ] **Step 3: Recompute the 6 geometry fixtures in `test_maze_sim.py`**

Apply these exact replacements:

1. `test_collides_true_near_wall_segment` — body:

```python
def test_collides_true_near_wall_segment():
    seg = [(1.0, -1.0, 1.0, 1.0)]    # vertical wall at x=1
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  robot_radius_m=0.35, wall_half_thickness_m=0.12)
    # collides(x,y) with no yaw uses the legacy bounding circle: hypot(FOOT_X_REAR,FOOT_HALF_W) =
    # hypot(0.468,0.292) = 0.5516 + margin 0.12 = 0.6716 (robot_radius_m=0.35 above is unused by
    # the no-yaw path -- see collides()).
    assert sim.collides(0.75, 0.0) is True       # 0.25 m to wall < bounding radius+margin (~0.6716)
    assert sim.collides(0.2, 0.0) is False       # 0.8 m to wall > bounding radius+margin (~0.6716)
```

2. `test_collides_rectangle_catches_rear_gripper` — body:

```python
def test_collides_rectangle_catches_rear_gripper():
    # Vertical wall at base_link x=-0.55: rear face is FOOT_X_REAR=-0.468, distance
    # 0.55-0.468 = 0.082 m < wall_half_thickness 0.12 -> collision. The gripper overhang is
    # BACK — this fixture again guards true rear contact through the exact rectangle path.
    sim = MazeSim([(-0.55, -0.30, -0.55, 0.30)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert sim.collides(0.0, 0.0, 0.0) is True
```

3. `test_collides_rectangle_tighter_on_sides_than_circle` — body:

```python
def test_collides_rectangle_tighter_on_sides_than_circle():
    # Horizontal wall at y=0.45 (to the left). Segment distance to side face (FOOT_HALF_W=0.292)
    # is 0.45-0.292 = 0.158 m > wall_half_thickness 0.12 -> NO collision, while the bounding
    # circle (0.6716) would have flagged a wall at 0.45 -- the lateral-tightness contrast
    # preserved.
    sim = MazeSim([(-0.30, 0.45, 0.30, 0.45)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert sim.collides(0.0, 0.0, 0.0) is False
```

4. `test_collides_front_wall_within_inflated_front` — body:

```python
def test_collides_front_wall_within_inflated_front():
    # Vertical wall at base_link x=0.35: front face FOOT_X_FRONT=0.262, 0.35-0.262 = 0.088
    # < 0.12 -> collision.
    sim = MazeSim([(0.35, -0.30, 0.35, 0.30)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert sim.collides(0.0, 0.0, 0.0) is True
```

5. `test_collides_yaw_aware_rotation` — body:

```python
def test_collides_yaw_aware_rotation():
    # A wall 0.55 m behind world-x (at yaw=0 rear clearance 0.55-0.468 = 0.082 m < 0.12 -> WOULD
    # collide), but the robot faces +y (yaw=+pi/2): in base_link the wall is 0.55 m to the left
    # (+y direction), clearance to side face (FOOT_HALF_W=0.292) is 0.55-0.292 = 0.258 m > 0.12
    # -> NO collision.
    sim = MazeSim([(-0.55, -0.30, -0.55, 0.30)], start_xy=(0.0, 0.0), start_yaw=0.0)
    import math
    assert sim.collides(0.0, 0.0, math.pi / 2) is False
```

6. `test_collides_yaw_none_uses_bounding_circle` — update the comment and keep
the assertions (wall at 0.30 < 0.6716 still collides):

```python
def test_collides_yaw_none_uses_bounding_circle():
    # Legacy (x,y)-only call: conservative bounding circle
    # (radius hypot(0.468,0.292)~0.5516 + margin 0.12 = ~0.6716).
    sim = MazeSim([(-0.30, -0.30, -0.30, 0.30)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert isinstance(sim.collides(0.0, 0.0), bool)
    assert sim.collides(0.0, 0.0) is True          # 0.30 < 0.6716 -> True
```

7. `test_collides_false_when_clear` — update the comment only (clearances still
clear: side 1.0−0.292=0.708 for the yaw path; 1.0 > 0.6716 for the circle).

8. `test_collides_exact_rejects_corner_near_miss` — body:

```python
def test_collides_exact_rejects_corner_near_miss():
    # A wall just off the rear-right CORNER (FOOT_X_REAR, -FOOT_HALF_W) = (-0.468, -0.292):
    # per-axis offsets (0.10, 0.10) beyond the rectangle -> corner - offsets = (-0.568, -0.392),
    # but Euclidean clearance hypot(0.10,0.10)=0.141 m > wall_half_thickness 0.12 -> NO collision.
    # A naive square-corner inflation WOULD have flagged this (both axis offsets <= 0.12). The
    # exact oracle correctly reports the ~0.14 m clearance.
    sim = MazeSim([(-0.568, -0.392, -0.567, -0.392)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert sim.collides(0.0, 0.0, 0.0) is False
```

9. `test_collides_exact_catches_true_corner_contact` — body:

```python
def test_collides_exact_catches_true_corner_contact():
    # Same corner (-0.468, -0.292), but per-axis offsets (0.06, 0.06) -> corner - offsets =
    # (-0.528, -0.352): Euclidean clearance hypot(0.06,0.06)=0.085 m < 0.12 -> real contact ->
    # True.
    sim = MazeSim([(-0.528, -0.352, -0.527, -0.352)], start_xy=(0.0, 0.0), start_yaw=0.0)
    assert sim.collides(0.0, 0.0, 0.0) is True
```

`test_step_blocked_by_wall_keeps_position_but_allows_rotation` needs no edit
(with the bigger rectangle the translation is rejected even more strongly; the
assertion set is unchanged).

- [ ] **Step 4: Run the two updated files**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_footprint.py src/tugbot_maze/test/test_maze_sim.py -q
```

Expected: PASS. If any fixture disagrees with your arithmetic, re-derive from
the constants — do NOT widen tolerances.

- [ ] **Step 5: Empirical-record adjudication — `test_wall_follow_maze_sim.py` + full suite**

Run:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_wall_follow_maze_sim.py -q
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -15
```

These are HISTORICAL-RECORD tests, footprint-coupled by design (the file
header says "revisit if any of those change"). Adjudicate, do not tune:

- `test_unsealed_left_hand_exterior_cheat_reopened_by_buggy_footprint`: the
  0.292 half-width sits between the buggy's 0.13 (cheat open) and the legged
  0.37 envelope (cheat closed). Whatever the run shows, rename the test to
  `test_unsealed_left_hand_exterior_cheat_era4_tugbot_footprint`, append an
  era-4 line to its history comment stating the OBSERVED outcome
  ("tugbot-return era (0.262/-0.468/0.292): exterior corridor <passable —
  outside > 0 | impassable — outside == 0>"), and assert the observed
  behavior.
- `test_report_faster_legitimate_hand`: the step counts and possibly the
  faster hand change with the envelope. Re-record the printed counts in the
  comment and update the `faster ==` lock ONLY after confirming both sealed
  runs stay inside (`out_l == 0 and out_r == 0` must hold, or the test is
  reporting a broken run — investigate, don't re-lock).
- Sealed interior solves / perturbation cases: if any go red, that is the
  historical legged-era behavior returning with the larger envelope. Do NOT
  weaken the follower. Record each red as a frozen failure in the suite
  baseline (commit message lists them by name next to the Task 0 quartet).
- Any OTHER suite delta (outside wall_follow + footprint files): STOP and
  report to the controller with the failure output — nothing else in the
  chain should move in this task.

- [ ] **Step 6: Commit (message carries the adjudication record)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727/src/tugbot_maze/tugbot_maze/footprint.py \
        ros2_ws_tugbot_nav_20260727/src/tugbot_maze/test/test_footprint.py \
        ros2_ws_tugbot_nav_20260727/src/tugbot_maze/test/test_maze_sim.py \
        ros2_ws_tugbot_nav_20260727/src/tugbot_maze/test/test_wall_follow_maze_sim.py
git commit -F - <<'EOF'
feat: honest tugbot footprint restored (0.262/-0.468/0.292)

Constants recovered verbatim from the pre-dog workspaces; SCAN_OFFSET_X
stays 0 (the -0.1855 belonged to the removed 2D scan_omni). Geometry
fixtures in test_maze_sim recomputed for the asymmetric rectangle
(bounding circle 0.5516+0.12). Wall-follow historical records
adjudicated for era 4:
<one line per adjudicated test with the observed outcome>
Suite after this task: <N> failed / <M> passed; new frozen entries:
<names, or "none">

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 4: Classic differential mode by default

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` (the `ackermann_drive` declare line, ~line 79)

- [ ] **Step 1: Flip the declared default**

Replace:

```python
        self.ackermann_drive = bool(self.declare_parameter('ackermann_drive', True).value)
```

with:

```python
        # tugbot-return: classic differential mode (in-place turns, x1 timeouts,
        # rotate-to-face centering). The Ackermann primitives stay in the code,
        # dormant — pass ackermann_drive:=true to re-engage them.
        self.ackermann_drive = bool(self.declare_parameter('ackermann_drive', False).value)
```

(No launch file passes `ackermann_drive` — verified; the declare default is
the single source of truth.)

- [ ] **Step 2: Regression-run the motion/solver test files**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/test_maze_motion.py src/tugbot_maze/test/test_maze_motion_sim.py \
                  src/tugbot_maze/test/test_ackermann_maneuvers.py src/tugbot_maze/test/test_flood_fill_maze_sim.py \
                  src/tugbot_maze/test/test_hop_controller.py -q
```

Expected: PASS. (These construct `MazeMotion(ackermann=...)` explicitly; the
Ackermann suites keep exercising the dormant primitives. If the flood_fill
offline solve shifts outcome, STOP and report — the offline harness runs
diff-mode already, so a delta here means a real regression, not the flip.)

- [ ] **Step 3: Full suite from the workspace root — compare against the Task 3 record**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -8
```

Expected: identical counts to the end of Task 3.

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: classic differential mode default (ackermann_drive False)

Single source of truth at the solver declare_parameter; no launch
override exists. Ackermann primitives remain dormant but tested.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 5: Build + smoke (plugin semantics recorded)

**Files:**
- Create: `tools/tugbot_odom_anchor_check.py` (repo-root `tools/` of ws 20260727)
- Create: `tools/verify_tugbot_model.sh`

NOTE: `tools/` here means `ros2_ws_tugbot_nav_20260727/tools/`.

- [ ] **Step 1: Rebuild so the SDF/launch edits land in install**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install
```

- [ ] **Step 2: Write the odom-anchor check node**

Create `tools/tugbot_odom_anchor_check.py`:

```python
#!/usr/bin/env python3
"""World-anchored-odom smoke check (tugbot-return Task 5). Drives a gentle arc
via /cmd_vel for DRIVE_S seconds, then compares the /odom displacement against
the ground-truth displacement from /gt/dynamic_pose (EVAL-ONLY readout — red
line: gt never feeds control; here it only grades the odom plugin).

PASS = displacement mismatch < 0.05 m AND both sources actually moved > 0.3 m
(catches both a drifting wheel-integrated odom and a silent/dead plugin).
Exit code 0 on PASS, 1 on FAIL, 2 on missing data."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

DRIVE_S = 6.0
SETTLE_S = 2.0
V, W = 0.25, 0.25        # gentle arc — diff drive, well inside accel limits
TOL_M = 0.05
MIN_TRAVEL_M = 0.3


class AnchorCheck(Node):
    def __init__(self):
        super().__init__('tugbot_odom_anchor_check')
        self.odom_xy = None
        self.gt_xy = None
        self.odom0 = None
        self.gt0 = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(TFMessage, '/gt/dynamic_pose', self._gt, 10)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def _gt(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'tugbot':
                tr = t.transform.translation
                self.gt_xy = (tr.x, tr.y)


def main():
    rclpy.init()
    node = AnchorCheck()
    deadline = time.monotonic() + 20.0
    while (node.odom_xy is None or node.gt_xy is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom_xy is None or node.gt_xy is None:
        print(f'ANCHOR_CHECK missing data: odom={node.odom_xy} gt={node.gt_xy}')
        return 2
    node.odom0, node.gt0 = node.odom_xy, node.gt_xy

    cmd = Twist()
    cmd.linear.x, cmd.angular.z = V, W
    t_end = time.monotonic() + DRIVE_S
    while time.monotonic() < t_end:
        node.pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.pub.publish(Twist())                      # stop
    t_end = time.monotonic() + SETTLE_S
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    d_odom = (node.odom_xy[0] - node.odom0[0], node.odom_xy[1] - node.odom0[1])
    d_gt = (node.gt_xy[0] - node.gt0[0], node.gt_xy[1] - node.gt0[1])
    err = math.hypot(d_odom[0] - d_gt[0], d_odom[1] - d_gt[1])
    trav = math.hypot(*d_gt)
    print(f'ANCHOR_CHECK d_odom={d_odom} d_gt={d_gt} err={err:.4f} travel={trav:.3f}')
    ok = err < TOL_M and trav > MIN_TRAVEL_M and math.hypot(*d_odom) > MIN_TRAVEL_M
    print(f'ANCHOR_CHECK {"PASS" if ok else "FAIL"}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 3: Write the smoke orchestrator**

Create `tools/verify_tugbot_model.sh`:

```bash
#!/usr/bin/env bash
# tugbot-return smoke (Task 5): model/plugin semantics in the REAL maze world,
# headless, no solver in the loop. Checks, in order:
#   1. gz has NO /scan topic (stock scan_omni really gone -- the projection
#      chain must be the only /scan source once the full stack runs)
#   2. /lidar/points/points delivers a PointCloud2 whose frame_id is the
#      tugbot/base_link/lidar_3d scoped frame (records the LIVE string)
#   3. /diffdrive/odom_unused exists on the gz graph (odom redirect took)
#   4. world-anchored odom: drive an arc, /odom displacement == gt displacement
#      (tools/tugbot_odom_anchor_check.py; gt readout is EVAL-ONLY)
# Same PRIME/env discipline as the run scripts.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

export DISPLAY="${DISPLAY:-:1}"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

kill_sim() {
    for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge robot_state_publisher \
               static_transform_publisher tugbot_odom_anchor_check; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}
kill_sim; sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))

LOGDIR="$(mktemp -d /tmp/tugbot_smoke.XXXXXX)"
echo "[SMOKE] logdir=$LOGDIR DOMAIN=$ROS_DOMAIN_ID"

ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true \
    > "$LOGDIR/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 25

PASS=0; FAIL=0
check() {  # check <name> <0-for-pass>
    if [ "$2" -eq 0 ]; then echo "[SMOKE] PASS $1"; PASS=$((PASS+1));
    else echo "[SMOKE] FAIL $1"; FAIL=$((FAIL+1)); fi
}

gz topic -l > "$LOGDIR/gz_topics.txt" 2>&1
! grep -qx "/scan" "$LOGDIR/gz_topics.txt"; check "no-model-/scan-topic" $?
grep -q "/lidar/points/points" "$LOGDIR/gz_topics.txt"; check "cloud-split-topic" $?
grep -q "/diffdrive/odom_unused" "$LOGDIR/gz_topics.txt"; check "diffdrive-odom-redirect" $?

timeout 15 ros2 topic echo --once /lidar/points --field header.frame_id \
    > "$LOGDIR/cloud_frame.txt" 2>&1
grep -q "tugbot/base_link/lidar_3d" "$LOGDIR/cloud_frame.txt"; check "live-lidar-frame" $?
echo "[SMOKE] live cloud frame: $(tr -d '\n' < "$LOGDIR/cloud_frame.txt")"

timeout 60 python3 "${SCRIPT_DIR}/tugbot_odom_anchor_check.py" \
    | tee "$LOGDIR/anchor.txt"
grep -q "ANCHOR_CHECK PASS" "$LOGDIR/anchor.txt"; check "world-anchored-odom" $?

kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 3; kill_sim
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[SMOKE] result: PASS=$PASS FAIL=$FAIL (logs: $LOGDIR)"
[ "$FAIL" -eq 0 ]
```

Then `chmod +x tools/verify_tugbot_model.sh tools/tugbot_odom_anchor_check.py`.

- [ ] **Step 4: Run the smoke (FOREGROUND)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
tools/verify_tugbot_model.sh
```

Expected: `PASS=5 FAIL=0`. If `live-lidar-frame` fails, read the printed live
frame string and fix the static TF child frames in
`tugbot_gazebo.launch.py` + the contract test to the OBSERVED string (the
`<model>/<link>/<sensor>` convention is an expectation, the sim is the
authority), then rebuild and re-run the smoke.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260727/tools/verify_tugbot_model.sh \
        ros2_ws_tugbot_nav_20260727/tools/tugbot_odom_anchor_check.py
git commit -F - <<'EOF'
feat: tugbot smoke harness - plugin semantics recorded

Headless maze-world smoke: no model-level /scan topic, cloud split-topic
present, DiffDrive odom redirect verified on the gz graph, live lidar
frame string recorded, world-anchored odom graded against the eval-only
gt readout via a driven arc (displacement mismatch < 0.05 m).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 6: Headless acceptance ×2 (MAIN SESSION ONLY)

**Files:** none (runs produce untracked artifacts under `ros2_ws_tugbot_nav_20260727/log/`).

This task is executed by the CONTROLLER in the main session — long sim runs
die with subagents. Subagents must not start it.

- [ ] **Step 1: Two headless runs**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
tools/run_flood_fill_maze.sh 1800 true false
tools/run_flood_fill_maze.sh 1800 true false
```

(1800 s budget: tugbot cruise 0.3 m/s with in-place turns finished in ~510-560 s
historically; headroom, not a prediction.)

- [ ] **Step 2: Gates per run** (ART = the printed `ARTIFACT_DIR`)

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260727
cat $ART/result.txt                                    # EXIT_REACHED required
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
python3 tools/replay_collision_oracle.py $ART          # 0.000% required
grep -ac "POS_GATE" $ART/launch.log                    # 0 or 1 (single recovery)
grep -ac "YAW_GATE" $ART/launch.log                    # 0 or 1 (single recovery)
grep -ac "ERROR" $ART/launch.log                       # 0 required
grep -a "CLOUDMAP voxels=" $ART/launch.log | tail -3   # final voxel count
grep -a "stale_dropped" $ART/launch.log | tail -1      # accumulator health
```

- [ ] **Step 3: Saturation-paradox check + honest comparison**

From each run's `CLOUDMAP voxels=` series compute new-voxel growth over the
run's final third vs its middle third; late growth must DECAY (sustained
late-run growth = ghost signal — first-class forensic finding; if it fires,
STOP and diagnose before proceeding, per the ackermann-physics playbook).
Record per-run final voxels against the buggy baseline
(GUI 449,948 / headless 454,733) and the dog baseline (2.15-2.47M). NO
tuning loop if tugbot lands higher — honest comparison per the spec's
adjudication 2.

- [ ] **Step 4: Record** the numbers in the working notes for the Task 7
addendum. No commit.

---

### Task 7: Results addendum + GUI gate

**Files:**
- Modify: `docs/superpowers/specs/2026-07-23-tugbot-return-design.md` (append addendum)

- [ ] **Step 1: Append the addendum** — a `## Addendum` section with: Task 0
baseline + final suite accounting (every frozen-name delta listed), smoke
results (live lidar frame string, anchor-check numbers), headless×2 table
(result / duration / oracle / gate counts / final voxels / saturation check),
and the honest three-platform comparison (dog / buggy / tugbot voxel counts).

- [ ] **Step 2: Commit docs**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-23-tugbot-return-design.md
git commit -F - <<'EOF'
docs: tugbot-return addendum - smoke + headless acceptance results

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

- [ ] **Step 3: STOP — GUI acceptance is user-triggered.** Report the
comparison table and await the user's explicit go-ahead for the GUI+RViz run
(`tools/run_flood_fill_maze.sh 1800 false true`), then the standard finishing
flow (final whole-branch review → merge decision) after the user's verdict.

---

## Self-review record

- Spec coverage: model surgery §1→Task 1; motion flip §2→Task 4; footprint §3→Task 3; world/bridge/launch §4→Task 2 (bridge verified platform-agnostic — no edit needed, contract asserts it); contract tests §5→Tasks 1-2; gates §6→Tasks 0/5/6 + GUI in Task 7; risks §7 embedded in Tasks 3/5/6.
- Placeholders: the two commit-message `<paste/one line per...>` slots are deliberate observation-recording slots, not plan gaps; everything else is concrete code/commands.
- Type consistency: contract helper names (`_ws_src`, `_block`) match the buggy-contract idiom; constants referenced in Tasks 3-5 all trace to the Task 3 block; `verify_tugbot_model.sh` calls `tugbot_odom_anchor_check.py` by its defined name.
