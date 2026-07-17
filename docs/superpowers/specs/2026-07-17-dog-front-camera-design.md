# Dog Front Camera Design (2026-07-17)

## 背景与目标

腿式行走阶段(20260717,merge `31eb9de`)完成后,ANYmal C 只有 2D 雷达 + 4 盏探照灯。
本阶段把摄像头**加回来**:给狗装 1 个前向 RGB 相机,纯观察输出——Gazebo 渲染 →
ROS 图像话题 → RViz 第一视角画面。为后续视觉阶段打地基。

**定位:只做观察输出。** 导航链一行不动(仍是纯雷达 ICP,`pose_source=online_slam`);
验收 = 画面正确 + 迷宫全回归不退化。

## 工作空间

- 新 workspace **`ros2_ws_tugbot_nav_20260718`**,完整克隆 `ros2_ws_tugbot_nav_20260717`
  (20260717 名字已被腿式阶段占用,按惯例顺延)。
- 构建必须 `colcon build --symlink-install`(plain build 会破坏 `maze_sim` 数据路径,20260705 教训)。

## 关键几何事实(实现前必读)

- **本模型行进方向 = 模型 +x**。实锤:`legged/trajectory.py` 支撑相足端从 +x 漂向 −x
  (蹬地),vx>0 → 机体沿 +x 平移;solver 航向 = odom yaw = body +x。
- **原版 CERBERUS 约定"前"= −x**(front_laser 在 x=−0.327,7 个相机全集中在 −x 端)。
  因此原版 7 个相机的姿态**任何一个都不能照搬**——照搬会朝行进反方向拍。
- 既有小怪癖:`light_forward` 在 (−0.41, 0, 0.265) 朝 −x 照 = 照着行进反方向,
  是同一约定遗留。**本阶段记录在案、不修**(灯光是纯视觉件,与验收无关)。

## 组件 1:相机传感器(SDF)

文件:`src/tugbot_description/models/anymal_c/model.sdf`,`base` link 内新增:

- `<sensor name="camera_front" type="camera">`
- **规格照抄原版 CERBERUS `camera_4`**(tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1
  /model.sdf 行 780 起):update_rate 20、hfov 2.19911(≈126° 广角)、720×540、
  clip 0.02–100 m、高斯噪声 stddev 0.007、内参 fx=fy=183.43 / cx=360.5 / cy=270.5、
  always_on 1。保持"传感器带原厂参数"的项目风格。
- **姿态全新定义:`<pose>0.45 0 0.20 0 0 0</pose>`,光轴朝 +x**(行进方向),
  装在机身 +x 端顶部,不被机壳/腿遮挡(实现时以 GUI/抓帧确认视野干净,
  必要时微调 z,决策记附记)。
- gz 话题显式命名(目标:ROS 侧 `/camera/front/image` + `/camera/front/camera_info`;
  gz 侧 topic/camera_info_topic 的确切 SDF 写法以 `gz topic -l` 实测为准,不猜)。

## 组件 2:桥接 + TF + RViz

- **桥接**:`src/tugbot_gazebo/config/tugbot_bridge.yaml` 新增两条 GZ→ROS 单向映射:
  `gz.msgs.Image` → `sensor_msgs/msg/Image`,`gz.msgs.CameraInfo` →
  `sensor_msgs/msg/CameraInfo`。不引入 ros_gz_image(单相机 720×540@20Hz 用
  parameter_bridge 足够,零新依赖——已评审过的方案 A)。
- **静态 TF**:照 `scan_omni` 先例,在 `tugbot_gazebo.launch.py` 加
  `static_transform_publisher`:`base_link` → 相机 frame(名字与 gz 图像消息
  header.frame_id 实测一致),平移/旋转与 SDF 姿态一致。
- **RViz**:默认 RViz 配置(`src/tugbot_bringup/rviz/`)加 **Image 面板**订阅
  `/camera/front/image`,与现有 SelfBuiltMap 视图并存。

## 数据流

```
gz camera_front (渲染, 20Hz) → gz topic → ros_gz_bridge → /camera/front/image
                                                        → /camera/front/camera_info
RViz Image 面板 ← /camera/front/image      (导航链不订阅任何图像话题)
```

## 验收标准(全回归门槛)

1. **程序化**:`/camera/front/image` hz ≈20、encoding/尺寸 720×540、抓一帧验证
   非全黑/非常数(相机真的在渲染场景)。
2. **回归守卫测试**:新增 pytest 解析 model.sdf,断言 `camera_front` 存在、
   pose x>0、yaw≈0(防"照搬原版 −x 姿态"回归);既有套件不新增失败
   (基线:7 个既有失败名单不变,411 pass / 3 xfail)。
3. **完整迷宫回归**:headless ×1 + GUI(带 RViz,**必须 PRIME offload**)×1,
   均 EXIT_REACHED、oracle `replay_collision_oracle.py` 0.000%、ICP rms 与基线
   同量级(中位 ~0.03)。渲染相机与 gpu_lidar 共用渲染引擎(headless 也吃 GPU),
   这是 GPU 余量的真实检验。
4. **人工**:RViz Image 面板看到狗行进的第一视角,用户确认。

## 风险与预案

- **GPU 争用**(本机主风险):若回归中 ICP 劣化/oracle 出现假碰撞,优先降相机
  帧率 20→10 Hz,其次降分辨率;**不动导航参数**。
- GUI/RViz 运行硬要求(既有):`export DISPLAY=:1 __NV_PRIME_RENDER_OFFLOAD=1
  __GLX_VENDOR_LIBRARY_NAME=nvidia`。

## 明确不做

- 不接入导航/避障(导航链冻结);不加第 2 个相机;不动 4 盏灯(含 light_forward
  朝向怪癖);不修 scan_match(继承缺口);不做图像录制/compressed 传输。

## 附记(实现期决策记录)

1. gz 话题名实测与配置一致(/camera/front/image、/camera/front/camera_info),无需回退;frame_id 实测 anymal_c/base/camera_front,静态 TF 与 checker 断言均用此名。
2. verify_front_camera.sh 必须显式 world_sdf:=迷宫世界 —— tugbot_gazebo.launch.py 默认世界是 tugbot_empty_world.sdf(无机器人)。
3. front_camera_check.py 非常数检查用全帧等距采样(data[::39]):迷宫无顶+126° 广角,顶部 ~50 行是真实天空带(实测仅 4 个字节值),头部采样会误杀正常相机。
4. 评审加固:verify_front_camera.sh 补 trap 清理(EXIT/INT/TERM)、kill 列表含 static_transform_publisher、/dev/shm/fastrtps_* 清理;boot 等待用可中断 sleep(前台 sleep 会吞挂起的 INT trap,实测)。
5. 本机直接 Bash 调 pkill -f "gz sim" 会自杀(wrapper eval 串含同名子串);变体:同一调用里 heredoc 写含该模式的脚本并执行同样自杀——写脚本与执行必须分两次 Bash 调用。
6. ⚠️ headless+相机首战失败记录:run 224928 EXIT_REACHED 但 oracle 4.225%(6/142,全为 ICP nan/退化 tick 上的单点闪报,姿态全程 nominal,非真实碰撞);复跑 233041 直接 TIMEOUT 活锁(111 STALL,oracle 大规模假阳性)。历史"6.9% headless 方差"说法经核查为误分类——那是 PRIME 修复前的 GUI 跑;真实 headless 无相机基线 2/2 全 0.000%。
7. 根因(A/B 实锤):headless 渲染落在 AMD 610M 核显(nvidia-smi 显示 4070 空闲);gpu_lidar 单独可承受,+相机 720×540@20Hz 随机压垮 → ICP nan 串 → 假碰撞/活锁。修复 = DISPLAY=:1 + __NV_PRIME_RENDER_OFFLOAD=1 + __GLX_VENDOR_LIBRARY_NAME=nvidia(gz server 以 533MiB 落上 4070);纯 __EGL_VENDOR_LIBRARY_FILENAMES 钉定无效。已固化进 run_flood_fill_maze.sh 与 verify_front_camera.sh(defaults-if-unset)。**本机从此所有带相机运行(含 headless)都吃此要求**。
8. 裁决跑(PRIME headless ×2):run 004150 EXIT_REACHED、oracle 0.000%、rms median 0.031、nan 82/135;run 005332 EXIT_REACHED、oracle 0.000%、rms median 0.024、nan 70/136;0 摔倒。均在无相机基线域(median 0.026–0.034,nan 51–65%)。相机 20→10Hz 回退预案未动用。
