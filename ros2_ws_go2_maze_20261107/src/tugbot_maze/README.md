# tugbot_maze：Go2 在线导航、步态与验收

[包索引](../README.md) · [成功经验](../../../docs/experience/2026-11-07-go2-maze-success.md) · [验收报告](../../../docs/validation/2026-11-07-go2-maze/README.md)

包名沿用原项目。当前入口为 `go2_maze_solver`、`locomotion_controller`、`go2_maze_audit`，由 go2_maze_bringup 统一加载；原有 Nav2 pilot、wall-follow 和其他实验算法保留为历史代码。

## 当前分工

| 源码 | 职责 |
| --- | --- |
| flood_fill_brain.py / maze_motion.py | 在线墙体记忆、格栅路由、居中/转向/步进/回退 |
| flood_fill_solver.py | ROS/TF 与在线增长参照定位；完成动作通过 _maze_completed 扩展 |
| go2_maze_solver.py | Go2 运动超时、出口精确收尾与持续站立 |
| go2_exit_approach.py | 只依赖在线位姿、已知出口和新鲜雷达的收尾命令 |
| legged/ / locomotion_controller.py | Go2 FK/IK、对角连续步态、姿态修正、加减速和失联停车 |
| scan_slice_projector.py / slice_to_scan.py | 16 线点云到 900 束扫描 |
| cloud_map_accumulator.py | RViz 在线三维点云累积 |
| footprint.py / go2_maze_audit.py | Go2 保守足迹与原生位姿独立验收 |

当前已知入口把 Gazebo 世界坐标里程计平移到 map。online_slam 只以外围边界和在线发现内部墙作参照；原项目完整墙图 scan_match 模式仍在，但不是 Go2 launch 的选项默认值。

## 运动与出口

Go2 的指令上限为 0.25 m/s / 0.5 rad/s；0.6 仿真秒无新命令时收步站立。巡航 0.22 m/s，hop 40 秒、转向 15 秒、回退 25 秒，按物理运动阶梯标定。

到离散出口格不会立刻完成。Go2 使用不超过 0.5 秒的新鲜扫描，低速上限 0.12 m/s，进入 `exit_radius-0.35`（默认 0.85 m）的内部目标区；连续保持两秒才发 EXIT_REACHED。缺少位姿/扫描时零命令；30 秒收尾超时进入 exit_failed，保持站立。

## 独立实际验收

验收器不发布运动命令。它接收原生动态位姿，并与另一条原生 odom 流比对以检查模型选择/桥接的一致性；二者都来源于 Gazebo。完整墙图仅在验收使用，不能把 ICP 修正后的估计轨迹当作真实碰撞证据。

成功要求：出口事件已发生、真实出口距离≤1.2 m、三维线速度范数<0.03 m/s 与角速度范数<0.08 rad/s 连续两秒、至少 100 个有效样本、零几何碰撞与非法越界、原生输出最大比对差<0.05 m、最低高度>0.22 m、最大总倾角≤0.1 rad。

足迹为 x=[-0.42,0.37]、|y|≤0.25 m；扫掠步长≤1 cm / 0.02 rad。它覆盖有限步态/姿态抽样与余量，不表示任意跌倒姿态下的完整几何或接触力认证。

`ground_truth.csv` 保留原始记录，`status.json` 为在线状态。严格重放还检查样本数完整、原始最大倾角及最终实际出口位置。成功与失败原始数据应一起保留。

## 检查

在工作区根目录运行 `./tools/test_go2_core.sh`。208 项核心检查包含官方 URDF 对照、关节限位、步态/停步、后小腿足迹、收尾反例、旧 PASS 拒绝与重复中断清理。

历史广泛测试的 7 项失败和未运行的 Nav2 pilot 已在验收报告列出。不要为让全量历史数字变绿而放宽当前出口、碰撞或倾角阈值。
