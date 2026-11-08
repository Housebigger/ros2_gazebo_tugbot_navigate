# Go2 工具与复核顺序

[工作区](../README.md) · [经验总结](../../docs/experience/2026-11-07-go2-maze-success.md)

以下命令从已构建的工作区根目录执行。Python 工具前先 `source setup.bash`；单独执行测试脚本不会把环境加载到父终端。

| 工具 | 用法与前置条件 |
| --- | --- |
| test_go2_core.sh | `./tools/test_go2_core.sh`；当前核心 218 项 |
| check_go2_motion.py | 先启动 `autonomous:=false` 的 go2_test.sdf，再执行；会发送真实运动命令 |
| check_go2_cameras.py | 只读三路实时图像、内参、TF 与 odom；`python3 tools/check_go2_cameras.py <全新目录> --seconds 90 --require-motion`，需已有自主走转实例 |
| run_checked_maze.sh | 全新目录、自动监控、只清理本轮进程、关闭后严格重放 |
| watch_maze_run.py | 监督器内部工具，读在线状态，不发布控制命令 |
| replay_go2_audit.py | 只读原始轨迹并重算，写 replay_audit.json；正式验证加 --require-success |
| plot_go2_run.py | 绘制已重放数据，需要可选 python3-matplotlib |
| check_go2_envelope.py | 官方碰撞体投影抽样；不启动机器人 |
| generate_go2_back_marker.py | 重生成两处相同的背部绿色箭头 DAE；纯视觉资源，之后重构建并重新加载模型 |
| generate_go2_model.py | 从官方 Xacro 的纯 SDF 转换生成原生模型与两份 world |
| center_twist_decompose.py / map_twist_metric.py | 继承的纯分析辅助，保留历史测试引用，不是当前控制入口 |

## 自动验收

相机检查输出起止 PNG 和 `camera_check.json`。除尺寸/内参外，还检查图像与跟随 TF 的完整运行窗口、最终帧/TF 新鲜度、时间单调性、流中断、同一仿真时钟、实际跟随偏移、光轴与地平线。画面变化使用区域均值抑制传感器噪声。`--require-motion` 要求实际行走超过 2 m、航向范围超过 0.4 rad；站立检查去掉该选项。默认墙钟超时 360 s，可用 `--wall-timeout` 调整；脚本不控制机器人。默认按固定世界方向验收；若手动将 SDF 改回 follow_yaw=true，检查器需加 `--follow-yaw`。原生几何检查另运行 `ctest --test-dir build/go2_observer_cameras --output-on-failure`。

```bash
./tools/run_checked_maze.sh
# 显式目录及 GUI：第一个参数必须是全新证据目录
./tools/run_checked_maze.sh "$PWD/log/runs/my-go2-run" gui:=true rviz:=true
```

无参数时创建唯一目录。目录非空会在启动仿真前拒绝；监督器不接受启动前的旧 status.json。上限 1800 仿真秒或 3600 墙钟秒，不通过、无有效原生位姿或超时均返回失败。

脚本通过完整重放才正常返回成功。重复中断清理有回归覆盖；本轮物理结果的严格重放是关闭后单独执行并归档的，随后完成了清理脚本修正。

## 原始数据复核与绘图

为保持版本化证据原样，可以先复制一份再复核：

```bash
source setup.bash
check_dir="$(mktemp -d /tmp/go2-audit-copy-XXXXXX)"
cp ../docs/validation/2026-11-07-go2-maze/accepted/{ground_truth.csv,status.json,replay_audit.json} "$check_dir/"
python3 tools/replay_go2_audit.py "$check_dir" --require-success
MPLCONFIGDIR=/tmp/go2-mpl python3 tools/plot_go2_run.py "$check_dir"
```

不加 `--require-success` 只要求几何/姿态检查通过，不能据其退出码宣布整轮通关。严格模式还要求在线出口停稳成功、样本数一致和最终实际出口距离。

## 几何与重生成

```bash
source setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/check_go2_envelope.py "$PWD"
xacro install/go2_description/share/go2_description/urdf/go2.urdf.xacro > /tmp/go2-native.urdf
gz sdf -p /tmp/go2-native.urdf > /tmp/go2-native.sdf
python3 tools/generate_go2_model.py /tmp/go2-native.sdf
```

生成器会改写当前模型/world 源文件，修改后重新构建和验证。足迹抽样输出不是任意姿态的安全证明，不能用改变足迹或增大出口半径掩盖碰撞/停车失败。

本轮完整通关数据归档在 docs/validation/2026-11-07-go2-three-cameras/accepted（相对仓库根目录）；重放前复制到新目录。此快照在首次成功后封存，排除了之后在出口的长时间空闲记录。
