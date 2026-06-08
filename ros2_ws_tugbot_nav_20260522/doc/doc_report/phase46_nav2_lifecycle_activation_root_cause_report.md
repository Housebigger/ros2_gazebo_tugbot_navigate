# Phase46 Nav2 Lifecycle Activation Root-Cause Diagnostics Report

## 结论

Phase46 bounded Nav2 lifecycle activation diagnostics 已完成。

最终分类：`NAV2_LIFECYCLE_ACTIVE_RECOVERED`

分类理由：`core Nav2 lifecycle nodes are active and /navigate_to_pose action server exists`

Phase45 人工验收通过，结论保持：`LIFECYCLE_NOT_ACTIVE`。本报告不声明自主探索成功，也不继续 `maze_explorer` / clearance root-cause 分析。

## Guardrails

- 未修改 Nav2/MPPI/controller 参数。
- 未修改 `maze_explorer`。
- 未运行 `maze_explorer` / `maze_goal_monitor` / `frontier_explorer`。
- 未使用旧 scaffold world/map。
- 未声明自主探索成功。
- 仅新增诊断脚本、测试、报告；未做 launch wiring 修复。
- cleanup 已执行，最终 live process check 与 cleanup artifact 为空。

## Active world / launch chain

- active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- manual launch: `src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py`
- Nav2 bringup: `nav2_bringup/launch/navigation_launch.py`
- Nav2 params: `src/tugbot_navigation/config/nav2_slam_params.yaml`
- SLAM params: `src/tugbot_navigation/config/slam_toolbox_params.yaml`

## Phase45 input carried forward

Phase45 classified the previous bounded run as `LIFECYCLE_NOT_ACTIVE`: Nav2 nodes and `/navigate_to_pose` existed, but lifecycle sampled inactive in the shorter capture window.

Phase46 specifically checked lifecycle manager presence, manager params, `node_names`, transition services, before/after lifecycle state, and whether the bounded wait simply needed more time.

## Runtime command

```bash
PHASE46_RUNTIME_SEC=5 PHASE46_STARTUP_WAIT_SEC=45 PHASE46_POST_CAPTURE_WAIT_SEC=10 PHASE46_HEADLESS=true PHASE46_USE_RVIZ=false tools/run_phase46_nav2_lifecycle_activation_root_cause.sh
```

## Key evidence

### Lifecycle states before/after

```text
## before_optional_wait
/lifecycle_manager_navigation: missing
/controller_server: active [3]
/planner_server: active [3]
/bt_navigator: active [3]
/behavior_server: active [3]
/waypoint_follower: active [3]
/velocity_smoother: active [3]
/smoother_server: active [3]
/route_server: active [3]
/collision_monitor: active [3]
/docking_server: active [3]
/slam_toolbox: active [3]
## after_optional_wait
/lifecycle_manager_navigation: missing
/controller_server: active [3]
/planner_server: active [3]
/bt_navigator: active [3]
/behavior_server: active [3]
/waypoint_follower: active [3]
/velocity_smoother: active [3]
/smoother_server: active [3]
/route_server: active [3]
/collision_monitor: active [3]
/docking_server: active [3]
/slam_toolbox: active [3]
```

### NavigateToPose / goal_pose

`navigate_to_pose_action_info.txt`:

```text
Action: /navigate_to_pose
Action clients: 3
    /bt_navigator
    /waypoint_follower
    /docking_server
Action servers: 1
    /bt_navigator
```

`goal_pose_topic_info.txt`:

```text
Type: geometry_msgs/msg/PoseStamped
Publisher count: 0
Subscription count: 1
```

### lifecycle_manager_navigation params

```text
# lifecycle_manager_navigation params
  /bond_disable_heartbeat_timeout
  attempt_respawn_reconnection
  autostart
  bond_respawn_max_duration
  bond_timeout
  diagnostic_updater.period
  diagnostic_updater.use_fqn
  node_names
  qos_overrides./clock.subscription.depth
  qos_overrides./clock.subscription.durability
  qos_overrides./clock.subscription.history
  qos_overrides./clock.subscription.reliability
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time

# autostart
Boolean value is: True

# node_names
String values are: ['controller_server', 'smoother_server', 'planner_server', 'route_server', 'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower', 'docking_server']

# use_sim_time
Boolean value is: True
```

### managed node_names

```text
String values are: ['controller_server', 'smoother_server', 'planner_server', 'route_server', 'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower', 'docking_server']
```

### transition service evidence

```text
# lifecycle service list
/behavior_server/change_state
/behavior_server/get_state
/bt_navigator/change_state
/bt_navigator/get_state
/collision_monitor/change_state
/collision_monitor/get_state
/controller_server/change_state
/controller_server/get_state
/docking_server/change_state
/docking_server/get_state
/global_costmap/global_costmap/change_state
/global_costmap/global_costmap/get_state
/lifecycle_manager_navigation/describe_parameters
/lifecycle_manager_navigation/get_parameter_types
/lifecycle_manager_navigation/get_parameters
/lifecycle_manager_navigation/get_type_description
/lifecycle_manager_navigation/is_active
/lifecycle_manager_navigation/list_parameters
/lifecycle_manager_navigation/manage_nodes
/lifecycle_manager_navigation/set_parameters
/lifecycle_manager_navigation/set_parameters_atomically
/local_costmap/local_costmap/change_state
/local_costmap/local_costmap/get_state
/planner_server/change_state
/planner_server/get_state
/route_server/change_state
/route_server/get_state
/slam_toolbox/change_state
/slam_toolbox/get_state
/smoother_server/change_state
/smoother_server/get_state
/velocity_smoother/change_state
/velocity_smoother/get_state
/waypoint_follower/change_state
/waypoint_follower/get_state

# manager node info
/lifecycle_manager_navigation
  Subscribers:
    /bond: bond/msg/Status
    /clock: rosgraph_msgs/msg/Clock
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /bond: bond/msg/Status
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /lifecycle_manager_navigation/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lifecycle_manager_navigation/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lifecycle_manager_navigation/get_parameters: rcl_interfaces/srv/GetParameters
    /lifecycle_manager_navigation/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /lifecycle_manager_navigation/is_active: std_srvs/srv/Trigger
    /lifecycle_manager_navigation/list_parameters: rcl_interfaces/srv/ListParameters
    /lifecycle_manager_navigation/manage_nodes: nav2_msgs/srv/ManageLifecycleNodes
    /lifecycle_manager_navigation/set_parameters: rcl_interfaces/srv/SetParameters
    /lifecycle_manager_navigation/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
    /behavior_server/change_state: lifecycle_msgs/srv/ChangeState
    /behavior_server/get_state: lifecycle_msgs/srv/GetState
    /bt_navigator/change_state: lifecycle_msgs/srv/ChangeState
    /bt_navigator/get_state: lifecycle_msgs/srv/GetState
    /collision_monitor/change_state: lifecycle_msgs/srv/ChangeState
    /collision_monitor/get_state: lifecycle_msgs/srv/GetState
    /controller_server/change_state: lifecycle_msgs/srv/ChangeState
    /controller_server/get_state: lifecycle_msgs/srv/GetState
    /docking_server/change_state: lifecycle_msgs/srv/ChangeState
    /docking_server/get_state: lifecycle_msgs/srv/GetState
    /planner_server/change_state: lifecycle_msgs/srv/ChangeState
    /planner_server/get_state: lifecycle_msgs/srv/GetState
    /route_server/change_state: lifecycle_msgs/srv/ChangeState
    /route_server/get_state: lifecycle_msgs/srv/GetState
    /smoother_server/change_state: lifecycle_msgs/srv/ChangeState
    /smoother_server/get_state: lifecycle_msgs/srv/GetState
    /velocity_smoother/change_state: lifecycle_msgs/srv/ChangeState
    /velocity_smoother/get_state: lifecycle_msgs/srv/GetState
    /waypoint_follower/change_state: lifecycle_msgs/srv/ChangeState
    /waypoint_follower/get_state: lifecycle_msgs/srv/GetState
  Action Servers:

  Action Clients:
```

## Interpretation

Phase46 shows the previous `LIFECYCLE_NOT_ACTIVE` symptom is not reproduced when the bounded startup window is extended to 45s plus an extra 10s before/after lifecycle capture:

- Core Nav2 lifecycle nodes are `active [3]`:
  - `/controller_server`
  - `/planner_server`
  - `/bt_navigator`
  - `/behavior_server`
  - `/waypoint_follower`
  - `/velocity_smoother`
  - `/smoother_server`
- Additional Nav2 Jazzy nodes sampled active:
  - `/route_server`
  - `/collision_monitor`
  - `/docking_server`
- `/slam_toolbox` is active.
- `/navigate_to_pose` action server exists and is served by `/bt_navigator`.
- `/goal_pose` has one subscriber.
- `lifecycle_manager_navigation` appears in the ROS node graph and its `node_names` list includes the Nav2 managed nodes.

Therefore Phase46 classifies the bounded run as `NAV2_LIFECYCLE_ACTIVE_RECOVERED` rather than namespace mismatch, manager exit, managed-node mismatch, blocked transition, or ineffective autostart.

Caveat: `ros2 lifecycle get /lifecycle_manager_navigation` reports `missing` because the lifecycle manager itself is not queried as a lifecycle node in the same way as managed lifecycle nodes; this matches the Phase45 skill pitfall and is not treated as manager exit when `/lifecycle_manager_navigation` is present in `nodes.txt` and manager params are readable.

## Artifacts

- `log/phase46_nav2_lifecycle_activation_root_cause/actions.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/cleanup_processes_after.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/diagnostics.json`
- `log/phase46_nav2_lifecycle_activation_root_cause/goal_pose_topic_info.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/launch.log`
- `log/phase46_nav2_lifecycle_activation_root_cause/lifecycle_states_before_after.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/managed_nodes.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/manager_params.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/nav2_config_diff.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/nav2_launch_static_analysis.json`
- `log/phase46_nav2_lifecycle_activation_root_cause/nav2_process_tree.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/navigate_to_pose_action_info.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/nodes.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/precheck.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/topics.txt`
- `log/phase46_nav2_lifecycle_activation_root_cause/transition_service_results.txt`

## Added / changed files

- `tools/analyze_phase46_nav2_lifecycle_activation_root_cause.py`
- `tools/run_phase46_nav2_lifecycle_activation_root_cause.sh`
- `src/tugbot_maze/test/test_phase46_nav2_lifecycle_activation_root_cause.py`
- `doc/doc_report/phase46_nav2_lifecycle_activation_root_cause_report.md`

## Verification

- `python3 -m py_compile tools/analyze_phase46_nav2_lifecycle_activation_root_cause.py src/tugbot_maze/test/test_phase46_nav2_lifecycle_activation_root_cause.py`: passed
- `bash -n tools/run_phase46_nav2_lifecycle_activation_root_cause.sh`: passed
- Phase46 focused tests: `5 passed in 0.00s`
- Phase36/37/41/42/43/44/45/46 focused regression: `45 passed in 0.36s`
- `git diff -- src/tugbot_navigation/config`: empty
- cleanup check: empty

## Phase46 final classification

`NAV2_LIFECYCLE_ACTIVE_RECOVERED`

Phase46 停止，等待人工验收；不进入 Phase47。
