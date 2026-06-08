# Phase54: Ingress Waypoint Nav2 Runtime Smoke

## Classification

INGRESS_NAV2_GOAL_REACHED

## Scope and guardrails

Phase54 validates only the runtime reachability of the Phase53 ingress waypoint:

- active world: `tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active metadata: `maze_20260528_scaled_instance.yaml`
- single Nav2 goal in map frame: `(x=1.0, y=0.0, yaw=0.0)`
- acceptance radius: `0.35 m`

Guardrails preserved:

- did not modify Nav2 / MPPI / controller parameters;
- did not modify `maze_explorer` strategy;
- did not run `maze_explorer`, `maze_goal_monitor`, or `frontier_explorer`;
- did not use fallback / terminal acceptance;
- did not use old scaffold world/map;
- does not claim autonomous exploration success;
- does not enter Phase55.

## Runtime command

```bash
PHASE54_RUN_TIMEOUT_SEC=300 \
PHASE54_RECORD_TIMEOUT_SEC=180 \
PHASE54_GOAL_TIMEOUT_SEC=100 \
PHASE54_READINESS_TIMEOUT_SEC=170 \
tools/run_phase54_ingress_waypoint_nav2_runtime_smoke.sh
```

Wrapper exit code: `0`.

## Artifacts

Artifact directory:

`log/phase54_ingress_waypoint_nav2_runtime_smoke/`

Primary artifacts:

- `phase54_ingress_waypoint_nav2_runtime_smoke.json`
- `phase54_ingress_waypoint_nav2_runtime_smoke_runtime_evidence.json`
- `phase54_ingress_waypoint_nav2_runtime_smoke_navigate_to_pose_action_result.json`
- `phase54_ingress_waypoint_nav2_runtime_smoke_lifecycle_readiness.txt`
- `phase54_ingress_waypoint_nav2_runtime_smoke_navigate_to_pose_action_info.txt`
- `phase54_ingress_waypoint_nav2_runtime_smoke_goal_pose_topic_info.txt`
- `phase54_ingress_waypoint_nav2_runtime_smoke_launch.log`
- `phase54_ingress_waypoint_nav2_runtime_smoke_ros_graph_snapshot.txt`
- `phase54_ingress_waypoint_nav2_runtime_smoke_ros_graph_final.txt`
- `phase54_ingress_waypoint_nav2_runtime_smoke_cleanup_processes_after.txt`

## Readiness evidence

The wrapper reached readiness before sending the single action goal:

- `/navigate_to_pose` Action servers: `1` (`/bt_navigator`)
- `/goal_pose` Subscription count: `1`
- lifecycle readiness file recorded `readiness_ready=1`
- `/planner_server`: `active [3]`
- `/bt_navigator`: `active [3]`

Note: the final text summary currently records `/controller_server_state=unconfigured [1]` because an early lifecycle sample contained that state; the wrapper has been patched after runtime to require same-snapshot lifecycle states for future runs. Runtime evidence still shows the Nav2 action server accepted and successfully completed the ingress goal, and the robot physically moved under Nav2.

## Nav2 action result

`phase54_ingress_waypoint_nav2_runtime_smoke_navigate_to_pose_action_result.json`:

- `goal_sent`: `true`
- `goal_accepted`: `true`
- `result_received`: `true`
- `status`: `4`
- `status_text`: `STATUS_SUCCEEDED`
- `error_code`: `0`
- `success`: `true`
- goal pose: map frame `(x=1.0, y=0.0, yaw=0.0)`

Feedback showed decreasing `distance_remaining`, ending below the Phase54 acceptance radius.

## Motion and arrival evidence

Runtime summary:

- first robot pose in map: approximately `(0.0, 0.0, 0.0)`
- final robot pose in map: `(0.8629334001675386, 0.020021996425704604, 0.029002209975015854)`
- final distance to ingress waypoint: `0.13852123711006567 m`
- acceptance radius: `0.35 m`
- `goal_reached_within_acceptance_radius`: `true`
- robot moved distance: `0.8631656465865274 m`
- `robot_moved`: `true`

## Plan evidence

After ingress, `/plan` evidence was available:

- frame: `map`
- pose count: `14`
- path length: `0.3926456965158254 m`
- first plan pose: `(0.6145000119315971, 0.023630620049579498)`
- last plan pose: `(1.0, 0.0)`

## Map / costmap / scan / TF evidence

Before ingress, near the entrance pose:

- `/map` inclusive near-robot known ratio: `0.4429369513168396`
- `/map` inclusive near-robot free ratio: `0.4429369513168396`
- out-of-bounds cells treated as unknown: `481`
- scan finite count: `322`
- nearest obstacle: `1.4781546592712402 m`
- TF available for `map->base_link`, `odom->base_link`, and `map->odom`

After ingress, near the final pose:

- `/map` inclusive near-robot known ratio: `0.8379446640316206`
- `/map` inclusive near-robot free ratio: `0.8308300395256917`
- `/map` inclusive near-robot out-of-bounds count: `0`
- `/local_costmap/costmap` inclusive known ratio: `1.0`
- `/local_costmap/costmap` inclusive free ratio: `0.6923688394276629`
- `/global_costmap/costmap` inclusive known ratio: `0.9185770750988143`
- `/global_costmap/costmap` inclusive free ratio: `0.7256916996047431`
- scan finite count: `406`
- nearest obstacle: `0.9287884831428528 m`
- `map->base_link`: available, translation `(0.8629334001675386, 0.020021996425704604, 0.0)`
- `odom->base_link`: available, same translation in this run
- `map->odom`: available, near-zero translation

This supports the Phase54 classification that the Phase53 ingress waypoint is Nav2-runtime reachable and that moving to the ingress waypoint improves near-robot inclusive `/map` evidence. This is not evidence of full autonomous exploration success.

## Cleanup

`phase54_ingress_waypoint_nav2_runtime_smoke_cleanup_processes_after.txt` is empty.

## Conclusion

Classification: `INGRESS_NAV2_GOAL_REACHED`.

Phase54 demonstrates that the ingress waypoint `(1.0, 0.0, 0.0)` is reachable by Nav2 in the active scaled2x world under bounded runtime conditions. It does not validate full maze exploration, does not run autonomous maze exploration, and does not authorize Phase55 without explicit user approval.
