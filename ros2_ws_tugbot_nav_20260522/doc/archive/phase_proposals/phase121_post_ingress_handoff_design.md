# Phase121 Post-ingress handoff to maze_explorer design review

Status: DESIGN_ONLY - Phase121 doc-only/design-only handoff review. Phase122 not entered.

## Purpose

Phase120 proved only that, in one visible-stack run, managed-active readiness wait completed, strict preflight passed, and the single explicit inner-ingress `NavigateToPose` goal succeeded:

```text
frame_id=map
x=2.0
y=0.0
yaw=0.0
classification=INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
maze_explorer_started=false
```

Phase120 single-goal success is not exploration success, not autonomous exploration success, not exit success, not Goal1 success, not carry-over success, and not staging success.

Phase121 defines a future Phase122+ handoff boundary from this stopped ingress success state to a safe `maze_explorer` startup check. Phase121 does not implement that boundary and does not start any runtime.

## Phase121 hard boundaries

This proposal is Phase121 DESIGN_ONLY and doc-only/design-only.

Rules:

- do not launch Gazebo/RViz/Nav2 runtime;
- do not send NavigateToPose goal;
- do not start maze_explorer;
- do not tune Nav2/MPPI/controller/config;
- do not change exploration strategy;
- do not remove preflight;
- do not claim autonomous exploration success;
- do not claim exit success;
- do not change branch scoring;
- do not change centerline gate;
- do not change fallback;
- do not change terminal acceptance;
- Phase122 not entered.

## Future handoff concept

Future Phase122+ may introduce a post-ingress handoff smoke that runs only after Phase120-style evidence has already established a stopped, successful inner-ingress result. The handoff is a safety/readiness gate between the completed ingress action and any `maze_explorer` process startup.

The handoff must be fail-closed. If evidence is missing, stale, contradictory, or out of tolerance, classify the handoff as not ready and do not start `maze_explorer`.

## Handoff preconditions

All of the following must be true in the same run before a future Phase122+ handoff may proceed:

1. same-run readiness wait passed
   - readiness wait did not time out;
   - launch log marker `Managed nodes are active` was found or equivalent multi-source readiness was satisfied;
   - action server readiness was observed after readiness wait.
2. preflight passed
   - `preflight.passed == true`;
   - `preflight.failed_gates == []`;
   - reject reason is null/absent;
   - preflight was not removed or bypassed.
3. inner-ingress result SUCCEEDED
   - the only dispatched goal was the explicit inner-ingress goal;
   - `frame_id=map`;
   - `x=2.0`;
   - `y=0.0`;
   - `yaw=0.0`;
   - goal was accepted;
   - result status label was `SUCCEEDED`.
4. robot pose near ingress goal
   - current robot pose is available in `map` frame;
   - distance to ingress goal is within the future Phase122 tolerance;
   - orientation error from ingress yaw is within the future Phase122 tolerance;
   - pose timestamp is fresh enough for the handoff window.
5. no residual active goal
   - Nav2 action state is idle;
   - no active, executing, canceling, or pending `NavigateToPose` goal remains;
   - no cancel request is still in flight;
   - no result ambiguity exists.
6. costmap/scan/TF freshness
   - global/local costmap freshness checks pass as applicable;
   - scan freshness passes;
   - TF freshness for map/odom/base and scan transforms passes;
   - evidence timestamps are captured in the handoff artifact.

If any precondition is false, the default classification is `INGRESS_SUCCESS_HANDOFF_NOT_READY`, or a more specific classification below.

## Handoff artifact schema

Future Phase122+ should write a `handoff_artifact` before starting `maze_explorer`. The artifact should be standalone and enough to explain why the handoff was or was not allowed.

Required top-level fields:

```text
phase
run_id
handoff_artifact
classification
handoff_allowed
maze_explorer_start_allowed
maze_explorer_started
phase120_source_artifact
created_wall_time_sec
```

Required `handoff_artifact` fields:

```text
ingress_goal_result
robot_pose_after_ingress
distance_to_ingress_goal
orientation_error
costmap_freshness
scan_freshness
tf_freshness
nav2_action_idle_state
preconditions
failure_reasons
```

`ingress_goal_result` should include:

```text
classification
accepted
result_received
result_status
result_status_label
abort_text
cancel_requested
cancel_result
send_wall_time_sec
goal_response_wall_time_sec
result_wait_started_wall_time_sec
goal_pose
```

`robot_pose_after_ingress` should include:

```text
frame_id
x
y
yaw
stamp
sample_wall_time_sec
source
pose_available
```

`distance_to_ingress_goal` should include:

```text
meters
tolerance_m
within_tolerance
```

`orientation_error` should include:

```text
radians
tolerance_rad
within_tolerance
```

`costmap_freshness` should include:

```text
global_costmap_available
local_costmap_available
global_costmap_age_sec
local_costmap_age_sec
max_allowed_age_sec
fresh
```

`scan_freshness` should include:

```text
scan_seen
scan_age_sec
max_allowed_age_sec
fresh
```

`tf_freshness` should include:

```text
map_odom_available
odom_base_available
map_base_available
scan_to_base_available
max_tf_age_sec
fresh
```

`nav2_action_idle_state` should include:

```text
action_server_ready
active_goal_count
pending_goal_count
executing_goal_count
canceling_goal_count
idle
last_result_status_label
```

The handoff artifact must explicitly include costmap/scan/TF freshness rather than inferring readiness only from Phase120 goal success.

## Failure classifications

Future Phase122+ handoff classifications:

```text
INGRESS_SUCCESS_HANDOFF_READY
INGRESS_SUCCESS_HANDOFF_NOT_READY
POSE_NOT_AT_INGRESS_GOAL
NAV2_ACTION_NOT_IDLE
TF_OR_SCAN_STALE
COSTMAP_NOT_READY
```

Meaning:

- `INGRESS_SUCCESS_HANDOFF_READY`: all preconditions passed; `maze_explorer` may be started only if Phase122+ explicitly authorizes startup.
- `INGRESS_SUCCESS_HANDOFF_NOT_READY`: generic fail-closed outcome for missing, stale, ambiguous, or contradictory evidence.
- `POSE_NOT_AT_INGRESS_GOAL`: Phase120 goal succeeded but current robot pose is not near the ingress goal or orientation error is out of tolerance.
- `NAV2_ACTION_NOT_IDLE`: a residual active/pending/executing/canceling goal remains or action state is ambiguous.
- `TF_OR_SCAN_STALE`: TF or scan evidence is unavailable/stale.
- `COSTMAP_NOT_READY`: required costmap freshness/availability evidence is missing or stale.

For every failure classification:

```text
handoff_allowed=false
maze_explorer_start_allowed=false
maze_explorer_started=false
```

There must be no maze_explorer startup on failed handoff.

## Future Phase122 allowed scope

Phase122 allowed scope is post-ingress handoff smoke only.

A future Phase122 may:

- read the Phase120 artifact;
- launch a visible stack only if explicitly authorized by the Phase122 task;
- reproduce the Phase120 readiness wait + ingress success if needed by the task;
- compute and write the handoff artifact;
- may start maze_explorer only after explicit Phase122+ authorization;
- start `maze_explorer` in `max_goals=0` or dry-start mode only;
- observe startup/initialization only;
- stop before any exploration dispatch.

A future Phase122 must:

- do not dispatch exploration goal;
- do not send Goal1;
- do not validate Goal1;
- does not validate Goal1;
- do not validate carry-over;
- does not validate carry-over;
- do not validate staging;
- does not validate staging;
- do not validate branch selection;
- do not validate centerline behavior;
- do not validate fallback;
- do not validate terminal acceptance;
- do not validate exit success;
- does not validate exit success;
- do not claim autonomous exploration success.

## Guardrails preserved for later phases

The future handoff does not authorize algorithm changes. It must preserve:

- do not change branch scoring;
- do not change centerline gate;
- do not change fallback;
- do not change terminal acceptance;
- do not change exploration order;
- do not tune Nav2/MPPI/controller/config;
- do not relax preflight.

## Acceptance criteria for Phase121

Phase121 acceptance requires only documentation and static tests:

- proposal exists at `doc/doc_proposal/phase121_post_ingress_handoff_design.md`;
- report exists at `doc/doc_report/phase121_post_ingress_handoff_design_report.md`;
- focused static tests cover design-only boundaries, preconditions, artifact schema, classifications, Phase122 allowed scope, and guardrails;
- no runtime runner or analyzer is added for Phase121;
- no Gazebo/RViz/Nav2 runtime is launched;
- no NavigateToPose goal is sent;
- no `maze_explorer` process is started;
- no Nav2 config or preflight implementation changes are made;
- Phase122 not entered.
