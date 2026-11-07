# Unitree Go2 autonomous latest-maze integration

The user selected the official Unitree Go2 already verified in ws_quadruped_sim and requested autonomous completion of the latest maze in this repository.

The base is main commit 2381e500e369b7a33d6ca9968e3b7ce7d45f762f and workspace ros2_ws_tugbot_nav_20260728. Its actual world includes Tugbot. Its README still describes an earlier ANYmal stage.

## Architecture

Deliver ros2_ws_go2_maze_20261107 using the existing online_slam flood-fill solver, growing wall reference, cell routing and motion FSM. Add a Go2 continuous velocity gait and native Gazebo joint torque controllers with real gravity, collisions and ground contact. Reuse the pinned official Go2 meshes and inertial data from ws_quadruped_sim.

The focused launch uses ROS 2 Lyrical and Gazebo Jetty already installed on this host. online_slam seeds its pose directly from the known entrance and odometry, so the focused solver entry does not require the legacy Nav2/slam_toolbox processes. The alternative full legacy bringup would add unrelated lifecycle and distribution migration work.

## Perception and frames

Retain the centered 16-ring 1800-column 10 Hz lidar, the projected 900-bin scan and the 720x540 front camera. Match every real sensor header to its TF mount. Preserve a centered horizontal scan convention with SCAN_OFFSET_X equal to zero.

The navigation contract is map to odom to base_link. Use the project's existing world-anchored Gazebo odometry design and exact entrance translation of 11.011281,9.025070. Runtime online_slam receives the perimeter and sensed interior walls. Full interior wall geometry is reserved for independent evaluation.

## Acceptance

First verify stand, actual forward displacement, turning, zero-command stop, lost-command stop and real sensor publication.

Then run autonomous exploration of the unchanged 20260528 scaled maze until the actual Go2 reaches the exit region around map21.072562,18.083566 within1.2m and stops. Require nonempty native pose evidence, no falls and independent collision evaluation against the true wall geometry. A solver success log alone is insufficient. Record exit error, elapsed simulation time, route and contact evidence.

## Baseline and choices

The initial bounded pure solver/localization baseline passed47 tests. The repository documents ten historical failures in the broad legacy suite, which require separate classification.

The new workspace follows the repository's dated-workspace convention. Control and navigation speed/time limits will be calibrated from measured Go2 response. Initial native torque PID uses the original actuator effort bounds.
