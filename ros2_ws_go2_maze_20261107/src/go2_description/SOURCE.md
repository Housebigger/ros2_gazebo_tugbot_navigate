# Unitree Go2 model source

The model and seven Collada meshes originate from Unitree Robotics.

- Repository https://github.com/unitreerobotics/unitree_ros
- Commit `41dec58c7a3ccb3050243ef16ac2ca74b2b0f9ca`
- Source directory `robots/go2_description`
- License BSD 3-Clause, reproduced in `LICENSE`
- Original URDF SHA-256 `7d19fe48e2e689ee1a032ab99f2a4a8b671d87e73de48d3e65811682a5b48b9e`

Local adaptations normalize the twelve actuator names and seventeen main link names to the existing workspace interface. Mesh URIs resolve through ament during Xacro expansion. Visuals use the materials embedded in the Collada files. Joint origins, axes, limits, inertias and collision shapes retain the upstream definitions. In this workspace, `go2_maze_bringup` renames actuator feedback to LF/RF/LH/RH_HAA/HFE/KFE for the existing legged controller. `tools/generate_go2_model.py` adds native Gazebo torque PID joints (P=320, D=4, original 23.7/45.43 Nm effort caps), joint feedback, world-anchored odometry, a centered lidar and a front camera. Base motion is produced by leg contact with the ground.

## Original mesh checksums

- `base.dae` `e52bebbb5c7ff1f6aedee620c9b5b349f6e965360ecc63a9bb743eeff2d0be64`
- `calf.dae` `29872fa1a435b92bc2b395fdfa7f0aee26101db3c494681896bd4ef24ee7883c`
- `calf_mirror.dae` `79c66be7ed06760bd5001c067a1fb0ac517f639d8b1a5f89be471903b3f359e1`
- `foot.dae` `ffbd95fd641866ce9bd277aa249b92dafc9148d70131fa2c52709a35ce5b710b`
- `hip.dae` `8735e10617afe252cef0844c1d139d15bc6d4c653660d742fea9fe05c2704d8f`
- `thigh.dae` `a622381ae308897ff143b8bbf02f075457c342b22984205040424a5a983246fa`
- `thigh_mirror.dae` `c7bae0b0565c2aa4b7cc15a538d795e9153e0f10c821cc18429e92693172bdc6`
