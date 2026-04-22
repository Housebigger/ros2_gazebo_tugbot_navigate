from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


def test_expected_workspace_files_exist():
    expected = [
        WORKSPACE_ROOT / 'README.md',
        WORKSPACE_ROOT / 'src' / 'tugbot_description' / 'package.xml',
        WORKSPACE_ROOT / 'src' / 'tugbot_description' / 'setup.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf',
        WORKSPACE_ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.config',
        WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'package.xml',
        WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'setup.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_lane_world.sdf',
        WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_lane_world_debug.sdf',
        WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'config' / 'bridge.yaml',
        WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'package.xml',
        WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'setup.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'config' / 'perception.yaml',
        WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'tugbot_perception' / 'lane_detector_node.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'package.xml',
        WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'setup.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'config' / 'control.yaml',
        WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'tugbot_control' / 'lane_controller_node.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'package.xml',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'setup.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'sim_minimal.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'full_system.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'full_system_zeroerr_outer.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'perception_debug.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'perception_debug_zeroerr_outer.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'full_system_rviz_zeroerr_outer.launch.py',
        WORKSPACE_ROOT / 'src' / 'tugbot_bringup' / 'config' / 'perception_debug.rviz',
    ]

    missing = [str(path.relative_to(WORKSPACE_ROOT)) for path in expected if not path.exists()]
    assert not missing, missing
