import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_controller_dynamics.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def test_phase21_controller_dynamics_recorder_contract():
    assert RECORDER.exists(), 'record_controller_dynamics.py must exist'
    source = RECORDER.read_text(encoding='utf-8')
    ast.parse(source)

    assert 'class ControllerDynamicsRecorder' in source
    assert "'/odom'" in source or '"/odom"' in source
    assert "'cmd_vel_nav'" in source or '"cmd_vel_nav"' in source
    assert 'nav_msgs.msg' in source and 'Odometry' in source
    assert 'geometry_msgs.msg' in source and 'Twist' in source
    assert 'source' in source and 'odom' in source and 'cmd_vel' in source
    assert 'linear_x' in source
    assert 'angular_z' in source
    assert 'yaw' in source
    assert 'ExternalShutdownException' in source
    assert 'rclpy.ok()' in source


def test_phase21_smoke_wrapper_contract():
    assert WRAPPER.exists(), 'run_phase21_controller_diagnostics_smoke.sh must exist'
    source = WRAPPER.read_text(encoding='utf-8')

    assert 'set -euo pipefail' in source
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in source
    assert 'record_explorer_state_series.py --output' in source
    assert 'record_controller_dynamics.py' in source
    assert 'ros2 launch tugbot_bringup tugbot_maze_explore.launch.py' in source
    assert 'analyze_goal_events_with_nav2_log.py' in source
    assert 'summarize_goal_event_local_costs.py' in source
    assert 'analyze_goal_controller_dynamics.py' in source

    # Cleanup must cover orphaned child nodes observed after Phase 20.
    for token in [
        'ros_gz_bridge',
        'static_transform_publisher',
        'maze_goal_monitor',
        'slam_toolbox',
        'controller_server',
        'planner_server',
        'bt_navigator',
        'gz sim',
    ]:
        assert token in source

    assert 'trap cleanup EXIT' in source
    assert 'phase21_run' in source
    assert 'phase22_run' in source
    assert 'maze_explorer' in source
    assert 'goal_timeout_sec:=35.0' in source
    assert 'branch_goal_step_m:=0.9' in source
    assert 'clearance_radius_m:=0.34' in source
