from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
BRINGUP = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
PACKAGE_XML = ROOT / 'src' / 'tugbot_maze' / 'package.xml'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase49_maze_explorer_declares_dispatch_entry_readiness_gate_inputs():
    source = _read(MAZE_EXPLORER)
    for token in [
        'WAIT_FOR_DISPATCH_ENTRY_READINESS',
        "declare_parameter('scan_topic'",
        "declare_parameter('goal_pose_topic'",
        "'dispatch_readiness_required_lifecycle_nodes'",
        "declare_parameter('dispatch_readiness_near_robot_radius_m'",
        "declare_parameter('dispatch_readiness_min_map_known_ratio'",
        "declare_parameter('dispatch_readiness_min_map_free_ratio'",
        "declare_parameter('dispatch_readiness_min_local_costmap_known_ratio'",
        "declare_parameter('dispatch_readiness_min_scan_finite_count'",
        'LaserScan',
        'GetState',
        'PRIMARY_STATE_ACTIVE',
        '_string_list_parameter',
    ]:
        assert token in source


def test_phase49_gate_runs_before_topology_sampling_and_blocks_failed_exhausted_when_not_ready():
    source = _read(MAZE_EXPLORER)
    explore_body = source[source.index('def _explore_once'):source.index('def _analyze_and_dispatch')]
    assert '_dispatch_entry_readiness_gate(robot_pose)' in explore_body
    assert explore_body.index('_dispatch_entry_readiness_gate(robot_pose)') < explore_body.index('self._analyze_and_dispatch(robot_pose)')
    assert 'self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS' in explore_body
    assert 'self.last_dispatch_readiness_gate' in explore_body
    assert 'self._publish_state()' in explore_body

    gate_block = explore_body[
        explore_body.index('gate = self._dispatch_entry_readiness_gate(robot_pose)'):
        explore_body.index('self.mode = AT_NODE_ANALYZE')
    ]
    assert '_mark_exhausted' not in gate_block
    assert 'FAILED_EXHAUSTED' not in gate_block


def test_phase49_state_payload_contains_structured_gate_diagnostics_without_topology_sampling():
    source = _read(MAZE_EXPLORER)
    for token in [
        'last_dispatch_readiness_gate',
        'dispatch_readiness_gate_passed',
        'dispatch_readiness_blocking_reasons',
        'dispatch_readiness_first_pass_time_sec',
        'nav2_lifecycle_active',
        'navigate_to_pose_action_ready',
        'goal_pose_subscriber_ready',
        'scan_sufficient',
        'map_sufficient',
        'tf_sufficient',
        'local_costmap_sufficient',
    ]:
        assert token in source

    publish_body = source[source.index('def _publish_state'):source.index('def _grid_view_from_msg')]
    assert "'dispatch_readiness_gate'" in publish_body
    assert "'dispatch_readiness_gate_passed'" in publish_body
    assert "'dispatch_readiness_blocking_reasons'" in publish_body


def test_phase49_gate_sufficiency_uses_full_subscribed_data_not_topic_echo_or_clearance_tuning():
    source = _read(MAZE_EXPLORER)
    for token in [
        'self.scan_sub = self.create_subscription(LaserScan, self.scan_topic',
        'self.map_view',
        'self.local_costmap_view',
        'self.tf_buffer.lookup_transform(self.map_frame, self.base_frame',
        '_grid_ratio_near_pose',
        '_scan_sufficiency',
        '_local_costmap_sufficiency',
        '_map_sufficiency',
    ]:
        assert token in source
    assert 'ros2 topic echo' not in source

    # Phase49 must not be a hidden clearance or Nav2 parameter retune.
    launch = _read(BRINGUP)
    assert "DeclareLaunchArgument('clearance_radius_m', default_value='0.38'" in launch
    assert "DeclareLaunchArgument('dispatch_readiness" in launch


def test_phase49_launch_and_dependencies_expose_gate_parameters_without_nav2_config_edits():
    launch = _read(BRINGUP)
    package_xml = _read(PACKAGE_XML)
    for token in [
        "'scan_topic': '/scan'",
        "'goal_pose_topic': '/goal_pose'",
        "'dispatch_readiness_min_map_known_ratio'",
        "'dispatch_readiness_min_local_costmap_known_ratio'",
        "'dispatch_readiness_min_scan_finite_count'",
        "'dispatch_readiness_required_lifecycle_nodes'",
        "DeclareLaunchArgument('dispatch_readiness_min_map_known_ratio'",
        "DeclareLaunchArgument('dispatch_readiness_min_local_costmap_known_ratio'",
        "DeclareLaunchArgument('dispatch_readiness_min_scan_finite_count'",
    ]:
        assert token in launch
    for dep in [
        '<exec_depend>sensor_msgs</exec_depend>',
        '<exec_depend>lifecycle_msgs</exec_depend>',
    ]:
        assert dep in package_xml
