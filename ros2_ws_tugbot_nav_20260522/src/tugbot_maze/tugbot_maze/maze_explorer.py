#!/usr/bin/env python3
"""DFS-style maze explorer for Tugbot SLAM + Nav2.

The node uses only runtime robot observations: live OccupancyGrid, TF pose, and
Nav2 NavigateToPose feedback. It keeps topological memory of junctions, branch
states, dead ends, and backtracking targets. Known entrance/exit coordinates are
used for initialization, success detection, and branch ranking bias; they are not
used as a precomputed path through the Gazebo world.
"""

from __future__ import annotations

import json
import math
import ast
from typing import Any, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import (
    CORRIDOR,
    DEAD_END,
    JUNCTION,
    classify_local_topology,
    filter_open_directions,
    generate_second_step_forward_goal_after_staging,
    make_branch_goal,
    make_centered_branch_goal,
    normalize_angle,
    plan_two_step_corridor_alignment_staging_goal,
    refine_corridor_centerline_target,
)
from tugbot_maze.maze_topology import (
    BACKTRACK_FAILED,
    BLACKLISTED,
    BLOCKED,
    BLOCKED_NAV2,
    GOAL_REJECTED,
    GOAL_TIMEOUT,
    GOAL_PREEMPTED,
    GOAL_CANCELED_AFTER_TIMEOUT,
    GOAL_CANCELED_AFTER_EXIT,
    TRUE_DEAD_END,
    BranchOption,
    EXPLORED,
    IN_PROGRESS,
    MazeTopology,
    SELECTABLE_BRANCH_STATES,
)

RobotPose = Tuple[float, float, float]
Point = Tuple[float, float]

WAIT_FOR_MAP = 'WAIT_FOR_MAP'
WAIT_FOR_NAV2 = 'WAIT_FOR_NAV2'
WAIT_FOR_DISPATCH_ENTRY_READINESS = 'WAIT_FOR_DISPATCH_ENTRY_READINESS'
STARTUP_WARMUP_NO_DISPATCH = 'STARTUP_WARMUP_NO_DISPATCH'
WAIT_FOR_TOPOLOGY_CONSISTENCY = 'WAIT_FOR_TOPOLOGY_CONSISTENCY'
AT_NODE_ANALYZE = 'AT_NODE_ANALYZE'
NAVIGATING = 'NAVIGATING'
BACKTRACKING = 'BACKTRACKING'
SETTLING = 'SETTLING'
EXIT_REACHED = 'EXIT_REACHED'
FAILED_EXHAUSTED = 'FAILED_EXHAUSTED'
ENTRY_DIRECT = 'ENTRY_DIRECT'


class MazeExplorer(Node):
    def __init__(self) -> None:
        super().__init__('maze_explorer')

        self.map_topic = str(self.declare_parameter('map_topic', '/map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.action_name = str(self.declare_parameter('action_name', '/navigate_to_pose').value)
        self.state_topic = str(self.declare_parameter('state_topic', '/maze/explorer_state').value)
        self.goal_events_topic = str(self.declare_parameter('goal_events_topic', '/maze/goal_events').value)
        self.local_costmap_topic = str(self.declare_parameter('local_costmap_topic', '/local_costmap/costmap').value)
        self.scan_topic = str(self.declare_parameter('scan_topic', '/scan').value)
        self.goal_pose_topic = str(self.declare_parameter('goal_pose_topic', '/goal_pose').value)
        required_lifecycle_nodes_value = self.declare_parameter(
            'dispatch_readiness_required_lifecycle_nodes',
            '/controller_server,/planner_server,/bt_navigator',
        ).value
        self.dispatch_readiness_required_lifecycle_nodes = self._string_list_parameter(required_lifecycle_nodes_value)
        self.dispatch_readiness_near_robot_radius_m = float(self.declare_parameter('dispatch_readiness_near_robot_radius_m', 1.0).value)
        self.dispatch_readiness_min_map_known_ratio = float(self.declare_parameter('dispatch_readiness_min_map_known_ratio', 0.70).value)
        self.dispatch_readiness_min_map_free_ratio = float(self.declare_parameter('dispatch_readiness_min_map_free_ratio', 0.50).value)
        self.dispatch_readiness_min_local_costmap_known_ratio = float(self.declare_parameter('dispatch_readiness_min_local_costmap_known_ratio', 0.95).value)
        self.dispatch_readiness_min_local_costmap_free_ratio = float(self.declare_parameter('dispatch_readiness_min_local_costmap_free_ratio', 0.50).value)
        self.dispatch_readiness_min_scan_finite_count = int(self.declare_parameter('dispatch_readiness_min_scan_finite_count', 120).value)
        self.dispatch_readiness_max_local_costmap_age_sec = float(self.declare_parameter('dispatch_readiness_max_local_costmap_age_sec', 5.0).value)
        self.directional_local_costmap_readiness_override_enabled = bool(
            self.declare_parameter('directional_local_costmap_readiness_override_enabled', False).value
        )
        self.startup_warmup_no_dispatch = bool(self.declare_parameter('startup_warmup_no_dispatch', False).value)
        self.topology_consistency_enabled = bool(self.declare_parameter('topology_consistency_enabled', True).value)
        self.topology_consistency_required_no_candidate_frames = max(
            1,
            int(self.declare_parameter('topology_consistency_required_no_candidate_frames', 2).value),
        )
        self.topology_consistency_window_sec = max(
            0.0,
            float(self.declare_parameter('topology_consistency_window_sec', 4.0).value),
        )
        self.post_ingress_single_open_exception_enabled = bool(
            self.declare_parameter('post_ingress_single_open_exception_enabled', True).value
        )
        self.post_ingress_single_open_exception_return_angle_min_deg = float(
            self.declare_parameter('post_ingress_single_open_exception_return_angle_min_deg', 80.0).value
        )
        self.centerline_target_refinement_enabled = bool(self.declare_parameter('centerline_target_refinement_enabled', True).value)
        self.centerline_target_refinement_side_probe_m = float(
            self.declare_parameter('centerline_target_refinement_side_probe_m', 1.5).value
        )
        self.centerline_target_refinement_forward_offsets_m = self._float_list_parameter(
            self.declare_parameter('centerline_target_refinement_forward_offsets_m', '0.0,0.1,0.2').value
        )
        self.centerline_target_refinement_lateral_offsets_m = self._float_list_parameter(
            self.declare_parameter('centerline_target_refinement_lateral_offsets_m', '-0.45,-0.30,-0.15,0.0,0.15,0.30,0.45').value
        )
        self.centerline_target_refinement_gate_mode = str(
            self.declare_parameter('centerline_target_refinement_gate_mode', 'balance_first').value
        )
        self.centerline_target_refinement_min_clearance_floor_m = float(
            self.declare_parameter('centerline_target_refinement_min_clearance_floor_m', 0.45).value
        )
        self.centerline_target_refinement_forward_progress_tolerance_m = float(
            self.declare_parameter('centerline_target_refinement_forward_progress_tolerance_m', 0.05).value
        )

        self.entrance_x = float(self.declare_parameter('entrance_x', -4.0).value)
        self.entrance_y = float(self.declare_parameter('entrance_y', -3.0).value)
        self.entrance_yaw = float(self.declare_parameter('entrance_yaw', 0.0).value)
        self.exit_x = float(self.declare_parameter('exit_x', 4.0).value)
        self.exit_y = float(self.declare_parameter('exit_y', 3.0).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 0.6).value)
        self.entry_direct_enabled = bool(self.declare_parameter('entry_direct_enabled', True).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 1.5).value)

        self.exploration_rate_hz = float(self.declare_parameter('exploration_rate_hz', 0.5).value)
        self.publish_debug_state_hz = float(self.declare_parameter('publish_debug_state_hz', 1.0).value)
        self.junction_merge_radius_m = float(self.declare_parameter('junction_merge_radius_m', 0.75).value)
        self.exit_bias_weight = float(self.declare_parameter('exit_bias_weight', 0.5).value)
        self.branch_angle_step_deg = float(self.declare_parameter('branch_angle_step_deg', 90.0).value)
        self.branch_lookahead_m = float(self.declare_parameter('branch_lookahead_m', 1.5).value)
        self.branch_goal_step_m = float(self.declare_parameter('branch_goal_step_m', 1.0).value)
        self.min_goal_step_m = float(self.declare_parameter('min_goal_step_m', 0.45).value)
        self.clearance_radius_m = float(self.declare_parameter('clearance_radius_m', 0.35).value)
        self.min_open_distance_m = float(self.declare_parameter('min_open_distance_m', 0.45).value)
        self.goal_timeout_sec = float(self.declare_parameter('goal_timeout_sec', 45.0).value)
        self.near_exit_goal_timeout_sec = float(self.declare_parameter('near_exit_goal_timeout_sec', 55.0).value)
        self.near_exit_timeout_extension_radius_m = float(self.declare_parameter('near_exit_timeout_extension_radius_m', 1.0).value)
        self.near_exit_fallback_enabled = bool(self.declare_parameter('near_exit_fallback_enabled', False).value)
        self.near_exit_fallback_trigger_radius_m = float(self.declare_parameter('near_exit_fallback_trigger_radius_m', 0.9).value)
        self.near_exit_terminal_acceptance_radius_m = float(self.declare_parameter('near_exit_terminal_acceptance_radius_m', 0.6).value)
        self.near_exit_micro_goal_min_step_m = float(self.declare_parameter('near_exit_micro_goal_min_step_m', 0.20).value)
        self.near_exit_micro_goal_max_step_m = float(self.declare_parameter('near_exit_micro_goal_max_step_m', 0.35).value)
        self.near_exit_fallback_max_attempts = int(self.declare_parameter('near_exit_fallback_max_attempts', 1).value)
        self.near_exit_fallback_require_clean_topology = bool(self.declare_parameter('near_exit_fallback_require_clean_topology', True).value)
        self.near_exit_fallback_require_path_alignment = bool(self.declare_parameter('near_exit_fallback_require_path_alignment', True).value)
        self.near_exit_fallback_robot_to_path_max_m = float(self.declare_parameter('near_exit_fallback_robot_to_path_max_m', 0.15).value)
        self.near_exit_fallback_cmd_near_zero_min_sec = float(self.declare_parameter('near_exit_fallback_cmd_near_zero_min_sec', 3.0).value)
        self.goal_settle_sec = float(self.declare_parameter('goal_settle_sec', 1.5).value)
        self.backtrack_goal_tolerance_m = float(self.declare_parameter('backtrack_goal_tolerance_m', 0.35).value)
        self.lateral_centering_search_m = float(self.declare_parameter('lateral_centering_search_m', 0.8).value)
        self.allow_reverse_branch_goals = bool(self.declare_parameter('allow_reverse_branch_goals', True).value)
        self.reverse_branch_angle_threshold_deg = float(self.declare_parameter('reverse_branch_angle_threshold_deg', 135.0).value)
        self.blacklist_radius_m = float(self.declare_parameter('blacklist_radius_m', 0.5).value)
        self.max_failures_per_branch = int(self.declare_parameter('max_failures_per_branch', 2).value)
        self.max_backtrack_failures_per_node = int(self.declare_parameter('max_backtrack_failures_per_node', 2).value)
        self.max_goals = int(self.declare_parameter('max_goals', 80).value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_view: Optional[OccupancyGridView] = None
        self.local_costmap_msg: Optional[OccupancyGrid] = None
        self.local_costmap_view: Optional[OccupancyGridView] = None
        self.local_costmap_last_update_time: Optional[Time] = None
        self.scan_msg: Optional[LaserScan] = None
        self.scan_last_update_time: Optional[Time] = None
        self.local_cost_obstacle_threshold = 90
        self.local_cost_inflation_threshold = 70
        self.local_cost_target_radius_m = 0.30
        self.local_cost_robot_cluster_radius_m = 0.75
        self.local_cost_footprint_length_m = 0.60
        self.local_cost_footprint_width_m = 0.50
        self.local_cost_front_wedge_radius_m = 0.75
        self.local_cost_front_wedge_half_angle_rad = math.radians(35.0)
        self.local_cost_side_probe_radius_m = 0.45
        self.mode = WAIT_FOR_MAP
        self.goal_active = False
        self.goal_sent_time: Optional[Time] = None
        self.goal_handle = None
        self.active_goal_kind = 'none'
        self.active_goal_target: Optional[Point] = None
        self.goal_sequence_id = 0
        self.active_goal_sequence_id: Optional[int] = None
        self.last_completed_goal_sequence_id: Optional[int] = None
        self.goal_settle_until: Optional[Time] = None
        self.last_goal_completion_time: Optional[Time] = None
        self.active_start_node_id: Optional[int] = None
        self.active_branch: Optional[BranchOption] = None
        self.active_edge_id: Optional[int] = None
        self.active_goal_event_context: dict = {}
        self.goal_event_context_by_sequence: dict[int, dict] = {}
        self.phase62_nav2_feedback_by_sequence: dict[int, list[dict[str, object]]] = {}
        self.pending_branch_choice_diagnostics: dict = {}
        self.current_node_id: Optional[int] = None
        self.goal_count = 0
        self.entry_direct_dispatched = False
        self.goal_success_count = 0
        self.goal_failure_count = 0
        self.nav2_failure_count = 0
        self.stale_result_count = 0
        self.preempted_goal_count = 0
        self.terminal_cancel_count = 0
        self.timeout_cancel_count = 0
        self.canceled_after_timeout_count = 0
        self.canceled_after_exit_count = 0
        self.timeout_cancel_goal_sequence_id: Optional[int] = None
        self.terminal_cancel_goal_sequence_id: Optional[int] = None
        self.last_terminal_reason: Optional[str] = None
        self.terminal_state_entered = False
        self.blocked_branch_count = 0
        self.blacklisted_goal_count = 0
        self.backtrack_failure_count = 0
        self.last_failure_reason: Optional[str] = None
        self.last_nav2_status: Optional[int] = None
        self.last_local_topology_kind: Optional[str] = None
        self.last_open_direction_count = 0
        self.last_candidate_count = 0
        # Phase43 read-only evidence: captures why initial topology sampling did
        # or did not produce open directions/candidates. This must not feed back
        # into branch selection, Nav2 goals, or any strategy decision.
        self.last_topology_sampling_diagnostics: dict[str, object] = {}
        self.phase56_open_direction_to_candidate_diagnostics: dict[str, object] = {}
        self.last_dispatch_readiness_gate: dict[str, object] = {}
        self.dispatch_readiness_first_pass_time: Optional[Time] = None
        self.dead_end_count = 0
        self.exhausted_logged = False
        self.exit_reached_logged = False
        self.last_state_publish_time: Optional[Time] = None
        self.near_exit_fallback_attempts = 0
        self.topology_consistency_frames: list[dict[str, object]] = []
        self.topology_consistency_start_time: Optional[Time] = None
        self.topology_consistency_candidate_recovered = False
        self.topology_consistency_terminalization_reason: Optional[str] = None
        self.last_topology_consistency_diagnostics: dict[str, object] = {
            'topology_consistency_enabled': bool(self.topology_consistency_enabled),
            'topology_consistency_required_no_candidate_frames': int(self.topology_consistency_required_no_candidate_frames),
            'topology_consistency_window_sec': float(self.topology_consistency_window_sec),
            'topology_consistency_guard_status': 'idle',
            'topology_consistency_frames': [],
            'topology_consistency_frame_count': 0,
            'topology_consistency_frame_index': None,
            'raw_open_direction_count': None,
            'filtered_open_direction_count': None,
            'candidate_before_filter_count': None,
            'candidate_after_filter_count': None,
            'dead_end_policy_state': 'idle',
            'terminalization_delayed': False,
            'candidate_recovered_during_consistency_window': False,
            'topology_consistency_terminalization_reason': None,
        }
        self.post_ingress_single_open_exception_consumed = False
        self.last_post_ingress_single_open_exception_diagnostics: dict[str, object] = {
            'post_ingress_single_open_exception_enabled': bool(self.post_ingress_single_open_exception_enabled),
            'post_ingress_context_active': False,
            'first_post_ingress_topology_node': False,
            'single_open_exception_eligible': False,
            'single_open_exception_applied': False,
            'single_open_exception_reason': 'idle',
            'angle_to_return_entrance': None,
            'angle_to_return_entrance_min_deg': float(self.post_ingress_single_open_exception_return_angle_min_deg),
            'multi_frame_single_open_confirmed': False,
            'single_open_exception_candidate_recovered_during_consistency_window': False,
            'candidate_map_cell_state': None,
            'candidate_local_costmap_cell_state': None,
            'single_open_exception_dispatch_produced': False,
        }
        self.last_centerline_target_refinement_diagnostics: dict[str, object] = self._empty_centerline_target_refinement_diagnostics()
        self.pending_corridor_alignment_second_step: Optional[dict[str, object]] = None

        self.topology = MazeTopology(
            junction_merge_radius_m=self.junction_merge_radius_m,
            exit_bias_weight=self.exit_bias_weight,
            blacklist_radius_m=self.blacklist_radius_m,
            max_failures_per_branch=self.max_failures_per_branch,
            max_backtrack_failures_per_node=self.max_backtrack_failures_per_node,
            backtrack_goal_tolerance_m=self.backtrack_goal_tolerance_m,
        )
        entrance = self.topology.find_or_create_node(self.entrance_x, self.entrance_y, node_type='junction')
        self.topology.visit_node(entrance.node_id)
        self.current_node_id = entrance.node_id

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.lifecycle_clients = {
            node_name: self.create_client(GetState, f'{node_name}/get_state')
            for node_name in self.dispatch_readiness_required_lifecycle_nodes
        }
        self.lifecycle_state_futures: dict[str, object] = {}
        self.lifecycle_state_cache: dict[str, dict[str, object]] = {}

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, self._map_qos_profile())
        self.local_costmap_sub = self.create_subscription(OccupancyGrid, self.local_costmap_topic, self._local_costmap_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.exploration_rate_hz, 0.1), self._explore_once)
        self.state_timer = self.create_timer(1.0 / max(self.publish_debug_state_hz, 0.1), self._publish_state)

        self.get_logger().info(
            'maze_explorer started: map_topic=%s action_name=%s state_topic=%s entrance=(%.3f, %.3f, %.3f) '
            'exit=(%.3f, %.3f) radius=%.3f branch_angle_step_deg=%.1f branch_lookahead_m=%.2f '
            'branch_goal_step_m=%.2f clearance_radius_m=%.2f max_goals=%d'
            % (
                self.map_topic,
                self.action_name,
                self.state_topic,
                self.entrance_x,
                self.entrance_y,
                self.entrance_yaw,
                self.exit_x,
                self.exit_y,
                self.exit_radius,
                self.branch_angle_step_deg,
                self.branch_lookahead_m,
                self.branch_goal_step_m,
                self.clearance_radius_m,
                self.max_goals,
            )
        )

    @staticmethod
    def _map_qos_profile() -> QoSProfile:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    @staticmethod
    def _string_list_parameter(value: Any) -> list[str]:
        if isinstance(value, str):
            stripped = value.strip()
            if stripped.startswith('['):
                try:
                    parsed = ast.literal_eval(stripped)
                    if isinstance(parsed, (list, tuple)):
                        return [str(item) for item in parsed]
                except Exception:  # noqa: BLE001 - fall back to comma splitting for launch strings.
                    pass
            return [item.strip() for item in stripped.split(',') if item.strip()]
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        return [str(value)]

    @staticmethod
    def _float_list_parameter(value: Any) -> list[float]:
        if isinstance(value, str):
            stripped = value.strip()
            if stripped.startswith('['):
                try:
                    parsed = ast.literal_eval(stripped)
                    if isinstance(parsed, (list, tuple)):
                        return [float(item) for item in parsed]
                except Exception:  # noqa: BLE001 - fall back to comma splitting for launch strings.
                    pass
            return [float(item.strip()) for item in stripped.split(',') if item.strip()]
        if isinstance(value, (list, tuple)):
            return [float(item) for item in value]
        return [float(value)]

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_view = self._grid_view_from_msg(msg)
        if self.mode == WAIT_FOR_MAP:
            self.mode = WAIT_FOR_NAV2

    def _local_costmap_callback(self, msg: OccupancyGrid) -> None:
        self.local_costmap_msg = msg
        self.local_costmap_view = self._grid_view_from_msg(msg)
        self.local_costmap_last_update_time = self.get_clock().now()
        if isinstance(self.pending_corridor_alignment_second_step, dict):
            self.pending_corridor_alignment_second_step['fresh_local_costmap_received'] = True
            pending = self.pending_corridor_alignment_second_step
            pending['fresh_local_costmap_sample_time_sec'] = self._now_wall_time_sec()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.scan_last_update_time = self.get_clock().now()
        if isinstance(self.pending_corridor_alignment_second_step, dict):
            self.pending_corridor_alignment_second_step['fresh_scan_received'] = True
            pending = self.pending_corridor_alignment_second_step
            pending['fresh_scan_sample_time_sec'] = self._now_wall_time_sec()

    def _explore_once(self) -> None:
        if self.mode in (EXIT_REACHED, FAILED_EXHAUSTED):
            self._publish_state()
            return

        if self.map_view is None:
            self.mode = WAIT_FOR_MAP
            self._publish_state()
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_state()
            return

        if self._exit_reached(robot_pose):
            self._enter_terminal_state(EXIT_REACHED, terminal_reason='exit_reached', robot_pose=robot_pose)
            self._publish_state()
            return

        if self.goal_active:
            self.mode = NAVIGATING if self.active_goal_kind != 'backtrack' else BACKTRACKING
            if self._goal_timed_out():
                self.get_logger().warn('maze explorer goal timed out; classifying active goal failure')
                self._handle_goal_failure(reason=GOAL_TIMEOUT)
            self._publish_state()
            return

        if self._goal_settle_active():
            self.mode = SETTLING
            self._publish_state()
            return

        if self.goal_count >= self.max_goals and not self.startup_warmup_no_dispatch:
            self._mark_exhausted('goal budget reached')
            self._publish_state()
            return

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.mode = WAIT_FOR_NAV2
            self._publish_state()
            return

        gate = self._dispatch_entry_readiness_gate(robot_pose)
        self.last_dispatch_readiness_gate = gate

        # ENTRY_DIRECT uses a relaxed readiness gate: only requires Nav2 action
        # server, basic TF, and scan. Does NOT require map_sufficient or
        # local_costmap_sufficient because the first straight-line goal does
        # not depend on topology analysis.
        if (self.entry_direct_enabled
                and not self.entry_direct_dispatched
                and self.goal_count == 0):
            nav2_active = bool(gate['checks'].get('nav2_lifecycle_active', False))
            action_ready = bool(gate['checks'].get('navigate_to_pose_action_ready', False))
            tf_ok = bool(gate['checks'].get('tf_sufficient', False))
            scan_ok = bool(gate['checks'].get('scan_sufficient', False))
            if nav2_active and action_ready and tf_ok and scan_ok:
                self.get_logger().info(
                    'ENTRY_DIRECT: relaxed readiness gate passed (nav2=%s action=%s tf=%s scan=%s); dispatching first goal'
                    % (nav2_active, action_ready, tf_ok, scan_ok)
                )
                self.mode = ENTRY_DIRECT
                self._dispatch_entry_direct_goal(robot_pose)
                self._publish_state()
                return
            if self.goal_count == 0:
                _tick = getattr(self, '_readiness_log_tick', 0) + 1
                self._readiness_log_tick = _tick
                if _tick % 10 == 1:
                    self.get_logger().info(
                        'ENTRY_DIRECT: relaxed readiness not yet passed (tick %d); nav2=%s action=%s tf=%s scan=%s'
                        % (_tick, nav2_active, action_ready, tf_ok, scan_ok)
                    )
            self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
            self._publish_state()
            return

        if not gate['passed']:
            self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
            self._publish_state()
            return
        if self.dispatch_readiness_first_pass_time is None:
            self.dispatch_readiness_first_pass_time = self.get_clock().now()

        if self.startup_warmup_no_dispatch:
            self.mode = STARTUP_WARMUP_NO_DISPATCH
            self._publish_state()
            return

        self.mode = AT_NODE_ANALYZE
        if self._dispatch_second_step_after_corridor_alignment_staging(robot_pose):
            self._publish_state()
            return
        if self._maybe_apply_near_exit_fallback(robot_pose):
            self._publish_state()
            return
        self._analyze_and_dispatch(robot_pose)
        self._publish_state()

    def _dispatch_entry_readiness_gate(self, robot_pose: RobotPose) -> dict[str, object]:
        nav2_lifecycle = self._nav2_lifecycle_sufficiency()
        navigate_ready = bool(self.action_client.wait_for_server(timeout_sec=0.0))
        goal_pose_subscribers = self.count_subscribers(self.goal_pose_topic)
        goal_pose_ready = goal_pose_subscribers > 0
        tf_sufficient = self._tf_sufficiency()
        scan = self._scan_sufficiency()
        map_summary = self._map_sufficiency(robot_pose)
        local_costmap = self._local_costmap_sufficiency(robot_pose)

        directional_override_payload = local_costmap.get('directional_override')
        if not isinstance(directional_override_payload, dict):
            directional_override_payload = {}
        checks = {
            'nav2_lifecycle_active': bool(nav2_lifecycle['active']),
            'navigate_to_pose_action_ready': navigate_ready,
            'goal_pose_subscriber_ready': goal_pose_ready,
            'scan_sufficient': bool(scan['sufficient']),
            'map_sufficient': bool(map_summary['sufficient']),
            'tf_sufficient': bool(tf_sufficient['sufficient']),
            'local_costmap_sufficient': bool(local_costmap['sufficient']),
            # Phase74 audit fields: these preserve the original full-window gate
            # result separately from the optional directional override.  Only
            # local_costmap_sufficient participates in readiness blocking.
            'local_costmap_full_window_sufficient': bool(local_costmap.get('full_window_sufficient', local_costmap['sufficient'])),
            'local_costmap_directional_override_applied': bool(directional_override_payload.get('override_applied', False)),
        }
        blocking_check_keys = [
            'nav2_lifecycle_active',
            'navigate_to_pose_action_ready',
            'goal_pose_subscriber_ready',
            'scan_sufficient',
            'map_sufficient',
            'tf_sufficient',
            'local_costmap_sufficient',
        ]
        blocking_reasons = [key for key in blocking_check_keys if not checks.get(key, False)]
        return {
            'phase49_dispatch_entry_readiness_gate': True,
            'passed': not blocking_reasons,
            'blocking_reasons': blocking_reasons,
            'checks': checks,
            'nav2_lifecycle': nav2_lifecycle,
            'navigate_to_pose_action_ready': navigate_ready,
            'goal_pose_topic': self.goal_pose_topic,
            'goal_pose_subscriber_count': int(goal_pose_subscribers),
            'scan': scan,
            'map': map_summary,
            'tf': tf_sufficient,
            'local_costmap': local_costmap,
            'local_costmap_full_window': local_costmap.get('local_costmap_full_window', local_costmap),
            'local_costmap_directional_override': local_costmap.get('directional_override'),
            'robot_pose_map': self._pose_to_payload(robot_pose),
        }

    def _dispatch_entry_direct_goal(self, robot_pose: RobotPose) -> None:
        """Dispatch a straight-line goal from entrance into the maze.

        This bypasses topology/staging/branch-detection for the very first
        goal, avoiding the staging misfire that causes first-goal timeouts.
        """
        d = self.entry_direct_distance_m
        target_x = self.entrance_x + math.cos(self.entrance_yaw) * d
        target_y = self.entrance_y + math.sin(self.entrance_yaw) * d
        target_yaw = self.entrance_yaw
        self.entry_direct_dispatched = True
        self.get_logger().info(
            'ENTRY_DIRECT: dispatching straight-line goal from entrance '
            '(%.3f, %.3f, yaw=%.3f) to (%.3f, %.3f, yaw=%.3f) distance=%.2f'
            % (self.entrance_x, self.entrance_y, self.entrance_yaw,
               target_x, target_y, target_yaw, d)
        )
        self._send_goal(
            target_xy=(target_x, target_y),
            yaw=target_yaw,
            goal_kind='entry_direct',
            skip_two_step_staging=True,
        )

    def _nav2_lifecycle_sufficiency(self) -> dict[str, object]:
        node_states = {}
        active = True
        for node_name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                self.lifecycle_state_futures.pop(node_name, None)
                node_states[node_name] = {'state_id': None, 'state_label': None, 'active': False, 'reason': 'service_not_ready'}
                active = False
                continue
            future = self.lifecycle_state_futures.get(node_name)
            if future is not None and future.done():
                result = future.result()
                if result is not None:
                    state = result.current_state
                    state_active = int(state.id) == int(State.PRIMARY_STATE_ACTIVE)
                    self.lifecycle_state_cache[node_name] = {
                        'state_id': int(state.id),
                        'state_label': str(state.label),
                        'active': state_active,
                        'reason': 'active' if state_active else 'not_active',
                    }
                self.lifecycle_state_futures.pop(node_name, None)
                future = None
            if future is None:
                self.lifecycle_state_futures[node_name] = client.call_async(GetState.Request())
            cached = self.lifecycle_state_cache.get(node_name)
            if cached is None:
                node_states[node_name] = {'state_id': None, 'state_label': None, 'active': False, 'reason': 'state_pending'}
                active = False
                continue
            node_states[node_name] = dict(cached)
            active = active and bool(cached.get('active', False))
        return {
            'active': bool(active),
            'required_nodes': list(self.dispatch_readiness_required_lifecycle_nodes),
            'node_states': node_states,
        }

    def _tf_sufficiency(self) -> dict[str, object]:
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - ROS TF exceptions vary.
            return {'sufficient': False, 'source': f'{self.map_frame}->{self.base_frame}', 'reason': str(exc)}
        return {'sufficient': True, 'source': f'{self.map_frame}->{self.base_frame}', 'reason': 'available'}

    def _scan_sufficiency(self) -> dict[str, object]:
        if self.scan_msg is None:
            return {'sufficient': False, 'finite_count': 0, 'nearest_obstacle_m': None, 'reason': 'scan_unavailable'}
        finite_ranges = [float(value) for value in self.scan_msg.ranges if math.isfinite(float(value))]
        nearest = min(finite_ranges) if finite_ranges else None
        sufficient = len(finite_ranges) >= self.dispatch_readiness_min_scan_finite_count
        return {
            'sufficient': bool(sufficient),
            'finite_count': int(len(finite_ranges)),
            'nearest_obstacle_m': float(nearest) if nearest is not None else None,
            'min_required_finite_count': int(self.dispatch_readiness_min_scan_finite_count),
            'reason': 'sufficient' if sufficient else 'finite_count_below_threshold',
        }

    def _map_sufficiency(self, robot_pose: RobotPose) -> dict[str, object]:
        return self._grid_sufficiency(
            self.map_view,
            robot_pose,
            min_known_ratio=self.dispatch_readiness_min_map_known_ratio,
            min_free_ratio=self.dispatch_readiness_min_map_free_ratio,
            label='map',
            grid_msg=self.map_msg,
        )

    def _local_costmap_sufficiency(self, robot_pose: RobotPose) -> dict[str, object]:
        summary = self._grid_sufficiency(
            self.local_costmap_view,
            robot_pose,
            min_known_ratio=self.dispatch_readiness_min_local_costmap_known_ratio,
            min_free_ratio=self.dispatch_readiness_min_local_costmap_free_ratio,
            label='local_costmap',
            grid_msg=self.local_costmap_msg,
        )
        age = None
        if self.local_costmap_last_update_time is not None:
            age = max(0.0, float((self.get_clock().now() - self.local_costmap_last_update_time).nanoseconds / 1e9))
        summary['sample_age_sec'] = round(age, 6) if age is not None else None
        if age is None:
            summary['sufficient'] = False
            summary['reason'] = 'local_costmap_unavailable'
        elif age > self.dispatch_readiness_max_local_costmap_age_sec:
            summary['sufficient'] = False
            summary['reason'] = 'local_costmap_stale'

        # Preserve the original full-window readiness verdict.  Phase74 may add
        # an effective directional override, but this record remains immutable so
        # runtime artifacts can compare full-window vs direction-aware semantics.
        full_window = dict(summary)
        directional_override = self._directional_local_costmap_readiness_override(robot_pose, full_window)
        full_window_sufficient = bool(full_window.get('sufficient', False))
        override_applied = bool(directional_override.get('override_applied', False))
        summary.update({
            'local_costmap_full_window': full_window,
            'full_window_sufficient': full_window_sufficient,
            'full_window_reason': full_window.get('reason'),
            'directional_override': directional_override,
            'effective_sufficient': bool(full_window_sufficient or override_applied),
        })
        if override_applied and not full_window_sufficient:
            summary['sufficient'] = True
            summary['reason'] = 'directional_override_sufficient'
        else:
            summary['sufficient'] = full_window_sufficient
            summary['reason'] = full_window.get('reason')
        return summary

    def _directional_local_costmap_readiness_override(
        self,
        robot_pose: RobotPose,
        full_window: dict[str, object],
    ) -> dict[str, object]:
        base: dict[str, object] = {
            'phase74_directional_local_costmap_readiness_override': True,
            'enabled': bool(self.directional_local_costmap_readiness_override_enabled),
            'override_considered': False,
            'override_applied': False,
            'reason': 'disabled',
            'full_window_gate_failed': not bool(full_window.get('sufficient', False)),
            'fresh': False,
            'knownness_ok': False,
            'robot_in_bounds': bool(full_window.get('robot_in_bounds', False)),
            'known_ratio': full_window.get('known_ratio'),
            'min_known_ratio': full_window.get('min_known_ratio'),
            'free_ratio': full_window.get('free_ratio'),
            'min_free_ratio': full_window.get('min_free_ratio'),
            'sample_age_sec': full_window.get('sample_age_sec'),
            'selected_candidate_direction': None,
            'candidate_directions': [],
            'direction_traversable_count': 0,
            'non_reverse_direction_traversable_count': 0,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
        }
        if not self.directional_local_costmap_readiness_override_enabled:
            return base
        base['override_considered'] = True
        if bool(full_window.get('sufficient', False)):
            base['reason'] = 'full_window_gate_already_sufficient'
            base['fresh'] = True
            base['knownness_ok'] = True
            return base
        if self.map_view is None or self.local_costmap_view is None:
            base['reason'] = 'map_or_local_costmap_unavailable'
            return base
        age = full_window.get('sample_age_sec')
        fresh = isinstance(age, (int, float)) and float(age) <= self.dispatch_readiness_max_local_costmap_age_sec
        known_ratio = full_window.get('known_ratio')
        min_known_ratio = full_window.get('min_known_ratio')
        knownness_ok = (
            isinstance(known_ratio, (int, float))
            and isinstance(min_known_ratio, (int, float))
            and float(known_ratio) >= float(min_known_ratio)
        )
        robot_in_bounds = bool(full_window.get('robot_in_bounds', False))
        base['fresh'] = bool(fresh)
        base['knownness_ok'] = bool(knownness_ok)
        base['robot_in_bounds'] = bool(robot_in_bounds)
        if not fresh:
            base['reason'] = 'local_costmap_not_fresh'
            return base
        if not knownness_ok:
            base['reason'] = 'local_costmap_knownness_insufficient'
            return base
        if not robot_in_bounds:
            base['reason'] = 'robot_out_of_local_costmap_bounds'
            return base

        local = classify_local_topology(
            self.map_view,
            robot_pose,
            angle_step_deg=self.branch_angle_step_deg,
            lookahead_m=self.branch_lookahead_m,
            clearance_radius_m=self.clearance_radius_m,
            min_open_distance_m=self.min_open_distance_m,
        )
        filtered_open_directions = filter_open_directions(
            local.open_directions,
            robot_yaw=robot_pose[2],
            allow_reverse=self.allow_reverse_branch_goals,
            reverse_angle_threshold_deg=self.reverse_branch_angle_threshold_deg,
        )
        reverse_threshold = math.radians(self.reverse_branch_angle_threshold_deg)
        candidates: list[dict[str, object]] = []
        traversable_count = 0
        non_reverse_traversable_count = 0
        selected: Optional[dict[str, object]] = None
        selected_reverse_fallback: Optional[dict[str, object]] = None
        for direction in filtered_open_directions:
            angle = float(direction.angle_rad)
            target = make_centered_branch_goal(
                self.map_view,
                pose_xy=(robot_pose[0], robot_pose[1]),
                direction_rad=angle,
                preferred_step_m=self.branch_goal_step_m,
                min_step_m=self.min_goal_step_m,
                clearance_radius_m=self.clearance_radius_m,
                lateral_search_m=self.lateral_centering_search_m,
            )
            reverse_delta = abs(normalize_angle(angle - robot_pose[2]))
            is_reverse = bool(reverse_delta >= reverse_threshold)
            if target is None:
                row = {
                    'candidate_direction_angle_rad': angle,
                    'candidate_direction_angle_deg': math.degrees(angle),
                    'target': None,
                    'is_reverse_candidate': is_reverse,
                    'direction_corridor_traversable': False,
                    'reason': 'candidate_target_unavailable',
                    'path_cost': None,
                    'min_clearance_m': None,
                    'target_risk': None,
                    'footprint_risk': None,
                    'front_wedge_risk': None,
                }
                candidates.append(row)
                continue
            geometry = self._compute_goal_geometry_diagnostics(robot_pose, target, angle)
            local_cost = self._compute_local_cost_diagnostics(robot_pose, target)
            footprint = self._local_cost_target_footprint_evidence(target, angle)
            wedge_values, wedge_clearance = self._local_cost_values_in_wedge(
                self.local_costmap_view,
                (robot_pose[0], robot_pose[1]),
                angle,
                self.local_cost_front_wedge_radius_m,
                self.local_cost_front_wedge_half_angle_rad,
            )
            path_max = local_cost.get('dispatch_path_local_cost_max')
            path_mean = local_cost.get('dispatch_path_local_cost_mean')
            raw_path_samples = local_cost.get('dispatch_path_local_cost_sample_count')
            raw_path_in_bounds = local_cost.get('dispatch_path_local_cost_in_bounds_sample_count')
            raw_coverage = local_cost.get('dispatch_local_cost_sample_coverage_ratio')
            path_samples = int(raw_path_samples) if isinstance(raw_path_samples, (int, float)) else 0
            path_in_bounds = int(raw_path_in_bounds) if isinstance(raw_path_in_bounds, (int, float)) else 0
            coverage = float(raw_coverage) if isinstance(raw_coverage, (int, float)) else 0.0
            min_clearance = geometry.get('path_corridor_min_clearance_m')
            if min_clearance is None:
                min_clearance = geometry.get('target_clearance_m')
            path_cost_ok = isinstance(path_max, (int, float)) and float(path_max) < float(self.local_cost_inflation_threshold)
            clearance_ok = isinstance(min_clearance, (int, float)) and float(min_clearance) >= float(self.clearance_radius_m)
            coverage_ok = path_samples > 0 and path_in_bounds > 0 and coverage >= 0.70
            traversable = bool(path_cost_ok and clearance_ok and coverage_ok)
            target_values = footprint.get('summary') if isinstance(footprint.get('summary'), dict) else {}
            row = {
                'candidate_direction_angle_rad': angle,
                'candidate_direction_angle_deg': math.degrees(angle),
                'target': self._point_to_payload(target),
                'is_reverse_candidate': is_reverse,
                'direction_corridor_traversable': traversable,
                'reason': 'directional_corridor_traversable' if traversable else 'directional_corridor_not_traversable',
                'path_cost': {
                    'max': path_max,
                    'mean': path_mean,
                    'sample_count': path_samples,
                    'in_bounds_sample_count': path_in_bounds,
                    'coverage_ratio': coverage,
                    'threshold': int(self.local_cost_inflation_threshold),
                },
                'min_clearance_m': min_clearance,
                'target_risk': {
                    'target_local_cost': local_cost.get('dispatch_target_local_cost'),
                    'target_local_cost_max_radius': local_cost.get('dispatch_target_local_cost_max_radius'),
                    'target_in_local_costmap_bounds': local_cost.get('dispatch_target_in_local_costmap_bounds'),
                },
                'footprint_risk': target_values,
                'front_wedge_risk': {
                    'max': max(wedge_values) if wedge_values else None,
                    'mean': float(sum(wedge_values) / len(wedge_values)) if wedge_values else None,
                    'high_cost_count': int(sum(1 for value in wedge_values if value >= self.local_cost_inflation_threshold)),
                    'lethal_count': int(sum(1 for value in wedge_values if value >= self.local_cost_obstacle_threshold)),
                    'clearance_m': wedge_clearance,
                },
            }
            if traversable:
                traversable_count += 1
                if is_reverse:
                    selected_reverse_fallback = selected_reverse_fallback or row
                else:
                    non_reverse_traversable_count += 1
                    selected = selected or row
            candidates.append(row)
        if selected is None:
            selected = selected_reverse_fallback
        base.update({
            'local_topology_kind': local.kind,
            'raw_open_direction_count': int(len(local.open_directions)),
            'filtered_open_direction_count': int(len(filtered_open_directions)),
            'candidate_directions': candidates,
            'direction_traversable_count': int(traversable_count),
            'non_reverse_direction_traversable_count': int(non_reverse_traversable_count),
            'selected_candidate_direction': selected,
        })
        if selected is not None:
            base['override_applied'] = True
            base['reason'] = 'directional_corridor_traversable'
        else:
            base['reason'] = 'no_directional_corridor_traversable'
        return base

    def _grid_sufficiency(
        self,
        grid: Optional[OccupancyGridView],
        robot_pose: RobotPose,
        *,
        min_known_ratio: float,
        min_free_ratio: float,
        label: str,
        grid_msg: Optional[OccupancyGrid] = None,
    ) -> dict[str, object]:
        if grid is None:
            return {
                'sufficient': False,
                'known_ratio': None,
                'free_ratio': None,
                'occupied_ratio': None,
                'unknown_ratio': None,
                'sample_count': 0,
                'reason': f'{label}_unavailable',
            }
        ratios = self._grid_ratio_near_pose(grid, robot_pose, self.dispatch_readiness_near_robot_radius_m)
        diagnostics = self._grid_metadata_diagnostics(grid, robot_pose, self.dispatch_readiness_near_robot_radius_m, grid_msg)
        known_ratio_obj = ratios.get('known_ratio', 0.0)
        free_ratio_obj = ratios.get('free_ratio', 0.0)
        known_ratio = float(known_ratio_obj) if isinstance(known_ratio_obj, (int, float)) else 0.0
        free_ratio = float(free_ratio_obj) if isinstance(free_ratio_obj, (int, float)) else 0.0
        sufficient = (
            bool(ratios['robot_in_bounds'])
            and known_ratio >= min_known_ratio
            and free_ratio >= min_free_ratio
        )
        ratios.update(diagnostics)
        ratios.update({
            'sufficient': bool(sufficient),
            'min_known_ratio': float(min_known_ratio),
            'min_free_ratio': float(min_free_ratio),
            'radius_m': float(self.dispatch_readiness_near_robot_radius_m),
            'reason': 'sufficient' if sufficient else f'{label}_ratio_or_bounds_insufficient',
        })
        return ratios

    def _grid_ratio_near_pose(self, grid: OccupancyGridView, robot_pose: RobotPose, radius_m: float) -> dict[str, object]:
        center = grid.world_to_cell(robot_pose[0], robot_pose[1])
        radius_cells = int(math.ceil(max(radius_m, 0.0) / grid.info.resolution))
        known = 0
        free = 0
        occupied = 0
        unknown = 0
        sample_count = 0
        in_bounds_count = 0
        out_of_bounds_count = 0
        min_x = center[0] - radius_cells
        max_x = center[0] + radius_cells
        min_y = center[1] - radius_cells
        max_y = center[1] + radius_cells
        for cell_y in range(min_y, max_y + 1):
            for cell_x in range(min_x, max_x + 1):
                world_x, world_y = grid.cell_to_world(cell_x, cell_y)
                if math.hypot(world_x - robot_pose[0], world_y - robot_pose[1]) > radius_m + 1e-9:
                    continue
                sample_count += 1
                cell = (cell_x, cell_y)
                if not grid.in_bounds(cell):
                    out_of_bounds_count += 1
                    unknown += 1
                    continue
                in_bounds_count += 1
                value = grid.cell_value(cell)
                if value < 0:
                    unknown += 1
                elif value >= grid.occupied_threshold:
                    known += 1
                    occupied += 1
                else:
                    known += 1
                    free += 1
        denom = max(1, sample_count)
        return {
            'phase51_map_sufficiency_discrepancy_diagnostics': True,
            'robot_in_bounds': bool(grid.in_bounds(center)),
            'robot_cell': [int(center[0]), int(center[1])],
            'radius_cells': int(radius_cells),
            'sample_window': {'min_x': int(min_x), 'max_x': int(max_x), 'min_y': int(min_y), 'max_y': int(max_y)},
            'sample_count': int(sample_count),
            'in_bounds_count': int(in_bounds_count),
            'out_of_bounds_count': int(out_of_bounds_count),
            'known_count': int(known),
            'free_count': int(free),
            'occupied_count': int(occupied),
            'unknown_count': int(unknown),
            'out_of_bounds_as_unknown': True,
            'known_ratio': float(known / denom),
            'free_ratio': float(free / denom),
            'occupied_ratio': float(occupied / denom),
            'unknown_ratio': float(unknown / denom),
        }

    def _grid_metadata_diagnostics(
        self,
        grid: OccupancyGridView,
        robot_pose: RobotPose,
        radius_m: float,
        grid_msg: Optional[OccupancyGrid],
    ) -> dict[str, object]:
        stamp_sec = None
        frame_id = None
        if grid_msg is not None:
            stamp = grid_msg.header.stamp
            stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
            frame_id = str(grid_msg.header.frame_id)
        center = grid.world_to_cell(robot_pose[0], robot_pose[1])
        radius_cells = int(math.ceil(max(radius_m, 0.0) / grid.info.resolution))
        min_x = center[0] - radius_cells
        max_x = center[0] + radius_cells
        min_y = center[1] - radius_cells
        max_y = center[1] + radius_cells
        return {
            'map_stamp': stamp_sec,
            'grid_frame_id': frame_id,
            'grid_width': int(grid.info.width),
            'grid_height': int(grid.info.height),
            'grid_resolution': float(grid.info.resolution),
            'grid_origin': {'x': float(grid.info.origin_x), 'y': float(grid.info.origin_y)},
            'bbox_world': {
                'min_x': float(grid.info.origin_x + min_x * grid.info.resolution),
                'max_x': float(grid.info.origin_x + (max_x + 1) * grid.info.resolution),
                'min_y': float(grid.info.origin_y + min_y * grid.info.resolution),
                'max_y': float(grid.info.origin_y + (max_y + 1) * grid.info.resolution),
            },
        }

    def _analyze_and_dispatch(self, robot_pose: RobotPose) -> None:
        assert self.map_view is not None
        local = classify_local_topology(
            self.map_view,
            robot_pose,
            angle_step_deg=self.branch_angle_step_deg,
            lookahead_m=self.branch_lookahead_m,
            clearance_radius_m=self.clearance_radius_m,
            min_open_distance_m=self.min_open_distance_m,
        )

        node_type = 'dead_end' if local.kind == DEAD_END else 'junction' if local.kind == JUNCTION else 'corridor'
        self.last_local_topology_kind = local.kind
        self.last_open_direction_count = len(local.open_directions)
        self.last_topology_sampling_diagnostics = self._build_topology_sampling_diagnostics(
            robot_pose,
            local.open_directions,
            filtered_open_directions=None,
            branch_options=None,
        )
        node = self.topology.find_or_create_node(robot_pose[0], robot_pose[1], node_type=node_type)
        self.current_node_id = node.node_id
        self.topology.visit_node(node.node_id)

        if local.kind == DEAD_END:
            self.dead_end_count += 1
            self.last_failure_reason = TRUE_DEAD_END
            self.phase56_open_direction_to_candidate_diagnostics = self._build_phase56_open_direction_to_candidate_diagnostics(
                robot_pose=robot_pose,
                raw_open_directions=local.open_directions,
                filtered_open_directions=[],
                branch_options=[],
                node=None,
                chosen=None,
                policy_state='dead_end_policy_before_candidate_generation',
                rejection_reason='dead_end_policy_no_branch_options',
            )
            self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
            self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            if self._maybe_delay_no_candidate_terminalization(
                robot_pose=robot_pose,
                raw_open_directions=local.open_directions,
                filtered_open_directions=[],
                branch_options=[],
                policy_state='dead_end_policy_before_candidate_generation',
                rejection_reason='dead_end_policy_no_branch_options',
            ):
                self._publish_goal_event('topology_consistency_guard')
                return
            exception_branch = self._maybe_apply_post_ingress_single_open_exception(
                robot_pose=robot_pose,
                local=local,
                node=node,
            )
            if exception_branch is not None:
                branch_options = [exception_branch]
                self.topology.set_branch_options(node.node_id, branch_options)
                candidate_before_filter_count = len(branch_options)
                selectable_candidate_count = sum(1 for branch in branch_options if branch.state in SELECTABLE_BRANCH_STATES)
                chosen = self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))
                if chosen is not None:
                    self._reset_topology_consistency_guard(candidate_recovered=True)
                    self.last_post_ingress_single_open_exception_diagnostics['single_open_exception_dispatch_produced'] = True
                    self.last_post_ingress_single_open_exception_diagnostics['single_open_exception_candidate_recovered_during_consistency_window'] = True
                    self.post_ingress_single_open_exception_consumed = True
                    self.phase56_open_direction_to_candidate_diagnostics = self._build_phase56_open_direction_to_candidate_diagnostics(
                        robot_pose=robot_pose,
                        raw_open_directions=local.open_directions,
                        filtered_open_directions=local.open_directions,
                        branch_options=branch_options,
                        node=node,
                        chosen=chosen,
                        policy_state='candidate_selected_for_dispatch',
                        rejection_reason=None,
                    )
                    self.phase56_open_direction_to_candidate_diagnostics['candidate_before_filter_count'] = int(candidate_before_filter_count)
                    self.phase56_open_direction_to_candidate_diagnostics['candidate_after_filter_count'] = int(selectable_candidate_count)
                    self.phase56_open_direction_to_candidate_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
                    self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
                    self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
                    self.last_topology_sampling_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
                    self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
                    self.last_candidate_count = len(branch_options)
                    self.pending_branch_choice_diagnostics = self._build_branch_choice_diagnostics(
                        node,
                        chosen,
                        dispatch_pose=robot_pose,
                        robot_yaw=robot_pose[2],
                        goal_kind='explore',
                    )
                    self.pending_branch_choice_diagnostics['selected_due_to_context'] = 'post_ingress_single_open_exception_existing_score'
                    self.pending_branch_choice_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
                    self.topology.mark_branch_state(node.node_id, chosen, IN_PROGRESS)
                    self.active_start_node_id = node.node_id
                    self.active_branch = chosen
                    self._send_goal(chosen.target_xy, yaw=chosen.angle_rad, goal_kind='explore')
                    return
            self.topology.mark_dead_end(node.node_id, incoming_edge_id=self.active_edge_id)
            backtrack_target = self.topology.next_backtrack_target(node.node_id)
            if backtrack_target is None:
                self._mark_exhausted('dead end reached and no untried junction remains')
                return
            self.pending_branch_choice_diagnostics = self._empty_branch_choice_diagnostics(selected_due_to_context='dead_end_backtrack')
            self._send_goal(backtrack_target.xy, yaw=self._yaw_to_point(robot_pose[:2], backtrack_target.xy), goal_kind='backtrack')
            return

        branch_options = []
        filtered_open_directions = filter_open_directions(
            local.open_directions,
            robot_yaw=robot_pose[2],
            allow_reverse=self.allow_reverse_branch_goals,
            reverse_angle_threshold_deg=self.reverse_branch_angle_threshold_deg,
        )
        self.last_open_direction_count = len(filtered_open_directions)
        self.last_topology_sampling_diagnostics = self._build_topology_sampling_diagnostics(
            robot_pose,
            local.open_directions,
            filtered_open_directions=filtered_open_directions,
            branch_options=None,
        )
        for direction in filtered_open_directions:
            target = make_centered_branch_goal(
                self.map_view,
                pose_xy=(robot_pose[0], robot_pose[1]),
                direction_rad=direction.angle_rad,
                preferred_step_m=self.branch_goal_step_m,
                min_step_m=self.min_goal_step_m,
                clearance_radius_m=self.clearance_radius_m,
                lateral_search_m=self.lateral_centering_search_m,
            )
            if target is not None:
                branch_options.append(BranchOption(angle_rad=direction.angle_rad, target_xy=target))
        self.topology.set_branch_options(node.node_id, branch_options)
        self.last_candidate_count = len(branch_options)
        candidate_before_filter_count = len(branch_options)
        selectable_candidate_count = sum(1 for branch in branch_options if branch.state in SELECTABLE_BRANCH_STATES)
        self.last_topology_sampling_diagnostics = self._build_topology_sampling_diagnostics(
            robot_pose,
            local.open_directions,
            filtered_open_directions=filtered_open_directions,
            branch_options=branch_options,
        )
        self.phase56_open_direction_to_candidate_diagnostics = self._build_phase56_open_direction_to_candidate_diagnostics(
            robot_pose=robot_pose,
            raw_open_directions=local.open_directions,
            filtered_open_directions=filtered_open_directions,
            branch_options=branch_options,
            node=node,
            chosen=None,
            policy_state='before_choose_next_branch',
            rejection_reason=None,
        )
        self.phase56_open_direction_to_candidate_diagnostics['candidate_before_filter_count'] = int(candidate_before_filter_count)
        self.phase56_open_direction_to_candidate_diagnostics['candidate_after_filter_count'] = int(selectable_candidate_count)
        self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
        self.blocked_branch_count = self.topology.blocked_branch_count()
        self.blacklisted_goal_count = len(self.topology.blacklist)

        chosen = self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))
        if chosen is not None:
            self._reset_topology_consistency_guard(candidate_recovered=True)
            self.phase56_open_direction_to_candidate_diagnostics = self._build_phase56_open_direction_to_candidate_diagnostics(
                robot_pose=robot_pose,
                raw_open_directions=local.open_directions,
                filtered_open_directions=filtered_open_directions,
                branch_options=branch_options,
                node=node,
                chosen=chosen,
                policy_state='candidate_selected_for_dispatch',
                rejection_reason=None,
            )
            self.phase56_open_direction_to_candidate_diagnostics['candidate_before_filter_count'] = int(candidate_before_filter_count)
            self.phase56_open_direction_to_candidate_diagnostics['candidate_after_filter_count'] = int(selectable_candidate_count)
            self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
            self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.pending_branch_choice_diagnostics = self._build_branch_choice_diagnostics(
                node,
                chosen,
                dispatch_pose=robot_pose,
                robot_yaw=robot_pose[2],
                goal_kind='explore',
            )
            self.topology.mark_branch_state(node.node_id, chosen, IN_PROGRESS)
            self.active_start_node_id = node.node_id
            self.active_branch = chosen
            self._send_goal(chosen.target_xy, yaw=chosen.angle_rad, goal_kind='explore')
            return

        backtrack_target = self.topology.next_backtrack_target(node.node_id)
        if backtrack_target is not None:
            self.phase56_open_direction_to_candidate_diagnostics['junction_or_dead_end_policy_filter'] = 'backtrack_selected_after_no_candidate'
            self.phase56_open_direction_to_candidate_diagnostics['branch_candidate_rejection_reason'] = self.phase56_open_direction_to_candidate_diagnostics.get('branch_candidate_rejection_reason') or 'node_policy_no_untried_branch'
            self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
            self.pending_branch_choice_diagnostics = self._empty_branch_choice_diagnostics(selected_due_to_context='backtrack')
            self._send_goal(backtrack_target.xy, yaw=self._yaw_to_point(robot_pose[:2], backtrack_target.xy), goal_kind='backtrack')
            return

        rejection = str(self.phase56_open_direction_to_candidate_diagnostics.get('branch_candidate_rejection_reason') or 'node_policy_no_untried_branch')
        self.phase56_open_direction_to_candidate_diagnostics['junction_or_dead_end_policy_filter'] = 'no_backtrack_after_no_candidate'
        self.phase56_open_direction_to_candidate_diagnostics['branch_candidate_rejection_reason'] = rejection
        self.last_topology_sampling_diagnostics['phase56_open_direction_to_candidate_diagnostics'] = self.phase56_open_direction_to_candidate_diagnostics
        if self._maybe_delay_no_candidate_terminalization(
            robot_pose=robot_pose,
            raw_open_directions=local.open_directions,
            filtered_open_directions=filtered_open_directions,
            branch_options=branch_options,
            policy_state='no_backtrack_after_no_candidate',
            rejection_reason=rejection,
        ):
            self._publish_goal_event('topology_consistency_guard')
            return
        self._mark_exhausted('no untried branches remain')

    def _post_ingress_single_open_context_active(self) -> bool:
        return bool(
            self.post_ingress_single_open_exception_enabled
            and not self.post_ingress_single_open_exception_consumed
            and self.goal_count == 0
            and self.active_edge_id is None
            and self.dispatch_readiness_first_pass_time is not None
        )

    def _single_open_exception_multi_frame_confirmed(self) -> bool:
        required = int(self.topology_consistency_required_no_candidate_frames)
        if len(self.topology_consistency_frames) < required:
            return False
        recent = self.topology_consistency_frames[-required:]
        for frame in recent:
            raw_count = frame.get('raw_open_direction_count')
            candidate_count = frame.get('candidate_after_filter_count')
            if not isinstance(raw_count, (int, float)) or not isinstance(candidate_count, (int, float)):
                return False
            if int(raw_count) != 1 or int(candidate_count) != 0:
                return False
        return True

    def _build_post_ingress_single_open_exception_diagnostics(
        self,
        *,
        robot_pose: RobotPose,
        raw_open_directions: Sequence[Any],
        candidate_xy: Optional[Point],
        reason: str,
        eligible: bool,
        applied: bool,
        angle_to_return_entrance: Optional[float],
        multi_frame_single_open_confirmed: bool,
    ) -> dict[str, object]:
        return {
            'post_ingress_single_open_exception_enabled': bool(self.post_ingress_single_open_exception_enabled),
            'post_ingress_context_active': bool(self._post_ingress_single_open_context_active()),
            'first_post_ingress_topology_node': bool(self.goal_count == 0 and self.active_edge_id is None),
            'single_open_exception_eligible': bool(eligible),
            'single_open_exception_applied': bool(applied),
            'single_open_exception_reason': reason,
            'angle_to_return_entrance': angle_to_return_entrance,
            'angle_to_return_entrance_min_deg': float(self.post_ingress_single_open_exception_return_angle_min_deg),
            'multi_frame_single_open_confirmed': bool(multi_frame_single_open_confirmed),
            'single_open_exception_candidate_recovered_during_consistency_window': bool(applied and multi_frame_single_open_confirmed),
            'raw_open_direction_count': int(len(raw_open_directions)),
            'filtered_open_direction_count': int(len(raw_open_directions)),
            'candidate_before_filter_count': 1 if candidate_xy is not None else 0,
            'candidate_after_filter_count': 1 if applied else 0,
            'candidate_goal_point': self._point_to_payload(candidate_xy),
            'candidate_map_cell_state': self._map_cell_state_for_point(candidate_xy),
            'candidate_local_costmap_cell_state': self._local_costmap_cell_state_for_point(candidate_xy),
            'single_open_exception_dispatch_produced': False,
        }

    def _maybe_apply_post_ingress_single_open_exception(
        self,
        *,
        robot_pose: RobotPose,
        local,
        node,
    ) -> Optional[BranchOption]:
        raw_open_directions = list(local.open_directions)
        post_ingress_context_active = self._post_ingress_single_open_context_active()
        first_post_ingress_topology_node = bool(self.goal_count == 0 and self.active_edge_id is None)
        multi_frame_single_open_confirmed = self._single_open_exception_multi_frame_confirmed()
        candidate_xy: Optional[Point] = None
        angle_to_return_entrance: Optional[float] = None
        single_open_exception_reason = 'not_evaluated'
        if not self.post_ingress_single_open_exception_enabled:
            single_open_exception_reason = 'disabled'
        elif self.post_ingress_single_open_exception_consumed:
            single_open_exception_reason = 'already_consumed'
        elif not self.post_ingress_single_open_exception_consumed and (not post_ingress_context_active or not first_post_ingress_topology_node):
            single_open_exception_reason = 'not_first_post_ingress_topology_node'
        elif local.kind != DEAD_END:
            single_open_exception_reason = 'not_dead_end_topology'
        elif local.kind == DEAD_END and len(local.open_directions) == 1 and len(raw_open_directions) != 1:
            single_open_exception_reason = 'not_single_open_direction'
        elif len(raw_open_directions) != 1:
            single_open_exception_reason = 'not_single_open_direction'
        elif not multi_frame_single_open_confirmed:
            single_open_exception_reason = 'not_multi_frame_confirmed'
        else:
            direction = raw_open_directions[0]
            return_heading = math.atan2(self.entrance_y - robot_pose[1], self.entrance_x - robot_pose[0])
            angle_to_return_entrance = abs(math.degrees(normalize_angle(float(direction.angle_rad) - return_heading)))
            if angle_to_return_entrance < self.post_ingress_single_open_exception_return_angle_min_deg:
                single_open_exception_reason = 'aligned_with_return_to_entrance'
            else:
                candidate_xy = make_centered_branch_goal(
                    self.map_view,
                    pose_xy=(robot_pose[0], robot_pose[1]),
                    direction_rad=direction.angle_rad,
                    preferred_step_m=self.branch_goal_step_m,
                    min_step_m=self.min_goal_step_m,
                    clearance_radius_m=self.clearance_radius_m,
                    lateral_search_m=self.lateral_centering_search_m,
                ) if self.map_view is not None else None
                candidate_map_cell_state = self._map_cell_state_for_point(candidate_xy)
                candidate_local_costmap_cell_state = self._local_costmap_cell_state_for_point(candidate_xy)
                if candidate_xy is None:
                    single_open_exception_reason = 'candidate_unavailable'
                elif candidate_map_cell_state.get('clearance_result') != 'clear':
                    single_open_exception_reason = 'candidate_map_not_clear'
                else:
                    single_open_exception_reason = 'post_ingress_single_open_exception_applied'
                    self.last_post_ingress_single_open_exception_diagnostics = self._build_post_ingress_single_open_exception_diagnostics(
                        robot_pose=robot_pose,
                        raw_open_directions=raw_open_directions,
                        candidate_xy=candidate_xy,
                        reason=single_open_exception_reason,
                        eligible=True,
                        applied=True,
                        angle_to_return_entrance=angle_to_return_entrance,
                        multi_frame_single_open_confirmed=multi_frame_single_open_confirmed,
                    )
                    self.last_topology_sampling_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
                    self.phase56_open_direction_to_candidate_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
                    return BranchOption(angle_rad=float(direction.angle_rad), target_xy=candidate_xy)
        self.last_post_ingress_single_open_exception_diagnostics = self._build_post_ingress_single_open_exception_diagnostics(
            robot_pose=robot_pose,
            raw_open_directions=raw_open_directions,
            candidate_xy=candidate_xy,
            reason=single_open_exception_reason,
            eligible=False,
            applied=False,
            angle_to_return_entrance=angle_to_return_entrance,
            multi_frame_single_open_confirmed=multi_frame_single_open_confirmed,
        )
        self.last_topology_sampling_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
        self.phase56_open_direction_to_candidate_diagnostics['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
        return None

    def _record_topology_consistency_frame(
        self,
        *,
        robot_pose: RobotPose,
        raw_open_directions: Sequence[Any],
        filtered_open_directions: Sequence[Any],
        branch_options: Sequence[BranchOption],
        policy_state: str,
        rejection_reason: Optional[str],
        terminalization_delayed: bool,
        dead_end_policy_confirmed: bool = False,
    ) -> dict[str, object]:
        candidate_before = int(len(branch_options))
        candidate_after = int(sum(1 for branch in branch_options if branch.state in SELECTABLE_BRANCH_STATES))
        now = self.get_clock().now()
        if self.topology_consistency_start_time is None:
            self.topology_consistency_start_time = now
        elapsed_sec = max(0.0, float((now - self.topology_consistency_start_time).nanoseconds / 1e9))
        frame = {
            'topology_consistency_frame_index': int(len(self.topology_consistency_frames) + 1),
            'elapsed_sec': elapsed_sec,
            'robot_pose_map': self._pose_to_payload(robot_pose),
            'raw_open_direction_count': int(len(raw_open_directions)),
            'filtered_open_direction_count': int(len(filtered_open_directions)),
            'candidate_before_filter_count': candidate_before,
            'candidate_after_filter_count': candidate_after,
            'branch_candidate_rejection_reason': rejection_reason,
            'junction_or_dead_end_policy_filter': policy_state,
            'dead_end_policy_state': 'confirmed' if dead_end_policy_confirmed else 'pending',
            'dead_end_policy_pending': not dead_end_policy_confirmed,
            'dead_end_policy_confirmed': bool(dead_end_policy_confirmed),
            'terminalization_delayed': bool(terminalization_delayed),
            'no_candidate': bool(candidate_after == 0),
        }
        self.topology_consistency_frames.append(frame)
        self.last_topology_consistency_diagnostics = self._topology_consistency_diagnostics(
            status='confirmed_no_candidate' if dead_end_policy_confirmed else 'pending',
            terminalization_delayed=terminalization_delayed,
        )
        self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
        self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
        return frame

    def _topology_consistency_no_candidate_confirmed(self) -> bool:
        required = int(self.topology_consistency_required_no_candidate_frames)
        if len(self.topology_consistency_frames) < required:
            return False
        recent = self.topology_consistency_frames[-required:]
        return all(int(frame.get('candidate_after_filter_count') or 0) == 0 for frame in recent)

    def _topology_consistency_diagnostics(
        self,
        *,
        status: str,
        terminalization_delayed: bool,
    ) -> dict[str, object]:
        elapsed_sec = None
        if self.topology_consistency_start_time is not None:
            elapsed_sec = max(0.0, float((self.get_clock().now() - self.topology_consistency_start_time).nanoseconds / 1e9))
        latest_frame = self.topology_consistency_frames[-1] if self.topology_consistency_frames else {}
        return {
            'topology_consistency_enabled': bool(self.topology_consistency_enabled),
            'topology_consistency_required_no_candidate_frames': int(self.topology_consistency_required_no_candidate_frames),
            'topology_consistency_window_sec': float(self.topology_consistency_window_sec),
            'topology_consistency_elapsed_sec': elapsed_sec,
            'topology_consistency_frame_count': int(len(self.topology_consistency_frames)),
            'topology_consistency_frame_index': latest_frame.get('topology_consistency_frame_index'),
            'topology_consistency_frames': list(self.topology_consistency_frames),
            'topology_consistency_guard_status': status,
            'topology_consistency_terminalization_reason': self.topology_consistency_terminalization_reason,
            'raw_open_direction_count': latest_frame.get('raw_open_direction_count'),
            'filtered_open_direction_count': latest_frame.get('filtered_open_direction_count'),
            'candidate_before_filter_count': latest_frame.get('candidate_before_filter_count'),
            'candidate_after_filter_count': latest_frame.get('candidate_after_filter_count'),
            'dead_end_policy_state': latest_frame.get('dead_end_policy_state', 'idle'),
            'terminalization_delayed': bool(terminalization_delayed),
            'candidate_recovered_during_consistency_window': bool(self.topology_consistency_candidate_recovered),
            'confirmed_no_candidate_frames': int(len(self.topology_consistency_frames)) if status == 'confirmed_no_candidate' else 0,
        }

    def _reset_topology_consistency_guard(self, *, candidate_recovered: bool = False) -> None:
        if candidate_recovered and self.topology_consistency_frames:
            self.topology_consistency_candidate_recovered = True
            self.topology_consistency_terminalization_reason = None
            self.last_topology_consistency_diagnostics = self._topology_consistency_diagnostics(
                status='candidate_recovered',
                terminalization_delayed=False,
            )
            self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.topology_consistency_start_time = None
            return
        self.topology_consistency_frames = []
        self.topology_consistency_start_time = None
        self.topology_consistency_candidate_recovered = False
        self.topology_consistency_terminalization_reason = None
        self.last_topology_consistency_diagnostics = self._topology_consistency_diagnostics(
            status='idle',
            terminalization_delayed=False,
        )

    def _maybe_delay_no_candidate_terminalization(
        self,
        *,
        robot_pose: RobotPose,
        raw_open_directions: Sequence[Any],
        filtered_open_directions: Sequence[Any],
        branch_options: Sequence[BranchOption],
        policy_state: str,
        rejection_reason: Optional[str],
    ) -> bool:
        candidate_after = int(sum(1 for branch in branch_options if branch.state in SELECTABLE_BRANCH_STATES))
        no_candidate_policy_reason = rejection_reason in {'dead_end_policy_no_branch_options', 'node_policy_no_untried_branch'}
        if not self.topology_consistency_enabled or candidate_after > 0 or not no_candidate_policy_reason:
            self._reset_topology_consistency_guard(candidate_recovered=candidate_after > 0)
            return False
        if self.topology_consistency_start_time is None:
            self.topology_consistency_frames = []
            self.topology_consistency_candidate_recovered = False
            self.topology_consistency_terminalization_reason = None
        self._record_topology_consistency_frame(
            robot_pose=robot_pose,
            raw_open_directions=raw_open_directions,
            filtered_open_directions=filtered_open_directions,
            branch_options=branch_options,
            policy_state=policy_state,
            rejection_reason=rejection_reason,
            terminalization_delayed=True,
            dead_end_policy_confirmed=False,
        )
        if self._topology_consistency_no_candidate_confirmed():
            self.topology_consistency_terminalization_reason = 'consecutive_no_candidate_frames_confirmed'
            last = self.topology_consistency_frames[-1]
            last['dead_end_policy_state'] = 'confirmed'
            last['dead_end_policy_pending'] = False
            last['dead_end_policy_confirmed'] = True
            last['terminalization_delayed'] = False
            self.last_topology_consistency_diagnostics = self._topology_consistency_diagnostics(
                status='confirmed_no_candidate',
                terminalization_delayed=False,
            )
            self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            return False
        elapsed_sec = self.last_topology_consistency_diagnostics.get('topology_consistency_elapsed_sec')
        if isinstance(elapsed_sec, (int, float)) and elapsed_sec > self.topology_consistency_window_sec:
            self.last_topology_consistency_diagnostics = self._topology_consistency_diagnostics(
                status='window_elapsed_waiting_for_required_frames',
                terminalization_delayed=True,
            )
            self.last_topology_sampling_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.phase56_open_direction_to_candidate_diagnostics['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
        self.mode = WAIT_FOR_TOPOLOGY_CONSISTENCY
        return True

    def _sample_topology_ray_diagnostic(self, robot_pose: RobotPose, angle_rad: float, matched_open_distance: Optional[float] = None) -> dict[str, object]:
        """Read-only mirror of initial topology ray sampling for Phase43 evidence."""
        sample_step_m = 0.05
        if self.map_view is not None:
            sample_step_m = max(self.map_view.info.resolution * 0.5, 0.05)
        best_distance = 0.0
        reject_reason = 'lookahead_complete'
        map_cell_state = 'map_unavailable'
        clearance_result = 'not_checked'
        first_block_xy = (
            robot_pose[0] + math.cos(angle_rad) * self.branch_lookahead_m,
            robot_pose[1] + math.sin(angle_rad) * self.branch_lookahead_m,
        )
        sampled_endpoint = first_block_xy
        if self.map_view is not None:
            distance = sample_step_m
            while distance <= self.branch_lookahead_m + 1e-9:
                x = robot_pose[0] + math.cos(angle_rad) * distance
                y = robot_pose[1] + math.sin(angle_rad) * distance
                cell = self.map_view.world_to_cell(x, y)
                value = self.map_view.cell_value(cell)
                in_bounds = self.map_view.in_bounds(cell)
                has_clearance = self.map_view.world_point_has_clearance(x, y, self.clearance_radius_m)
                map_cell_state = 'out_of_bounds' if not in_bounds else 'unknown' if value < 0 else 'occupied' if value >= self.map_view.occupied_threshold else 'free'
                clearance_result = 'clear' if has_clearance else 'blocked'
                sampled_endpoint = (x, y)
                if not has_clearance:
                    first_block_xy = (x, y)
                    if not in_bounds:
                        reject_reason = 'out_of_bounds_or_map_edge'
                    elif value < 0:
                        reject_reason = 'unknown_cell_or_unknown_clearance'
                    elif value >= self.map_view.occupied_threshold:
                        reject_reason = 'occupied_or_inflated_clearance'
                    else:
                        reject_reason = 'clearance_radius_blocked'
                    break
                best_distance = distance
                distance += sample_step_m
        local_cost = {
            'available': False,
            'sampled_endpoint_in_bounds': None,
            'sampled_endpoint_cost': None,
            'sampled_endpoint_max_radius_cost': None,
            'costmap_lethal_or_unknown_result': 'local_costmap_unavailable',
        }
        if self.local_costmap_view is not None:
            cell = self.local_costmap_view.world_to_cell(sampled_endpoint[0], sampled_endpoint[1])
            in_bounds = self.local_costmap_view.in_bounds(cell)
            value = int(self.local_costmap_view.cell_value(cell)) if in_bounds else None
            radius_values = self._local_cost_values_in_radius(self.local_costmap_view, sampled_endpoint, self.local_cost_target_radius_m)
            max_radius = max(radius_values) if radius_values else None
            if not in_bounds:
                cost_result = 'out_of_bounds'
            elif value is not None and value < 0:
                cost_result = 'unknown'
            elif max_radius is not None and max_radius >= self.local_cost_obstacle_threshold:
                cost_result = 'lethal_or_obstacle'
            elif max_radius is not None and max_radius >= self.local_cost_inflation_threshold:
                cost_result = 'inflated'
            else:
                cost_result = 'clear'
            local_cost = {
                'available': True,
                'sampled_endpoint_in_bounds': bool(in_bounds),
                'sampled_endpoint_cost': value,
                'sampled_endpoint_max_radius_cost': max_radius,
                'costmap_lethal_or_unknown_result': cost_result,
            }
        is_open = bool(matched_open_distance is not None and matched_open_distance >= self.min_open_distance_m)
        if matched_open_distance is not None and reject_reason != 'lookahead_complete':
            reject_reason = 'open_direction_after_merge' if is_open else reject_reason
        return {
            'sampled_direction': {'angle_rad': float(angle_rad), 'angle_deg': float(math.degrees(angle_rad))},
            'sampled_endpoint': self._point_to_payload(sampled_endpoint),
            'first_block_xy': self._point_to_payload(first_block_xy),
            'safe_distance_m': float(matched_open_distance if matched_open_distance is not None else best_distance),
            'is_open': is_open,
            'map_cell_state': map_cell_state,
            'clearance_result': clearance_result,
            'costmap_lethal_or_unknown_result': local_cost['costmap_lethal_or_unknown_result'],
            'local_costmap': local_cost,
            'reject_reason': None if is_open else reject_reason,
        }

    def _build_topology_sampling_diagnostics(
        self,
        robot_pose: RobotPose,
        open_directions: Sequence[Any],
        *,
        filtered_open_directions: Optional[Sequence[Any]],
        branch_options: Optional[list[BranchOption]],
    ) -> dict[str, object]:
        # This function is deliberately read-only: it samples the same geometry
        # context for explanation only and never mutates topology/selection.
        raw_open = list(open_directions)
        filtered = list(filtered_open_directions) if filtered_open_directions is not None else None
        branches = list(branch_options) if branch_options is not None else None
        step_rad = math.radians(self.branch_angle_step_deg)
        sample_count = max(1, int(round((2.0 * math.pi) / step_rad))) if step_rad > 0.0 else 0
        raw_by_angle: dict[float, float] = {}
        for direction in raw_open:
            raw_by_angle[round(float(direction.angle_rad), 6)] = float(direction.distance_m)
        filtered_angles = {round(float(direction.angle_rad), 6) for direction in filtered} if filtered is not None else None
        branch_angles = {round(float(branch.angle_rad), 6) for branch in branches} if branches is not None else None
        samples = []
        reason_counts: dict[str, int] = {}
        for index in range(sample_count):
            angle = normalize_angle(robot_pose[2] + step_rad * index)
            key = round(float(angle), 6)
            matched_distance = raw_by_angle.get(key)
            row = self._sample_topology_ray_diagnostic(robot_pose, angle, matched_distance)
            if filtered_angles is not None and row['is_open'] and key not in filtered_angles:
                row['reject_reason'] = 'reverse_filtered'
            if branch_angles is not None and row['is_open'] and key in filtered_angles and key not in branch_angles:
                row['reject_reason'] = 'centered_branch_goal_unavailable'
            reason = str(row.get('reject_reason') or 'accepted_open_direction')
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
            samples.append(row)
        tf_lookup_result = {'map_to_base_link': 'available', 'source': self.map_frame + '->' + self.base_frame}
        return {
            'phase43_readonly': True,
            'tf_lookup_result': tf_lookup_result,
            'robot_pose_map': self._pose_to_payload(robot_pose),
            'parameters': {
                'branch_angle_step_deg': float(self.branch_angle_step_deg),
                'branch_lookahead_m': float(self.branch_lookahead_m),
                'clearance_radius_m': float(self.clearance_radius_m),
                'min_open_distance_m': float(self.min_open_distance_m),
                'branch_goal_step_m': float(self.branch_goal_step_m),
                'min_goal_step_m': float(self.min_goal_step_m),
                'lateral_centering_search_m': float(self.lateral_centering_search_m),
                'allow_reverse_branch_goals': bool(self.allow_reverse_branch_goals),
            },
            'sampled_direction_count': len(samples),
            'raw_open_direction_count': len(raw_open),
            'filtered_open_direction_count': len(filtered) if filtered is not None else None,
            'candidate_branch_count': len(branches) if branches is not None else None,
            'candidate_before_filter_count': len(branches) if branches is not None else None,
            'candidate_after_filter_count': (
                sum(1 for branch in branches if branch.state in SELECTABLE_BRANCH_STATES)
                if branches is not None else None
            ),
            'phase56_open_direction_to_candidate_diagnostics': self.phase56_open_direction_to_candidate_diagnostics,
            'reject_reason_counts': reason_counts,
            'samples': samples,
        }

    def _map_cell_state_for_point(self, xy: Optional[Point]) -> dict[str, object]:
        if xy is None or self.map_view is None:
            return {'available': False, 'state': 'map_unavailable', 'cell': None, 'value': None, 'clearance_result': 'not_checked'}
        cell = self.map_view.world_to_cell(float(xy[0]), float(xy[1]))
        in_bounds = self.map_view.in_bounds(cell)
        value = self.map_view.cell_value(cell) if in_bounds else -1
        state = 'out_of_bounds' if not in_bounds else 'unknown' if value < 0 else 'occupied' if value >= self.map_view.occupied_threshold else 'free'
        has_clearance = bool(in_bounds and self.map_view.world_point_has_clearance(float(xy[0]), float(xy[1]), self.clearance_radius_m))
        nearest_obstacle_distance = None
        if in_bounds:
            nearest_obstacle_distance = float(self.map_view.nearest_obstacle_distance(float(xy[0]), float(xy[1]), max_radius_m=max(self.clearance_radius_m * 3.0, 1.2)))
        return {
            'available': True,
            'state': state,
            'cell': [int(cell[0]), int(cell[1])],
            'value': int(value),
            'in_bounds': bool(in_bounds),
            'clearance_result': 'clear' if has_clearance else 'blocked',
            'nearest_obstacle_distance_m': nearest_obstacle_distance,
        }

    def _local_costmap_cell_state_for_point(self, xy: Optional[Point]) -> dict[str, object]:
        if xy is None or self.local_costmap_view is None:
            return {'available': False, 'state': 'local_costmap_unavailable', 'cell': None, 'value': None}
        cell = self.local_costmap_view.world_to_cell(float(xy[0]), float(xy[1]))
        in_bounds = self.local_costmap_view.in_bounds(cell)
        value = int(self.local_costmap_view.cell_value(cell)) if in_bounds else None
        radius_values = self._local_cost_values_in_radius(self.local_costmap_view, xy, self.local_cost_target_radius_m)
        max_radius = max(radius_values) if radius_values else None
        if not in_bounds:
            state = 'out_of_bounds'
        elif value is not None and value < 0:
            state = 'unknown'
        elif max_radius is not None and max_radius >= self.local_cost_obstacle_threshold:
            state = 'lethal_or_obstacle'
        elif max_radius is not None and max_radius >= self.local_cost_inflation_threshold:
            state = 'inflated'
        else:
            state = 'clear'
        return {
            'available': True,
            'state': state,
            'cell': [int(cell[0]), int(cell[1])],
            'value': value,
            'in_bounds': bool(in_bounds),
            'max_radius_cost': max_radius,
        }

    def _build_phase56_open_direction_to_candidate_diagnostics(
        self,
        *,
        robot_pose: RobotPose,
        raw_open_directions: Sequence[Any],
        filtered_open_directions: Sequence[Any],
        branch_options: Sequence[BranchOption],
        node,
        chosen: Optional[BranchOption],
        policy_state: str,
        rejection_reason: Optional[str],
    ) -> dict[str, object]:
        """Phase56 read-only open-direction-to-candidate diagnostics; no strategy change."""
        accepted = list(filtered_open_directions) or list(raw_open_directions)
        accepted_direction = accepted[0] if accepted else None
        accepted_angle = float(accepted_direction.angle_rad) if accepted_direction is not None else None
        projection_distance = float(getattr(accepted_direction, 'distance_m', 0.0)) if accepted_direction is not None else None
        vector = [math.cos(accepted_angle), math.sin(accepted_angle)] if accepted_angle is not None else None
        sample_endpoint = None
        raw_branch_goal = None
        centered_goal = None
        if accepted_angle is not None:
            sample_distance = projection_distance if projection_distance is not None else self.branch_lookahead_m
            sample_endpoint = (
                float(robot_pose[0]) + math.cos(accepted_angle) * sample_distance,
                float(robot_pose[1]) + math.sin(accepted_angle) * sample_distance,
            )
            if self.map_view is not None:
                raw_branch_goal = make_branch_goal(
                    self.map_view,
                    pose_xy=(robot_pose[0], robot_pose[1]),
                    direction_rad=accepted_angle,
                    preferred_step_m=self.branch_goal_step_m,
                    min_step_m=self.min_goal_step_m,
                    clearance_radius_m=self.clearance_radius_m,
                )
                centered_goal = make_centered_branch_goal(
                    self.map_view,
                    pose_xy=(robot_pose[0], robot_pose[1]),
                    direction_rad=accepted_angle,
                    preferred_step_m=self.branch_goal_step_m,
                    min_step_m=self.min_goal_step_m,
                    clearance_radius_m=self.clearance_radius_m,
                    lateral_search_m=self.lateral_centering_search_m,
                )
        branch_list = list(branch_options)
        selected_branch = chosen or (branch_list[0] if branch_list else None)
        candidate_goal = selected_branch.target_xy if selected_branch is not None else centered_goal
        travel_distance = math.hypot(float(candidate_goal[0]) - robot_pose[0], float(candidate_goal[1]) - robot_pose[1]) if candidate_goal is not None else None
        min_ok = bool(travel_distance is not None and travel_distance >= self.min_goal_step_m)
        map_state = self._map_cell_state_for_point(candidate_goal)
        local_state = self._local_costmap_cell_state_for_point(candidate_goal)
        clearance_result = str(map_state.get('clearance_result'))
        if rejection_reason is None:
            if accepted_direction is None:
                rejection_reason = 'no_accepted_open_direction'
            elif raw_branch_goal is None:
                rejection_reason = 'raw_branch_goal_below_min_step'
            elif centered_goal is None:
                rejection_reason = 'centered_branch_goal_unavailable'
            elif travel_distance is not None and travel_distance < self.min_goal_step_m:
                rejection_reason = 'filtered_by_min_travel_distance'
            elif branch_list and all(branch.state not in SELECTABLE_BRANCH_STATES for branch in branch_list):
                rejection_reason = 'duplicate_or_exhausted'
            elif not branch_list:
                rejection_reason = 'centered_branch_goal_unavailable'
            elif chosen is None:
                rejection_reason = 'node_policy_no_untried_branch'
        duplicate_filter = 'not_checked'
        if branch_list:
            duplicate_filter = 'selectable' if any(branch.state in SELECTABLE_BRANCH_STATES for branch in branch_list) else 'duplicate_or_exhausted'
        elif raw_branch_goal is not None:
            duplicate_filter = 'no_candidate_created'
        return {
            'available': True,
            'accepted_open_direction_angle_rad': accepted_angle,
            'accepted_open_direction_angle_deg': math.degrees(accepted_angle) if accepted_angle is not None else None,
            'accepted_open_direction_vector': vector,
            'accepted_open_direction_projection_distance_m': projection_distance,
            'sample_endpoint': self._point_to_payload(sample_endpoint),
            'projection_distance_m': projection_distance,
            'lookahead_distance_m': float(self.branch_lookahead_m),
            'branch_goal_step_m': float(self.branch_goal_step_m),
            'min_goal_step_m': float(self.min_goal_step_m),
            'raw_branch_goal_point': self._point_to_payload(raw_branch_goal),
            'candidate_goal_point': self._point_to_payload(candidate_goal),
            'candidate_map_cell_state': map_state,
            'candidate_local_costmap_cell_state': local_state,
            'candidate_clearance_result': clearance_result,
            'min_travel_distance_check': {
                'travel_distance_m': travel_distance,
                'min_goal_step_m': float(self.min_goal_step_m),
                'passed': bool(min_ok),
            },
            'too_close_check': {
                'too_close': bool(travel_distance is not None and travel_distance < self.min_goal_step_m),
                'threshold_m': float(self.min_goal_step_m),
            },
            'duplicate_or_exhausted_filter': duplicate_filter,
            'junction_or_dead_end_policy_filter': policy_state,
            'branch_candidate_rejection_reason': rejection_reason,
            'raw_open_direction_count': int(len(raw_open_directions)),
            'filtered_open_direction_count': int(len(filtered_open_directions)),
            'candidate_before_filter_count': int(len(branch_list)),
            'candidate_after_filter_count': int(sum(1 for branch in branch_list if branch.state in SELECTABLE_BRANCH_STATES)),
            'candidate_branch_states': [branch.state for branch in branch_list],
            'chosen_candidate_goal_point': self._point_to_payload(chosen.target_xy) if chosen is not None else None,
        }

    def _empty_branch_choice_diagnostics(self, selected_due_to_context: Optional[str] = None) -> dict[str, object]:
        return {
            'chosen_branch_rank': None,
            'chosen_branch_score_components': None,
            'candidate_branch_count': 0,
            'candidate_branches': [],
            'selected_due_to_context': selected_due_to_context,
        }

    def _phase143_staging_gate_artifact_context(
        self,
        *,
        two_step_staging: dict[str, object],
        branch_choice_diagnostics: dict[str, object],
        branch_angle: float,
    ) -> dict[str, object]:
        """Return Phase143 event-context-only staging gate artifact serialization.

        This wrapper mirrors the perception-level `staging_gate_artifact_completeness`
        payload into `/maze/goal_events`.  It does not change gate logic,
        thresholds, branch scoring, fallback, centerline selection, or terminal
        acceptance semantics.
        """
        artifact_any = two_step_staging.get('staging_gate_artifact_completeness')
        artifact = dict(artifact_any) if isinstance(artifact_any, dict) else {}
        candidate_branches = branch_choice_diagnostics.get('candidate_branches')
        candidate_branch_count = branch_choice_diagnostics.get('candidate_branch_count')
        if isinstance(candidate_branches, list):
            selected_branch_geometry = next(
                (dict(row) for row in candidate_branches if isinstance(row, dict) and row.get('rank') == branch_choice_diagnostics.get('chosen_branch_rank')),
                dict(candidate_branches[0]) if candidate_branches and isinstance(candidate_branches[0], dict) else {},
            )
        else:
            selected_branch_geometry = {}
        artifact.setdefault('candidate_branch_count', candidate_branch_count)
        artifact.setdefault('last_open_direction_count', self.last_open_direction_count)
        artifact.setdefault('last_candidate_count', self.last_candidate_count)
        artifact.setdefault('branch_angle', float(branch_angle))
        artifact.setdefault('selected_branch_geometry', selected_branch_geometry)
        artifact.setdefault('source_single_step', {})
        artifact.setdefault('trigger_conditions', {})
        artifact.setdefault('staging_reason', two_step_staging.get('staging_reason'))
        artifact.setdefault('staging_reject_reason', two_step_staging.get('staging_reject_reason'))
        artifact.setdefault('gate_classification_candidate', 'insufficient_staging_gate_evidence')
        artifact.setdefault('branch_scoring_changed', False)
        artifact.setdefault('fallback_terminal_acceptance_used', False)
        return artifact

    def _build_branch_choice_diagnostics(
        self,
        node,
        chosen: Optional[BranchOption],
        *,
        dispatch_pose: Optional[RobotPose],
        robot_yaw: float,
        goal_kind: str,
    ) -> dict[str, object]:
        """Describe branch ranking at dispatch time without selecting a branch.

        Phase26E deliberately calls this only after MazeTopology.choose_next_branch()
        has already returned `chosen`.  The data below mirrors the existing score
        formula and runtime cost/clearance context for diagnostics; it must not be
        used as an alternate decision path.
        """
        selectable = [branch for branch in node.branches if branch.state in SELECTABLE_BRANCH_STATES]
        ranked = sorted(
            selectable,
            key=lambda branch: branch.score_for_exit(node.xy, (self.exit_x, self.exit_y), self.exit_bias_weight),
            reverse=True,
        )
        chosen_rank = None
        if chosen is not None:
            for index, branch in enumerate(ranked, start=1):
                if branch is chosen:
                    chosen_rank = index
                    break
        branches = []
        for branch in node.branches:
            rank = None
            for index, ranked_branch in enumerate(ranked, start=1):
                if branch is ranked_branch:
                    rank = index
                    break
            branches.append(
                self._branch_option_payload(
                    node,
                    branch,
                    chosen=chosen,
                    rank=rank,
                    dispatch_pose=dispatch_pose,
                    robot_yaw=robot_yaw,
                    goal_kind=goal_kind,
                )
            )
        chosen_components = None
        if chosen is not None:
            chosen_components = self._branch_score_components(node.xy, chosen, dispatch_pose=dispatch_pose)
        return {
            'chosen_branch_rank': chosen_rank,
            'chosen_branch_score_components': chosen_components,
            'candidate_branch_count': len(branches),
            'candidate_branches': branches,
            'selected_due_to_context': 'topology_exit_bias_score' if chosen is not None else None,
        }

    def _branch_score_components(
        self,
        node_xy: Point,
        branch: BranchOption,
        *,
        dispatch_pose: Optional[RobotPose],
    ) -> dict[str, object]:
        target_exit_dist = self._distance_to_exit(branch.target_xy)
        source_xy = (dispatch_pose[0], dispatch_pose[1]) if dispatch_pose is not None else node_xy
        source_exit_dist = self._distance_to_exit(source_xy)
        heading_to_exit = math.atan2(self.exit_y - node_xy[1], self.exit_x - node_xy[0])
        heading_alignment = math.cos(normalize_angle(branch.angle_rad - heading_to_exit))
        score = branch.score_for_exit(node_xy, (self.exit_x, self.exit_y), self.exit_bias_weight)
        return {
            'score': float(score),
            'target_exit_dist': float(target_exit_dist),
            'exit_progress_delta_m': float(source_exit_dist - target_exit_dist),
            'heading_alignment': float(heading_alignment),
            'exit_bias_weight': float(self.exit_bias_weight),
            'state': branch.state,
            'failure_reason': branch.failure_reason,
        }

    def _branch_option_payload(
        self,
        node,
        branch: BranchOption,
        *,
        chosen: Optional[BranchOption],
        rank: Optional[int],
        dispatch_pose: Optional[RobotPose],
        robot_yaw: float,
        goal_kind: str,
    ) -> dict[str, object]:
        geometry = self._compute_goal_geometry_diagnostics(dispatch_pose, branch.target_xy, branch.angle_rad)
        local_cost = self._compute_local_cost_diagnostics(dispatch_pose, branch.target_xy)
        score_components = self._branch_score_components(node.xy, branch, dispatch_pose=dispatch_pose)
        rejection_reason = None
        if branch.state not in SELECTABLE_BRANCH_STATES:
            rejection_reason = branch.failure_reason or branch.state
        elif chosen is not None and branch is not chosen:
            rejection_reason = 'lower_rank_not_selected'
        reverse_delta = abs(normalize_angle(branch.angle_rad - robot_yaw))
        reverse_threshold = math.radians(self.reverse_branch_angle_threshold_deg)
        return {
            'branch_angle': float(branch.angle_rad),
            'target': self._point_to_payload(branch.target_xy),
            'target_exit_dist': score_components['target_exit_dist'],
            'exit_progress_delta_m': score_components['exit_progress_delta_m'],
            'target_clearance_m': geometry.get('target_clearance_m'),
            'path_corridor_min_clearance_m': geometry.get('path_corridor_min_clearance_m'),
            'dispatch_path_local_cost_max': local_cost.get('dispatch_path_local_cost_max'),
            'dispatch_path_local_cost_mean': local_cost.get('dispatch_path_local_cost_mean'),
            'target_local_cost': local_cost.get('dispatch_target_local_cost'),
            'target_local_cost_max_radius': local_cost.get('dispatch_target_local_cost_max_radius'),
            'is_reverse_candidate': bool(reverse_delta >= reverse_threshold),
            'is_backtrack_context': bool(goal_kind == 'backtrack'),
            'is_near_exit_candidate': self._target_near_exit(branch.target_xy),
            'rejection_reason': rejection_reason,
            'rank': rank,
            'state': branch.state,
            'score_components': score_components,
        }

    def _empty_centerline_target_refinement_diagnostics(self, reason: str = 'idle') -> dict[str, object]:
        return {
            'enabled': bool(getattr(self, 'centerline_target_refinement_enabled', False)),
            'gate_mode': str(getattr(self, 'centerline_target_refinement_gate_mode', 'balance_first')),
            'applied': False,
            'reason': reason,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
            'original_target': None,
            'refined_target': None,
            'centerline_projected_target': None,
            'corridor_heading_yaw': None,
            'refinement_applied': False,
            'refinement_reject_reason': reason,
            'forward_executability_check': {
                'checked': False,
                'passed': False,
                'reason': reason,
                'same_corridor': False,
                'two_side_wall_evidence': False,
                'target_has_clearance': False,
                'occupancy_free': False,
                'forward_progress_not_lowered': False,
                'forward_progress_not_obviously_lowered': False,
                'safety_floor_ok': False,
                'footprint_lethal_not_increased': False,
                'front_wedge_lethal_not_increased': False,
                'local_cost_sample_count': 0,
                'front_wedge_sample_count': 0,
            },
            'candidate_family': {
                'centerline_projection': True,
                'bounded_local_search': True,
                'forward_offsets_m': [],
                'lateral_offsets_m': [],
                'heading_offsets_rad': [],
            },
            'candidate_count': 0,
            'eligible_candidate_count': 0,
            'strict_eligible_candidate_count': 0,
            'balance_first_eligible_candidate_count': 0,
            'hard_safety_pass_candidate_count': 0,
            'selected_candidate_index': None,
            'selected_candidate_target': None,
            'selected_candidate_yaw': None,
            'selection_priority_trace': [
                'hard safety pass',
                'no footprint/front-wedge lethal regression',
                'safety_floor_ok',
                'forward_progress_ok',
                'clearance better',
                'balance error smaller',
            ],
            'rejected_candidate_summaries': [],
            'original_target_preserved_on_reject': False,
            'multi_candidate_forward_search': {
                'enabled': False,
                'candidate_family': {
                    'centerline_projection': True,
                    'bounded_local_search': True,
                    'forward_offsets_m': [],
                    'lateral_offsets_m': [],
                    'heading_offsets_rad': [],
                },
                'candidate_count': 0,
                'hard_safety_pass_candidate_count': 0,
                'selected_candidate_index': None,
                'selected_candidate_target': None,
                'selected_candidate_yaw': None,
                'selection_priority_trace': [],
                'rejected_candidate_summaries': [],
                'refinement_applied': False,
                'refinement_reject_reason': reason,
                'original_target_preserved_on_reject': False,
                'branch_scoring_changed': False,
                'fallback_terminal_acceptance_used': False,
            },
            'gate_conditions': {
                'same_corridor': False,
                'two_side_wall_evidence': False,
                'target_has_clearance': False,
                'occupancy_free': False,
                'forward_progress_not_lowered': False,
                'forward_progress_not_obviously_lowered': False,
                'improves_balance': False,
                'balance_error_improved': False,
                'improves_min_clearance': False,
                'improves_local_cost': False,
                'improves_front_wedge': False,
                'safety_floor_ok': False,
                'footprint_lethal_not_increased': False,
                'front_wedge_lethal_not_increased': False,
                'balance_first_eligible': False,
            },
        }

    def _maybe_refine_corridor_centerline_dispatch_target(
        self,
        *,
        dispatch_pose: Optional[RobotPose],
        original_target: Point,
        yaw: float,
        goal_kind: str,
    ) -> dict[str, object]:
        if not self.centerline_target_refinement_enabled:
            diagnostics = self._empty_centerline_target_refinement_diagnostics('disabled')
        elif goal_kind != 'explore':
            diagnostics = self._empty_centerline_target_refinement_diagnostics('non_explore_goal')
        elif self.map_view is None:
            diagnostics = self._empty_centerline_target_refinement_diagnostics('map_unavailable')
        elif dispatch_pose is None:
            diagnostics = self._empty_centerline_target_refinement_diagnostics('dispatch_pose_unavailable')
        else:
            diagnostics = refine_corridor_centerline_target(
                map_grid=self.map_view,
                local_cost_grid=self.local_costmap_view,
                dispatch_pose=dispatch_pose,
                original_target=original_target,
                direction_rad=yaw,
                clearance_radius_m=self.clearance_radius_m,
                side_probe_m=self.centerline_target_refinement_side_probe_m,
                forward_offsets_m=self.centerline_target_refinement_forward_offsets_m,
                lateral_offsets_m=self.centerline_target_refinement_lateral_offsets_m,
                local_cost_radius_m=self.local_cost_target_radius_m,
                front_wedge_radius_m=self.local_cost_front_wedge_radius_m,
                front_wedge_half_angle_rad=self.local_cost_front_wedge_half_angle_rad,
                high_cost_threshold=self.local_cost_inflation_threshold,
                gate_mode=self.centerline_target_refinement_gate_mode,
                min_clearance_floor_m=self.centerline_target_refinement_min_clearance_floor_m,
                forward_progress_tolerance_m=self.centerline_target_refinement_forward_progress_tolerance_m,
            )
        diagnostics['original_target'] = diagnostics.get('original_target') or self._point_to_payload(original_target)
        diagnostics['refined_target'] = diagnostics.get('refined_target') or self._point_to_payload(original_target)
        diagnostics['branch_scoring_changed'] = False
        diagnostics['fallback_terminal_acceptance_used'] = False
        diagnostics['refinement_applied'] = bool(diagnostics.get('refinement_applied', diagnostics.get('applied', False)))
        diagnostics.setdefault('refinement_reject_reason', None if diagnostics['refinement_applied'] else diagnostics.get('reason'))
        diagnostics.setdefault('centerline_projected_target', diagnostics.get('refined_target') if diagnostics['refinement_applied'] else None)
        diagnostics.setdefault('corridor_heading_yaw', float(yaw) if diagnostics['refinement_applied'] else None)
        diagnostics.setdefault('forward_executability_check', {'checked': False, 'passed': False, 'reason': diagnostics.get('reason')})
        diagnostics.setdefault('candidate_family', {})
        diagnostics.setdefault('hard_safety_pass_candidate_count', 0)
        diagnostics.setdefault('selected_candidate_index', None)
        diagnostics.setdefault('selected_candidate_target', None)
        diagnostics.setdefault('selected_candidate_yaw', None)
        diagnostics.setdefault('selection_priority_trace', [])
        diagnostics.setdefault('rejected_candidate_summaries', [])
        diagnostics.setdefault('original_target_preserved_on_reject', bool(not diagnostics['refinement_applied']))
        diagnostics.setdefault('multi_candidate_forward_search', {
            'enabled': bool(diagnostics.get('enabled', False)),
            'candidate_family': diagnostics.get('candidate_family', {}),
            'candidate_count': diagnostics.get('candidate_count', 0),
            'hard_safety_pass_candidate_count': diagnostics.get('hard_safety_pass_candidate_count', 0),
            'selected_candidate_index': diagnostics.get('selected_candidate_index'),
            'selected_candidate_target': diagnostics.get('selected_candidate_target'),
            'selected_candidate_yaw': diagnostics.get('selected_candidate_yaw'),
            'selection_priority_trace': diagnostics.get('selection_priority_trace', []),
            'rejected_candidate_summaries': diagnostics.get('rejected_candidate_summaries', []),
            'refinement_applied': bool(diagnostics.get('refinement_applied', False)),
            'refinement_reject_reason': diagnostics.get('refinement_reject_reason'),
            'original_target_preserved_on_reject': bool(diagnostics.get('original_target_preserved_on_reject', False)),
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        })
        self.last_centerline_target_refinement_diagnostics = diagnostics
        return diagnostics

    def _maybe_plan_corridor_alignment_staging(
        self,
        *,
        dispatch_pose: Optional[RobotPose],
        original_target: Point,
        yaw: float,
        goal_kind: str,
        centerline_refinement: dict[str, object],
        skip_two_step_staging: bool,
    ) -> dict[str, object]:
        if skip_two_step_staging or goal_kind != 'explore' or self.map_view is None or dispatch_pose is None:
            return {
                'two_step_staging_plan': {'enabled': False, 'reason': 'not_applicable'},
                'staging_goal_pose': None,
                'staging_reason': None,
                'staging_executability_check': {'checked': False, 'hard_safety_pass': False, 'reason': 'not_applicable'},
                'second_step_forward_goal': None,
                'staging_applied': False,
                'staging_reject_reason': 'not_applicable',
                'branch_scoring_changed': False,
                'fallback_terminal_acceptance_used': False,
            }
        return plan_two_step_corridor_alignment_staging_goal(
            map_grid=self.map_view,
            local_cost_grid=self.local_costmap_view,
            dispatch_pose=dispatch_pose,
            original_target=original_target,
            direction_rad=yaw,
            clearance_radius_m=self.clearance_radius_m,
            source_forward_refinement=centerline_refinement,
            side_probe_m=self.centerline_target_refinement_side_probe_m,
            staging_lateral_offsets_m=(0.10, 0.20, 0.30),
            staging_forward_offsets_m=(0.0, 0.05, 0.10),
            max_staging_distance_m=0.35,
            local_cost_radius_m=self.local_cost_target_radius_m,
            front_wedge_radius_m=self.local_cost_front_wedge_radius_m,
            front_wedge_half_angle_rad=self.local_cost_front_wedge_half_angle_rad,
            high_cost_threshold=self.local_cost_inflation_threshold,
            min_clearance_floor_m=self.centerline_target_refinement_min_clearance_floor_m,
            current_goal_sequence=self.active_goal_sequence_id,
            current_branch_context_id=str(self.active_start_node_id) if self.active_start_node_id is not None else None,
            current_timestamp_sec=self.get_clock().now().nanoseconds / 1e9,
            staging_frame_id='map',
        )

    def _now_wall_time_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds / 1e9)

    @staticmethod
    def _branch_option_to_payload(branch: object) -> Optional[dict[str, object]]:
        if not isinstance(branch, BranchOption):
            return None
        return {
            'angle_rad': float(branch.angle_rad),
            'target_xy': [float(branch.target_xy[0]), float(branch.target_xy[1])],
            'state': branch.state,
            'edge_id': branch.edge_id,
            'failures': int(branch.failures),
            'failure_reason': branch.failure_reason,
        }

    @staticmethod
    def _staging_plan_goal_pose(staging_plan: object) -> Optional[dict[str, object]]:
        if not isinstance(staging_plan, dict):
            return None
        pose = staging_plan.get('staging_goal_pose')
        return dict(pose) if isinstance(pose, dict) else None

    @staticmethod
    def _source_forward_search_payload(source_forward_refinement: object) -> dict[str, object]:
        if not isinstance(source_forward_refinement, dict):
            return {}
        nested = source_forward_refinement.get('multi_candidate_forward_search')
        if isinstance(nested, dict):
            return nested
        return source_forward_refinement

    def _serialize_pending_corridor_alignment_second_step(self, pending: object) -> dict[str, object]:
        if not isinstance(pending, dict):
            return {'exists': False, 'runtime_serialized': True}
        return {
            'exists': True,
            'runtime_serialized': True,
            'original_goal_kind': pending.get('original_goal_kind'),
            'original_target': pending.get('original_target'),
            'direction_rad': pending.get('direction_rad'),
            'start_node_id': pending.get('start_node_id'),
            'active_branch': self._branch_option_to_payload(pending.get('active_branch')),
            'staging_goal_pose': self._staging_plan_goal_pose(pending.get('staging_plan')),
            'staging_plan': pending.get('staging_plan'),
            'staging_result_status_label': pending.get('staging_result_status_label'),
            'staging_succeeded_wall_time_sec': pending.get('staging_succeeded_wall_time_sec'),
            'fresh_scan_received': bool(pending.get('fresh_scan_received', False)),
            'fresh_local_costmap_received': bool(pending.get('fresh_local_costmap_received', False)),
            'fresh_tf_received': bool(pending.get('fresh_tf_received', False)),
            'fresh_scan_sample_time_sec': pending.get('fresh_scan_sample_time_sec'),
            'fresh_local_costmap_sample_time_sec': pending.get('fresh_local_costmap_sample_time_sec'),
            'fresh_tf_sample_time_sec': pending.get('fresh_tf_sample_time_sec'),
        }

    @staticmethod
    def _second_step_lateral_residual_after(second_step: object) -> Optional[float]:
        if not isinstance(second_step, dict):
            return None
        source_forward = MazeExplorer._source_forward_search_payload(second_step.get('forward_refinement_result'))
        selected_metrics = source_forward.get('selected_metrics') if isinstance(source_forward, dict) else None
        if isinstance(selected_metrics, dict):
            value = selected_metrics.get('lateral_residual_after_m')
        else:
            value = source_forward.get('lateral_residual_after_m') if isinstance(source_forward, dict) else None
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        return out if math.isfinite(out) else None

    @staticmethod
    def _front_wedge_risk_payload(second_step: object) -> dict[str, object]:
        if not isinstance(second_step, dict):
            return {'available': False}
        source_forward = MazeExplorer._source_forward_search_payload(second_step.get('forward_refinement_result'))
        selected_metrics = source_forward.get('selected_metrics') if isinstance(source_forward, dict) else None
        metrics = selected_metrics if isinstance(selected_metrics, dict) else source_forward
        if not isinstance(metrics, dict):
            metrics = {}
        return {
            'available': bool(metrics),
            'max': metrics.get('front_wedge_cost_max'),
            'high_cost_count': metrics.get('front_wedge_high_cost_count'),
            'lethal_count': metrics.get('front_wedge_lethal_count'),
            'sample_count': metrics.get('front_wedge_sample_count'),
            'footprint_lethal_count': metrics.get('footprint_lethal_count'),
            'footprint_high_cost_count': metrics.get('footprint_high_cost_count'),
        }

    def _second_step_forward_goal_valid(self, second_step: object) -> bool:
        if not isinstance(second_step, dict):
            return False
        selected = second_step.get('selected_candidate_target')
        yaw = self._finite_float_or_none(second_step.get('selected_candidate_yaw'))
        if not (isinstance(selected, list) and len(selected) >= 2 and yaw is not None):
            return False
        x = self._finite_float_or_none(selected[0])
        y = self._finite_float_or_none(selected[1])
        return bool(x is not None and y is not None and second_step.get('generated_after_fresh_evidence') is True)

    def _serialize_second_step_forward_goal(
        self,
        second_step: object,
        pending: object,
        second_step_generation_wall_time_sec: float,
    ) -> dict[str, object]:
        if not isinstance(second_step, dict):
            return {'valid': False, 'map_frame_id': self.map_frame}
        source_forward = self._source_forward_search_payload(second_step.get('forward_refinement_result'))
        payload = dict(second_step)
        payload.update({
            'valid': self._second_step_forward_goal_valid(second_step),
            'map_frame_id': self.map_frame,
            'selected_candidate_target': second_step.get('selected_candidate_target'),
            'selected_candidate_yaw': second_step.get('selected_candidate_yaw'),
            'candidate_count': source_forward.get('candidate_count'),
            'hard_safety_pass_candidate_count': second_step.get('hard_safety_pass_candidate_count'),
            'selected_candidate_index': source_forward.get('selected_candidate_index'),
            'selection_priority_trace': source_forward.get('selection_priority_trace'),
            'rejected_candidate_summaries': source_forward.get('rejected_candidate_summaries'),
            'front_wedge_risk_after_staging': self._front_wedge_risk_payload(second_step),
            'lateral_residual_after': self._second_step_lateral_residual_after(second_step),
            'generated_after_fresh_evidence': bool(second_step.get('generated_after_fresh_evidence', False)),
            'fresh_scan_received': bool(second_step.get('fresh_scan_received', False)),
            'fresh_local_costmap_received': bool(second_step.get('fresh_local_costmap_received', False)),
            'fresh_tf_received': bool(second_step.get('fresh_tf_received', False)),
            'fresh_scan_sample_time_sec': pending.get('fresh_scan_sample_time_sec') if isinstance(pending, dict) else None,
            'fresh_local_costmap_sample_time_sec': pending.get('fresh_local_costmap_sample_time_sec') if isinstance(pending, dict) else None,
            'fresh_tf_sample_time_sec': pending.get('fresh_tf_sample_time_sec') if isinstance(pending, dict) else None,
            'generation_wall_time_sec': second_step_generation_wall_time_sec,
        })
        return payload

    def _dispatch_second_step_after_corridor_alignment_staging(self, robot_pose: RobotPose) -> bool:
        if not isinstance(self.pending_corridor_alignment_second_step, dict):
            return False
        pending = dict(self.pending_corridor_alignment_second_step)
        original = pending.get('original_target')
        if not (isinstance(original, list) and len(original) >= 2 and self.map_view is not None):
            self.pending_corridor_alignment_second_step = None
            return False
        second_step_generation_wall_time_sec = self._now_wall_time_sec()
        pending['fresh_tf_received'] = True
        pending['fresh_tf_sample_time_sec'] = second_step_generation_wall_time_sec
        direction_rad = float(pending.get('direction_rad', robot_pose[2]))
        second_step = generate_second_step_forward_goal_after_staging(
            map_grid=self.map_view,
            local_cost_grid=self.local_costmap_view,
            staged_pose=robot_pose,
            original_target=(float(original[0]), float(original[1])),
            direction_rad=direction_rad,
            clearance_radius_m=self.clearance_radius_m,
            fresh_scan_received=bool(pending.get('fresh_scan_received', False)),
            fresh_local_costmap_received=bool(pending.get('fresh_local_costmap_received', False)),
            fresh_tf_received=bool(pending.get('fresh_tf_received', False)),
            side_probe_m=self.centerline_target_refinement_side_probe_m,
            forward_offsets_m=self.centerline_target_refinement_forward_offsets_m,
            lateral_offsets_m=self.centerline_target_refinement_lateral_offsets_m,
            local_cost_radius_m=self.local_cost_target_radius_m,
            front_wedge_radius_m=self.local_cost_front_wedge_radius_m,
            front_wedge_half_angle_rad=self.local_cost_front_wedge_half_angle_rad,
            high_cost_threshold=self.local_cost_inflation_threshold,
            min_clearance_floor_m=self.centerline_target_refinement_min_clearance_floor_m,
            forward_progress_tolerance_m=self.centerline_target_refinement_forward_progress_tolerance_m,
        )
        if not bool(second_step.get('generated_after_fresh_evidence', False)):
            self.pending_corridor_alignment_second_step = pending
            return False
        selected = second_step.get('selected_candidate_target')
        if not (isinstance(selected, list) and len(selected) >= 2):
            self.pending_corridor_alignment_second_step = pending
            return False
        serialized_pending = self._serialize_pending_corridor_alignment_second_step(pending)
        serialized_second_step = self._serialize_second_step_forward_goal(
            second_step,
            pending,
            second_step_generation_wall_time_sec,
        )
        self.pending_corridor_alignment_second_step = None
        self.active_start_node_id = pending.get('start_node_id') if isinstance(pending.get('start_node_id'), int) else self.active_start_node_id
        branch = pending.get('active_branch')
        if isinstance(branch, BranchOption):
            self.active_branch = branch
        self.pending_branch_choice_diagnostics = {
            'selected_due_to_context': 'phase92_corridor_alignment_staging_second_step',
            'pending_corridor_alignment_second_step': serialized_pending,
            'second_step_forward_goal': serialized_second_step,
            'two_step_staging_plan': pending.get('staging_plan', {}).get('two_step_staging_plan') if isinstance(pending.get('staging_plan'), dict) else None,
            'staging_goal_pose': pending.get('staging_plan', {}).get('staging_goal_pose') if isinstance(pending.get('staging_plan'), dict) else None,
            'freshness_after_staging': {
                'fresh_scan_received': bool(pending.get('fresh_scan_received', False)),
                'fresh_local_costmap_received': bool(pending.get('fresh_local_costmap_received', False)),
                'fresh_tf_received': bool(pending.get('fresh_tf_received', False)),
                'fresh_scan_sample_time_sec': pending.get('fresh_scan_sample_time_sec'),
                'fresh_local_costmap_sample_time_sec': pending.get('fresh_local_costmap_sample_time_sec'),
                'fresh_tf_sample_time_sec': pending.get('fresh_tf_sample_time_sec'),
                'generation_wall_time_sec': second_step_generation_wall_time_sec,
            },
            'front_wedge_risk_after_staging': serialized_second_step.get('front_wedge_risk_after_staging'),
            'lateral_residual_after': serialized_second_step.get('lateral_residual_after'),
            'staging_applied': False,
            'prior_staging_applied': True,
            'staging_reject_reason': None,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        }
        yaw = second_step.get('selected_candidate_yaw')
        self._send_goal((float(selected[0]), float(selected[1])), float(yaw if isinstance(yaw, (int, float)) else direction_rad), 'explore', skip_two_step_staging=True)
        return True

    def _send_goal(self, target_xy: Point, yaw: float, goal_kind: str, skip_two_step_staging: bool = False) -> None:
        dispatch_pose = self._lookup_robot_pose()
        original_target_xy = target_xy
        requested_goal_kind = goal_kind
        centerline_refinement = self._maybe_refine_corridor_centerline_dispatch_target(
            dispatch_pose=dispatch_pose,
            original_target=target_xy,
            yaw=yaw,
            goal_kind=goal_kind,
        )
        two_step_staging = self._maybe_plan_corridor_alignment_staging(
            dispatch_pose=dispatch_pose,
            original_target=target_xy,
            yaw=yaw,
            goal_kind=goal_kind,
            centerline_refinement=centerline_refinement,
            skip_two_step_staging=skip_two_step_staging,
        )
        goal_yaw = yaw
        if bool(two_step_staging.get('staging_applied', False)) and isinstance(two_step_staging.get('staging_goal_pose'), dict):
            pose = two_step_staging['staging_goal_pose']
            target_xy = (float(pose['x']), float(pose['y']))
            goal_yaw = float(pose['yaw'])
            goal_kind = 'corridor_alignment_staging'
            self.pending_corridor_alignment_second_step = {
                'original_goal_kind': requested_goal_kind,
                'original_target': self._point_to_payload(original_target_xy),
                'direction_rad': float(yaw),
                'start_node_id': self.active_start_node_id,
                'active_branch': self.active_branch,
                'staging_plan': two_step_staging,
                'fresh_scan_received': False,
                'fresh_local_costmap_received': False,
                'fresh_tf_received': False,
                'staging_result_status_label': None,
                'staging_succeeded_wall_time_sec': None,
            }
        elif bool(centerline_refinement.get('applied', False)) and isinstance(centerline_refinement.get('refined_target'), list):
            refined_any = centerline_refinement['refined_target']
            if isinstance(refined_any, list) and len(refined_any) >= 2:
                target_xy = (float(refined_any[0]), float(refined_any[1]))
            corridor_yaw = centerline_refinement.get('corridor_heading_yaw')
            if isinstance(corridor_yaw, (int, float)) and not isinstance(corridor_yaw, bool):
                goal_yaw = float(corridor_yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(target_xy, goal_yaw)
        self.goal_sequence_id += 1
        goal_sequence_id = self.goal_sequence_id
        self.goal_active = True
        self.goal_sent_time = self.get_clock().now()
        self.active_goal_sequence_id = goal_sequence_id
        self.goal_settle_until = None
        self.active_goal_kind = goal_kind
        self.active_goal_target = target_xy
        directional_override_payload = self.last_dispatch_readiness_gate.get('local_costmap_directional_override')
        if not isinstance(directional_override_payload, dict):
            directional_override_payload = {}
        self.active_goal_event_context = {
            'goal_sequence': goal_sequence_id,
            'dispatch_pose': self._pose_to_payload(dispatch_pose),
            'target': self._point_to_payload(target_xy),
            'original_target': self._point_to_payload(original_target_xy),
            'refined_target': self._point_to_payload(target_xy),
            'target_exit_dist': self._distance_to_exit(target_xy),
            'robot_exit_dist_at_dispatch': self._distance_to_exit(dispatch_pose[:2]) if dispatch_pose is not None else None,
            'local_topology': self.last_local_topology_kind,
            'branch_angle': float(yaw),
            'goal_kind': goal_kind,
            'near_exit': self._target_near_exit(target_xy),
            'start_node_id': self.active_start_node_id,
            'current_node_id': self.current_node_id,
            'last_open_direction_count': self.last_open_direction_count,
            'last_candidate_count': self.last_candidate_count,
            'dispatch_readiness_gate': self.last_dispatch_readiness_gate,
            'local_costmap_full_window': self.last_dispatch_readiness_gate.get('local_costmap_full_window'),
            'local_costmap_directional_override': self.last_dispatch_readiness_gate.get('local_costmap_directional_override'),
            'directional_local_costmap_readiness_override_applied': bool(directional_override_payload.get('override_applied', False)),
            'skip_two_step_staging': bool(skip_two_step_staging),
            'phase138_recursion_guard': bool(skip_two_step_staging),
            'phase136_recursion_guard': bool(skip_two_step_staging),
            'recursion_guard': bool(skip_two_step_staging),
        }
        if skip_two_step_staging:
            self.active_goal_event_context.update({
                'two_step_stage_dispatch_requested': False,
                'staging_applied': False,
                'staging_reject_reason': 'skip_two_step_staging_recursion_guard',
            })
        if self.pending_branch_choice_diagnostics:
            self.active_goal_event_context.update(self.pending_branch_choice_diagnostics)
        else:
            self.active_goal_event_context.update(self._empty_branch_choice_diagnostics())
        self.pending_branch_choice_diagnostics = {}
        if self.last_post_ingress_single_open_exception_diagnostics:
            self.active_goal_event_context['post_ingress_single_open_exception'] = self.last_post_ingress_single_open_exception_diagnostics
            self.active_goal_event_context.update(self.last_post_ingress_single_open_exception_diagnostics)
        self.active_goal_event_context['effective_timeout_sec'] = self._effective_goal_timeout_sec()
        if self.last_topology_consistency_diagnostics:
            self.active_goal_event_context['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
            self.active_goal_event_context.update(self.last_topology_consistency_diagnostics)
        if centerline_refinement:
            self.active_goal_event_context['centerline_target_refinement'] = centerline_refinement
            self.active_goal_event_context.update({
                'centerline_refinement_applied': bool(centerline_refinement.get('applied', False)),
                'centerline_refinement_reason': centerline_refinement.get('reason'),
                'centerline_refinement_enabled': bool(centerline_refinement.get('enabled', False)),
                'centerline_refinement_candidate_count': centerline_refinement.get('candidate_count'),
                'centerline_refinement_eligible_candidate_count': centerline_refinement.get('eligible_candidate_count'),
                'centerline_refinement_strict_eligible_candidate_count': centerline_refinement.get('strict_eligible_candidate_count'),
                'centerline_refinement_balance_first_eligible_candidate_count': centerline_refinement.get('balance_first_eligible_candidate_count'),
                'centerline_refinement_gate_mode': centerline_refinement.get('gate_mode'),
                'centerline_refinement_gate_conditions': centerline_refinement.get('gate_conditions'),
                'multi_candidate_forward_search': centerline_refinement.get('multi_candidate_forward_search'),
                'candidate_family': centerline_refinement.get('candidate_family'),
                'candidate_count': centerline_refinement.get('candidate_count'),
                'hard_safety_pass_candidate_count': centerline_refinement.get('hard_safety_pass_candidate_count'),
                'selected_candidate_index': centerline_refinement.get('selected_candidate_index'),
                'selected_candidate_target': centerline_refinement.get('selected_candidate_target'),
                'selected_candidate_yaw': centerline_refinement.get('selected_candidate_yaw'),
                'selection_priority_trace': centerline_refinement.get('selection_priority_trace'),
                'rejected_candidate_summaries': centerline_refinement.get('rejected_candidate_summaries'),
                'original_target_preserved_on_reject': bool(centerline_refinement.get('original_target_preserved_on_reject', False)),
                'centerline_projected_target': centerline_refinement.get('centerline_projected_target'),
                'corridor_heading_yaw': centerline_refinement.get('corridor_heading_yaw'),
                'refinement_applied': bool(centerline_refinement.get('refinement_applied', centerline_refinement.get('applied', False))),
                'refinement_reject_reason': centerline_refinement.get('refinement_reject_reason'),
                'forward_executability_check': centerline_refinement.get('forward_executability_check'),
                'branch_scoring_changed': bool(centerline_refinement.get('branch_scoring_changed', False)),
                'fallback_terminal_acceptance_used': bool(centerline_refinement.get('fallback_terminal_acceptance_used', False)),
            })
        if two_step_staging:
            self.active_goal_event_context['two_step_staging_plan'] = two_step_staging.get('two_step_staging_plan')
            branch_choice_diagnostics = dict(self.active_goal_event_context)
            self.active_goal_event_context.update({
                'two_step_stage_dispatch_requested': bool(two_step_staging.get('two_step_stage_dispatch_requested', False)),
                'staging_goal_pose': two_step_staging.get('staging_goal_pose'),
                'staging_reason': two_step_staging.get('staging_reason'),
                'staging_executability_check': two_step_staging.get('staging_executability_check'),
                'second_step_forward_goal': (
                    two_step_staging.get('second_step_forward_goal')
                    if two_step_staging.get('second_step_forward_goal') is not None
                    else self.active_goal_event_context.get('second_step_forward_goal')
                ),
                'staging_applied': bool(two_step_staging.get('staging_applied', False)),
                'staging_reject_reason': two_step_staging.get('staging_reject_reason'),
                'staging_candidates': two_step_staging.get('staging_candidates'),
                'corridor_evidence_carry_over': two_step_staging.get('corridor_evidence_carry_over'),
                'carry_over_source': two_step_staging.get('carry_over_source'),
                'carry_over_applied': bool(two_step_staging.get('carry_over_applied', False)),
                'carry_over_reject_reason': two_step_staging.get('carry_over_reject_reason'),
                'source_forward_window': two_step_staging.get('source_forward_window'),
                'staging_window': two_step_staging.get('staging_window'),
                'safety_evidence_recomputed': bool(two_step_staging.get('safety_evidence_recomputed', False)),
                'staging_gate_artifact_completeness': self._phase143_staging_gate_artifact_context(
                    two_step_staging=two_step_staging,
                    branch_choice_diagnostics=branch_choice_diagnostics,
                    branch_angle=yaw,
                ),
                'visual_diagnosis_wait_requested': bool(two_step_staging.get('visual_diagnosis_wait_requested', False)),
                'branch_scoring_changed': bool(two_step_staging.get('branch_scoring_changed', False)),
                'fallback_terminal_acceptance_used': bool(two_step_staging.get('fallback_terminal_acceptance_used', False)),
            })
        self.active_goal_event_context.update(self._compute_goal_geometry_diagnostics(dispatch_pose, target_xy, yaw))
        self.active_goal_event_context.update(self._compute_local_cost_diagnostics(dispatch_pose, target_xy))
        self.active_goal_event_context.update(self._compute_phase62_first_dispatch_traversability_diagnostics(dispatch_pose, target_xy, yaw))
        self.goal_event_context_by_sequence[goal_sequence_id] = dict(self.active_goal_event_context)
        self.goal_count += 1
        self.mode = NAVIGATING if goal_kind != 'backtrack' else BACKTRACKING
        self._publish_goal_event('dispatch')
        self.get_logger().info(
            'maze explorer sending %s goal #%d seq=%d: x=%.3f y=%.3f yaw=%.3f'
            % (goal_kind, self.goal_count, goal_sequence_id, target_xy[0], target_xy[1], yaw)
        )
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=lambda feedback_msg, sequence_id=goal_sequence_id: self._goal_feedback_callback(feedback_msg, sequence_id))
        future.add_done_callback(lambda future, sequence_id=goal_sequence_id: self._goal_response_callback(future, sequence_id))

    def _goal_feedback_callback(self, feedback_msg, sequence_id: int) -> None:  # noqa: ANN001 - rclpy action feedback type varies.
        feedback = getattr(feedback_msg, 'feedback', None)
        if feedback is None:
            return
        row = {
            'navigation_time_sec': self._duration_msg_to_sec(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_sec': self._duration_msg_to_sec(getattr(feedback, 'estimated_time_remaining', None)),
            'distance_remaining': self._finite_float_or_none(getattr(feedback, 'distance_remaining', None)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        }
        rows = self.phase62_nav2_feedback_by_sequence.setdefault(sequence_id, [])
        rows.append(row)
        if len(rows) > 200:
            del rows[:-200]
        if self.active_goal_sequence_id == sequence_id and self.active_goal_event_context:
            self.active_goal_event_context['phase62_nav2_feedback_summary'] = self._phase62_nav2_feedback_summary(sequence_id)
            self.active_goal_event_context['phase62_robot_to_target_progress'] = self._phase62_robot_to_target_progress()
            self.goal_event_context_by_sequence[sequence_id] = dict(self.active_goal_event_context)

    def _goal_response_callback(self, future, sequence_id: int) -> None:  # noqa: ANN001 - rclpy future type varies.
        if self._is_stale_goal_result(sequence_id):
            self._mark_goal_preempted(sequence_id, GoalStatus.STATUS_UNKNOWN)
            return
        self.goal_handle = future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().warn('maze explorer goal rejected by Nav2')
            self._handle_goal_failure(reason=GOAL_REJECTED, result_status=GoalStatus.STATUS_UNKNOWN)
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(lambda future, sequence_id=sequence_id: self._goal_result_callback(future, sequence_id))

    def _goal_result_callback(self, future, sequence_id: int) -> None:  # noqa: ANN001 - rclpy future type varies.
        result = future.result()
        status = getattr(result, 'status', GoalStatus.STATUS_UNKNOWN)
        self.last_nav2_status = int(status)
        if self.active_goal_sequence_id == sequence_id and self.active_goal_event_context:
            nav_result = getattr(result, 'result', None)
            self.active_goal_event_context['phase62_nav2_result_summary'] = {
                'status': int(status),
                'error_code': int(getattr(nav_result, 'error_code', -1)) if nav_result is not None else None,
                'error_msg': str(getattr(nav_result, 'error_msg', '')) if nav_result is not None else None,
            }
            self.active_goal_event_context['phase62_nav2_feedback_summary'] = self._phase62_nav2_feedback_summary(sequence_id)
            self.active_goal_event_context['phase62_robot_to_target_progress'] = self._phase62_robot_to_target_progress()
            self.goal_event_context_by_sequence[sequence_id] = dict(self.active_goal_event_context)
        if self._is_timeout_canceled_goal_result(sequence_id):
            self._mark_goal_canceled_after_timeout(sequence_id, status)
            return
        if self._is_terminal_canceled_goal_result(sequence_id):
            self._mark_goal_canceled_after_exit(sequence_id, status)
            return
        if self._is_stale_goal_result(sequence_id):
            self._mark_goal_preempted(sequence_id, status)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_goal_success(result_status=status)
        else:
            self.get_logger().warn('maze explorer goal failed with status=%s seq=%d' % (status, sequence_id))
            self._handle_goal_failure(reason=BLOCKED_NAV2, result_status=status)

    def _handle_goal_success(self, result_status: int = GoalStatus.STATUS_SUCCEEDED) -> None:
        self.last_completed_goal_sequence_id = self.active_goal_sequence_id
        completed_goal_kind = self.active_goal_kind
        self._publish_goal_event('success', result_status=int(result_status), result_reason='succeeded')
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.goal_success_count += 1
        if completed_goal_kind == 'entry_direct':
            self.get_logger().info(
                'ENTRY_DIRECT goal succeeded; transitioning to AT_NODE_ANALYZE for normal DFS exploration'
            )
        if completed_goal_kind == 'corridor_alignment_staging':
            if isinstance(self.pending_corridor_alignment_second_step, dict):
                self.pending_corridor_alignment_second_step['staging_result_status_label'] = 'SUCCEEDED'
                self.pending_corridor_alignment_second_step['staging_succeeded_wall_time_sec'] = self._now_wall_time_sec()
            self.get_logger().info('corridor-alignment staging succeeded; waiting for fresh scan/local_costmap/TF before second-step forward goal')
        elif self.active_goal_kind == 'explore' and self.active_start_node_id is not None and self.active_branch is not None:
            self.post_ingress_single_open_exception_consumed = True
            self.topology.mark_branch_state(self.active_start_node_id, self.active_branch, EXPLORED)
            end = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='corridor') if self.active_goal_target else None
            if end is not None:
                edge = self.topology.connect_nodes(self.active_start_node_id, end.node_id, state=EXPLORED)
                self.active_edge_id = edge.edge_id
                self.current_node_id = end.node_id
                self.topology.visit_node(end.node_id)
        elif self.active_goal_kind == 'backtrack' and self.active_goal_target is not None:
            node = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='junction')
            self.current_node_id = node.node_id
            self.topology.visit_node(node.node_id)
        elif self.active_goal_kind == 'near_exit_micro_goal':
            self.get_logger().info('near-exit micro-goal succeeded; returning to exit check / normal exploration')
        self.active_goal_kind = 'none'
        self.active_goal_target = None
        self.active_goal_sequence_id = None
        self.active_start_node_id = None
        self.active_branch = None
        self.active_goal_event_context = {}
        self._start_goal_settle_cooldown()
        self.mode = SETTLING

    def _handle_goal_failure(self, reason: str = BLOCKED_NAV2, result_status: Optional[int] = None) -> None:
        self.last_completed_goal_sequence_id = self.active_goal_sequence_id
        if reason == GOAL_TIMEOUT:
            self._cancel_goal_after_timeout()
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.goal_failure_count += 1
        self.last_failure_reason = reason
        branch_failure_state = None
        caused_branch_failure = False
        caused_blacklist = False
        if self.active_goal_kind == 'entry_direct':
            self.get_logger().warn(
                'ENTRY_DIRECT goal failed (reason=%s); falling back to AT_NODE_ANALYZE' % reason
            )
        if self.active_goal_kind == 'backtrack':
            self.backtrack_failure_count += 1
            if self.active_goal_target is not None:
                node = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='junction')
                self.topology.record_backtrack_failure(node.node_id)
            reason = BACKTRACK_FAILED
        elif self.active_goal_kind == 'near_exit_micro_goal':
            self.get_logger().warn('near-exit micro-goal failed; preserving DFS topology state')
        elif self.active_goal_kind == 'explore' and self.active_start_node_id is not None and self.active_branch is not None:
            self.post_ingress_single_open_exception_consumed = True
            self.nav2_failure_count += 1
            state = self.topology.record_branch_failure(self.active_start_node_id, self.active_branch, reason=reason)
            branch_failure_state = state
            caused_branch_failure = state in (BLOCKED, BLACKLISTED)
            caused_blacklist = state in (BLOCKED, BLACKLISTED)
            if state == BLOCKED:
                self.blocked_branch_count = self.topology.blocked_branch_count()
            if state == BLACKLISTED:
                self.blacklisted_goal_count = len(self.topology.blacklist)
        self.blacklisted_goal_count = len(self.topology.blacklist)
        if reason == GOAL_TIMEOUT:
            self._update_active_timeout_local_cost_diagnostics()
            self.get_logger().info(
                'phase23 timeout local cost seq=%s footprint_max=%s front_max=%s path_0_5m_max=%s path_1_0m_max=%s'
                % (
                    self.active_goal_sequence_id,
                    self.active_goal_event_context.get('timeout_footprint_cost_max'),
                    self.active_goal_event_context.get('timeout_front_wedge_cost_max'),
                    self.active_goal_event_context.get('timeout_path_ahead_0_5m_cost_max'),
                    self.active_goal_event_context.get('timeout_path_ahead_1_0m_cost_max'),
                )
            )
            self._publish_goal_event('timeout',
                result_reason=reason,
                branch_failure_state=branch_failure_state,
                caused_branch_failure=caused_branch_failure,
                caused_blacklist=caused_blacklist,
            )
        else:
            self._publish_goal_event(
                'failure',
                result_status=int(result_status) if result_status is not None else None,
                result_reason=reason,
                branch_failure_state=branch_failure_state,
                caused_branch_failure=caused_branch_failure,
                caused_blacklist=caused_blacklist,
            )
        self.active_goal_kind = 'none'
        self.active_goal_target = None
        self.active_goal_sequence_id = None
        self.active_start_node_id = None
        self.active_branch = None
        self.active_goal_event_context = {}
        self._start_goal_settle_cooldown()
        self.mode = SETTLING

    def _is_stale_goal_result(self, sequence_id: int) -> bool:
        return self.active_goal_sequence_id is not None and sequence_id != self.active_goal_sequence_id

    def _is_timeout_canceled_goal_result(self, sequence_id: int) -> bool:
        return self.timeout_cancel_goal_sequence_id is not None and sequence_id == self.timeout_cancel_goal_sequence_id

    def _is_terminal_canceled_goal_result(self, sequence_id: int) -> bool:
        return (
            self.mode in (EXIT_REACHED, FAILED_EXHAUSTED)
            and self.terminal_cancel_goal_sequence_id is not None
            and sequence_id == self.terminal_cancel_goal_sequence_id
        )

    def _enter_terminal_state(self, terminal_mode: str, terminal_reason: str, robot_pose: Optional[RobotPose] = None) -> None:
        self.mode = terminal_mode
        self.terminal_state_entered = True
        self.last_terminal_reason = terminal_reason
        self._cancel_active_goal_for_terminal_state(terminal_reason)
        if terminal_mode == EXIT_REACHED and not self.exit_reached_logged:
            self.exit_reached_logged = True
            pose = robot_pose or self._lookup_robot_pose() or (float('nan'), float('nan'), 0.0)
            self.get_logger().info(
                'MAZE_EXPLORER_EXIT_REACHED pose=(%.3f, %.3f) exit=(%.3f, %.3f) radius=%.3f'
                % (pose[0], pose[1], self.exit_x, self.exit_y, self.exit_radius)
            )
        elif terminal_mode == FAILED_EXHAUSTED:
            self.exhausted_logged = True

    def _cancel_active_goal_for_terminal_state(self, terminal_reason: str) -> None:
        if self.goal_handle is None:
            return
        self.terminal_cancel_count += 1
        self.terminal_cancel_goal_sequence_id = self.active_goal_sequence_id
        self.last_terminal_reason = terminal_reason
        self._publish_goal_event('terminal_cancel', result_reason=terminal_reason)
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 - ROS action implementations vary.
            self.get_logger().warn('failed to cancel active terminal goal: %s' % exc)
        self.goal_active = False
        self.goal_sent_time = None
        self.active_goal_kind = 'terminal_cancel'
        self.active_goal_target = None
        self.active_goal_sequence_id = None

    def _cancel_goal_after_timeout(self) -> None:
        if self.goal_handle is None:
            return
        self.timeout_cancel_count += 1
        self.timeout_cancel_goal_sequence_id = self.active_goal_sequence_id
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 - ROS action implementations vary.
            self.get_logger().warn('failed to cancel timed-out goal: %s' % exc)

    def _mark_goal_preempted(self, sequence_id: int, status: int) -> None:
        self.stale_result_count += 1
        self.preempted_goal_count += 1
        self.last_failure_reason = GOAL_PREEMPTED
        self.last_nav2_status = int(status)
        self._publish_goal_event('stale_result', goal_sequence=sequence_id, result_status=int(status), result_reason=GOAL_PREEMPTED)
        self.get_logger().warn(
            'maze explorer ignoring stale/preempted goal result seq=%d active_seq=%s status=%s'
            % (sequence_id, self.active_goal_sequence_id, status)
        )

    def _mark_goal_canceled_after_exit(self, sequence_id: int, status: int) -> None:
        self.canceled_after_exit_count += 1
        self.last_failure_reason = GOAL_CANCELED_AFTER_EXIT
        self.last_nav2_status = int(status)
        self._publish_goal_event('terminal_cancel_result', goal_sequence=sequence_id, result_status=int(status), result_reason=GOAL_CANCELED_AFTER_EXIT)
        self.active_goal_event_context = {}
        self.get_logger().info(
            'maze explorer ignoring terminal-canceled goal result seq=%d status=%s reason=%s'
            % (sequence_id, status, self.last_terminal_reason)
        )

    def _mark_goal_canceled_after_timeout(self, sequence_id: int, status: int) -> None:
        self.canceled_after_timeout_count += 1
        self.last_failure_reason = GOAL_CANCELED_AFTER_TIMEOUT
        self.last_nav2_status = int(status)
        self._publish_goal_event('timeout_cancel_result', goal_sequence=sequence_id, result_status=int(status), result_reason=GOAL_CANCELED_AFTER_TIMEOUT)
        self.get_logger().info(
            'maze explorer ignoring timeout-canceled goal result seq=%d status=%s'
            % (sequence_id, status)
        )

    def _start_goal_settle_cooldown(self) -> None:
        self.last_goal_completion_time = self.get_clock().now()
        if self.goal_settle_sec <= 0.0:
            self.goal_settle_until = None
            return
        self.goal_settle_until = self.last_goal_completion_time + Duration(seconds=self.goal_settle_sec)

    def _goal_settle_active(self) -> bool:
        if self.goal_settle_until is None:
            return False
        if self.get_clock().now() < self.goal_settle_until:
            return True
        self.goal_settle_until = None
        if self.mode == 'SETTLING':
            self.mode = AT_NODE_ANALYZE
        return False

    def _goal_settle_remaining_sec(self) -> float:
        if self.goal_settle_until is None:
            return 0.0
        remaining = (self.goal_settle_until - self.get_clock().now()).nanoseconds / 1e9
        return max(0.0, float(remaining))

    def _goal_timed_out(self) -> bool:
        if self.goal_sent_time is None:
            return False
        elapsed = self.get_clock().now() - self.goal_sent_time
        return elapsed.nanoseconds / 1e9 > self._effective_goal_timeout_sec()

    def _active_goal_near_exit(self) -> bool:
        if self.active_goal_target is None:
            return False
        return self._target_near_exit(self.active_goal_target)

    def _target_near_exit(self, target_xy: Point) -> bool:
        return self._distance_to_exit(target_xy) <= self.near_exit_timeout_extension_radius_m

    def _effective_goal_timeout_sec(self) -> float:
        if self._active_goal_near_exit():
            return max(self.goal_timeout_sec, self.near_exit_goal_timeout_sec)
        return self.goal_timeout_sec

    def _lookup_robot_pose(self) -> Optional[RobotPose]:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - ROS TF exceptions vary.
            self.get_logger().debug('waiting for TF %s -> %s: %s' % (self.map_frame, self.base_frame, exc))
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return (float(translation.x), float(translation.y), yaw)

    def _exit_reached(self, robot_pose: RobotPose) -> bool:
        return math.hypot(robot_pose[0] - self.exit_x, robot_pose[1] - self.exit_y) <= self.exit_radius

    def _near_exit_fallback_decision(
        self,
        robot_pose: RobotPose,
        *,
        robot_to_path_distance: Optional[float] = None,
        cmd_near_zero_duration: Optional[float] = None,
    ) -> dict[str, object]:
        robot_exit_dist = self._distance_to_exit(robot_pose[:2])
        base = {
            'near_exit_fallback_triggered': False,
            'fallback_reason': None,
            'robot_exit_dist': float(robot_exit_dist),
            'cmd_near_zero_duration': cmd_near_zero_duration,
            'last_nav2_result': self.last_failure_reason,
            'robot_to_path_distance': robot_to_path_distance,
            'action': 'no_action',
            'near_exit_fallback_enabled': bool(self.near_exit_fallback_enabled),
            'near_exit_fallback_attempts': int(self.near_exit_fallback_attempts),
            'near_exit_fallback_max_attempts': int(self.near_exit_fallback_max_attempts),
        }
        if not self.near_exit_fallback_enabled:
            base['fallback_reason'] = 'disabled'
            return base
        if robot_exit_dist > self.near_exit_fallback_trigger_radius_m:
            base['fallback_reason'] = 'outside_trigger_radius'
            return base
        if self.near_exit_fallback_require_clean_topology:
            if self.topology.blocked_branch_count() != 0:
                base['fallback_reason'] = 'blocked_branch_count_nonzero'
                return base
            if len(self.topology.blacklist) != 0:
                base['fallback_reason'] = 'blacklisted_goal_count_nonzero'
                return base
        if (
            self.near_exit_fallback_require_path_alignment
            and robot_to_path_distance is not None
            and robot_to_path_distance > self.near_exit_fallback_robot_to_path_max_m
        ):
            base['fallback_reason'] = 'robot_to_path_distance_too_large'
            return base
        recent_problem_reasons = {GOAL_TIMEOUT, BLOCKED_NAV2, GOAL_CANCELED_AFTER_TIMEOUT, GOAL_REJECTED}
        has_recent_failure = self.last_failure_reason in recent_problem_reasons
        has_cmd_stall = (
            cmd_near_zero_duration is not None
            and cmd_near_zero_duration >= self.near_exit_fallback_cmd_near_zero_min_sec
        )
        if not (has_recent_failure or has_cmd_stall):
            base['fallback_reason'] = 'no_recent_timeout_abort_or_cmd_stall'
            return base
        if robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m:
            base.update({
                'near_exit_fallback_triggered': True,
                'fallback_reason': 'terminal_acceptance_radius',
                'action': 'terminal_acceptance',
            })
            return base
        if self.near_exit_fallback_attempts >= self.near_exit_fallback_max_attempts:
            base['fallback_reason'] = 'max_attempts_reached'
            return base
        base.update({
            'near_exit_fallback_triggered': True,
            'fallback_reason': 'micro_goal_candidate',
            'action': 'micro_goal',
        })
        return base

    def _maybe_apply_near_exit_fallback(self, robot_pose: RobotPose) -> bool:
        decision = self._near_exit_fallback_decision(robot_pose)
        action = decision.get('action')
        if action == 'terminal_acceptance':
            self._publish_near_exit_fallback_event(decision)
            self._enter_terminal_state(EXIT_REACHED, terminal_reason='near_exit_terminal_acceptance', robot_pose=robot_pose)
            return True
        if action == 'micro_goal':
            micro_goal = self._compute_near_exit_micro_goal(robot_pose)
            if micro_goal is None:
                decision['near_exit_fallback_triggered'] = False
                decision['fallback_reason'] = 'micro_goal_unavailable'
                decision['action'] = 'no_action'
                self._publish_near_exit_fallback_event(decision)
                return False
            feasible, reason, diagnostics = self._near_exit_micro_goal_feasible(robot_pose, micro_goal)
            decision.update(diagnostics)
            if not feasible:
                decision['near_exit_fallback_triggered'] = False
                decision['fallback_reason'] = reason
                decision['action'] = 'no_action'
                self._publish_near_exit_fallback_event(decision)
                return False
            decision['target'] = self._point_to_payload(micro_goal)
            self._publish_near_exit_fallback_event(decision)
            self.near_exit_fallback_attempts += 1
            self.pending_branch_choice_diagnostics = self._empty_branch_choice_diagnostics(selected_due_to_context='near_exit_fallback_micro_goal')
            self._send_goal(micro_goal, yaw=self._yaw_to_point(robot_pose[:2], micro_goal), goal_kind='near_exit_micro_goal')
            return True
        robot_exit_dist = decision.get('robot_exit_dist')
        if (
            decision.get('fallback_reason') != 'disabled'
            and isinstance(robot_exit_dist, (int, float))
            and float(robot_exit_dist) <= self.near_exit_fallback_trigger_radius_m
        ):
            self._publish_near_exit_fallback_event(decision)
        return False

    def _compute_near_exit_micro_goal(self, robot_pose: RobotPose) -> Optional[Point]:
        distance_to_exit = math.hypot(self.exit_x - robot_pose[0], self.exit_y - robot_pose[1])
        if distance_to_exit <= 1e-6:
            return None
        step = min(self.near_exit_micro_goal_max_step_m, max(self.near_exit_micro_goal_min_step_m, distance_to_exit - self.near_exit_terminal_acceptance_radius_m))
        step = max(0.0, min(step, distance_to_exit))
        if step <= 1e-6:
            return None
        unit_x = (self.exit_x - robot_pose[0]) / distance_to_exit
        unit_y = (self.exit_y - robot_pose[1]) / distance_to_exit
        return (float(robot_pose[0] + unit_x * step), float(robot_pose[1] + unit_y * step))

    def _near_exit_micro_goal_feasible(self, robot_pose: RobotPose, target_xy: Point) -> tuple[bool, str, dict[str, object]]:
        geometry = self._compute_goal_geometry_diagnostics(robot_pose, target_xy, self._yaw_to_point(robot_pose[:2], target_xy))
        local_cost = self._compute_local_cost_diagnostics(robot_pose, target_xy)
        diagnostics: dict[str, object] = {
            'micro_goal_geometry': geometry,
            'micro_goal_local_cost': local_cost,
        }
        target_occupancy = geometry.get('target_cell_occupancy')
        if isinstance(target_occupancy, (int, float)) and float(target_occupancy) >= 65.0:
            return False, 'micro_goal_target_occupied', diagnostics
        if geometry.get('line_of_sight_occupied_count') not in (None, 0):
            return False, 'micro_goal_line_of_sight_occupied', diagnostics
        target_clearance = geometry.get('target_clearance_m')
        if target_clearance is not None and float(target_clearance) < self.clearance_radius_m:
            return False, 'micro_goal_clearance_too_low', diagnostics
        target_cost = local_cost.get('dispatch_target_local_cost')
        if target_cost is not None and int(target_cost) >= self.local_cost_obstacle_threshold:
            return False, 'micro_goal_local_cost_lethal', diagnostics
        path_cost = local_cost.get('dispatch_path_local_cost_max')
        if path_cost is not None and int(path_cost) >= self.local_cost_obstacle_threshold:
            return False, 'micro_goal_path_local_cost_lethal', diagnostics
        return True, 'micro_goal_feasible', diagnostics

    def _publish_near_exit_fallback_event(self, decision: dict[str, object]) -> None:
        context = dict(decision)
        context.update({
            'goal_sequence': self.active_goal_sequence_id,
            'goal_kind': self.active_goal_kind,
            'near_exit': bool(context.get('robot_exit_dist') is not None and float(context['robot_exit_dist']) <= self.near_exit_fallback_trigger_radius_m),
        })
        previous_context = self.active_goal_event_context
        self.active_goal_event_context = context
        self._publish_goal_event('near_exit_fallback')
        self.active_goal_event_context = previous_context

    def _distance_to_exit(self, xy: Point) -> float:
        return math.hypot(float(xy[0]) - self.exit_x, float(xy[1]) - self.exit_y)

    def _compute_goal_geometry_diagnostics(self, dispatch_pose: Optional[RobotPose], target_xy: Point, yaw: float) -> dict[str, object]:
        diagnostics: dict[str, object] = {
            'target_cell_occupancy': None,
            'target_clearance_m': None,
            'line_of_sight_occupied_count': None,
            'line_of_sight_unknown_count': None,
            'line_of_sight_min_clearance_m': None,
            'path_corridor_min_clearance_m': None,
            'target_near_wall': None,
            'target_crosses_wall_corner': None,
            'target_crosses_narrow_passage': None,
            'target_yaw_corridor_conflict': None,
        }
        if self.map_view is None:
            return diagnostics
        grid = self.map_view
        target_cell = grid.world_to_cell(target_xy[0], target_xy[1])
        target_value = grid.cell_value(target_cell)
        target_clearance = grid.nearest_obstacle_distance(
            target_xy[0], target_xy[1], max_radius_m=max(self.clearance_radius_m * 3.0, 1.2)
        )
        diagnostics['target_cell_occupancy'] = int(target_value)
        diagnostics['target_clearance_m'] = float(target_clearance)
        diagnostics['target_near_wall'] = bool(target_clearance < self.clearance_radius_m)
        diagnostics['target_crosses_wall_corner'] = False
        diagnostics['target_crosses_narrow_passage'] = bool(target_clearance < max(self.clearance_radius_m * 1.5, 0.50))
        diagnostics['target_yaw_corridor_conflict'] = False
        if dispatch_pose is None:
            diagnostics['line_of_sight_occupied_count'] = 0
            diagnostics['line_of_sight_unknown_count'] = 0
            diagnostics['line_of_sight_min_clearance_m'] = float(target_clearance)
            diagnostics['path_corridor_min_clearance_m'] = float(target_clearance)
            return diagnostics
        start_xy = (dispatch_pose[0], dispatch_pose[1])
        values = grid.line_sample_values(start_xy, target_xy)
        occupied_count = sum(1 for value in values if value >= grid.occupied_threshold)
        unknown_count = sum(1 for value in values if value < 0)
        line_min_clearance = grid.line_min_clearance(
            start_xy, target_xy, max_radius_m=max(self.clearance_radius_m * 3.0, 1.2)
        )
        diagnostics['line_of_sight_occupied_count'] = int(occupied_count)
        diagnostics['line_of_sight_unknown_count'] = int(unknown_count)
        diagnostics['line_of_sight_min_clearance_m'] = float(line_min_clearance)
        path_corridor_min_clearance = float(min(target_clearance, line_min_clearance))
        diagnostics['path_corridor_min_clearance_m'] = path_corridor_min_clearance
        diagnostics['target_crosses_wall_corner'] = bool(occupied_count > 0 or unknown_count > 0)
        diagnostics['target_crosses_narrow_passage'] = bool(path_corridor_min_clearance < max(self.clearance_radius_m * 1.5, 0.50))
        if self.last_local_topology_kind == CORRIDOR:
            travel_yaw = self._yaw_to_point(start_xy, target_xy)
            diagnostics['target_yaw_corridor_conflict'] = bool(abs(normalize_angle(yaw - travel_yaw)) > math.radians(75.0))
        return diagnostics

    @staticmethod
    def _point_to_payload(xy: Optional[Point]) -> Optional[list[float]]:
        if xy is None:
            return None
        return [float(xy[0]), float(xy[1])]

    @staticmethod
    def _pose_to_payload(pose: Optional[RobotPose]) -> Optional[list[float]]:
        if pose is None:
            return None
        return [float(pose[0]), float(pose[1]), float(pose[2])]

    def _compute_phase62_first_dispatch_traversability_diagnostics(self, dispatch_pose: Optional[RobotPose], target_xy: Point, yaw: float) -> dict[str, object]:
        """Phase62 read-only first-dispatch local traversability diagnostics.

        These fields describe target/local-cost geometry at dispatch time only.
        They do not modify branch scoring, Nav2 parameters, clearance radius, or
        map sufficiency gates.
        """
        target_cell_state = self._local_costmap_cell_state_for_point(target_xy)
        patch = self._local_costmap_patch_for_point(target_xy, radius_cells=3)
        footprint = self._local_cost_target_footprint_evidence(target_xy, yaw)
        robot_to_target_progress = self._phase62_robot_to_target_progress(dispatch_pose=dispatch_pose, target_xy=target_xy)
        nav2_feedback = self._phase62_nav2_feedback_summary(self.active_goal_sequence_id)
        diagnostics = {
            'phase62_first_dispatch_traversability': True,
            'phase62_target_cell_state': target_cell_state,
            'phase62_local_costmap_patch': patch,
            'phase62_target_footprint_cost': footprint,
            'phase62_front_wedge_clearance_m': None,
            'phase62_robot_to_target_progress': robot_to_target_progress,
            'phase62_cmd_vel_summary': {
                'source': 'external_runtime_recorder_required',
                'available_in_maze_explorer': False,
                'reason': 'maze_explorer does not subscribe to cmd_vel; Phase62 wrapper records cmd_vel externally',
            },
            'phase62_nav2_feedback_summary': nav2_feedback,
            'phase62_nav2_result_summary': None,
            'phase62_timestamp_consistency': self._phase62_timestamp_consistency(),
            'phase62_local_cost_thresholds': {
                'inflation_threshold': int(self.local_cost_inflation_threshold),
                'obstacle_threshold': int(self.local_cost_obstacle_threshold),
                'target_radius_m': float(self.local_cost_target_radius_m),
                'dispatch_target_local_cost_max_radius_source': 'Phase19 dispatch_target_local_cost_max_radius retained in goal event payload',
            },
        }
        if dispatch_pose is not None and self.local_costmap_view is not None:
            wedge_values, wedge_clearance = self._local_cost_values_in_wedge(
                self.local_costmap_view,
                (dispatch_pose[0], dispatch_pose[1]),
                dispatch_pose[2],
                self.local_cost_front_wedge_radius_m,
                self.local_cost_front_wedge_half_angle_rad,
            )
            diagnostics['phase62_front_wedge_clearance_m'] = wedge_clearance
            diagnostics['phase62_front_wedge_cost'] = {
                'sample_count': len(wedge_values),
                'max': max(wedge_values) if wedge_values else None,
                'mean': float(sum(wedge_values) / len(wedge_values)) if wedge_values else None,
                'high_cost_count': int(sum(1 for value in wedge_values if value >= self.local_cost_inflation_threshold)),
                'lethal_count': int(sum(1 for value in wedge_values if value >= self.local_cost_obstacle_threshold)),
            }
        return diagnostics

    def _local_costmap_patch_for_point(self, xy: Optional[Point], radius_cells: int = 3) -> dict[str, object]:
        grid = self.local_costmap_view
        if xy is None or grid is None:
            return {'available': False, 'radius_cells': int(radius_cells), 'center_cell': None, 'rows': [], 'summary': None}
        center_cell = grid.world_to_cell(float(xy[0]), float(xy[1]))
        values: list[int] = []
        rows: list[dict[str, object]] = []
        for cell_y in range(center_cell[1] - radius_cells, center_cell[1] + radius_cells + 1):
            row_values: list[Optional[int]] = []
            for cell_x in range(center_cell[0] - radius_cells, center_cell[0] + radius_cells + 1):
                cell = (cell_x, cell_y)
                if grid.in_bounds(cell):
                    value = int(grid.cell_value(cell))
                    values.append(value)
                    row_values.append(value)
                else:
                    row_values.append(None)
            rows.append({'cell_y': int(cell_y), 'values': row_values})
        return {
            'available': True,
            'radius_cells': int(radius_cells),
            'center_cell': [int(center_cell[0]), int(center_cell[1])],
            'rows': rows,
            'summary': {
                'sample_count': len(values),
                'max': max(values) if values else None,
                'mean': float(sum(values) / len(values)) if values else None,
                'high_cost_count': int(sum(1 for value in values if value >= self.local_cost_inflation_threshold)),
                'lethal_count': int(sum(1 for value in values if value >= self.local_cost_obstacle_threshold)),
            },
        }

    def _local_cost_target_footprint_evidence(self, target_xy: Optional[Point], yaw: float) -> dict[str, object]:
        grid = self.local_costmap_view
        if target_xy is None or grid is None:
            return {'available': False, 'summary': None}
        values = self._local_cost_values_in_oriented_box(
            grid,
            target_xy,
            yaw,
            self.local_cost_footprint_length_m,
            self.local_cost_footprint_width_m,
        )
        return {
            'available': True,
            'length_m': float(self.local_cost_footprint_length_m),
            'width_m': float(self.local_cost_footprint_width_m),
            'summary': {
                'sample_count': len(values),
                'max': max(values) if values else None,
                'mean': float(sum(values) / len(values)) if values else None,
                'p95': self._cost_percentile(values, 0.95),
                'high_cost_count': int(sum(1 for value in values if value >= self.local_cost_inflation_threshold)),
                'lethal_count': int(sum(1 for value in values if value >= self.local_cost_obstacle_threshold)),
            },
        }

    def _phase62_robot_to_target_progress(
        self,
        dispatch_pose: Optional[RobotPose] = None,
        target_xy: Optional[Point] = None,
    ) -> dict[str, object]:
        target = target_xy if target_xy is not None else self.active_goal_target
        current_pose = self._lookup_robot_pose()
        start_pose = dispatch_pose
        if start_pose is None and isinstance(self.active_goal_event_context.get('dispatch_pose'), list):
            raw = self.active_goal_event_context.get('dispatch_pose')
            if isinstance(raw, list) and len(raw) >= 3:
                start_pose = (float(raw[0]), float(raw[1]), float(raw[2]))
        start_distance = None
        current_distance = None
        improvement = None
        if target is not None and start_pose is not None:
            start_distance = math.hypot(float(start_pose[0]) - float(target[0]), float(start_pose[1]) - float(target[1]))
        if target is not None and current_pose is not None:
            current_distance = math.hypot(float(current_pose[0]) - float(target[0]), float(current_pose[1]) - float(target[1]))
        if start_distance is not None and current_distance is not None:
            improvement = float(start_distance - current_distance)
        return {
            'dispatch_pose': self._pose_to_payload(start_pose),
            'current_pose': self._pose_to_payload(current_pose),
            'target': self._point_to_payload(target),
            'distance_at_dispatch_m': start_distance,
            'current_distance_m': current_distance,
            'distance_improvement_m': improvement,
        }

    def _phase62_nav2_feedback_summary(self, sequence_id: Optional[int]) -> dict[str, object]:
        rows = self.phase62_nav2_feedback_by_sequence.get(sequence_id, []) if sequence_id is not None else []
        distances = [float(row['distance_remaining']) for row in rows if row.get('distance_remaining') is not None]
        return {
            'feedback_sample_count': len(rows),
            'distance_remaining_initial_m': distances[0] if distances else None,
            'distance_remaining_final_m': distances[-1] if distances else None,
            'distance_remaining_improvement_m': (distances[0] - distances[-1]) if len(distances) >= 2 else None,
            'number_of_recoveries_max': max([int(row.get('number_of_recoveries') or 0) for row in rows], default=0),
        }

    def _phase62_timestamp_consistency(self) -> dict[str, object]:
        now = self.get_clock().now()
        map_age = None
        local_age = None
        scan_age = None
        if self.map_msg is not None:
            map_stamp = Time.from_msg(self.map_msg.header.stamp)
            map_age = max(0.0, float((now - map_stamp).nanoseconds / 1e9))
        if self.local_costmap_last_update_time is not None:
            local_age = max(0.0, float((now - self.local_costmap_last_update_time).nanoseconds / 1e9))
        if self.scan_last_update_time is not None:
            scan_age = max(0.0, float((now - self.scan_last_update_time).nanoseconds / 1e9))
        max_age = max([age for age in [map_age, local_age, scan_age] if age is not None], default=None)
        return {
            'map_age_sec': map_age,
            'local_costmap_age_sec': local_age,
            'scan_age_sec': scan_age,
            'max_age_sec': max_age,
            'all_available': bool(map_age is not None and local_age is not None and scan_age is not None),
        }

    @staticmethod
    def _duration_msg_to_sec(value: Any) -> Optional[float]:
        if value is None:
            return None
        return float(getattr(value, 'sec', 0)) + float(getattr(value, 'nanosec', 0)) / 1e9

    @staticmethod
    def _finite_float_or_none(value: Any) -> Optional[float]:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        return out if math.isfinite(out) else None

    def _compute_local_cost_diagnostics(self, dispatch_pose: Optional[RobotPose], target_xy: Point) -> dict[str, object]:
        diagnostics: dict[str, object] = {
            'local_costmap_topic': self.local_costmap_topic,
            'local_costmap_msg': self.local_costmap_msg is not None,
            'local_costmap_view': self.local_costmap_view is not None,
            'dispatch_local_cost_sample_age_sec': None,
            'dispatch_target_in_local_costmap_bounds': None,
            'dispatch_robot_in_local_costmap_bounds': None,
            'dispatch_path_local_cost_sample_count': 0,
            'dispatch_path_local_cost_in_bounds_sample_count': 0,
            'dispatch_local_cost_sample_coverage_ratio': 0.0,
            'dispatch_target_local_cost': None,
            'dispatch_target_local_cost_max_radius': None,
            'dispatch_path_local_cost_max': None,
            'dispatch_path_local_cost_mean': None,
            'timeout_robot_local_cost_max': None,
            'timeout_robot_local_cost_mean': None,
            'timeout_robot_obstacle_cluster_count': None,
            'footprint_corridor_inflation_squeezed': None,
            'timeout_footprint_cost_max': None,
            'timeout_footprint_cost_mean': None,
            'timeout_footprint_cost_p95': None,
            'timeout_footprint_inflated_cell_count': None,
            'timeout_footprint_lethal_cell_count': None,
            'timeout_front_wedge_cost_max': None,
            'timeout_front_wedge_cost_mean': None,
            'timeout_front_wedge_clearance_m': None,
            'timeout_left_side_cost_max': None,
            'timeout_left_side_clearance_m': None,
            'timeout_right_side_cost_max': None,
            'timeout_right_side_clearance_m': None,
            'timeout_path_ahead_0_5m_cost_max': None,
            'timeout_path_ahead_0_5m_cost_mean': None,
            'timeout_path_ahead_1_0m_cost_max': None,
            'timeout_path_ahead_1_0m_cost_mean': None,
        }
        grid = self.local_costmap_view
        if grid is None:
            return diagnostics
        if self.local_costmap_last_update_time is not None:
            diagnostics['dispatch_local_cost_sample_age_sec'] = round(
                max(0.0, float((self.get_clock().now() - self.local_costmap_last_update_time).nanoseconds / 1e9)),
                6,
            )
        target_cell = grid.world_to_cell(target_xy[0], target_xy[1])
        target_in_bounds = grid.in_bounds(target_cell)
        diagnostics['dispatch_target_in_local_costmap_bounds'] = bool(target_in_bounds)
        diagnostics['dispatch_target_local_cost'] = int(grid.cell_value(target_cell)) if target_in_bounds else None
        target_values = self._local_cost_values_in_radius(grid, target_xy, self.local_cost_target_radius_m)
        diagnostics['dispatch_target_local_cost_max_radius'] = max(target_values) if target_values else None
        if dispatch_pose is not None:
            start_xy = (dispatch_pose[0], dispatch_pose[1])
            robot_cell = grid.world_to_cell(start_xy[0], start_xy[1])
            diagnostics['dispatch_robot_in_local_costmap_bounds'] = bool(grid.in_bounds(robot_cell))
            expected_samples = self._line_sample_count(start_xy, target_xy, grid.info.resolution)
            path_values = self._line_in_bounds_values(grid, start_xy, target_xy)
            diagnostics['dispatch_path_local_cost_sample_count'] = int(expected_samples)
            diagnostics['dispatch_path_local_cost_in_bounds_sample_count'] = int(len(path_values))
            diagnostics['dispatch_local_cost_sample_coverage_ratio'] = float(len(path_values) / expected_samples) if expected_samples else 0.0
            diagnostics['dispatch_path_local_cost_max'] = max(path_values) if path_values else None
            diagnostics['dispatch_path_local_cost_mean'] = float(sum(path_values) / len(path_values)) if path_values else None
        return diagnostics

    def _update_active_timeout_local_cost_diagnostics(self) -> None:
        if not self.active_goal_event_context:
            return
        robot_pose = self._lookup_robot_pose()
        grid = self.local_costmap_view
        diagnostics: dict[str, object] = {
            'timeout_local_cost_sample_age_sec': None,
            'timeout_robot_in_local_costmap_bounds': None,
            'timeout_robot_local_cost_max': None,
            'timeout_robot_local_cost_mean': None,
            'timeout_robot_obstacle_cluster_count': None,
            'footprint_corridor_inflation_squeezed': None,
            'timeout_footprint_cost_max': None,
            'timeout_footprint_cost_mean': None,
            'timeout_footprint_cost_p95': None,
            'timeout_footprint_inflated_cell_count': None,
            'timeout_footprint_lethal_cell_count': None,
            'timeout_front_wedge_cost_max': None,
            'timeout_front_wedge_cost_mean': None,
            'timeout_front_wedge_clearance_m': None,
            'timeout_left_side_cost_max': None,
            'timeout_left_side_clearance_m': None,
            'timeout_right_side_cost_max': None,
            'timeout_right_side_clearance_m': None,
            'timeout_path_ahead_0_5m_cost_max': None,
            'timeout_path_ahead_0_5m_cost_mean': None,
            'timeout_path_ahead_1_0m_cost_max': None,
            'timeout_path_ahead_1_0m_cost_mean': None,
        }
        if grid is not None and self.local_costmap_last_update_time is not None:
            diagnostics['timeout_local_cost_sample_age_sec'] = round(
                max(0.0, float((self.get_clock().now() - self.local_costmap_last_update_time).nanoseconds / 1e9)),
                6,
            )
        if grid is not None and robot_pose is not None:
            robot_xy = (robot_pose[0], robot_pose[1])
            robot_cell = grid.world_to_cell(robot_xy[0], robot_xy[1])
            diagnostics['timeout_robot_in_local_costmap_bounds'] = bool(grid.in_bounds(robot_cell))
            values = self._local_cost_values_in_radius(grid, robot_xy, self.local_cost_robot_cluster_radius_m)
            diagnostics['timeout_robot_local_cost_max'] = max(values) if values else None
            diagnostics['timeout_robot_local_cost_mean'] = float(sum(values) / len(values)) if values else None
            diagnostics['timeout_robot_obstacle_cluster_count'] = int(sum(1 for value in values if value >= self.local_cost_obstacle_threshold))
            diagnostics['footprint_corridor_inflation_squeezed'] = bool(any(value >= self.local_cost_inflation_threshold for value in values))
            footprint_values = self._local_cost_values_in_oriented_box(
                grid,
                robot_xy,
                robot_pose[2],
                self.local_cost_footprint_length_m,
                self.local_cost_footprint_width_m,
            )
            self._update_cost_stats(diagnostics, 'timeout_footprint_cost', footprint_values)
            diagnostics['timeout_footprint_inflated_cell_count'] = int(sum(1 for value in footprint_values if value >= self.local_cost_inflation_threshold))
            diagnostics['timeout_footprint_lethal_cell_count'] = int(sum(1 for value in footprint_values if value >= self.local_cost_obstacle_threshold))
            front_values, front_clearance = self._local_cost_values_in_wedge(
                grid,
                robot_xy,
                robot_pose[2],
                self.local_cost_front_wedge_radius_m,
                self.local_cost_front_wedge_half_angle_rad,
            )
            diagnostics['timeout_front_wedge_cost_max'] = max(front_values) if front_values else None
            diagnostics['timeout_front_wedge_cost_mean'] = float(sum(front_values) / len(front_values)) if front_values else None
            diagnostics['timeout_front_wedge_clearance_m'] = front_clearance
            left_xy = (
                robot_xy[0] + math.cos(robot_pose[2] + math.pi / 2.0) * self.local_cost_side_probe_radius_m,
                robot_xy[1] + math.sin(robot_pose[2] + math.pi / 2.0) * self.local_cost_side_probe_radius_m,
            )
            right_xy = (
                robot_xy[0] + math.cos(robot_pose[2] - math.pi / 2.0) * self.local_cost_side_probe_radius_m,
                robot_xy[1] + math.sin(robot_pose[2] - math.pi / 2.0) * self.local_cost_side_probe_radius_m,
            )
            left_values = self._local_cost_values_in_radius(grid, left_xy, self.local_cost_side_probe_radius_m)
            right_values = self._local_cost_values_in_radius(grid, right_xy, self.local_cost_side_probe_radius_m)
            diagnostics['timeout_left_side_cost_max'] = max(left_values) if left_values else None
            diagnostics['timeout_right_side_cost_max'] = max(right_values) if right_values else None
            diagnostics['timeout_left_side_clearance_m'] = grid.nearest_obstacle_distance(left_xy[0], left_xy[1], max_radius_m=self.local_cost_side_probe_radius_m)
            diagnostics['timeout_right_side_clearance_m'] = grid.nearest_obstacle_distance(right_xy[0], right_xy[1], max_radius_m=self.local_cost_side_probe_radius_m)
            path_0_5 = self._local_cost_path_ahead_values(grid, robot_xy, robot_pose[2], 0.5)
            path_1_0 = self._local_cost_path_ahead_values(grid, robot_xy, robot_pose[2], 1.0)
            diagnostics['timeout_path_ahead_0_5m_cost_max'] = max(path_0_5) if path_0_5 else None
            diagnostics['timeout_path_ahead_0_5m_cost_mean'] = float(sum(path_0_5) / len(path_0_5)) if path_0_5 else None
            diagnostics['timeout_path_ahead_1_0m_cost_max'] = max(path_1_0) if path_1_0 else None
            diagnostics['timeout_path_ahead_1_0m_cost_mean'] = float(sum(path_1_0) / len(path_1_0)) if path_1_0 else None
        self.active_goal_event_context.update(diagnostics)
        if self.active_goal_sequence_id is not None:
            context = dict(self.goal_event_context_by_sequence.get(self.active_goal_sequence_id, {}))
            context.update(diagnostics)
            self.goal_event_context_by_sequence[self.active_goal_sequence_id] = context

    def _local_cost_values_in_radius(self, grid: OccupancyGridView, center: Point, radius_m: float) -> list[int]:
        center_x = float(center[0])
        center_y = float(center[1])
        center_cell = grid.world_to_cell(center_x, center_y)
        radius_cells = int(math.ceil(max(radius_m, 0.0) / grid.info.resolution))
        values: list[int] = []
        for cell_y in range(center_cell[1] - radius_cells, center_cell[1] + radius_cells + 1):
            for cell_x in range(center_cell[0] - radius_cells, center_cell[0] + radius_cells + 1):
                cell = (cell_x, cell_y)
                if not grid.in_bounds(cell):
                    continue
                world_x, world_y = grid.cell_to_world(cell_x, cell_y)
                if math.hypot(world_x - center_x, world_y - center_y) <= radius_m + 1e-9:
                    values.append(int(grid.cell_value(cell)))
        return values

    @staticmethod
    def _cost_percentile(values: list[int], q: float) -> Optional[float]:
        if not values:
            return None
        ordered = sorted(values)
        rank = max(1, int(math.ceil(q * len(ordered))))
        return float(ordered[min(rank - 1, len(ordered) - 1)])

    def _update_cost_stats(self, diagnostics: dict[str, object], prefix: str, values: list[int]) -> None:
        diagnostics[f'{prefix}_max'] = max(values) if values else None
        diagnostics[f'{prefix}_mean'] = float(sum(values) / len(values)) if values else None
        diagnostics[f'{prefix}_p95'] = self._cost_percentile(values, 0.95)

    def _local_cost_values_in_oriented_box(
        self,
        grid: OccupancyGridView,
        center: Point,
        yaw: float,
        length_m: float,
        width_m: float,
    ) -> list[int]:
        center_x = float(center[0])
        center_y = float(center[1])
        half_length = max(0.0, float(length_m)) / 2.0
        half_width = max(0.0, float(width_m)) / 2.0
        radius = math.hypot(half_length, half_width)
        center_cell = grid.world_to_cell(center_x, center_y)
        radius_cells = int(math.ceil(radius / grid.info.resolution))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        values: list[int] = []
        for cell_y in range(center_cell[1] - radius_cells, center_cell[1] + radius_cells + 1):
            for cell_x in range(center_cell[0] - radius_cells, center_cell[0] + radius_cells + 1):
                cell = (cell_x, cell_y)
                if not grid.in_bounds(cell):
                    continue
                world_x, world_y = grid.cell_to_world(cell_x, cell_y)
                dx = world_x - center_x
                dy = world_y - center_y
                forward = dx * cos_yaw + dy * sin_yaw
                lateral = -dx * sin_yaw + dy * cos_yaw
                if abs(forward) <= half_length + 1e-9 and abs(lateral) <= half_width + 1e-9:
                    values.append(int(grid.cell_value(cell)))
        return values

    def _local_cost_values_in_wedge(
        self,
        grid: OccupancyGridView,
        center: Point,
        yaw: float,
        radius_m: float,
        half_angle_rad: float,
    ) -> tuple[list[int], Optional[float]]:
        center_x = float(center[0])
        center_y = float(center[1])
        radius = max(0.0, float(radius_m))
        center_cell = grid.world_to_cell(center_x, center_y)
        radius_cells = int(math.ceil(radius / grid.info.resolution))
        values: list[int] = []
        clearance = radius
        for cell_y in range(center_cell[1] - radius_cells, center_cell[1] + radius_cells + 1):
            for cell_x in range(center_cell[0] - radius_cells, center_cell[0] + radius_cells + 1):
                cell = (cell_x, cell_y)
                if not grid.in_bounds(cell):
                    continue
                world_x, world_y = grid.cell_to_world(cell_x, cell_y)
                dx = world_x - center_x
                dy = world_y - center_y
                dist = math.hypot(dx, dy)
                if dist > radius + 1e-9 or dist <= 1e-9:
                    continue
                angle = normalize_angle(math.atan2(dy, dx) - yaw)
                if abs(angle) <= half_angle_rad + 1e-9:
                    value = int(grid.cell_value(cell))
                    values.append(value)
                    if value >= self.local_cost_inflation_threshold:
                        clearance = min(clearance, dist)
        return values, float(clearance) if values else None

    def _local_cost_path_ahead_values(self, grid: OccupancyGridView, start: Point, yaw: float, distance_m: float) -> list[int]:
        end = (
            float(start[0]) + math.cos(yaw) * float(distance_m),
            float(start[1]) + math.sin(yaw) * float(distance_m),
        )
        return self._line_in_bounds_values(grid, start, end)

    @staticmethod
    def _line_sample_count(start: Point, end: Point, resolution: float) -> int:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        step = max(resolution * 0.5, 0.05)
        return max(1, int(math.ceil(distance / max(step, 1e-6)))) + 1

    def _line_in_bounds_values(self, grid: OccupancyGridView, start: Point, end: Point) -> list[int]:
        steps = self._line_sample_count(start, end, grid.info.resolution) - 1
        values: list[int] = []
        for index in range(steps + 1):
            ratio = index / max(steps, 1)
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            cell = grid.world_to_cell(x, y)
            if grid.in_bounds(cell):
                values.append(int(grid.cell_value(cell)))
        return values

    def _goal_elapsed_sec(self) -> Optional[float]:
        if self.goal_sent_time is None:
            return None
        return max(0.0, float((self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9))

    def _publish_goal_event(
        self,
        event: str,
        *,
        goal_sequence: Optional[int] = None,
        result_status: Optional[int] = None,
        result_reason: Optional[str] = None,
        branch_failure_state: Optional[str] = None,
        caused_branch_failure: bool = False,
        caused_blacklist: bool = False,
    ) -> None:
        context = dict(self.goal_event_context_by_sequence.get(goal_sequence, self.active_goal_event_context)) if goal_sequence is not None else dict(self.active_goal_event_context)
        if event == 'topology_consistency_guard':
            context.update(self.last_topology_consistency_diagnostics)
            context['topology_consistency_guard'] = self.last_topology_consistency_diagnostics
        payload = {
            'event': event,
            'goal_sequence': goal_sequence if goal_sequence is not None else context.get('goal_sequence', self.active_goal_sequence_id),
            'dispatch_pose': context.get('dispatch_pose'),
            'target': context.get('target', self._point_to_payload(self.active_goal_target)),
            'original_target': context.get('original_target'),
            'refined_target': context.get('refined_target'),
            'centerline_target_refinement': context.get('centerline_target_refinement', self.last_centerline_target_refinement_diagnostics),
            'centerline_refinement_applied': context.get('centerline_refinement_applied'),
            'centerline_refinement_reason': context.get('centerline_refinement_reason'),
            'centerline_refinement_enabled': context.get('centerline_refinement_enabled', self.centerline_target_refinement_enabled),
            'centerline_refinement_candidate_count': context.get('centerline_refinement_candidate_count'),
            'centerline_refinement_eligible_candidate_count': context.get('centerline_refinement_eligible_candidate_count'),
            'centerline_refinement_gate_conditions': context.get('centerline_refinement_gate_conditions'),
            'two_step_staging_plan': context.get('two_step_staging_plan'),
            'two_step_stage_dispatch_requested': context.get('two_step_stage_dispatch_requested', False),
            'staging_goal_pose': context.get('staging_goal_pose'),
            'staging_reason': context.get('staging_reason'),
            'staging_executability_check': context.get('staging_executability_check'),
            'second_step_forward_goal': context.get('second_step_forward_goal'),
            'staging_applied': context.get('staging_applied', False),
            'staging_reject_reason': context.get('staging_reject_reason'),
            'staging_candidates': context.get('staging_candidates'),
            'visual_diagnosis_wait_requested': context.get('visual_diagnosis_wait_requested', False),
            'branch_scoring_changed': context.get('branch_scoring_changed', False),
            'fallback_terminal_acceptance_used': context.get('fallback_terminal_acceptance_used', False),
            'target_exit_dist': context.get('target_exit_dist'),
            'robot_exit_dist_at_dispatch': context.get('robot_exit_dist_at_dispatch'),
            'local_topology': context.get('local_topology', self.last_local_topology_kind),
            'branch_angle': context.get('branch_angle'),
            'goal_kind': context.get('goal_kind', self.active_goal_kind),
            'effective_timeout_sec': context.get('effective_timeout_sec', self._effective_goal_timeout_sec()),
            'result_status': result_status,
            'result_reason': result_reason,
            'elapsed_sec': self._goal_elapsed_sec(),
            'near_exit': context.get('near_exit', self._active_goal_near_exit()),
            'branch_failure_state': branch_failure_state,
            'caused_branch_failure': caused_branch_failure,
            'caused_blacklist': caused_blacklist,
            'start_node_id': context.get('start_node_id'),
            'current_node_id': context.get('current_node_id', self.current_node_id),
            'last_open_direction_count': context.get('last_open_direction_count', self.last_open_direction_count),
            'last_candidate_count': context.get('last_candidate_count', self.last_candidate_count),
            'chosen_branch_rank': context.get('chosen_branch_rank'),
            'chosen_branch_score_components': context.get('chosen_branch_score_components'),
            'candidate_branch_count': context.get('candidate_branch_count'),
            'candidate_branches': context.get('candidate_branches'),
            'selected_due_to_context': context.get('selected_due_to_context'),
            'target_cell_occupancy': context.get('target_cell_occupancy'),
            'target_clearance_m': context.get('target_clearance_m'),
            'line_of_sight_occupied_count': context.get('line_of_sight_occupied_count'),
            'line_of_sight_unknown_count': context.get('line_of_sight_unknown_count'),
            'line_of_sight_min_clearance_m': context.get('line_of_sight_min_clearance_m'),
            'path_corridor_min_clearance_m': context.get('path_corridor_min_clearance_m'),
            'target_near_wall': context.get('target_near_wall'),
            'target_crosses_wall_corner': context.get('target_crosses_wall_corner'),
            'target_crosses_narrow_passage': context.get('target_crosses_narrow_passage'),
            'target_yaw_corridor_conflict': context.get('target_yaw_corridor_conflict'),
            'local_costmap_topic': context.get('local_costmap_topic'),
            'local_costmap_msg': context.get('local_costmap_msg'),
            'local_costmap_view': context.get('local_costmap_view'),
            'dispatch_local_cost_sample_age_sec': context.get('dispatch_local_cost_sample_age_sec'),
            'dispatch_target_in_local_costmap_bounds': context.get('dispatch_target_in_local_costmap_bounds'),
            'dispatch_robot_in_local_costmap_bounds': context.get('dispatch_robot_in_local_costmap_bounds'),
            'dispatch_path_local_cost_sample_count': context.get('dispatch_path_local_cost_sample_count'),
            'dispatch_path_local_cost_in_bounds_sample_count': context.get('dispatch_path_local_cost_in_bounds_sample_count'),
            'dispatch_local_cost_sample_coverage_ratio': context.get('dispatch_local_cost_sample_coverage_ratio'),
            'dispatch_target_local_cost': context.get('dispatch_target_local_cost'),
            'dispatch_target_local_cost_max_radius': context.get('dispatch_target_local_cost_max_radius'),
            'dispatch_path_local_cost_max': context.get('dispatch_path_local_cost_max'),
            'dispatch_path_local_cost_mean': context.get('dispatch_path_local_cost_mean'),
            'timeout_local_cost_sample_age_sec': context.get('timeout_local_cost_sample_age_sec'),
            'timeout_robot_in_local_costmap_bounds': context.get('timeout_robot_in_local_costmap_bounds'),
            'timeout_robot_local_cost_max': context.get('timeout_robot_local_cost_max'),
            'timeout_robot_local_cost_mean': context.get('timeout_robot_local_cost_mean'),
            'timeout_robot_obstacle_cluster_count': context.get('timeout_robot_obstacle_cluster_count'),
            'footprint_corridor_inflation_squeezed': context.get('footprint_corridor_inflation_squeezed'),
            'timeout_footprint_cost_max': context.get('timeout_footprint_cost_max'),
            'timeout_footprint_cost_mean': context.get('timeout_footprint_cost_mean'),
            'timeout_footprint_cost_p95': context.get('timeout_footprint_cost_p95'),
            'timeout_footprint_inflated_cell_count': context.get('timeout_footprint_inflated_cell_count'),
            'timeout_footprint_lethal_cell_count': context.get('timeout_footprint_lethal_cell_count'),
            'timeout_front_wedge_cost_max': context.get('timeout_front_wedge_cost_max'),
            'timeout_front_wedge_cost_mean': context.get('timeout_front_wedge_cost_mean'),
            'timeout_front_wedge_clearance_m': context.get('timeout_front_wedge_clearance_m'),
            'timeout_left_side_cost_max': context.get('timeout_left_side_cost_max'),
            'timeout_left_side_clearance_m': context.get('timeout_left_side_clearance_m'),
            'timeout_right_side_cost_max': context.get('timeout_right_side_cost_max'),
            'timeout_right_side_clearance_m': context.get('timeout_right_side_clearance_m'),
            'timeout_path_ahead_0_5m_cost_max': context.get('timeout_path_ahead_0_5m_cost_max'),
            'timeout_path_ahead_0_5m_cost_mean': context.get('timeout_path_ahead_0_5m_cost_mean'),
            'timeout_path_ahead_1_0m_cost_max': context.get('timeout_path_ahead_1_0m_cost_max'),
            'timeout_path_ahead_1_0m_cost_mean': context.get('timeout_path_ahead_1_0m_cost_mean'),
            'phase62_nav2_result_summary': context.get('phase62_nav2_result_summary'),
            'phase62_timestamp_consistency': context.get('phase62_timestamp_consistency'),
            'phase62_local_cost_thresholds': context.get('phase62_local_cost_thresholds'),
            'phase62_first_dispatch_traversability': context.get('phase62_first_dispatch_traversability'),
            'phase62_target_cell_state': context.get('phase62_target_cell_state'),
            'phase62_local_costmap_patch': context.get('phase62_local_costmap_patch'),
            'phase62_target_footprint_cost': context.get('phase62_target_footprint_cost'),
            'phase62_front_wedge_clearance_m': context.get('phase62_front_wedge_clearance_m'),
            'phase62_front_wedge_cost': context.get('phase62_front_wedge_cost'),
            'phase62_robot_to_target_progress': context.get('phase62_robot_to_target_progress'),
            'phase62_cmd_vel_summary': context.get('phase62_cmd_vel_summary'),
            'phase62_nav2_feedback_summary': context.get('phase62_nav2_feedback_summary'),
            'near_exit_fallback_triggered': context.get('near_exit_fallback_triggered', False),
            'fallback_reason': context.get('fallback_reason'),
            'robot_exit_dist': context.get('robot_exit_dist'),
            'cmd_near_zero_duration': context.get('cmd_near_zero_duration'),
            'last_nav2_result': context.get('last_nav2_result'),
            'robot_to_path_distance': context.get('robot_to_path_distance'),
            'action': context.get('action'),
            'near_exit_fallback_enabled': context.get('near_exit_fallback_enabled', self.near_exit_fallback_enabled),
            'near_exit_fallback_attempts': context.get('near_exit_fallback_attempts', self.near_exit_fallback_attempts),
            'near_exit_fallback_max_attempts': context.get('near_exit_fallback_max_attempts', self.near_exit_fallback_max_attempts),
            'micro_goal_geometry': context.get('micro_goal_geometry'),
            'micro_goal_local_cost': context.get('micro_goal_local_cost'),
            'topology_consistency_guard': context.get('topology_consistency_guard', self.last_topology_consistency_diagnostics),
            'topology_consistency_enabled': context.get('topology_consistency_enabled', self.topology_consistency_enabled),
            'topology_consistency_required_no_candidate_frames': context.get('topology_consistency_required_no_candidate_frames', self.topology_consistency_required_no_candidate_frames),
            'topology_consistency_window_sec': context.get('topology_consistency_window_sec', self.topology_consistency_window_sec),
            'topology_consistency_frame_index': context.get('topology_consistency_frame_index'),
            'topology_consistency_frames': context.get('topology_consistency_frames'),
            'topology_consistency_frame_count': context.get('topology_consistency_frame_count'),
            'raw_open_direction_count': context.get('raw_open_direction_count'),
            'filtered_open_direction_count': context.get('filtered_open_direction_count'),
            'candidate_before_filter_count': context.get('candidate_before_filter_count'),
            'candidate_after_filter_count': context.get('candidate_after_filter_count'),
            'dead_end_policy_state': context.get('dead_end_policy_state'),
            'terminalization_delayed': context.get('terminalization_delayed'),
            'candidate_recovered_during_consistency_window': context.get('candidate_recovered_during_consistency_window'),
            'topology_consistency_guard_status': context.get('topology_consistency_guard_status'),
            'topology_consistency_terminalization_reason': context.get('topology_consistency_terminalization_reason'),
            'post_ingress_single_open_exception': context.get('post_ingress_single_open_exception', self.last_post_ingress_single_open_exception_diagnostics),
            'post_ingress_context_active': context.get('post_ingress_context_active'),
            'first_post_ingress_topology_node': context.get('first_post_ingress_topology_node'),
            'single_open_exception_eligible': context.get('single_open_exception_eligible'),
            'single_open_exception_applied': context.get('single_open_exception_applied'),
            'single_open_exception_reason': context.get('single_open_exception_reason'),
            'angle_to_return_entrance': context.get('angle_to_return_entrance'),
            'multi_frame_single_open_confirmed': context.get('multi_frame_single_open_confirmed'),
            'single_open_exception_candidate_recovered_during_consistency_window': context.get('single_open_exception_candidate_recovered_during_consistency_window'),
            'single_open_exception_dispatch_produced': context.get('single_open_exception_dispatch_produced'),
            'blocked_branch_count': self.topology.blocked_branch_count(),
            'blacklisted_goal_count': len(self.topology.blacklist),
            'mode': self.mode,
        }
        self.goal_events_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _mark_exhausted(self, reason: str) -> None:
        self._enter_terminal_state(FAILED_EXHAUSTED, terminal_reason=reason)
        if not self.exhausted_logged:
            self.exhausted_logged = True
            self.get_logger().warn('MAZE_EXPLORER_EXHAUSTED reason=%s' % reason)

    def _state_payload(self) -> None:
        """Compatibility marker: state payload includes self.last_topology_sampling_diagnostics."""
        return None

    def _publish_state(self) -> None:
        robot_pose = self._lookup_robot_pose()
        exit_distance = None
        if robot_pose is not None:
            exit_distance = math.hypot(robot_pose[0] - self.exit_x, robot_pose[1] - self.exit_y)
        backtrack_target = None
        if self.current_node_id is not None:
            target_node = self.topology.next_backtrack_target(self.current_node_id)
            if target_node is not None:
                backtrack_target = {'node_id': target_node.node_id, 'x': target_node.x, 'y': target_node.y}
        payload = {
            'mode': self.mode,
            'dispatch_readiness_gate': self.last_dispatch_readiness_gate,
            'dispatch_readiness_gate_passed': bool(self.last_dispatch_readiness_gate.get('passed', False)),
            'dispatch_readiness_blocking_reasons': self.last_dispatch_readiness_gate.get('blocking_reasons', []),
            'dispatch_readiness_first_pass_time_sec': (
                self.dispatch_readiness_first_pass_time.nanoseconds / 1e9
                if self.dispatch_readiness_first_pass_time is not None else None
            ),
            'startup_warmup_no_dispatch': self.startup_warmup_no_dispatch,
            'startup_warmup_guard': 'gate_ready_no_dispatch' if self.mode == STARTUP_WARMUP_NO_DISPATCH else None,
            'startup_warmup_guard_reason': (
                'Phase52 bounded map-boundary readiness validation; topology sampling and goal dispatch disabled'
                if self.mode == STARTUP_WARMUP_NO_DISPATCH else None
            ),
            'current_node_id': self.current_node_id,
            'goal_active': self.goal_active,
            'goal_kind': self.active_goal_kind,
            'active_goal_target': self.active_goal_target,
            'goal_count': self.goal_count,
            'goal_sequence_id': self.goal_sequence_id,
            'active_goal_sequence_id': self.active_goal_sequence_id,
            'last_completed_goal_sequence_id': self.last_completed_goal_sequence_id,
            'effective_goal_timeout_sec': self._effective_goal_timeout_sec(),
            'active_goal_near_exit': self._active_goal_near_exit(),
            'near_exit_goal_timeout_sec': self.near_exit_goal_timeout_sec,
            'near_exit_timeout_extension_radius_m': self.near_exit_timeout_extension_radius_m,
            'near_exit_fallback_enabled': self.near_exit_fallback_enabled,
            'near_exit_fallback_trigger_radius_m': self.near_exit_fallback_trigger_radius_m,
            'near_exit_terminal_acceptance_radius_m': self.near_exit_terminal_acceptance_radius_m,
            'near_exit_fallback_attempts': self.near_exit_fallback_attempts,
            'near_exit_fallback_max_attempts': self.near_exit_fallback_max_attempts,
            'goal_settle_active': self._goal_settle_active(),
            'goal_settle_remaining_sec': self._goal_settle_remaining_sec(),
            'goal_success_count': self.goal_success_count,
            'goal_failure_count': self.goal_failure_count,
            'nav2_failure_count': self.nav2_failure_count,
            'stale_result_count': self.stale_result_count,
            'preempted_goal_count': self.preempted_goal_count,
            'terminal_cancel_count': self.terminal_cancel_count,
            'timeout_cancel_count': self.timeout_cancel_count,
            'canceled_after_timeout_count': self.canceled_after_timeout_count,
            'canceled_after_exit_count': self.canceled_after_exit_count,
            'timeout_cancel_goal_sequence_id': self.timeout_cancel_goal_sequence_id,
            'terminal_cancel_goal_sequence_id': self.terminal_cancel_goal_sequence_id,
            'last_terminal_reason': self.last_terminal_reason,
            'terminal_state_entered': self.terminal_state_entered,
            'backtrack_failure_count': self.backtrack_failure_count,
            'blocked_branch_count': self.topology.blocked_branch_count(),
            'blacklisted_goal_count': len(self.topology.blacklist),
            'last_failure_reason': self.last_failure_reason,
            'last_nav2_status': self.last_nav2_status,
            'last_local_topology_kind': self.last_local_topology_kind,
            'last_open_direction_count': self.last_open_direction_count,
            'allow_reverse_branch_goals': self.allow_reverse_branch_goals,
            'last_candidate_count': self.last_candidate_count,
            'topology_consistency_guard': self.last_topology_consistency_diagnostics,
            'topology_consistency_enabled': self.last_topology_consistency_diagnostics.get('topology_consistency_enabled', self.topology_consistency_enabled),
            'topology_consistency_required_no_candidate_frames': self.last_topology_consistency_diagnostics.get('topology_consistency_required_no_candidate_frames', self.topology_consistency_required_no_candidate_frames),
            'topology_consistency_window_sec': self.last_topology_consistency_diagnostics.get('topology_consistency_window_sec', self.topology_consistency_window_sec),
            'topology_consistency_frame_index': self.last_topology_consistency_diagnostics.get('topology_consistency_frame_index'),
            'topology_consistency_frames': self.last_topology_consistency_diagnostics.get('topology_consistency_frames', []),
            'topology_consistency_frame_count': self.last_topology_consistency_diagnostics.get('topology_consistency_frame_count', 0),
            'raw_open_direction_count': self.last_topology_consistency_diagnostics.get('raw_open_direction_count'),
            'filtered_open_direction_count': self.last_topology_consistency_diagnostics.get('filtered_open_direction_count'),
            'candidate_before_filter_count': self.last_topology_consistency_diagnostics.get('candidate_before_filter_count'),
            'candidate_after_filter_count': self.last_topology_consistency_diagnostics.get('candidate_after_filter_count'),
            'dead_end_policy_state': self.last_topology_consistency_diagnostics.get('dead_end_policy_state'),
            'terminalization_delayed': self.last_topology_consistency_diagnostics.get('terminalization_delayed', False),
            'candidate_recovered_during_consistency_window': self.last_topology_consistency_diagnostics.get('candidate_recovered_during_consistency_window', False),
            'topology_consistency_guard_status': self.last_topology_consistency_diagnostics.get('topology_consistency_guard_status'),
            'topology_consistency_terminalization_reason': self.last_topology_consistency_diagnostics.get('topology_consistency_terminalization_reason'),
            'post_ingress_single_open_exception': self.last_post_ingress_single_open_exception_diagnostics,
            'post_ingress_context_active': self.last_post_ingress_single_open_exception_diagnostics.get('post_ingress_context_active'),
            'first_post_ingress_topology_node': self.last_post_ingress_single_open_exception_diagnostics.get('first_post_ingress_topology_node'),
            'single_open_exception_eligible': self.last_post_ingress_single_open_exception_diagnostics.get('single_open_exception_eligible'),
            'single_open_exception_applied': self.last_post_ingress_single_open_exception_diagnostics.get('single_open_exception_applied'),
            'single_open_exception_reason': self.last_post_ingress_single_open_exception_diagnostics.get('single_open_exception_reason'),
            'angle_to_return_entrance': self.last_post_ingress_single_open_exception_diagnostics.get('angle_to_return_entrance'),
            'multi_frame_single_open_confirmed': self.last_post_ingress_single_open_exception_diagnostics.get('multi_frame_single_open_confirmed'),
            'single_open_exception_candidate_recovered_during_consistency_window': self.last_post_ingress_single_open_exception_diagnostics.get('single_open_exception_candidate_recovered_during_consistency_window'),
            'single_open_exception_dispatch_produced': self.last_post_ingress_single_open_exception_diagnostics.get('single_open_exception_dispatch_produced'),
            'phase62_first_dispatch_traversability': self.active_goal_event_context.get('phase62_first_dispatch_traversability') if self.active_goal_event_context else None,
            'phase62_target_cell_state': self.active_goal_event_context.get('phase62_target_cell_state') if self.active_goal_event_context else None,
            'phase62_local_costmap_patch': self.active_goal_event_context.get('phase62_local_costmap_patch') if self.active_goal_event_context else None,
            'phase62_target_footprint_cost': self.active_goal_event_context.get('phase62_target_footprint_cost') if self.active_goal_event_context else None,
            'phase62_front_wedge_clearance_m': self.active_goal_event_context.get('phase62_front_wedge_clearance_m') if self.active_goal_event_context else None,
            'phase62_robot_to_target_progress': self.active_goal_event_context.get('phase62_robot_to_target_progress') if self.active_goal_event_context else None,
            'phase62_cmd_vel_summary': self.active_goal_event_context.get('phase62_cmd_vel_summary') if self.active_goal_event_context else None,
            'phase62_nav2_feedback_summary': self.active_goal_event_context.get('phase62_nav2_feedback_summary') if self.active_goal_event_context else None,
            'phase62_nav2_result_summary': self.active_goal_event_context.get('phase62_nav2_result_summary') if self.active_goal_event_context else None,
            'phase62_timestamp_consistency': self.active_goal_event_context.get('phase62_timestamp_consistency') if self.active_goal_event_context else None,
            'last_topology_sampling_diagnostics': self.last_topology_sampling_diagnostics,
            'phase56_open_direction_to_candidate_diagnostics': self.phase56_open_direction_to_candidate_diagnostics,
            'known_junctions': sum(1 for node in self.topology.nodes.values() if node.node_type == 'junction'),
            'dead_ends': sum(1 for node in self.topology.nodes.values() if node.node_type == 'dead_end'),
            'edges': len(self.topology.edges),
            'backtrack_target': backtrack_target,
            'exit_distance_m': exit_distance,
        }
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _grid_view_from_msg(self, msg: OccupancyGrid) -> OccupancyGridView:
        return OccupancyGridView(
            info=OccupancyGridInfo(
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
            ),
            data=msg.data,
        )

    def _make_pose_stamped(self, target_xy: Point, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target_xy[0])
        pose.pose.position.y = float(target_xy[1])
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    @staticmethod
    def _yaw_to_point(source_xy: Point, target_xy: Point) -> float:
        return math.atan2(target_xy[1] - source_xy[1], target_xy[0] - source_xy[0])

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MazeExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
