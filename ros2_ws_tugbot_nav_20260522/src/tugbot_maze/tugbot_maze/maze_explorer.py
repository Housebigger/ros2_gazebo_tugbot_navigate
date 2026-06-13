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
from geometry_msgs.msg import Point as RosPoint, PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import (
    CORRIDOR,
    DEAD_END,
    JUNCTION,
    classify_local_topology,
    compute_junction_center,
    filter_open_directions,
    generate_second_step_forward_goal_after_staging,
    make_branch_goal,
    make_centered_branch_goal,
    normalize_angle,
    plan_two_step_corridor_alignment_staging_goal,
    refine_corridor_centerline_target,
)
from tugbot_maze.corridor_navigator import CorridorNavigator
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
RELOCALIZING = 'RELOCALIZING'
GUIDED_CORRIDOR = 'GUIDED_CORRIDOR'
CORRIDOR_REACTIVE = 'CORRIDOR_REACTIVE'


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
        self.centerline_target_refinement_enabled = bool(self.declare_parameter('centerline_target_refinement_enabled', False).value)
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
        # Maze interior bounds (map frame). Exploration goals must stay inside
        # the maze; in particular the entrance opening is at x=0, so any target
        # with x<0 is exterior (behind the robot, out the entrance) and must be
        # rejected — otherwise DFS wastes goals driving back outside.
        self.maze_bound_x_min = float(self.declare_parameter('maze_bound_x_min', 0.0).value)
        self.maze_bound_x_max = float(self.declare_parameter('maze_bound_x_max', 22.0).value)
        self.maze_bound_y_min = float(self.declare_parameter('maze_bound_y_min', -1.0).value)
        self.maze_bound_y_max = float(self.declare_parameter('maze_bound_y_max', 20.5).value)
        self.entry_direct_enabled = bool(self.declare_parameter('entry_direct_enabled', True).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 1.5).value)
        # Use reactive forward drive (not Nav2) for the entry move. Nav2 entry
        # goals frequently fail at the entrance before the costmap populates,
        # and each failure triggers Nav2's BackUp recovery which shoves the
        # robot backward out the entrance (observed drift to x=-2.0). Reactive
        # drive needs only scan+TF, drives straight in, and never invokes Nav2
        # recovery — so the robot reliably ends up inside the maze.
        self.entry_direct_use_reactive = bool(
            self.declare_parameter('entry_direct_use_reactive', True).value)
        self.entry_direct_dispatch_timeout_sec = float(
            self.declare_parameter('entry_direct_dispatch_timeout_sec', 90.0).value
        )

        self.exploration_rate_hz = float(self.declare_parameter('exploration_rate_hz', 0.5).value)
        self.publish_debug_state_hz = float(self.declare_parameter('publish_debug_state_hz', 1.0).value)
        self.junction_merge_radius_m = float(self.declare_parameter('junction_merge_radius_m', 0.75).value)
        # Branch scoring. Pure exit-bias (exploration_bonus=0) makes DFS greedy
        # toward the exit and unable to take corridors that lead *away* from it
        # (e.g. the C4 westward detour) — it gets stuck where the maze requires
        # moving away from the goal before coming back. Enabling a novelty bonus
        # and softening the exit bias lets DFS explore those detours while still
        # being gently pulled toward the exit.
        self.exit_bias_weight = float(self.declare_parameter('exit_bias_weight', 0.3).value)
        self.exploration_bonus_weight = float(self.declare_parameter('exploration_bonus_weight', 0.6).value)
        self.distance_to_exit_weight = float(self.declare_parameter('distance_to_exit_weight', 0.15).value)
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
        # Optimization 2: periodic re-localization rotation.  Every N completed
        # goals, perform an in-place 360° rotation to give SLAM scan coverage
        # from all directions.  This reduces odometry drift accumulation.
        self.relocalize_interval_goals = int(self.declare_parameter('relocalize_interval_goals', 10).value)
        self.relocalize_angular_speed = float(self.declare_parameter('relocalize_angular_speed', 0.5).value)
        self.relocalize_enabled = bool(self.declare_parameter('relocalize_enabled', True).value)
        self.backtrack_goal_tolerance_m = float(self.declare_parameter('backtrack_goal_tolerance_m', 0.35).value)
        self.lateral_centering_search_m = float(self.declare_parameter('lateral_centering_search_m', 0.8).value)
        self.allow_reverse_branch_goals = bool(self.declare_parameter('allow_reverse_branch_goals', True).value)
        self.reverse_branch_angle_threshold_deg = float(self.declare_parameter('reverse_branch_angle_threshold_deg', 135.0).value)
        self.blacklist_radius_m = float(self.declare_parameter('blacklist_radius_m', 0.5).value)
        self.max_failures_per_branch = int(self.declare_parameter('max_failures_per_branch', 2).value)
        self.max_backtrack_failures_per_node = int(self.declare_parameter('max_backtrack_failures_per_node', 2).value)
        self.max_goals = int(self.declare_parameter('max_goals', 80).value)

        # Guided Corridor Navigation (GCN) mode
        self.guided_corridor_mode = bool(self.declare_parameter('guided_corridor_mode', False).value)
        self.corridor_nav: Optional[CorridorNavigator] = None
        self.corridor_goal_step_m = float(self.declare_parameter('corridor_goal_step_m', 3.0).value)
        self.corridor_advance_tolerance_m = float(self.declare_parameter('corridor_advance_tolerance_m', 1.5).value)
        self.corridor_max_nav2_fails = int(self.declare_parameter('corridor_max_nav2_fails', 6).value)
        self.corridor_max_reactive = int(self.declare_parameter('corridor_max_reactive', 5).value)

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
        self.entry_direct_failed_time: Optional[Time] = None
        self.entry_direct_wait_start_time: Optional[Time] = None
        self.entry_direct_retry_count = 0
        self.entry_direct_max_retries = 2
        self.entry_direct_retry_after: Optional[Time] = None
        self.goal_success_count = 0
        self.goal_failure_count = 0
        self.nav2_failure_count = 0
        # Explore-goal retry: when Nav2 aborts (status=6) due to transient TF
        # issues, retry the same goal once after a short delay instead of
        # immediately marking the branch as BLOCKED.
        self.explore_retry_pending = False
        self.explore_retry_count = 0
        self.explore_max_retries = 1
        self.explore_retry_after: Optional[Time] = None
        self.explore_retry_target: Optional[tuple] = None
        self.explore_retry_start_node_id: Optional[int] = None
        self.explore_retry_branch = None  # BranchOption
        self.explore_retry_yaw: Optional[float] = None
        # Stagnation detector: when the robot spends too many goals in a small
        # area without progress, force a long-distance Dijkstra backtrack to
        # break out of local DFS loops in dense corridor networks.
        self.stagnation_recent_targets: list[tuple[float, float]] = []
        self.stagnation_window = 10       # number of recent goals to check
        self.stagnation_radius_m = 3.0    # if all N goals within this radius → stagnant
        self.stagnation_forced = False     # flag: force distant backtrack on next opportunity
        # Stuck escape: when Nav2 planner fails repeatedly from the same position
        # (all backtrack goals fail with status=6), send a short escape goal in
        # the most open direction to move the robot out of the SLAM artifact zone.
        self.stuck_escape_consecutive_failures = 0
        self.stuck_escape_last_position: Optional[tuple[float, float]] = None
        self.stuck_escape_active = False
        self.stuck_escape_max_consecutive = 3     # trigger after N failures from same area
        self.stuck_escape_position_radius_m = 0.5 # robot must not have moved more than this
        self.stuck_escape_attempts = 0
        self.stuck_escape_max_attempts = 3        # give up after N escape attempts
        self.stuck_escape_step_m = 1.0            # distance of escape goal
        # Frontier push: when DFS exhausts all branches, scan the SLAM map for
        # frontier cells (free cells adjacent to unknown) in the exit direction
        # and dispatch a goal to push the exploration boundary outward.
        self.frontier_push_active = False
        self.frontier_push_attempts = 0
        self.frontier_push_max_attempts = 25      # increased: micro-pushes need more attempts
        self.frontier_push_scan_radius_m = 25.0   # cover entire maze for frontier cells
        self.frontier_push_tried_targets: set[tuple[int, int]] = set()  # cell coords already tried
        # DFS retry: after frontier push exhausts all reachable candidates, reset
        # BLOCKED branches so DFS can retry previously-failed corridors.
        self.dfs_retry_cycle: int = 0
        self.dfs_retry_max_cycles: int = 5        # more cycles: frontier push skips bad targets quickly
        self.heading_push_angles_tried: list[float] = []  # angles tried by heading push (for dedup)
        # Eastward progress monitor: if DFS spends N goals without advancing
        # eastward by at least threshold_m, interrupt DFS and force a heading
        # push toward unexplored territory to the east/northeast.
        self.eastward_progress_max_x: float = 0.0       # farthest east (x) reached so far
        self.eastward_progress_goal_at_max: int = 0      # goal_count when max_x was last updated
        # Disabled by default (huge limit): forcing eastward on stagnation is a
        # maze-specific hack that fights legitimate west/south detours (e.g. C4).
        # With the novelty exploration bonus enabled, systematic exploration
        # replaces this heuristic.
        self.eastward_progress_stagnation_limit: int = int(
            self.declare_parameter('eastward_progress_stagnation_limit', 100000).value)
        self.eastward_progress_advance_threshold: float = 0.5  # min meters of eastward advance
        self.eastward_push_active: bool = False          # currently in forced eastward push
        self.eastward_push_attempts: int = 0
        self.eastward_push_max_attempts: int = 10        # max forced pushes before giving up
        # Reactive drive: bypasses Nav2 planner when SLAM drift creates
        # impassable walls in the global costmap.  Uses direct cmd_vel
        # with laser scan safety to drive through SLAM artifact walls.
        self.reactive_drive_active: bool = False
        self.reactive_drive_state: str = 'idle'  # idle|rotating|driving
        self.reactive_drive_target_yaw: float = 0.0
        self.reactive_drive_target_distance: float = 1.0
        self.reactive_drive_start_xy: Optional[tuple[float, float]] = None
        self.reactive_drive_count: int = 0       # total reactive drives this run
        self.reactive_drive_max_count: int = 8    # max reactive drives before giving up
        # Reactive-drive watchdog: a reactive drive must never run forever.
        # Without these, a wedged robot whose forward laser cone stays clear
        # publishes linear.x indefinitely (observed ~300s freeze at C5).
        self.reactive_drive_start_time = None            # sim-clock Time when drive started
        self.reactive_drive_max_seconds: float = 12.0    # hard cap per reactive drive
        self.reactive_drive_progress_ref_xy: Optional[tuple[float, float]] = None
        self.reactive_drive_progress_ref_time = None     # sim-clock Time of last real progress
        self.reactive_drive_no_progress_sec: float = 3.0  # wedge if no >0.1m move within this
        self.reactive_drive_backup_ref_xy: Optional[tuple[float, float]] = None
        self.reactive_drive_rot_ref_yaw: Optional[float] = None    # yaw at last rotation progress
        self.reactive_drive_rot_ref_time = None          # sim-clock Time of last rotation progress
        self.reactive_drive_no_rot_sec: float = 3.0      # jammed if can't rotate within this
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
        # Optimization 1: track local costmap failure duration for drift-aware
        # degraded dispatch.  When local_costmap_sufficient stays false for
        # longer than this grace period (seconds) during active exploration
        # (topology nodes exist), relax the gate to essentials-only so the
        # robot can continue navigating using the global SLAM map.
        self.local_costmap_fail_start_time: Optional[Time] = None
        self.local_costmap_degrade_grace_sec: float = 30.0
        self.local_costmap_degrade_triggered: bool = False
        # Optimization 2: re-localization rotation state
        self.relocalize_active: bool = False
        self.relocalize_start_yaw: Optional[float] = None
        self.relocalize_accumulated_rad: float = 0.0
        self.relocalize_last_yaw: Optional[float] = None
        self.relocalize_goals_at_last_rotation: int = 0

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
            exploration_bonus_weight=self.exploration_bonus_weight,
            distance_to_exit_weight=self.distance_to_exit_weight,
        )
        entrance = self.topology.find_or_create_node(self.entrance_x, self.entrance_y, node_type='junction')
        self.topology.visit_node(entrance.node_id)
        self.current_node_id = entrance.node_id

        # GCN: initialize corridor navigator when in guided mode
        if self.guided_corridor_mode:
            self.corridor_nav = CorridorNavigator()
            self.get_logger().info(
                'GCN: Guided Corridor Navigation mode enabled — %d corridors loaded'
                % len(self.corridor_nav.corridors)
            )
        # Publish maze boundary OccupancyGrid so Nav2's global costmap knows
        # about ALL maze walls from the start, preventing it from routing
        # through unmapped exterior space. REQUIRED in every mode: the global
        # costmap config has a maze_boundary_layer (StaticLayer subscribed to
        # /maze_boundary_map, transient-local). If nothing publishes that topic,
        # the layer never receives a map and the whole global costmap reports
        # "Costmap timed out waiting for update", aborting every Nav2 plan. This
        # used to be GCN-only, which silently broke all DFS (autonomous) planning.
        self._publish_maze_boundary_map()

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
        # Optimization 2: cmd_vel publisher for reactive drive.
        # Publish to /cmd_vel_nav (input of velocity_smoother → collision_monitor → /cmd_vel pipeline)
        # instead of /cmd_vel directly, because collision_monitor overrides /cmd_vel with zeros
        # when Nav2 controller is idle.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        # GCN: corridor path marker publisher for RViz visualization
        self.gcn_marker_pub = self.create_publisher(MarkerArray, '/maze/gcn_corridor_path', 10)
        self._gcn_markers_published = False
        self.timer = self.create_timer(1.0 / max(self.exploration_rate_hz, 0.1), self._explore_once)
        self.state_timer = self.create_timer(1.0 / max(self.publish_debug_state_hz, 0.1), self._publish_state)
        # Reactive drive needs a fast control loop: the main tick is 0.5 Hz, but
        # the velocity_smoother zeroes commands after velocity_timeout (1.0s), so
        # reactive rotation/drive at 0.5 Hz stutters and never completes. A 10 Hz
        # loop keeps fresh commands flowing so the robot actually moves.
        self.reactive_timer = self.create_timer(0.1, self._reactive_control_tick)

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

    def _publish_maze_boundary_map(self) -> None:
        """Publish a rectangular exterior boundary as a static OccupancyGrid.

        Instead of loading the maze image (whose wall pixels don't align with
        the corridor guide coordinates), this creates a simple rectangular
        boundary that blocks ALL area outside the maze perimeter.  Interior
        cells are set to -1 (unknown) so the SLAM static_layer provides the
        actual wall data; only the exterior is explicitly marked occupied (100).

        A narrow entrance gap at the bottom-left allows the robot to enter the
        maze from its starting position (~0, -2.5).  From inside the maze, this
        gap is a dead-end — Nav2 won't plan through it because there's nowhere
        to go beyond the gap.

        This prevents Nav2 from planning paths through unmapped exterior space,
        which is the root cause of the robot repeatedly escaping outside the maze.
        """
        try:
            import numpy as np
        except ImportError:
            self.get_logger().error(
                'GCN: numpy not available — cannot publish maze boundary map'
            )
            return

        # Grid parameters: 24m × 24m at ~0.067 m/pixel
        resolution = 24.0 / 359.0  # ~0.0669 m/pixel
        width, height = 360, 360
        origin_x, origin_y = -0.989, -2.975

        # Maze interior boundary (slightly larger than actual maze walls
        # which span x ≈ 0.95–21.07, y ≈ -1.04–19.09 in map coords).
        # Interior cells stay at -1 (unknown) — transparent to SLAM.
        maze_x_min = 0.0
        maze_x_max = 22.0
        maze_y_min = -1.5
        maze_y_max = 22.0

        # Entrance gap: narrow opening at bottom-left for robot to enter
        # from start position (~0, -2.5).  From inside the maze this is a
        # dead-end cul-de-sac — Nav2 won't plan through it.
        entrance_x_min = -0.5
        entrance_x_max = 3.0
        entrance_y_min = origin_y  # -2.975 (grid bottom)
        entrance_y_max = -0.5

        # Build coordinate arrays (OccupancyGrid: row 0 = lowest y)
        xs = np.arange(width) * resolution + origin_x
        ys = np.arange(height) * resolution + origin_y
        XX, YY = np.meshgrid(xs, ys)

        # Outside the maze interior rectangle
        outside = ((XX < maze_x_min) | (XX > maze_x_max) |
                   (YY < maze_y_min) | (YY > maze_y_max))

        # Inside the entrance gap (open corridor from start to entrance)
        in_entrance = ((XX >= entrance_x_min) & (XX <= entrance_x_max) &
                       (YY >= entrance_y_min) & (YY <= entrance_y_max))

        # Mark: exterior = occupied (100), interior = unknown (-1)
        grid = np.full((height, width), -1, dtype=np.int8)
        grid[outside & ~in_entrance] = 100

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = grid.flatten().tolist()

        # Publish with TRANSIENT_LOCAL so late-joining StaticLayer subscribers
        # receive the message even if they weren't subscribed yet.
        self._boundary_map_pub = self.create_publisher(
            OccupancyGrid, '/maze_boundary_map',
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )
        self._boundary_map_pub.publish(grid_msg)

        blocked = int((grid == 100).sum())
        self.get_logger().info(
            'GCN: published rectangular boundary map (%dx%d, res=%.4f, '
            '%d exterior cells blocked, entrance gap x=[%.1f,%.1f] y=[%.1f,%.1f])'
            % (width, height, resolution, blocked,
               entrance_x_min, entrance_x_max,
               entrance_y_min, entrance_y_max)
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
        if self.goal_active or self._goal_settle_active():
            pass  # no log spam during normal nav/settle
        else:
            self.get_logger().debug(
                '_explore_once active: mode=%s goal_count=%d goal_active=%s'
                % (self.mode, self.goal_count, self.goal_active)
            )

        if self.map_view is None:
            self.mode = WAIT_FOR_MAP
            self._publish_state()
            return

        self._map_jump_diagnostic()

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_state()
            return

        if self._exit_reached(robot_pose):
            self._enter_terminal_state(EXIT_REACHED, terminal_reason='exit_reached', robot_pose=robot_pose)
            self._publish_state()
            return

        if self.goal_active:
            # GCN proactive boundary enforcement: cancel Nav2 goal immediately
            # if the robot drifts outside the maze during corridor navigation.
            # Nav2 may plan paths through unmapped exterior space (treated as
            # free by the global costmap), routing the robot around the outside
            # of the maze walls.  Detect this early and abort before the robot
            # goes far outside.
            if (self.guided_corridor_mode
                    and self.active_goal_kind == 'gcn_corridor'
                    and robot_pose is not None):
                rx, ry = robot_pose[0], robot_pose[1]
                # Tight boundary: maze interior is roughly (-0.5, -0.5) to (22.5, 22.5).
                # Use conservative threshold so we catch drift early.
                outside_boundary = ry < -0.3 or rx < -0.3 or rx > 22.3 or ry > 22.3
                # Also check corridor deviation: if robot is >1.5m from the corridor
                # guide coordinate, Nav2 has diverged from the corridor path.
                corridor_deviation = False
                if self.corridor_nav is not None and not self.corridor_nav.finished:
                    c = self.corridor_nav.current
                    guide_axis, guide_val = c['guide']
                    deviation = abs((rx if guide_axis == 'x' else ry) - guide_val)
                    corridor_deviation = deviation > 1.8  # corridor is ~2m wide
                if outside_boundary or corridor_deviation:
                    self.get_logger().error(
                        'GCN BOUNDARY BREACH: robot at (%.1f,%.1f) outside=%s corridor_deviation=%s — '
                        'Nav2 routing outside maze. Canceling goal and re-entering.'
                        % (rx, ry, outside_boundary, corridor_deviation)
                    )
                    # Cancel Nav2 goal
                    if self.goal_handle is not None:
                        try:
                            self.goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    # Clear goal state WITHOUT going through _handle_goal_failure
                    # (which would set mode=SETTLING and trigger DFS code paths).
                    self.goal_active = False
                    self.goal_handle = None
                    self.goal_sent_time = None
                    self.active_goal_kind = 'none'
                    self.active_goal_target = None
                    self.active_goal_sequence_id = None
                    self.active_start_node_id = None
                    self.active_branch = None
                    self.active_goal_event_context = {}
                    self.goal_failure_count += 1
                    # Use reactive drive (cmd_vel) to push back into corridor 1
                    # heading north.  This bypasses Nav2 entirely, avoiding the
                    # exterior routing issue.
                    reentry_angle = math.pi / 2  # north
                    reentry_dist = 1.5
                    self.get_logger().info(
                        'GCN: re-entering maze via reactive drive %.0f° %.1fm'
                        % (math.degrees(reentry_angle), reentry_dist)
                    )
                    if self._start_reactive_drive(reentry_angle, distance=reentry_dist):
                        self.mode = CORRIDOR_REACTIVE
                    else:
                        # Reactive drive blocked — set GUIDED_CORRIDOR so GCN
                        # tick can dispatch a short re-entry goal next cycle.
                        self.mode = GUIDED_CORRIDOR
                    self._publish_state()
                    return

            self.mode = NAVIGATING if self.active_goal_kind != 'backtrack' else BACKTRACKING
            if self._goal_timed_out():
                elapsed_sec = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9 if self.goal_sent_time else -1
                self.get_logger().warn(
                    'maze explorer goal timed out; classifying active goal failure seq=%s elapsed=%.1fs timeout=%.1fs'
                    % (self.active_goal_sequence_id, elapsed_sec, self._effective_goal_timeout_sec())
                )
                self._handle_goal_failure(reason=GOAL_TIMEOUT)
            self._publish_state()
            return

        if self._goal_settle_active():
            self.mode = SETTLING
            self._publish_state()
            return

        if self.goal_count >= self.max_goals and not self.startup_warmup_no_dispatch and not (self.guided_corridor_mode and self.mode in (GUIDED_CORRIDOR, CORRIDOR_REACTIVE)):
            self._mark_exhausted('goal budget reached')
            self._publish_state()
            return

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('action server not ready, mode=WAIT_FOR_NAV2')
            self.mode = WAIT_FOR_NAV2
            self._publish_state()
            return

        gate = self._dispatch_entry_readiness_gate(robot_pose)
        self.last_dispatch_readiness_gate = gate

        # ENTRY_DIRECT readiness gate: requires Nav2 lifecycle, action server,
        # TF, and scan.  We do NOT require map_sufficient because the robot is
        # at the maze entrance where the SLAM map may never reach 70%/50%
        # thresholds (half the area behind the robot is outside the maze).
        # The global costmap typically has enough corridor data after Nav2
        # lifecycle activates for Navfn to plan the short (1.5m) straight-line
        # entry goal.
        #
        # Dispatch timeout: if essential checks never all pass (unlikely),
        # skip ENTRY_DIRECT after entry_direct_dispatch_timeout_sec and
        # proceed to regular exploration via the relaxed gate.
        if (self.entry_direct_enabled
                and not self.entry_direct_dispatched
                and (self.goal_count == 0
                     or (self.entry_direct_retry_count > 0
                         and self.entry_direct_retry_count <= self.entry_direct_max_retries))):
            nav2_active = bool(gate['checks'].get('nav2_lifecycle_active', False))
            action_ready = bool(gate['checks'].get('navigate_to_pose_action_ready', False))
            tf_ok = bool(gate['checks'].get('tf_sufficient', False))
            scan_ok = bool(gate['checks'].get('scan_sufficient', False))
            map_ok = bool(gate['checks'].get('map_sufficient', False))
            # Respect retry delay: after a failed ENTRY_DIRECT attempt, wait
            # 5 seconds before retrying to give the global costmap time to
            # populate with SLAM data.
            now = self.get_clock().now()
            if (self.entry_direct_retry_after is not None
                    and now < self.entry_direct_retry_after):
                remaining = (self.entry_direct_retry_after - now).nanoseconds / 1e9
                if self.entry_direct_retry_count == 1:  # log once per retry
                    self.get_logger().info(
                        'ENTRY_DIRECT: retry %d/%d waiting %.1fs for global costmap; nav2=%s action=%s tf=%s scan=%s map=%s'
                        % (self.entry_direct_retry_count, self.entry_direct_max_retries,
                           remaining, nav2_active, action_ready, tf_ok, scan_ok, map_ok)
                    )
                self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                self._publish_state()
                return
            self.entry_direct_retry_after = None  # delay elapsed
            # Reactive entry needs only TF + scan (no Nav2). Dispatch as soon as
            # those are ready so the robot drives in immediately, before Nav2
            # entry-goal failures can back it out the entrance.
            entry_ready = (tf_ok and scan_ok) if self.entry_direct_use_reactive \
                else (nav2_active and action_ready and tf_ok and scan_ok)
            if entry_ready:
                self.get_logger().info(
                    'ENTRY_DIRECT: readiness gate passed (reactive=%s nav2=%s action=%s tf=%s scan=%s map=%s); dispatching first goal'
                    % (self.entry_direct_use_reactive, nav2_active, action_ready, tf_ok, scan_ok, map_ok)
                )
                self.mode = ENTRY_DIRECT
                self._dispatch_entry_direct_goal(robot_pose)
                self._publish_state()
                return
            # Track when ENTRY_DIRECT waiting started
            now = self.get_clock().now()
            if self.entry_direct_wait_start_time is None:
                self.entry_direct_wait_start_time = now
            elapsed_wait = (now - self.entry_direct_wait_start_time).nanoseconds / 1e9
            # Check dispatch timeout
            if elapsed_wait >= self.entry_direct_dispatch_timeout_sec:
                self.get_logger().warn(
                    'ENTRY_DIRECT: dispatch timeout (%.1fs > %.1fs); essential checks never all passed. '
                    'Skipping ENTRY_DIRECT, falling back to regular exploration.'
                    % (elapsed_wait, self.entry_direct_dispatch_timeout_sec)
                )
                self.entry_direct_dispatched = True  # prevent retry
                # Set failed_time to wait start so the grace period (30s) is
                # already expired — the relaxed gate activates immediately.
                self.entry_direct_failed_time = self.entry_direct_wait_start_time
                # Fall through to relaxed gate below
            else:
                if self.goal_count == 0:
                    _tick = getattr(self, '_readiness_log_tick', 0) + 1
                    self._readiness_log_tick = _tick
                    if _tick % 10 == 1:
                        self.get_logger().info(
                            'ENTRY_DIRECT: readiness not yet passed (tick %d, %.0fs/%.0fs timeout); nav2=%s action=%s tf=%s scan=%s map=%s'
                            % (_tick, elapsed_wait, self.entry_direct_dispatch_timeout_sec,
                               nav2_active, action_ready, tf_ok, scan_ok, map_ok)
                        )
                self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                self._publish_state()
                return

        if not gate['passed']:
            # After ENTRY_DIRECT dispatched, relax readiness: only require
            # essential infrastructure (action server, TF, scan).  Skip
            # map_sufficient / local_costmap_sufficient which can fail in
            # narrow corridors during normal DFS exploration.
            # When topology is established (nodes exist), always use relaxed gate.
            # When ENTRY_DIRECT failed and no topology nodes exist yet, the map
            # is likely still unknown.  Wait up to ENTRY_DIRECT_FAILED_GRACE_SEC
            # for the full gate (map_sufficient).  If the grace period expires,
            # fall back to relaxed gate — classify_local_topology will attempt
            # analysis with whatever map data is available, which is better than
            # being stuck forever.
            use_relaxed = False
            if self.entry_direct_dispatched and len(self.topology.nodes) > 0:
                # Topology established: always use relaxed gate
                use_relaxed = True
            elif self.entry_direct_dispatched and len(self.topology.nodes) == 0:
                # ENTRY_DIRECT failed, no topology yet.  Check grace period.
                essential_keys = (
                    'navigate_to_pose_action_ready',
                    'tf_sufficient',
                    'scan_sufficient',
                )
                essential_ok = all(
                    gate['checks'].get(k, False) for k in essential_keys)
                if essential_ok:
                    grace_sec = 30.0
                    if self.entry_direct_failed_time is not None:
                        elapsed = (self.get_clock().now() - self.entry_direct_failed_time).nanoseconds / 1e9
                    else:
                        elapsed = 0.0
                    if elapsed >= grace_sec:
                        self.get_logger().warn(
                            'ENTRY_DIRECT failed grace period expired (%.1fs / %.1fs); '
                            'falling back to relaxed gate for first topology analysis'
                            % (elapsed, grace_sec)
                        )
                        use_relaxed = True
                    elif self.dispatch_readiness_first_pass_time is None:
                        # First time essential checks passed — log once
                        self.get_logger().info(
                            'Waiting for full readiness gate after ENTRY_DIRECT failure '
                            '(%.1fs / %.1fs grace period); map_sufficient=%s local_costmap_sufficient=%s'
                            % (elapsed, grace_sec,
                               gate['checks'].get('map_sufficient', False),
                               gate['checks'].get('local_costmap_sufficient', False)))
                    # Grace period not yet expired — keep waiting for full gate
                    if not use_relaxed:
                        self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                        self._publish_state()
                        return
                else:
                    self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                    self._publish_state()
                    return
            else:
                self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                self._publish_state()
                return

            if use_relaxed:
                essential_keys = (
                    'navigate_to_pose_action_ready',
                    'tf_sufficient',
                    'scan_sufficient',
                )
                essential_ok = all(
                    gate['checks'].get(k, False) for k in essential_keys)
                if essential_ok:
                    if self.dispatch_readiness_first_pass_time is None:
                        self.dispatch_readiness_first_pass_time = (
                            self.get_clock().now())
                else:
                    self.mode = WAIT_FOR_DISPATCH_ENTRY_READINESS
                    self._publish_state()
                    return
        if self.dispatch_readiness_first_pass_time is None:
            self.dispatch_readiness_first_pass_time = self.get_clock().now()

        if self.startup_warmup_no_dispatch:
            self.mode = STARTUP_WARMUP_NO_DISPATCH
            self._publish_state()
            return

        # Optimization 2: periodic re-localization rotation.  When due (every
        # N completed goals), perform an in-place 360° rotation to give SLAM
        # full scan coverage from all directions, reducing odometry drift.
        if self.relocalize_active:
            still_rotating = self._execute_relocalization_step(robot_pose)
            if still_rotating:
                self.mode = RELOCALIZING
                self._publish_state()
                return
            # Rotation just finished — fall through to normal dispatch
        elif self._should_start_relocalization():
            self.relocalize_active = True
            still_rotating = self._execute_relocalization_step(robot_pose)
            if still_rotating:
                self.mode = RELOCALIZING
                self._publish_state()
                return

        # Reactive drive: bypass Nav2 planner when SLAM drift walls cage
        # the robot.  Actual execution runs in the fast 10 Hz reactive timer
        # (_reactive_control_tick); here we just yield (no dispatch) while it
        # is active.  When it finishes, reactive_drive_active flips False and a
        # later tick falls through to normal dispatch.
        if self.reactive_drive_active:
            self.mode = RELOCALIZING  # reuse mode (no-dispatch state)
            self._publish_state()
            return

        # ── Guided Corridor Navigation (GCN) mode ──
        # When enabled, bypass the DFS topology analysis entirely and follow
        # the pre-computed corridor sequence from the 2D maze solution.
        if self.guided_corridor_mode and self.corridor_nav is not None:
            self._gcn_explore_tick(robot_pose)
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

    # ──────────────────────────────────────────────────────────────────
    # Guided Corridor Navigation (GCN) methods
    # ──────────────────────────────────────────────────────────────────

    def _target_in_maze(self, x: float, y: float) -> bool:
        """True if (x, y) is inside the maze interior bounds (map frame).

        Used to reject exploration/branch/frontier goals that fall outside the
        maze (e.g. west of the entrance opening at x=0), which would send the
        robot back out the entrance instead of deeper into the maze.
        """
        return (self.maze_bound_x_min <= x <= self.maze_bound_x_max
                and self.maze_bound_y_min <= y <= self.maze_bound_y_max)

    def _gcn_reactive_angle(self, robot_xy) -> float:
        """Heading for reactive drive: aim at the centerline-snapped waypoint.

        Using the pure compass direction (nav.current_angle) drives the robot
        straight along its current (possibly off-center) lateral position,
        which can keep it jammed against a side wall. The snapped goal point
        lies on the corridor centerline, so heading toward it naturally
        corrects lateral drift while still advancing.
        """
        nav = self.corridor_nav
        if nav is None:
            return 0.0
        goal = nav.current_goal(robot_xy, max_step=self.corridor_goal_step_m)
        dx = goal[0] - robot_xy[0]
        dy = goal[1] - robot_xy[1]
        if math.hypot(dx, dy) < 0.15:
            return nav.current_angle
        return math.atan2(dy, dx)

    def _gcn_explore_tick(self, robot_pose: RobotPose) -> None:
        """One tick of the GCN state machine.

        Flow:
        1. Publish RViz corridor path markers
        2. If all corridors done → check exit reached / mark exhausted
        3. Check if current corridor target reached → advance
        3. Dispatch next corridor goal via Nav2, or reactive drive if stuck
        """
        nav = self.corridor_nav
        if nav is None:
            return

        # Publish RViz corridor path markers (every tick for live target indicator)
        self._publish_gcn_markers()

        rx, ry = robot_pose[0], robot_pose[1]

        # Check if all corridors completed
        if nav.finished:
            dist = self._distance_to_exit(robot_pose[:2])
            if dist < self.exit_radius * 2:
                self.get_logger().info(
                    'GCN: all corridors completed, within %.1fm of exit — declaring EXIT_REACHED'
                    % dist
                )
                self._enter_terminal_state(EXIT_REACHED, terminal_reason='gcn_all_corridors_done', robot_pose=robot_pose)
                return
            else:
                self.get_logger().warn(
                    'GCN: all corridors completed but %.1fm from exit — marking exhausted'
                    % dist
                )
                self._mark_exhausted('gcn_corridors_done_but_not_at_exit')
                return

        # Check if current corridor/segment target reached → advance
        if nav.check_advance(robot_pose[:2], tolerance=self.corridor_advance_tolerance_m):
            self.get_logger().info(
                'GCN: reached target for %s at (%.1f,%.1f) — advancing'
                % (nav.corridor_label, rx, ry)
            )
            nav.advance()
            if nav.finished:
                # Will be picked up on next tick
                self.mode = GUIDED_CORRIDOR
                return
            nav.set_entry(robot_pose[:2])
            self.get_logger().info(
                'GCN: now on %s target=(%.1f,%.1f)'
                % (nav.corridor_label, nav.current_target[0], nav.current_target[1])
            )

        self.mode = GUIDED_CORRIDOR

        # If corridor is exhausted (too many failures), try reactive drive
        if nav.corridor_exhausted:
            if not self.reactive_drive_active and not self.goal_active:
                angle = self._gcn_reactive_angle(robot_pose[:2])
                dist = min(1.5, nav.dist_to_current_target(robot_pose[:2]))
                self.get_logger().warn(
                    'GCN: corridor %s exhausted (nav2_fail=%d reactive=%d) — forcing reactive drive %.0f° %.1fm'
                    % (nav.corridor_label, nav.nav2_fail_count, nav.reactive_count,
                       math.degrees(angle), dist)
                )
                if self._start_reactive_drive_gcn(angle, distance=dist):
                    nav.record_reactive_drive()
                    self.mode = CORRIDOR_REACTIVE
                    return
                else:
                    # All angles blocked — try perpendicular nudge to reposition
                    perp = angle + math.pi / 2  # try left
                    if self._start_reactive_drive(perp, distance=0.5,
                                                  min_clearance=0.5):
                        self.get_logger().info(
                            'GCN: lateral nudge %.0f° to escape wall'
                            % math.degrees(perp)
                        )
                        self.mode = CORRIDOR_REACTIVE
                        return
                    perp = angle - math.pi / 2  # try right
                    if self._start_reactive_drive(perp, distance=0.5,
                                                  min_clearance=0.5):
                        self.get_logger().info(
                            'GCN: lateral nudge %.0f° to escape wall'
                            % math.degrees(perp)
                        )
                        self.mode = CORRIDOR_REACTIVE
                        return
                    self.get_logger().error(
                        'GCN: reactive drive blocked for %s at ALL angles + lateral — cannot proceed'
                        % nav.corridor_label
                    )
                    self._mark_exhausted('gcn_corridor_stuck')
                    return

        # Try reactive drive if Nav2 has failed multiple times
        if nav.should_try_reactive:
            if not self.reactive_drive_active and not self.goal_active:
                angle = self._gcn_reactive_angle(robot_pose[:2])
                dist = min(1.5, nav.dist_to_current_target(robot_pose[:2]))
                self.get_logger().info(
                    'GCN: Nav2 failed %d times for %s — trying reactive drive %.0f° %.1fm'
                    % (nav.nav2_fail_count, nav.corridor_label, math.degrees(angle), dist)
                )
                if self._start_reactive_drive_gcn(angle, distance=dist):
                    nav.record_reactive_drive()
                    self.mode = CORRIDOR_REACTIVE
                    return
                else:
                    # Reactive drive blocked — count the failed attempt so
                    # corridor_exhausted can eventually fire instead of looping
                    nav.record_reactive_drive()

        # Dispatch corridor goal via Nav2 — ONLY if no goal currently active
        if self.goal_active:
            # A Nav2 goal is already in flight; wait for it to complete
            return

        # Maze boundary guard: if robot drifted outside the maze, it means
        # Nav2 routed around the exterior walls (SLAM hasn't mapped them yet).
        # Cancel and force re-entry through the entrance corridor.
        if ry < -1.5 or rx < -1.0 or rx > 22.0 or ry > 22.0:
            self.get_logger().error(
                'GCN: robot OUTSIDE maze boundary at (%.1f,%.1f) — '
                'Nav2 likely routed along exterior. Re-entering via entrance.'
                % (rx, ry)
            )
            reentry_goal = (self.entrance_x + 1.0, max(0.5, min(ry, 2.0)))
            self._send_goal(reentry_goal, yaw=math.pi / 2,
                            goal_kind='gcn_corridor', skip_two_step_staging=True)
            return

        goal = nav.current_goal(robot_pose[:2], max_step=self.corridor_goal_step_m)
        yaw = nav.current_angle
        self.get_logger().info(
            'GCN: dispatching %s goal=(%.1f,%.1f) yaw=%.0f° %s'
            % (nav.corridor_label, goal[0], goal[1], math.degrees(yaw), nav.summary())
        )
        self._send_goal(goal, yaw=yaw, goal_kind='gcn_corridor', skip_two_step_staging=True)

    def _gcn_handle_goal_success(self, robot_pose: RobotPose) -> None:
        """Handle a successful GCN corridor goal."""
        nav = self.corridor_nav
        if nav is None:
            return
        self.get_logger().info(
            'GCN: goal succeeded for %s at (%.1f,%.1f) progress=%.1fm'
            % (nav.corridor_label, robot_pose[0], robot_pose[1],
               nav.relative_progress(robot_pose[:2]))
        )

    def _gcn_handle_goal_failure(self, robot_pose: RobotPose, reason: str) -> None:
        """Handle a failed GCN corridor goal."""
        nav = self.corridor_nav
        if nav is None:
            return
        nav.record_nav2_failure()
        self.get_logger().warn(
            'GCN: goal FAILED for %s reason=%s nav2_fail=%d at (%.1f,%.1f)'
            % (nav.corridor_label, reason, nav.nav2_fail_count,
               robot_pose[0], robot_pose[1])
        )

    def _gcn_handle_reactive_complete(self, robot_pose: RobotPose) -> None:
        """Handle reactive drive completion in GCN mode."""
        nav = self.corridor_nav
        if nav is None:
            return
        self.get_logger().info(
            'GCN: reactive drive completed for %s at (%.1f,%.1f) progress=%.1fm'
            % (nav.corridor_label, robot_pose[0], robot_pose[1],
               nav.relative_progress(robot_pose[:2]))
        )

    # ── GCN RViz Marker Publisher ──────────────────────────────────

    # RGBA colors for each of the 10 corridors (matching static visualization)
    _GCN_CORRIDOR_COLORS = [
        (1.0, 0.0, 0.0, 0.9),    # C1 red
        (1.0, 0.53, 0.0, 0.9),   # C2 orange
        (1.0, 1.0, 0.0, 0.9),    # C3 yellow
        (0.0, 1.0, 0.0, 0.9),    # C4 green
        (0.0, 1.0, 1.0, 0.9),    # C5 cyan
        (0.0, 0.53, 1.0, 0.9),   # C6 blue
        (0.53, 0.0, 1.0, 0.9),   # C7 purple
        (1.0, 0.0, 1.0, 0.9),    # C8 magenta
        (1.0, 0.27, 0.53, 0.9),  # C9 pink
        (0.53, 1.0, 0.27, 0.9),  # C10 lime
        (0.8, 0.8, 0.0, 0.9),    # C11 olive
        (0.0, 0.8, 0.53, 0.9),   # C12 teal
        (0.53, 0.53, 1.0, 0.9),  # C13 periwinkle
        (0.8, 0.27, 0.0, 0.9),   # C14 rust
        (0.27, 0.8, 0.8, 0.9),   # C15 aquamarine
        (0.8, 0.0, 0.53, 0.9),   # C16 crimson
        (0.4, 0.8, 0.0, 0.9),    # C17 chartreuse
        (0.0, 0.4, 0.8, 0.9),    # C18 steel blue
    ]

    def _publish_gcn_markers(self) -> None:
        """Publish the GCN corridor path as RViz MarkerArray.

        Publishes:
        - One LINE_STRIP per corridor (colored by corridor index)
        - SPHERE markers at each corridor waypoint (start/end + sub-segment joints)
        - A large SPHERE for the maze exit
        - Text labels for each corridor

        Markers are published once and latched via a DELETE_ALL + re-publish
        pattern to avoid accumulation.
        """
        nav = self.corridor_nav
        if nav is None:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker namespace to avoid collisions
        ns = 'gcn_corridors'
        base_id = 0

        # ── Entrance marker ──
        entrance = Marker()
        entrance.header.frame_id = self.map_frame
        entrance.header.stamp = now
        entrance.ns = ns
        entrance.id = base_id
        entrance.type = Marker.SPHERE
        entrance.action = Marker.ADD
        entrance.pose.position.x = self.entrance_x
        entrance.pose.position.y = self.entrance_y
        entrance.pose.position.z = 0.1
        entrance.pose.orientation.w = 1.0
        entrance.scale.x = 0.5
        entrance.scale.y = 0.5
        entrance.scale.z = 0.5
        entrance.color.r = 0.0
        entrance.color.g = 1.0
        entrance.color.b = 0.0
        entrance.color.a = 0.9
        entrance.lifetime.sec = 0  # persistent
        entrance.text = 'ENTER'
        marker_array.markers.append(entrance)
        base_id += 1

        # ── Entrance text label ──
        et = Marker()
        et.header.frame_id = self.map_frame
        et.header.stamp = now
        et.ns = ns
        et.id = base_id
        et.type = Marker.TEXT_VIEW_FACING
        et.action = Marker.ADD
        et.pose.position.x = self.entrance_x
        et.pose.position.y = self.entrance_y
        et.pose.position.z = 0.6
        et.pose.orientation.w = 1.0
        et.scale.z = 0.4
        et.color.r = 0.0
        et.color.g = 1.0
        et.color.b = 0.0
        et.color.a = 0.9
        et.text = 'ENTER'
        et.lifetime.sec = 0
        marker_array.markers.append(et)
        base_id += 1

        # ── Exit marker ──
        exit_m = Marker()
        exit_m.header.frame_id = self.map_frame
        exit_m.header.stamp = now
        exit_m.ns = ns
        exit_m.id = base_id
        exit_m.type = Marker.SPHERE
        exit_m.action = Marker.ADD
        exit_m.pose.position.x = self.exit_x
        exit_m.pose.position.y = self.exit_y
        exit_m.pose.position.z = 0.1
        exit_m.pose.orientation.w = 1.0
        exit_m.scale.x = 0.6
        exit_m.scale.y = 0.6
        exit_m.scale.z = 0.6
        exit_m.color.r = 1.0
        exit_m.color.g = 0.0
        exit_m.color.b = 0.0
        exit_m.color.a = 1.0
        exit_m.lifetime.sec = 0
        marker_array.markers.append(exit_m)
        base_id += 1

        # ── Exit text label ──
        xt = Marker()
        xt.header.frame_id = self.map_frame
        xt.header.stamp = now
        xt.ns = ns
        xt.id = base_id
        xt.type = Marker.TEXT_VIEW_FACING
        xt.action = Marker.ADD
        xt.pose.position.x = self.exit_x
        xt.pose.position.y = self.exit_y
        xt.pose.position.z = 0.7
        xt.pose.orientation.w = 1.0
        xt.scale.z = 0.5
        xt.color.r = 1.0
        xt.color.g = 0.0
        xt.color.b = 0.0
        xt.color.a = 1.0
        xt.text = 'EXIT (%.1f,%.1f)' % (self.exit_x, self.exit_y)
        xt.lifetime.sec = 0
        marker_array.markers.append(xt)
        base_id += 1

        # ── Per-corridor markers ──
        for i, c in enumerate(nav.corridors):
            color = self._GCN_CORRIDOR_COLORS[i] if i < len(self._GCN_CORRIDOR_COLORS) else (1.0, 1.0, 1.0, 0.9)
            corridor_idx = c['id']

            # Build waypoint list for this corridor
            waypoints = [c['start']]
            if 'sub_segs' in c and c['sub_segs']:
                for seg in c['sub_segs']:
                    waypoints.append(seg[2])  # end point of each sub-segment
            else:
                waypoints.append(c['end'])

            # ── Line strip for corridor path ──
            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = now
            line.ns = ns
            line.id = base_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.15  # line width
            line.color.r = color[0]
            line.color.g = color[1]
            line.color.b = color[2]
            line.color.a = color[3]
            line.lifetime.sec = 0
            for wp in waypoints:
                p = RosPoint()
                p.x = float(wp[0])
                p.y = float(wp[1])
                p.z = 0.05
                line.points.append(p)
            marker_array.markers.append(line)
            base_id += 1

            # ── Sphere at start waypoint ──
            sph_start = Marker()
            sph_start.header.frame_id = self.map_frame
            sph_start.header.stamp = now
            sph_start.ns = ns
            sph_start.id = base_id
            sph_start.type = Marker.SPHERE
            sph_start.action = Marker.ADD
            sph_start.pose.position.x = float(c['start'][0])
            sph_start.pose.position.y = float(c['start'][1])
            sph_start.pose.position.z = 0.1
            sph_start.pose.orientation.w = 1.0
            sph_start.scale.x = 0.3
            sph_start.scale.y = 0.3
            sph_start.scale.z = 0.3
            sph_start.color.r = color[0]
            sph_start.color.g = color[1]
            sph_start.color.b = color[2]
            sph_start.color.a = 1.0
            sph_start.lifetime.sec = 0
            marker_array.markers.append(sph_start)
            base_id += 1

            # ── Sphere at end waypoint ──
            sph_end = Marker()
            sph_end.header.frame_id = self.map_frame
            sph_end.header.stamp = now
            sph_end.ns = ns
            sph_end.id = base_id
            sph_end.type = Marker.SPHERE
            sph_end.action = Marker.ADD
            sph_end.pose.position.x = float(c['end'][0])
            sph_end.pose.position.y = float(c['end'][1])
            sph_end.pose.position.z = 0.1
            sph_end.pose.orientation.w = 1.0
            sph_end.scale.x = 0.3
            sph_end.scale.y = 0.3
            sph_end.scale.z = 0.3
            sph_end.color.r = color[0]
            sph_end.color.g = color[1]
            sph_end.color.b = color[2]
            sph_end.color.a = 1.0
            sph_end.lifetime.sec = 0
            marker_array.markers.append(sph_end)
            base_id += 1

            # ── Text label at corridor midpoint ──
            mid_x = (c['start'][0] + c['end'][0]) / 2.0
            mid_y = (c['start'][1] + c['end'][1]) / 2.0
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = now
            label.ns = ns
            label.id = base_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(mid_x)
            label.pose.position.y = float(mid_y)
            label.pose.position.z = 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = color[0]
            label.color.g = color[1]
            label.color.b = color[2]
            label.color.a = 1.0
            label.text = 'C%d:%s %.1fm' % (corridor_idx, c['dir'], c.get('len', 0))
            label.lifetime.sec = 0
            marker_array.markers.append(label)
            base_id += 1

        # ── Current corridor progress indicator (updated each tick) ──
        if not nav.finished:
            cur = nav.current
            cur_color = self._GCN_CORRIDOR_COLORS[nav.idx] if nav.idx < len(self._GCN_CORRIDOR_COLORS) else (1, 1, 1, 1)
            target = nav.current_target
            prog = Marker()
            prog.header.frame_id = self.map_frame
            prog.header.stamp = now
            prog.ns = ns
            prog.id = base_id
            prog.type = Marker.SPHERE
            prog.action = Marker.ADD
            prog.pose.position.x = float(target[0])
            prog.pose.position.y = float(target[1])
            prog.pose.position.z = 0.3
            prog.pose.orientation.w = 1.0
            prog.scale.x = 0.45
            prog.scale.y = 0.45
            prog.scale.z = 0.45
            # Pulsing yellow target marker
            prog.color.r = 1.0
            prog.color.g = 1.0
            prog.color.b = 0.0
            prog.color.a = 0.95
            prog.lifetime.sec = 0
            marker_array.markers.append(prog)
            base_id += 1

            # Target text
            tgt_label = Marker()
            tgt_label.header.frame_id = self.map_frame
            tgt_label.header.stamp = now
            tgt_label.ns = ns
            tgt_label.id = base_id
            tgt_label.type = Marker.TEXT_VIEW_FACING
            tgt_label.action = Marker.ADD
            tgt_label.pose.position.x = float(target[0])
            tgt_label.pose.position.y = float(target[1])
            tgt_label.pose.position.z = 0.8
            tgt_label.pose.orientation.w = 1.0
            tgt_label.scale.z = 0.35
            tgt_label.color.r = 1.0
            tgt_label.color.g = 1.0
            tgt_label.color.b = 0.0
            tgt_label.color.a = 1.0
            tgt_label.text = 'TARGET %s' % nav.corridor_label
            tgt_label.lifetime.sec = 0
            marker_array.markers.append(tgt_label)
            base_id += 1

        self.gcn_marker_pub.publish(marker_array)

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

        # Optimization 1: drift-aware local costmap degradation.
        # When local_costmap_sufficient is false for > grace period and
        # topology is established (nodes exist), the local costmap is likely
        # suffering from odom drift rather than genuine unexplored terrain.
        # Remove it from blocking reasons so navigation continues using the
        # global SLAM map (which uses the 'map' frame and is drift-free).
        now = self.get_clock().now()
        lcs_ok = checks.get('local_costmap_sufficient', False)
        has_topology = len(self.topology.nodes) > 0
        if not lcs_ok and has_topology:
            if self.local_costmap_fail_start_time is None:
                self.local_costmap_fail_start_time = now
            elapsed = (now - self.local_costmap_fail_start_time).nanoseconds / 1e9
            if elapsed >= self.local_costmap_degrade_grace_sec:
                if not self.local_costmap_degrade_triggered:
                    self.get_logger().warn(
                        'Optimization1: local_costmap_sufficient=false for %.0fs (>%.0fs grace) '
                        'with %d topology nodes — removing local_costmap from dispatch gate '
                        '(likely odom drift; global SLAM map is still usable)'
                        % (elapsed, self.local_costmap_degrade_grace_sec, len(self.topology.nodes))
                    )
                    self.local_costmap_degrade_triggered = True
                blocking_reasons = [r for r in blocking_reasons if r != 'local_costmap_sufficient']
        else:
            # Reset tracker when costmap recovers or no topology yet
            self.local_costmap_fail_start_time = None
            if lcs_ok:
                self.local_costmap_degrade_triggered = False

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

        if self.entry_direct_use_reactive:
            # Drive straight into the maze with the reactive controller (no
            # Nav2, so no failed-goal BackUp recovery to push us back out).
            started = self._start_reactive_drive(
                self.entrance_yaw, distance=d, min_clearance=0.8)
            self.get_logger().info(
                'ENTRY_DIRECT: reactive straight-line entry %s from (%.2f,%.2f) '
                '%.1fm @ %.0f deg' % ('started' if started else 'BLOCKED',
                                      robot_pose[0], robot_pose[1], d,
                                      math.degrees(self.entrance_yaw))
            )
            if started:
                return
            # If reactive is somehow blocked, fall through to a Nav2 goal.
            self.get_logger().warn(
                'ENTRY_DIRECT: reactive entry blocked; falling back to Nav2 goal')

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

        # Stuck escape: when Nav2 planner fails repeatedly from the same position,
        # dispatch a short goal in the most open direction to move the robot out
        # of the SLAM artifact zone before resuming normal DFS.
        if self.stuck_escape_active:
            local_esc = classify_local_topology(
                self.map_view,
                robot_pose,
                angle_step_deg=self.branch_angle_step_deg,
                lookahead_m=self.branch_lookahead_m,
                clearance_radius_m=self.clearance_radius_m,
                min_open_distance_m=self.min_open_distance_m,
            )
            if local_esc.open_directions:
                # Pick the most open direction (longest clear distance)
                best_dir = max(local_esc.open_directions, key=lambda d: d.distance_m)
                step = min(self.stuck_escape_step_m, best_dir.distance_m * 0.6)
                if step >= 0.3:
                    escape_x = robot_pose[0] + step * math.cos(best_dir.angle_rad)
                    escape_y = robot_pose[1] + step * math.sin(best_dir.angle_rad)
                    self.get_logger().warn(
                        'STUCK ESCAPE: dispatching %.2fm goal in direction %.1f° '
                        '(%.3f, %.3f) attempt %d/%d'
                        % (step, math.degrees(best_dir.angle_rad),
                           escape_x, escape_y,
                           self.stuck_escape_attempts, self.stuck_escape_max_attempts)
                    )
                    self._send_goal(
                        (escape_x, escape_y),
                        yaw=best_dir.angle_rad,
                        goal_kind='stuck_escape',
                    )
                    return
            # No open direction or step too short — cannot escape
            self.get_logger().error(
                'STUCK ESCAPE: no viable open direction (open_count=%d); marking exhausted'
                % len(local_esc.open_directions)
            )
            self._mark_exhausted('stuck: Nav2 planner fails from this position and no open escape direction')
            return

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

        # Compute corrected junction position using Chebyshev center algorithm.
        # This places the topology node at the geometric intersection center
        # rather than the robot's position in the corridor entrance.
        if node_type == 'junction' and local.kind == JUNCTION:
            junction_xy = compute_junction_center(
                robot_xy=(robot_pose[0], robot_pose[1]),
                open_directions=local.open_directions,
            )
        else:
            junction_xy = (robot_pose[0], robot_pose[1])

        node = self.topology.find_or_create_node(junction_xy[0], junction_xy[1], node_type=node_type)
        self.current_node_id = node.node_id
        self.topology.visit_node(node.node_id)

        junction_center_offset_m = math.hypot(junction_xy[0] - robot_pose[0], junction_xy[1] - robot_pose[1])
        if junction_center_offset_m > 0.05:
            self.get_logger().info(
                'junction center corrected: (%.3f, %.3f) -> (%.3f, %.3f) offset=%.3fm'
                % (robot_pose[0], robot_pose[1], junction_xy[0], junction_xy[1], junction_center_offset_m)
            )

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
                    self._send_goal(chosen.target_xy, yaw=chosen.angle_rad, goal_kind='explore', skip_two_step_staging=True)
                    return
            self.topology.mark_dead_end(node.node_id, incoming_edge_id=self.active_edge_id)
            # Dijkstra smart backtracking: find nearest unexplored junction via shortest known path.
            # Falls back to stack-based backtracking if topology graph is disconnected.
            backtrack_target = self._dijkstra_backtrack_target(node.node_id)
            if backtrack_target is None:
                self._mark_exhausted('dead end reached and no unexplored junction reachable via known paths')
                return
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
            if target is not None and not self._target_in_maze(target[0], target[1]):
                # Lateral centering pushed the target outside the maze (common
                # at the entrance, where free space opens west into the entrance
                # chamber and biases the centroid). Fall back to the straight
                # projection along the direction, which stays in the corridor.
                straight = (
                    robot_pose[0] + math.cos(direction.angle_rad) * self.branch_goal_step_m,
                    robot_pose[1] + math.sin(direction.angle_rad) * self.branch_goal_step_m,
                )
                if self._target_in_maze(straight[0], straight[1]):
                    self.get_logger().info(
                        'DFS: centered target (%.2f,%.2f) out of maze; using '
                        'straight projection (%.2f,%.2f) at %.0f deg'
                        % (target[0], target[1], straight[0], straight[1],
                           math.degrees(direction.angle_rad))
                    )
                    target = straight
                else:
                    self.get_logger().info(
                        'DFS: rejecting out-of-maze branch target (%.2f, %.2f) '
                        'at angle %.0f deg' % (target[0], target[1],
                                               math.degrees(direction.angle_rad))
                    )
                    target = None
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

        # Eastward progress monitor: if DFS has spent many goals without
        # advancing eastward, interrupt DFS and force a heading push toward
        # unexplored territory.  This prevents DFS from getting stuck exploring
        # junctions with many branches in the same western area.
        if self._check_eastward_stagnation(robot_pose):
            return

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
            self._send_goal(chosen.target_xy, yaw=chosen.angle_rad, goal_kind='explore', skip_two_step_staging=True)
            return

        backtrack_target = self._dijkstra_backtrack_target(node.node_id)
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

    def _dijkstra_backtrack_target(self, current_node_id: int):
        """Find unexplored junction via Dijkstra, with stack-based fallback.

        Normally uses dijkstra_nearest_unexplored() for shortest-path target.
        When stagnation is detected (robot looping in a small area), switches to
        dijkstra_farthest_unexplored() to force a long-distance backtrack that
        breaks out of local DFS loops in dense corridor networks.
        Falls back to next_backtrack_target() if the topology graph is disconnected
        or Dijkstra finds no reachable unexplored junction.
        """
        # Check for stagnation before selecting backtrack target
        if self._is_stagnant():
            self.stagnation_forced = True

        if self.stagnation_forced:
            robot_pose = self._lookup_robot_pose()
            current_xy = (robot_pose[0], robot_pose[1]) if robot_pose else (0.0, 0.0)
            exit_xy = (self.exit_x, self.exit_y)
            far_result = self.topology.dijkstra_farthest_unexplored(
                current_node_id, exit_xy=exit_xy, current_xy=current_xy,
            )
            if far_result is not None:
                path, target = far_result
                self.get_logger().warn(
                    'STAGNATION backtrack (exit-directed): forced backtrack to junction %d at (%.3f, %.3f) via %d-hop path'
                    % (target.node_id, target.x, target.y, len(path) - 1)
                )
                self.stagnation_forced = False
                self.stagnation_recent_targets.clear()
                return target
            # Dijkstra found no reachable unexplored junction — try farthest on visit stack
            farthest = self.topology.farthest_stack_backtrack_target(current_node_id, current_xy)
            if farthest is not None:
                self.get_logger().warn(
                    'STAGNATION backtrack (stack farthest): forced backtrack to junction %d at (%.3f, %.3f)'
                    % (farthest.node_id, farthest.x, farthest.y)
                )
                self.stagnation_forced = False
                self.stagnation_recent_targets.clear()
                return farthest
            self.stagnation_forced = False

        dijkstra_result = self.topology.dijkstra_nearest_unexplored(current_node_id)
        if dijkstra_result is not None:
            path, target = dijkstra_result
            self.get_logger().info(
                'Dijkstra backtrack: selected junction %d at (%.3f, %.3f) via %d-hop path'
                % (target.node_id, target.x, target.y, len(path) - 1)
            )
            return target
        # Fallback: stack-based backtracking for disconnected topology or early exploration
        fallback = self.topology.next_backtrack_target(current_node_id)
        if fallback is not None:
            self.get_logger().info(
                'Dijkstra found no path; fallback stack-based backtrack to junction %d at (%.3f, %.3f)'
                % (fallback.node_id, fallback.x, fallback.y)
            )
        return fallback

    def _is_stagnant(self) -> bool:
        """Detect if the robot is stuck in a local DFS loop.

        Returns True if the last stagnation_window explore goals all landed
        within stagnation_radius_m of their centroid.
        """
        targets = self.stagnation_recent_targets
        if len(targets) < self.stagnation_window:
            return False
        # Compute centroid
        cx = sum(t[0] for t in targets) / len(targets)
        cy = sum(t[1] for t in targets) / len(targets)
        # Check if all targets are within radius of centroid
        for tx, ty in targets:
            dist = math.hypot(tx - cx, ty - cy)
            if dist > self.stagnation_radius_m:
                return False
        self.get_logger().warn(
            'STAGNATION detected: %d recent explore goals within %.1fm of centroid (%.1f, %.1f)'
            % (len(targets), self.stagnation_radius_m, cx, cy)
        )
        return True

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
            key=lambda branch: branch.score_for_exit(node.xy, (self.exit_x, self.exit_y), self.exit_bias_weight, distance_to_exit_weight=self.distance_to_exit_weight),
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
        score = branch.score_for_exit(node_xy, (self.exit_x, self.exit_y), self.exit_bias_weight, distance_to_exit_weight=self.distance_to_exit_weight)
        return {
            'score': float(score),
            'target_exit_dist': float(target_exit_dist),
            'exit_progress_delta_m': float(source_exit_dist - target_exit_dist),
            'heading_alignment': float(heading_alignment),
            'exit_bias_weight': float(self.exit_bias_weight),
            'distance_to_exit_weight': float(self.distance_to_exit_weight),
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
        # Recovery goals (stuck_escape, frontier_push) do not consume exploration budget
        if goal_kind not in ('stuck_escape', 'frontier_push', 'eastward_push'):
            self.goal_count += 1
        # Track eastward progress ONLY for actual movement goals (explore, entry_direct),
        # NOT for speculative pushes (frontier_push, eastward_push) that may fail.
        # If a speculative push succeeds, the next DFS explore goal from the new
        # position will naturally update the progress tracker.
        if goal_kind in ('explore', 'entry_direct', 'backtrack', 'stuck_escape', 'near_exit_micro_goal'):
            if target_xy[0] > self.eastward_progress_max_x + self.eastward_progress_advance_threshold:
                old_max = self.eastward_progress_max_x
                self.eastward_progress_max_x = target_xy[0]
                self.eastward_progress_goal_at_max = self.goal_count
                self.get_logger().info(
                    'EASTWARD PROGRESS: new max_x=%.1f (was %.1f) at goal #%d'
                    % (self.eastward_progress_max_x, old_max, self.goal_count)
                )
        # Track recent explore targets for stagnation detection
        if goal_kind in ('explore', 'entry_direct'):
            self.stagnation_recent_targets.append((float(target_xy[0]), float(target_xy[1])))
            if len(self.stagnation_recent_targets) > self.stagnation_window:
                self.stagnation_recent_targets = self.stagnation_recent_targets[-self.stagnation_window:]
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
            self.get_logger().warn('goal_response STALE seq=%d active_seq=%s' % (sequence_id, self.active_goal_sequence_id))
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
        try:
            self._goal_result_callback_impl(future, sequence_id)
        except Exception as exc:
            import traceback
            self.get_logger().error(
                'FATAL EXCEPTION in _goal_result_callback seq=%d: %s\n%s'
                % (sequence_id, exc, traceback.format_exc())
            )

    def _goal_result_callback_impl(self, future, sequence_id: int) -> None:
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
            # Clear any stale retry state on success
            self.explore_retry_count = 0
            self.explore_retry_pending = False
            self._handle_goal_success(result_status=status)
        else:
            # Explore-goal retry: if status=6 (ABORTED) and goal is an explore
            # goal that hasn't been retried yet, retry once after a short delay.
            # This handles transient TF buffer clears that cause Nav2 to abort
            # before the transform recovers.
            if (status == GoalStatus.STATUS_ABORTED
                    and self.active_goal_kind == 'explore'
                    and self.explore_retry_count < self.explore_max_retries):
                self.explore_retry_count += 1
                self.explore_retry_pending = True
                self.explore_retry_after = self.get_clock().now() + Duration(seconds=3.0)
                self.explore_retry_target = self.active_goal_target
                self.explore_retry_start_node_id = self.active_start_node_id
                self.explore_retry_branch = self.active_branch
                self.explore_retry_yaw = self.active_goal_event_context.get('target_yaw')
                self.get_logger().warn(
                    'explore goal ABORTED (status=6 seq=%d); retry %d/%d in 3.0s '
                    'target=(%.3f, %.3f) branch_yaw=%.3f'
                    % (sequence_id, self.explore_retry_count, self.explore_max_retries,
                       self.active_goal_target[0] if self.active_goal_target else 0,
                       self.active_goal_target[1] if self.active_goal_target else 0,
                       self.explore_retry_yaw or 0)
                )
                # Reset goal state WITHOUT marking branch as failed
                self.goal_active = False
                self.goal_handle = None
                self.goal_sent_time = None
                self.active_goal_kind = 'none'
                self.active_goal_target = None
                self.active_goal_sequence_id = None
                self.active_start_node_id = None
                self.active_branch = None
                self.active_goal_event_context = {}
                self._start_goal_settle_cooldown()
                self.mode = SETTLING
                return
            self.get_logger().warn('maze explorer goal failed with status=%s seq=%d' % (status, sequence_id))
            self._handle_goal_failure(reason=BLOCKED_NAV2, result_status=status)

    def _handle_goal_success(self, result_status: int = GoalStatus.STATUS_SUCCEEDED) -> None:
        self.last_completed_goal_sequence_id = self.active_goal_sequence_id
        completed_goal_kind = self.active_goal_kind
        # Clear stuck-escape state on any successful goal — the robot can
        # navigate from this position, so it's no longer stuck.
        if completed_goal_kind != 'stuck_escape':
            if self.stuck_escape_consecutive_failures > 0:
                self.stuck_escape_consecutive_failures = 0
                self.stuck_escape_last_position = None
                self.stuck_escape_active = False
                self.stuck_escape_attempts = 0
        self._publish_goal_event('success', result_status=int(result_status), result_reason='succeeded')
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.goal_success_count += 1
        if completed_goal_kind == 'entry_direct':
            self.get_logger().info(
                'ENTRY_DIRECT goal succeeded; transitioning to AT_NODE_ANALYZE for normal DFS exploration'
            )
        # GCN: track corridor goal success
        if completed_goal_kind == 'gcn_corridor' and self.guided_corridor_mode:
            robot_pose = self._lookup_robot_pose()
            if robot_pose is not None:
                self._gcn_handle_goal_success(robot_pose)
        if completed_goal_kind == 'corridor_alignment_staging':
            if isinstance(self.pending_corridor_alignment_second_step, dict):
                self.pending_corridor_alignment_second_step['staging_result_status_label'] = 'SUCCEEDED'
                self.pending_corridor_alignment_second_step['staging_succeeded_wall_time_sec'] = self._now_wall_time_sec()
            self.get_logger().info('corridor-alignment staging succeeded; waiting for fresh scan/local_costmap/TF before second-step forward goal')
        elif self.active_goal_kind == 'explore' and self.active_start_node_id is not None and self.active_branch is not None:
            self.post_ingress_single_open_exception_consumed = True
            try:
                self.topology.mark_branch_state(self.active_start_node_id, self.active_branch, EXPLORED)
                end = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='corridor') if self.active_goal_target else None
                if end is not None:
                    edge = self.topology.connect_nodes(self.active_start_node_id, end.node_id, state=EXPLORED)
                    self.active_edge_id = edge.edge_id
                    self.current_node_id = end.node_id
                    self.topology.visit_node(end.node_id)
            except Exception as exc:
                self.get_logger().error('EXCEPTION in explore success topology update: %s' % exc)
                import traceback
                self.get_logger().error('traceback: %s' % traceback.format_exc())
        elif self.active_goal_kind == 'backtrack' and self.active_goal_target is not None:
            node = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='junction')
            self.current_node_id = node.node_id
            self.topology.visit_node(node.node_id)
        elif self.active_goal_kind == 'near_exit_micro_goal':
            self.get_logger().info('near-exit micro-goal succeeded; returning to exit check / normal exploration')
        elif self.active_goal_kind == 'frontier_push':
            self.get_logger().info(
                'frontier push goal succeeded; clearing state and resuming DFS from new position'
            )
            self.frontier_push_active = False
            self.eastward_push_active = False
            self.frontier_push_tried_targets.clear()
            self.heading_push_angles_tried.clear()  # allow fresh micro-push angles from new position
            # Register new position in topology so DFS can discover new branches
            if self.active_goal_target is not None:
                node = self.topology.find_or_create_node(
                    self.active_goal_target[0], self.active_goal_target[1], node_type='corridor'
                )
                self.current_node_id = node.node_id
                self.topology.visit_node(node.node_id)
            # Clear stuck state as well — the robot moved successfully
            self.stuck_escape_consecutive_failures = 0
            self.stuck_escape_last_position = None
            self.stuck_escape_active = False
            self.stuck_escape_attempts = 0
            # Reset DFS retry cycle so we get fresh frontier pushes from new position
            self.dfs_retry_cycle = max(0, self.dfs_retry_cycle - 1)
        elif self.active_goal_kind == 'stuck_escape':
            self.get_logger().info('stuck escape goal succeeded; clearing stuck state and resuming DFS')
            self.stuck_escape_consecutive_failures = 0
            self.stuck_escape_last_position = None
            self.stuck_escape_active = False
        elif self.active_goal_kind == 'eastward_push':
            self.get_logger().info(
                'eastward push goal succeeded; resetting progress monitor and resuming DFS from new position'
            )
            self.eastward_push_active = False
            self.heading_push_angles_tried.clear()  # allow fresh micro-push angles from new position
            # Register new position in topology so DFS can discover new branches
            if self.active_goal_target is not None:
                node = self.topology.find_or_create_node(
                    self.active_goal_target[0], self.active_goal_target[1], node_type='corridor'
                )
                self.current_node_id = node.node_id
                self.topology.visit_node(node.node_id)
            # Clear stuck state
            self.stuck_escape_consecutive_failures = 0
            self.stuck_escape_last_position = None
            self.stuck_escape_active = False
            # Reset BLOCKED branches so DFS can explore from new position
            reset_count = self.topology.reset_blocked_branches()
            if reset_count > 0:
                self.get_logger().warn(
                    'EASTWARD PUSH: reset %d BLOCKED branches after successful push to new position'
                    % reset_count
                )
        # GCN mode isolation: for gcn_corridor goals, bypass SETTLING mode
        # and keep the GCN state machine in control.  Going through SETTLING
        # triggers DFS code paths that can interfere with corridor navigation.
        if completed_goal_kind == 'gcn_corridor' and self.guided_corridor_mode:
            self.active_goal_kind = 'none'
            self.active_goal_target = None
            self.active_goal_sequence_id = None
            self.active_start_node_id = None
            self.active_branch = None
            self.active_goal_event_context = {}
            # No settle cooldown — GCN dispatches next corridor goal immediately
            self.mode = GUIDED_CORRIDOR
            return

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
            self.entry_direct_retry_count += 1
            can_retry = self.entry_direct_retry_count <= self.entry_direct_max_retries
            if can_retry:
                # Reset for retry: allow ENTRY_DIRECT block to run again after a
                # 5-second delay so the global costmap has time to populate.
                self.entry_direct_dispatched = False
                self.entry_direct_retry_after = self.get_clock().now() + Duration(seconds=5.0)
                self.get_logger().warn(
                    'ENTRY_DIRECT goal failed (reason=%s, retry %d/%d); '
                    'will retry in 5s to allow global costmap to populate'
                    % (reason, self.entry_direct_retry_count, self.entry_direct_max_retries)
                )
            else:
                # Nav2 planner kept failing (typically because the global
                # costmap/map never populated — and it can't populate until the
                # robot moves and SLAM sees the corridor). Break the chicken-
                # and-egg with a reactive forward drive: it bypasses the planner
                # and just drives into the entrance with laser safety, so the
                # robot physically enters and SLAM starts building the map.
                self.entry_direct_failed_time = self.get_clock().now()
                self.entry_direct_dispatched = True  # don't retry Nav2 entry
                started = self._start_reactive_drive(
                    self.entrance_yaw,
                    distance=self.entry_direct_distance_m,
                    min_clearance=0.8)
                if started:
                    self.get_logger().warn(
                        'ENTRY_DIRECT goal failed (reason=%s, retries exhausted %d); '
                        'reactive forward entry started (%.1fm @ %.0f deg) to break '
                        'planner/map chicken-and-egg'
                        % (reason, self.entry_direct_retry_count,
                           self.entry_direct_distance_m,
                           math.degrees(self.entrance_yaw))
                    )
                else:
                    self.get_logger().warn(
                        'ENTRY_DIRECT goal failed (reason=%s, retries exhausted %d); '
                        'reactive entry blocked — falling back to AT_NODE_ANALYZE'
                        % (reason, self.entry_direct_retry_count)
                    )
        if self.active_goal_kind == 'backtrack':
            self.backtrack_failure_count += 1
            if self.active_goal_target is not None:
                node = self.topology.find_or_create_node(self.active_goal_target[0], self.active_goal_target[1], node_type='junction')
                self.topology.record_backtrack_failure(node.node_id)
            reason = BACKTRACK_FAILED
        elif self.active_goal_kind == 'near_exit_micro_goal':
            self.get_logger().warn('near-exit micro-goal failed; preserving DFS topology state')
        elif self.active_goal_kind == 'frontier_push':
            self.frontier_push_active = False
            if self.eastward_push_active:
                self.eastward_push_active = False
                self.eastward_progress_goal_at_max = self.goal_count
            # Do NOT clear heading_push_angles_tried on failure — we want to
            # avoid retrying the same angle from the same position.  Only clear
            # on success (when the robot has moved to a new position).
            self.get_logger().warn(
                'frontier push goal failed (attempt %d/%d); will try another frontier target or exhaust'
                % (self.frontier_push_attempts, self.frontier_push_max_attempts)
            )
            # Don't return — let the normal flow continue. On next cycle,
            # _mark_exhausted will be called again and _maybe_frontier_push
            # will try another target if attempts remain.
        elif self.active_goal_kind == 'stuck_escape':
            self.stuck_escape_attempts += 1
            self.get_logger().warn(
                'stuck escape goal failed (attempt %d/%d)'
                % (self.stuck_escape_attempts, self.stuck_escape_max_attempts)
            )
            if self.stuck_escape_attempts >= self.stuck_escape_max_attempts:
                self.get_logger().error('stuck escape attempts exhausted; marking FAILED_EXHAUSTED')
                self.active_goal_kind = 'none'
                self.active_goal_target = None
                self.active_goal_sequence_id = None
                self.active_start_node_id = None
                self.active_branch = None
                self.active_goal_event_context = {}
                self._mark_exhausted('stuck: Nav2 planner cannot find paths and all escape attempts failed')
                return
            # Reset for another escape attempt on next cycle
            self.stuck_escape_active = True
        elif self.active_goal_kind == 'eastward_push':
            self.eastward_push_active = False
            # Reset stagnation counter so DFS gets another window before
            # the next forced push attempt
            self.eastward_progress_goal_at_max = self.goal_count
            # Do NOT clear heading_push_angles_tried on failure — we want to
            # try different angles from the same position.  Only clear on success.
            self.get_logger().warn(
                'eastward push goal failed (attempt %d/%d); resetting stagnation counter, '
                'resuming DFS from current position'
                % (self.eastward_push_attempts, self.eastward_push_max_attempts)
            )
        elif self.active_goal_kind == 'gcn_corridor' and self.guided_corridor_mode:
            robot_pose = self._lookup_robot_pose()
            if robot_pose is not None:
                self._gcn_handle_goal_failure(robot_pose, reason)
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
        # Track consecutive failures from same position for stuck escape detection.
        # When Nav2 can't plan from the robot's current position to ANY target
        # (all backtrack/explore goals fail with status=6), the robot is stuck in
        # a SLAM artifact zone.  After N consecutive failures without significant
        # movement, activate stuck_escape to send a short goal in the most open
        # direction to escape the artifact zone.
        if self.active_goal_kind not in ('stuck_escape', 'entry_direct', 'frontier_push', 'gcn_corridor'):
            failure_pose = self._lookup_robot_pose()
            if failure_pose:
                failure_pos = (failure_pose[0], failure_pose[1])
                if self.stuck_escape_last_position is not None:
                    move_since_last = math.hypot(
                        failure_pos[0] - self.stuck_escape_last_position[0],
                        failure_pos[1] - self.stuck_escape_last_position[1],
                    )
                    if move_since_last < self.stuck_escape_position_radius_m:
                        self.stuck_escape_consecutive_failures += 1
                    else:
                        # Robot moved — reset counter, not stuck
                        self.stuck_escape_consecutive_failures = 1
                        self.stuck_escape_last_position = failure_pos
                else:
                    self.stuck_escape_consecutive_failures = 1
                    self.stuck_escape_last_position = failure_pos
                if (self.stuck_escape_consecutive_failures >= self.stuck_escape_max_consecutive
                        and not self.stuck_escape_active
                        and self.stuck_escape_attempts < self.stuck_escape_max_attempts):
                    self.stuck_escape_active = True
                    self.stuck_escape_attempts += 1
                    self.get_logger().warn(
                        'STUCK DETECTED: %d consecutive failures from same position (%.3f, %.3f); '
                        'activating escape mechanism (attempt %d/%d)'
                        % (self.stuck_escape_consecutive_failures,
                           failure_pos[0], failure_pos[1],
                           self.stuck_escape_attempts, self.stuck_escape_max_attempts)
                    )
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
        # GCN mode isolation: for gcn_corridor goals, bypass SETTLING mode.
        # The GCN failure handler already recorded the failure; keeping the
        # mode as GUIDED_CORRIDOR allows the GCN tick to retry or fall back
        # to reactive drive on the next cycle.
        if self.active_goal_kind == 'gcn_corridor' and self.guided_corridor_mode:
            self.active_goal_kind = 'none'
            self.active_goal_target = None
            self.active_goal_sequence_id = None
            self.active_start_node_id = None
            self.active_branch = None
            self.active_goal_event_context = {}
            self.mode = GUIDED_CORRIDOR
            return

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

    # ---- Optimization 2: periodic re-localization rotation ----

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi)."""
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _should_start_relocalization(self) -> bool:
        """Check if a re-localization rotation is due."""
        if not self.relocalize_enabled:
            return False
        # GCN follows a pre-validated known path; periodic 360 deg in-place
        # spins are unnecessary and actively harm scan-matching in the narrow
        # maze corridors (observed disrupting the C5 transition). Skip them.
        if self.guided_corridor_mode:
            return False
        if self.relocalize_interval_goals <= 0:
            return False
        if self.goal_count < self.relocalize_interval_goals:
            return False
        goals_since = self.goal_count - self.relocalize_goals_at_last_rotation
        return goals_since >= self.relocalize_interval_goals

    def _execute_relocalization_step(self, robot_pose: RobotPose) -> bool:
        """Execute one tick of the re-localization rotation.

        Returns True while rotation is in progress, False when complete.
        """
        if not self.relocalize_active:
            return False

        yaw = robot_pose[2]

        if self.relocalize_start_yaw is None:
            # First tick: initialize
            self.relocalize_start_yaw = yaw
            self.relocalize_last_yaw = yaw
            self.relocalize_accumulated_rad = 0.0
            self.get_logger().info(
                'Optimization2: starting re-localization rotation (goals=%d, interval=%d)'
                % (self.goal_count, self.relocalize_interval_goals)
            )
            # Publish rotation command
            twist = Twist()
            twist.angular.z = self.relocalize_angular_speed
            self.cmd_vel_pub.publish(twist)
            return True

        # Track accumulated rotation (handle wrap-around)
        if self.relocalize_last_yaw is not None:
            delta = self._normalize_angle(yaw - self.relocalize_last_yaw)
            self.relocalize_accumulated_rad += delta
        self.relocalize_last_yaw = yaw

        if abs(self.relocalize_accumulated_rad) >= 2.0 * math.pi:
            # Rotation complete
            twist = Twist()  # zero velocity = stop
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                'Optimization2: re-localization rotation complete (%.1f rad accumulated, goals=%d)'
                % (self.relocalize_accumulated_rad, self.goal_count)
            )
            self.relocalize_active = False
            self.relocalize_start_yaw = None
            self.relocalize_goals_at_last_rotation = self.goal_count
            return False

        # Continue rotating
        twist = Twist()
        twist.angular.z = self.relocalize_angular_speed
        self.cmd_vel_pub.publish(twist)
        return True

    # -----------------------------------------------------------------
    # Reactive drive: bypass Nav2 planner when SLAM drift walls cage
    # the robot.  Uses direct cmd_vel with laser scan safety checking.
    # -----------------------------------------------------------------

    def _reactive_control_tick(self) -> None:
        """Fast (10 Hz) reactive-drive control loop.

        Runs independently of the 0.5 Hz exploration tick so that reactive
        rotation/forward-drive commands are published continuously, keeping the
        velocity_smoother fed (it zeroes after a 1.0s gap). No-op unless a
        reactive drive is active.
        """
        if not self.reactive_drive_active:
            return
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return
        self._execute_reactive_drive_step(robot_pose)

    def _execute_reactive_drive_step(self, robot_pose: RobotPose) -> bool:
        """Execute one tick of reactive drive (bypasses Nav2 planner).

        State machine:
        - rotating: publish angular velocity to face target direction
        - driving:  publish linear velocity, check laser scan, track distance

        Returns True while reactive drive is in progress, False when done.
        """
        if not self.reactive_drive_active:
            return False

        robot_x, robot_y, robot_yaw = robot_pose

        # ── Watchdog: hard timeout for the whole reactive drive ──
        now = self.get_clock().now()
        if self.reactive_drive_start_time is not None:
            elapsed = (now - self.reactive_drive_start_time).nanoseconds / 1e9
            if elapsed > self.reactive_drive_max_seconds:
                self.get_logger().warn(
                    'REACTIVE DRIVE: watchdog timeout after %.1fs (state=%s) — aborting'
                    % (elapsed, self.reactive_drive_state)
                )
                self.cmd_vel_pub.publish(Twist())
                self._finish_reactive_drive()
                return False

        if self.reactive_drive_state == 'backup':
            # Briefly reverse to unwedge, then finish so the GCN loop can retry.
            if self.reactive_drive_backup_ref_xy is None:
                self.reactive_drive_backup_ref_xy = (robot_x, robot_y)
            bx, by = self.reactive_drive_backup_ref_xy
            backed = math.hypot(robot_x - bx, robot_y - by)
            if backed >= 0.4:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info(
                    'REACTIVE DRIVE: backed up %.2fm to unwedge — finishing' % backed
                )
                self._finish_reactive_drive()
                return False
            twist = Twist()
            twist.linear.x = -0.15
            self.cmd_vel_pub.publish(twist)
            return True

        if self.reactive_drive_state == 'rotating':
            angle_diff = normalize_angle(self.reactive_drive_target_yaw - robot_yaw)

            if abs(angle_diff) < math.radians(8):
                # Rotation complete → start driving
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.reactive_drive_state = 'driving'
                self.reactive_drive_start_xy = (robot_x, robot_y)
                self.get_logger().info(
                    'REACTIVE DRIVE: rotation complete, driving forward'
                )
                return True

            # No-rotation (jammed) detection: if the robot is physically caught
            # against a wall it won't actually turn even while commanded. If yaw
            # barely changes for a few seconds, back up to free it.
            if self.reactive_drive_rot_ref_yaw is None:
                self.reactive_drive_rot_ref_yaw = robot_yaw
                self.reactive_drive_rot_ref_time = now
            else:
                yaw_moved = abs(normalize_angle(robot_yaw - self.reactive_drive_rot_ref_yaw))
                if yaw_moved > math.radians(8):
                    self.reactive_drive_rot_ref_yaw = robot_yaw
                    self.reactive_drive_rot_ref_time = now
                elif self.reactive_drive_rot_ref_time is not None:
                    stuck = (now - self.reactive_drive_rot_ref_time).nanoseconds / 1e9
                    if stuck > self.reactive_drive_no_rot_sec:
                        self.get_logger().warn(
                            'REACTIVE DRIVE: cannot rotate (yaw moved %.0f deg in %.1fs) '
                            '— jammed, backing up' % (math.degrees(yaw_moved), stuck)
                        )
                        self.reactive_drive_state = 'backup'
                        self.reactive_drive_backup_ref_xy = None
                        return True

            # Publish rotation (0.5 rad/s = velocity_smoother max for snappier turns)
            twist = Twist()
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            self.cmd_vel_pub.publish(twist)
            return True

        if self.reactive_drive_state == 'driving':
            # Distance traveled
            if self.reactive_drive_start_xy is not None:
                dx = robot_x - self.reactive_drive_start_xy[0]
                dy = robot_y - self.reactive_drive_start_xy[1]
                distance_traveled = math.sqrt(dx * dx + dy * dy)
            else:
                distance_traveled = 0.0

            if distance_traveled >= self.reactive_drive_target_distance:
                # Target reached
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(
                    'REACTIVE DRIVE: completed %.2fm / %.1fm'
                    % (distance_traveled, self.reactive_drive_target_distance)
                )
                self._finish_reactive_drive()
                return False

            # Safety: check laser ahead
            if not self._is_laser_clear_ahead(min_range=0.5, cone_half_angle=math.radians(20)):
                self.get_logger().warn(
                    'REACTIVE DRIVE: obstacle ahead after %.2fm, stopping'
                    % distance_traveled
                )
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self._finish_reactive_drive()
                return False

            # No-progress (wedge) detection: the forward laser cone can stay
            # clear while the robot is physically caught on a corner wall and
            # odom barely advances. If we don't actually move for a few seconds,
            # treat it as wedged and back up to unstick.
            if self.reactive_drive_progress_ref_xy is None:
                self.reactive_drive_progress_ref_xy = (robot_x, robot_y)
                self.reactive_drive_progress_ref_time = now
            else:
                rx0, ry0 = self.reactive_drive_progress_ref_xy
                moved = math.hypot(robot_x - rx0, robot_y - ry0)
                if moved > 0.1:
                    self.reactive_drive_progress_ref_xy = (robot_x, robot_y)
                    self.reactive_drive_progress_ref_time = now
                elif self.reactive_drive_progress_ref_time is not None:
                    stuck = (now - self.reactive_drive_progress_ref_time).nanoseconds / 1e9
                    if stuck > self.reactive_drive_no_progress_sec:
                        self.get_logger().warn(
                            'REACTIVE DRIVE: no progress for %.1fs (moved %.2fm) '
                            'after %.2fm — wedged, backing up'
                            % (stuck, moved, distance_traveled)
                        )
                        self.reactive_drive_state = 'backup'
                        self.reactive_drive_backup_ref_xy = None
                        return True

            # Drive forward
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            return True

        # Unknown state → cleanup
        self._finish_reactive_drive()
        return False

    def _finish_reactive_drive(self) -> None:
        """Clean up reactive drive state and allow fresh heading pushes."""
        self.reactive_drive_active = False
        self.reactive_drive_state = 'idle'
        self.reactive_drive_start_xy = None
        # GCN: notify corridor navigator of reactive drive completion
        if self.guided_corridor_mode and self.corridor_nav is not None:
            robot_pose = self._lookup_robot_pose()
            if robot_pose is not None:
                self._gcn_handle_reactive_complete(robot_pose)
            # Return to GUIDED_CORRIDOR mode so next tick dispatches new goal
            self.mode = GUIDED_CORRIDOR
        # Clear frontier_push_active since reactive drive bypassed Nav2 action
        self.frontier_push_active = False
        if self.eastward_push_active:
            self.eastward_push_active = False
            self.eastward_progress_goal_at_max = self.goal_count
        # Clear heading push angles so the next round tries fresh angles
        # from the new position (reactive drive may have moved the robot).
        self.heading_push_angles_tried.clear()

    def _start_reactive_drive(self, angle: float, distance: float = 1.0,
                              min_clearance: float = 1.5) -> bool:
        """Start a reactive drive toward angle.  Returns False if blocked.

        Args:
            angle: Target yaw in MAP frame (radians).
            distance: How far to drive (metres).
            min_clearance: Minimum laser range required in the target
                           direction (metres).  Default 1.5m for general use;
                           GCN callers pass 0.5m because the BFS guarantees the
                           gap exists and SLAM drift may place the robot very
                           close to a wall gap that the robot has already partly
                           crossed.
        """
        if self.scan_msg is None:
            return False
        if self.reactive_drive_count >= self.reactive_drive_max_count:
            self.get_logger().info(
                'REACTIVE DRIVE: max count (%d) reached'
                % self.reactive_drive_max_count
            )
            return False

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False

        _rx, _ry, robot_yaw = robot_pose
        angle_in_robot = normalize_angle(angle - robot_yaw)

        # Check laser in the desired direction
        if not self._is_laser_clear_in_direction(
                angle_in_robot, min_range=min_clearance,
                cone_half_angle=math.radians(20)):
            self.get_logger().info(
                'REACTIVE DRIVE: direction %.0f° blocked (min_range=%.1fm)'
                % (math.degrees(angle), min_clearance)
            )
            return False

        self.reactive_drive_active = True
        self.reactive_drive_state = 'rotating'
        self.reactive_drive_target_yaw = angle
        self.reactive_drive_target_distance = distance
        self.reactive_drive_start_xy = None
        self.reactive_drive_count += 1
        # Arm the watchdog (sim clock).
        self.reactive_drive_start_time = self.get_clock().now()
        self.reactive_drive_progress_ref_xy = None
        self.reactive_drive_progress_ref_time = None
        self.reactive_drive_backup_ref_xy = None
        self.reactive_drive_rot_ref_yaw = None
        self.reactive_drive_rot_ref_time = None

        self.get_logger().warn(
            'REACTIVE DRIVE: #%d — angle=%.0f° dist=%.1fm '
            '(bypassing Nav2 planner due to SLAM drift cage)'
            % (self.reactive_drive_count, math.degrees(angle), distance)
        )
        return True

    def _start_reactive_drive_gcn(self, prefer_angle: float,
                                   distance: float = 1.0) -> bool:
        """Start a GCN reactive drive with angular search if direct path blocked.

        GCN corridors are pre-validated by BFS so the gap is guaranteed to exist.
        SLAM drift may place the robot at the gap edge rather than centered,
        causing the direct laser check to fail.  This method searches ±5°..±25°
        for a clear path before giving up.

        Returns True if reactive drive started, False if all angles blocked.
        """
        # Try direct angle first with reduced clearance (BFS guarantees gap)
        if self._start_reactive_drive(prefer_angle, distance=distance,
                                      min_clearance=0.5):
            return True

        # Direct path blocked — search angular offsets for the gap
        for offset_deg in [5, -5, 10, -10, 15, -15, 20, -20, 25, -25]:
            try_angle = prefer_angle + math.radians(offset_deg)
            if self._start_reactive_drive(try_angle, distance=distance * 0.7,
                                          min_clearance=0.5):
                self.get_logger().info(
                    'GCN: reactive drive found gap at %.0f° (offset %+d° from %.0f°)'
                    % (math.degrees(try_angle), offset_deg,
                       math.degrees(prefer_angle))
                )
                return True

        self.get_logger().warn(
            'GCN: reactive drive blocked at ALL angles ±25° from %.0f°'
            % math.degrees(prefer_angle)
        )
        return False

    def _find_best_reactive_angle(self, prefer_angle: float) -> Optional[float]:
        """Find the best reactive drive angle using laser scan corridor detection.

        Scans the laser in sectors and picks the one with the longest clear
        range, weighted toward prefer_angle (typically angle_to_exit).

        Returns angle in MAP frame, or None if no viable direction.
        """
        if self.scan_msg is None:
            return None
        scan = self.scan_msg
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return None
        _rx, _ry, robot_yaw = robot_pose
        prefer_in_robot = normalize_angle(prefer_angle - robot_yaw)

        # Scan in 30° sectors, find the one with longest median range
        # IMPORTANT: only consider sectors within ±90° of the exit direction.
        # Previous version picked the longest corridor (often perpendicular or
        # opposite to exit), driving the robot AWAY from the goal.
        sector_width = math.radians(30)
        best_angle_robot: Optional[float] = None
        best_score: float = 0.0

        for sector_center_deg in range(-180, 180, 30):
            sector_center = math.radians(sector_center_deg)

            # Skip directions more than ±90° from exit — never drive backwards
            angle_diff = abs(normalize_angle(sector_center - prefer_in_robot))
            if angle_diff > math.pi / 2:
                continue

            # Collect ranges in this sector
            ranges_in_sector: list[float] = []
            for i in range(len(scan.ranges)):
                r = scan.ranges[i]
                if not math.isfinite(r):
                    continue
                scan_angle = scan.angle_min + i * scan.angle_increment
                if abs(normalize_angle(scan_angle - sector_center)) <= sector_width / 2:
                    ranges_in_sector.append(r)

            if not ranges_in_sector:
                continue

            # Use 25th percentile (robust to occasional noisy readings)
            sorted_r = sorted(ranges_in_sector)
            p25 = sorted_r[len(sorted_r) // 4]
            median_r = sorted_r[len(sorted_r) // 2]

            # Need at least 1.0m clearance to consider this sector
            if p25 < 1.0:
                continue

            # Score: cosine weighting strongly prefers alignment with exit.
            # alignment goes from 1.0 (exact exit direction) to 0.0 (±90°).
            alignment = math.cos(angle_diff)
            score = median_r * alignment
            if score > best_score:
                best_score = score
                best_angle_robot = sector_center

        if best_angle_robot is None:
            return None

        # Convert back to map frame
        result_angle = normalize_angle(best_angle_robot + robot_yaw)
        return result_angle

    def _is_laser_clear_in_direction(self, direction: float, min_range: float,
                                      cone_half_angle: float) -> bool:
        """Check laser scan for clear path in direction (robot frame)."""
        if self.scan_msg is None:
            return False
        scan = self.scan_msg
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            if not math.isfinite(r):
                continue
            scan_angle = scan.angle_min + i * scan.angle_increment
            if abs(normalize_angle(scan_angle - direction)) <= cone_half_angle:
                if r < min_range:
                    return False
        return True

    def _is_laser_clear_ahead(self, min_range: float, cone_half_angle: float) -> bool:
        """Check laser scan for clear path directly ahead (0° robot frame)."""
        return self._is_laser_clear_in_direction(0.0, min_range, cone_half_angle)

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
            # Check for pending explore-goal retry before transitioning to
            # AT_NODE_ANALYZE.  This handles transient Nav2 aborts caused by
            # TF buffer clears without marking the branch as BLOCKED.
            if self.explore_retry_pending:
                now = self.get_clock().now()
                if self.explore_retry_after is not None and now < self.explore_retry_after:
                    # Still waiting for retry delay to elapse
                    return True  # keep SETTLING
                self.explore_retry_pending = False
                self._dispatch_explore_retry()
                return False
            self.mode = AT_NODE_ANALYZE
        return False

    def _dispatch_explore_retry(self) -> None:
        """Dispatch a previously-saved explore goal as a retry."""
        if self.explore_retry_target is None:
            self.get_logger().warn('explore retry: no saved target; falling back to AT_NODE_ANALYZE')
            self.explore_retry_count = 0
            self.mode = AT_NODE_ANALYZE
            return
        # Restore the branch context so _send_goal has the right topology
        self.active_start_node_id = self.explore_retry_start_node_id
        self.active_branch = self.explore_retry_branch
        target = self.explore_retry_target
        yaw = self.explore_retry_yaw or 0.0
        self.get_logger().info(
            'dispatching explore retry: target=(%.3f, %.3f) yaw=%.3f retry=%d/%d'
            % (target[0], target[1], yaw, self.explore_retry_count, self.explore_max_retries)
        )
        # Reset retry state after dispatching
        saved_target = self.explore_retry_target
        self.explore_retry_target = None
        self.explore_retry_start_node_id = None
        self.explore_retry_branch = None
        self.explore_retry_yaw = None
        self._send_goal(saved_target, yaw, 'explore', skip_two_step_staging=True)
        self.mode = WAIT_FOR_NAV2

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

    def _lookup_odom_pose(self) -> Optional[RobotPose]:
        """Robot pose in the odom frame (drift-free over short horizons, but
        not loop-closed). Used to detect map-frame pose jumps from SLAM."""
        try:
            transform = self.tf_buffer.lookup_transform('odom', self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = self._yaw_from_quaternion(r.x, r.y, r.z, r.w)
        return (float(t.x), float(t.y), yaw)

    def _map_jump_diagnostic(self) -> None:
        """Log map-frame vs odom-frame motion each tick to expose SLAM map
        jumps (map-frame pose teleporting while odom moves smoothly)."""
        m = self._lookup_robot_pose()
        o = self._lookup_odom_pose()
        if m is None or o is None:
            return
        prev_m = getattr(self, '_diag_last_map_xy', None)
        prev_o = getattr(self, '_diag_last_odom_xy', None)
        if prev_m is not None and prev_o is not None:
            dmap = math.hypot(m[0] - prev_m[0], m[1] - prev_m[1])
            dodom = math.hypot(o[0] - prev_o[0], o[1] - prev_o[1])
            jump = abs(dmap - dodom)
            tag = '  <<< MAP JUMP' if jump > 0.3 else ''
            self.get_logger().info(
                'POSEDIAG map=(%.2f,%.2f) odom=(%.2f,%.2f) dmap=%.2f dodom=%.2f jump=%.2f%s'
                % (m[0], m[1], o[0], o[1], dmap, dodom, jump, tag)
            )
        self._diag_last_map_xy = (m[0], m[1])
        self._diag_last_odom_xy = (o[0], o[1])

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
        # Before giving up, try frontier push: scan the SLAM map for frontier
        # cells (free adjacent to unknown) in the exit direction and dispatch
        # a goal to push the exploration boundary outward. This breaks through
        # cases where DFS has explored all known junctions but the explored
        # region doesn't reach the exit yet.
        if self._maybe_frontier_push(reason):
            return
        # Before truly giving up, try resetting BLOCKED branches for DFS retry.
        if self._maybe_dfs_reset(reason):
            return
        self._enter_terminal_state(FAILED_EXHAUSTED, terminal_reason=reason)
        if not self.exhausted_logged:
            self.exhausted_logged = True
            self.get_logger().warn('MAZE_EXPLORER_EXHAUSTED reason=%s' % reason)

    def _maybe_frontier_push(self, exhaustion_reason: str) -> bool:
        """Attempt a frontier push goal before declaring FAILED_EXHAUSTED.

        Scans the SLAM map for frontier cells (free cells adjacent to unknown)
        reachable from the robot's position via connected free space.  Cycles
        through different candidates so that repeated failures don't retry the
        same unreachable target.

        Returns True if a frontier push goal was dispatched (exhaustion deferred).
        Returns False if no viable frontier target found or max attempts reached.
        """
        # Fast-fail on early stuck: if the robot hasn't made meaningful eastward
        # progress (max_x < 5.0) after <15 goals, SLAM drift has likely corrupted
        # the costmap.  Limit frontier pushes to 3 instead of 25 to avoid wasting
        # 10+ minutes on clearly doomed runs.
        effective_max_attempts = self.frontier_push_max_attempts
        if (self.goal_count < 15
                and self.eastward_progress_max_x < 5.0):
            effective_max_attempts = 3
            if self.frontier_push_attempts == 0:
                self.get_logger().warn(
                    'FRONTIER PUSH: early stuck detected (goal #%d, max_x=%.1f); '
                    'limiting to %d frontier push attempts instead of %d'
                    % (self.goal_count, self.eastward_progress_max_x,
                       effective_max_attempts, self.frontier_push_max_attempts)
                )

        if self.frontier_push_attempts >= effective_max_attempts:
            self.get_logger().info(
                'FRONTIER PUSH: max attempts (%d) reached; clearing tried targets'
                % effective_max_attempts
            )
            self.frontier_push_tried_targets.clear()
            return False

        if self.map_view is None:
            return False

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False

        robot_xy = (robot_pose[0], robot_pose[1])
        robot_exit_dist = math.hypot(robot_xy[0] - self.exit_x, robot_xy[1] - self.exit_y)

        # If robot is already very close to exit, don't frontier push
        if robot_exit_dist < 3.0:
            return False

        target = self._find_frontier_push_target(robot_xy, robot_exit_dist)
        if target is None:
            self.get_logger().info(
                'FRONTIER PUSH: no viable frontier target found '
                '(tried %d, exhausted all reachable candidates)'
                % len(self.frontier_push_tried_targets)
            )
            self.frontier_push_tried_targets.clear()
            return False

        # Critical check: if the best frontier target is FURTHER from the exit
        # than the robot, frontier push is counterproductive — it will move us
        # away from the goal.  Try a heading push toward the exit instead.
        target_exit_dist = math.hypot(target[0] - self.exit_x, target[1] - self.exit_y)
        if target_exit_dist > robot_exit_dist + 1.0:
            self.get_logger().info(
                'FRONTIER PUSH: best target at (%.1f, %.1f) is %.1fm from exit '
                '(robot is %.1fm) — moving AWAY from exit; trying heading push instead [tried=%d]'
                % (target[0], target[1], target_exit_dist, robot_exit_dist,
                   len(self.frontier_push_tried_targets))
            )
            # Do NOT clear tried_targets — heading push registers its own targets
            # in the tried set so it won't repeat the same angle/location.
            return self._heading_push_toward_exit(robot_xy, robot_exit_dist)

        # Record this target as tried using 1m grid dedup (same grid as _find_frontier_push_target)
        target_cell = self.map_view.world_to_cell(target[0], target[1])
        dedup_key = (target_cell[0] // 20, target_cell[1] // 20)
        self.frontier_push_tried_targets.add(dedup_key)

        self.frontier_push_attempts += 1
        self.frontier_push_active = True
        yaw = math.atan2(target[1] - robot_xy[1], target[0] - robot_xy[0])
        self.get_logger().warn(
            'FRONTIER PUSH: DFS exhausted (%s); dispatching frontier push goal #%d '
            'to (%.3f, %.3f) — %.1fm from exit (robot at %.1fm) [tried=%d]'
            % (exhaustion_reason, self.frontier_push_attempts,
               target[0], target[1],
               math.hypot(target[0] - self.exit_x, target[1] - self.exit_y),
               robot_exit_dist, len(self.frontier_push_tried_targets))
        )
        self._send_goal(target, yaw=yaw, goal_kind='frontier_push', skip_two_step_staging=True)
        return True

    def _maybe_dfs_reset(self, reason: str) -> bool:
        """Reset BLOCKED branches to UNTRIED so DFS can retry them.

        Called after frontier push exhaustion.  Gives DFS a second chance by
        reopening corridors that were blocked by Nav2 failures — with SLAM drift
        these corridors may now be reachable.

        Returns True if branches were reset and DFS can continue.
        Returns False if no more reset cycles remain.
        """
        if self.dfs_retry_cycle >= self.dfs_retry_max_cycles:
            self.get_logger().info(
                'DFS RESET: max cycles (%d) reached; proceeding to FAILED_EXHAUSTED'
                % self.dfs_retry_max_cycles
            )
            return False

        reset_count = self.topology.reset_blocked_branches()
        if reset_count == 0:
            self.get_logger().info(
                'DFS RESET: no BLOCKED branches to reset; proceeding to FAILED_EXHAUSTED'
            )
            return False

        self.dfs_retry_cycle += 1
        # Reset frontier push state so a fresh round of frontier pushes is possible
        self.frontier_push_tried_targets.clear()
        self.frontier_push_attempts = 0
        self.heading_push_angles_tried.clear()

        self.get_logger().warn(
            'DFS RESET: cycle %d/%d — reset %d BLOCKED branches to UNTRIED, '
            'clearing frontier state for fresh round (reason: %s)'
            % (self.dfs_retry_cycle, self.dfs_retry_max_cycles, reset_count, reason)
        )
        return True

    def _check_eastward_stagnation(self, robot_pose: RobotPose) -> bool:
        """Check if DFS has been stagnating (no eastward progress) and force a push.

        If the robot has spent more than eastward_progress_stagnation_limit goals
        without advancing eastward by at least eastward_progress_advance_threshold,
        dispatch a forced heading push toward the exit (east/northeast) to break
        out of the local DFS loop.

        Uses the multi-angle heading push mechanism to try different angles if
        the direct-to-exit angle fails.

        Returns True if a forced eastward push was dispatched (caller should return).
        Returns False if no stagnation detected (caller should proceed with DFS).
        """
        if self.eastward_push_active:
            return False  # already in a forced push, let goal result handle it

        if self.eastward_push_attempts >= self.eastward_push_max_attempts:
            return False  # already tried enough forced pushes

        goals_since_progress = self.goal_count - self.eastward_progress_goal_at_max

        if goals_since_progress < self.eastward_progress_stagnation_limit:
            return False  # not stagnant yet

        robot_xy = (robot_pose[0], robot_pose[1])
        robot_exit_dist = math.hypot(robot_xy[0] - self.exit_x, robot_xy[1] - self.exit_y)

        # Don't force push if already close to exit
        if robot_exit_dist < 5.0:
            return False

        self.eastward_push_attempts += 1
        self.eastward_push_active = True

        self.get_logger().warn(
            'EASTWARD STAGNATION: %d goals without eastward progress (max_x=%.1f at goal #%d, '
            'current goal #%d). Forcing eastward push #%d/%d via heading push '
            '(robot at (%.1f, %.1f), %.1fm from exit)'
            % (goals_since_progress, self.eastward_progress_max_x,
               self.eastward_progress_goal_at_max, self.goal_count,
               self.eastward_push_attempts, self.eastward_push_max_attempts,
               robot_xy[0], robot_xy[1], robot_exit_dist)
        )

        # Use the multi-angle heading push mechanism — it cycles through
        # different angles (N, NNE, NNW, NE, NW) with dedup tracking
        pushed = self._heading_push_toward_exit(robot_xy, robot_exit_dist)
        if not pushed:
            # All heading push angles exhausted — reset and let DFS continue
            self.eastward_push_active = False
            self.eastward_progress_goal_at_max = self.goal_count
            self.get_logger().warn(
                'EASTWARD STAGNATION: heading push exhausted all angles; '
                'resetting stagnation counter and resuming DFS'
            )
        return pushed

    def _heading_push_toward_exit(
        self,
        robot_xy: tuple[float, float],
        robot_exit_dist: float,
    ) -> bool:
        """Push the robot toward the exit through unknown territory via angle cycling.

        When BFS-connected frontier push can't find reachable frontier cells that
        move closer to the exit, this method dispatches goals at multiple angles,
        letting Nav2 plan through unknown cells (allow_unknown).

        Uses SHORT micro-pushes (1.5-2.0m) that stay within or just beyond the
        explored frontier, so Nav2 can plan through known/near-known territory.
        After a successful micro-push, angles are cleared for a fresh round from
        the new position.

        Returns True if a heading push goal was dispatched.
        Returns False if all candidate angles have been exhausted.
        """
        # Direction from robot to exit
        dx = self.exit_x - robot_xy[0]
        dy = self.exit_y - robot_xy[1]
        angle_to_exit = math.atan2(dy, dx)

        # Candidate angles in priority order:
        # 1. Pure north (90°) — vertical corridors are key path to exit
        # 2. NNE (67.5°) — slightly toward exit
        # 3. NNW (112.5°) — slightly toward x≈18 corridor
        # 4. NE (angle_to_exit) — straight toward exit (original behavior)
        # 5. East (0°) — pure east along horizontal corridors
        # 6. NW (135°) — more west, may find alternate corridor
        # 7. ENE (22.5°) — if exit is nearly east
        # 8. ESE (-22.5°) — southeast corridors
        candidate_angles = [
            math.pi / 2,                # 90°  pure north
            math.pi / 2 - math.pi / 8,  # 67.5° NNE
            math.pi / 2 + math.pi / 8,  # 112.5° NNW
            angle_to_exit,               # toward exit (often NE)
            0.0,                         # 0° pure east
            math.pi / 2 + math.pi / 4,  # 135° NW
            math.pi / 4,                # 45°  NE
            -math.pi / 8,               # -22.5° ESE
        ]

        for raw_angle in candidate_angles:
            # Normalize to [-pi, pi]
            angle = raw_angle
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Skip if this angle (within 15°) has already been tried
            if any(abs(angle - t) < math.radians(15) for t in self.heading_push_angles_tried):
                continue

            # MICRO-PUSH: 1.5-2.0m — keep target within known corridors
            # so Nav2 can actually plan to it.  These short pushes advance
            # the frontier incrementally instead of jumping into unknown.
            push_dist = min(2.0, max(1.5, robot_exit_dist * 0.12))
            push_x = robot_xy[0] + push_dist * math.cos(angle)
            push_y = robot_xy[1] + push_dist * math.sin(angle)

            # Validate target isn't in a known wall — if SLAM map shows
            # this cell as occupied, skip this angle (it hits a wall).
            if self.map_view is not None:
                target_cell = self.map_view.world_to_cell(push_x, push_y)
                if self.map_view.in_bounds(target_cell) and self.map_view.is_occupied(target_cell):
                    self.get_logger().info(
                        'HEADING PUSH: skipping angle %.0f° — target (%.1f, %.1f) is in a known wall'
                        % (math.degrees(angle), push_x, push_y)
                    )
                    self.heading_push_angles_tried.append(angle)
                    continue

            # Register this angle as tried
            self.heading_push_angles_tried.append(angle)
            # Also register target in frontier dedup set
            if self.map_view is not None:
                target_cell = self.map_view.world_to_cell(push_x, push_y)
                dedup_key = (target_cell[0] // 20, target_cell[1] // 20)
                self.frontier_push_tried_targets.add(dedup_key)

            self.frontier_push_attempts += 1
            self.frontier_push_active = True
            self.get_logger().warn(
                'HEADING PUSH: dispatching micro-push #%d at (%.1f, %.1f) angle=%.0f° '
                '(%.1fm from exit, robot at %.1fm, push_dist=%.1fm, angles_tried=%d)'
                % (self.frontier_push_attempts, push_x, push_y, math.degrees(angle),
                   math.hypot(push_x - self.exit_x, push_y - self.exit_y),
                   robot_exit_dist, push_dist, len(self.heading_push_angles_tried))
            )
            self._send_goal(
                (push_x, push_y),
                yaw=angle,
                goal_kind='frontier_push',
                skip_two_step_staging=True,
            )
            return True

        # All Nav2 heading push angles exhausted — try reactive drive
        # which bypasses the corrupted global costmap entirely.
        self.get_logger().warn(
            'HEADING PUSH: all %d Nav2 candidate angles exhausted; '
            'trying reactive drive (robot at %.1fm from exit)'
            % (len(self.heading_push_angles_tried), robot_exit_dist)
        )
        # Reactive drive: use laser scan to find the best corridor
        # direction (longest clear range, weighted toward exit).
        # This follows the actual corridor structure instead of fixed angles.
        laser_angle = self._find_best_reactive_angle(angle_to_exit)
        if laser_angle is not None:
            if self._start_reactive_drive(laser_angle, distance=1.5):
                self.frontier_push_attempts += 1
                self.frontier_push_active = True
                return True
            self.get_logger().info(
                'REACTIVE DRIVE: laser-best angle %.0f° blocked or failed'
                % math.degrees(laser_angle)
            )

        # Fallback: try fixed angles as last resort
        reactive_angles = [
            angle_to_exit,    # straight toward exit (often NE)
            math.pi / 2,      # pure north
            math.pi / 4,      # northeast
            0.0,               # pure east
            -math.pi / 4,     # southeast
            3 * math.pi / 4,  # northwest
        ]
        for r_angle in reactive_angles:
            if self._start_reactive_drive(r_angle, distance=1.5):
                self.frontier_push_attempts += 1
                self.frontier_push_active = True
                return True

        # Reactive drive also failed — truly stuck
        self.get_logger().warn(
            'HEADING PUSH: Nav2 angles + reactive drive all failed '
            '(robot at %.1fm from exit)'
            % robot_exit_dist
        )
        return False

    def _find_frontier_push_target(
        self,
        robot_xy: tuple[float, float],
        robot_exit_dist: float,
    ) -> Optional[Point]:
        """Find the best frontier cell to push exploration toward the exit.

        Two-phase approach:
        1. BFS from robot cell to find all reachable free cells (connected component).
        2. Among reachable frontier cells, score by exit proximity, angular alignment,
           and unexplored volume beyond the frontier.  Skip previously-tried targets.

        Returns the (x, y) world coordinates of the best untried frontier target, or None.
        """
        if self.map_view is None:
            return None

        res = self.map_view.info.resolution
        robot_cell = self.map_view.world_to_cell(robot_xy[0], robot_xy[1])

        # Direction from robot to exit for angular bias
        exit_angle = math.atan2(self.exit_y - robot_xy[1], self.exit_x - robot_xy[0])

        # Phase 1: BFS to find all free cells reachable from the robot.
        # This ensures we only consider frontier cells the robot can actually
        # navigate to through the SLAM map, avoiding targets separated by walls.
        reachable: set[tuple[int, int]] = set()
        bfs_queue: list[tuple[int, int]] = [robot_cell]
        reachable.add(robot_cell)
        scan_radius_cells = int(math.ceil(self.frontier_push_scan_radius_m / res))
        rx, ry = robot_cell

        while bfs_queue:
            cx, cy = bfs_queue.pop(0)
            for ndx, ndy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = cx + ndx, cy + ndy
                # Stay within scan radius
                if abs(nx - rx) > scan_radius_cells or abs(ny - ry) > scan_radius_cells:
                    continue
                neighbor = (nx, ny)
                if neighbor in reachable:
                    continue
                if not self.map_view.in_bounds(neighbor):
                    continue
                if not self.map_view.is_free(neighbor):
                    continue
                reachable.add(neighbor)
                bfs_queue.append(neighbor)

        self.get_logger().info(
            'FRONTIER PUSH: BFS found %d reachable free cells within %.0fm radius'
            % (len(reachable), self.frontier_push_scan_radius_m)
        )

        # Phase 2: Score reachable frontier cells.
        stride = max(3, int(0.15 / res))
        unknown_check_r = int(2.0 / res)  # 2m radius for unknown volume count
        candidates: list[tuple[float, float, float, int]] = []  # (x, y, score, unknown_count)

        for cell in reachable:
            # Only evaluate at stride intervals to keep scoring fast
            if (cell[0] - rx) % stride != 0 or (cell[1] - ry) % stride != 0:
                continue

            # Check if this is a frontier cell (free with at least one unknown neighbor)
            has_unknown_neighbor = False
            for ndx, ndy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                neighbor = (cell[0] + ndx, cell[1] + ndy)
                if self.map_view.is_unknown(neighbor):
                    has_unknown_neighbor = True
                    break
            if not has_unknown_neighbor:
                continue

            # Skip previously tried targets using 1m grid dedup
            # (20 cells = 1m at 0.05 resolution — coarse enough to avoid nearby re-tries)
            dedup_key = (cell[0] // 20, cell[1] // 20)
            if dedup_key in self.frontier_push_tried_targets:
                continue

            # Convert to world coordinates
            wx, wy = self.map_view.cell_to_world(cell[0], cell[1])

            # Reject frontier targets outside the maze interior (e.g. exterior
            # space west of the entrance opening).
            if not self._target_in_maze(wx, wy):
                continue

            # Use relaxed clearance: allow unknown cells within radius
            if not self.map_view.world_point_has_clearance(
                wx, wy, self.clearance_radius_m, unknown_is_safe=True,
            ):
                continue

            # Distance from robot
            dist_from_robot = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if dist_from_robot < 1.0:
                continue  # Too close — DFS should handle nearby frontiers
            if dist_from_robot > 12.0:
                continue  # Too far — Nav2 unlikely to plan path through unknown

            # Count unknown cells in a 2m radius — measures unexplored volume
            unknown_count = 0
            for udy in range(-unknown_check_r, unknown_check_r + 1, stride):
                for udx in range(-unknown_check_r, unknown_check_r + 1, stride):
                    uc = (cell[0] + udx, cell[1] + udy)
                    if self.map_view.is_unknown(uc):
                        unknown_count += 1

            # Exit proximity score: how much closer to exit vs robot (0 to ~1)
            cell_exit_dist = math.hypot(wx - self.exit_x, wy - self.exit_y)
            closeness = max(0.0, (robot_exit_dist - cell_exit_dist) / max(robot_exit_dist, 0.01))

            # Angular alignment with exit direction (0 = opposite, 1 = aligned)
            cell_angle = math.atan2(wy - robot_xy[1], wx - robot_xy[0])
            angle_diff = abs(cell_angle - exit_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            angular_score = max(0.0, 1.0 - angle_diff / math.pi)

            # Combined score: exit proximity + alignment + unexplored volume - distance
            # Heavy weight on exit proximity to avoid being dominated by unknown volume
            # in areas far from exit. Unknown volume is a tiebreaker, not the main driver.
            score = (
                closeness * 5.0          # 0-5 points for being closer to exit (MAIN driver)
                + angular_score * 3.0    # 0-3 points for exit-direction alignment
                + unknown_count * 0.002  # small bonus for large unknown areas (tiebreaker)
                - dist_from_robot * 0.1  # moderate distance penalty to prefer nearby frontiers
            )
            candidates.append((wx, wy, score, unknown_count))

        if not candidates:
            return None

        # Pick the best-scoring untried candidate
        candidates.sort(key=lambda c: c[2], reverse=True)
        best = candidates[0]
        # Log top 3 for debugging
        for i, c in enumerate(candidates[:3]):
            self.get_logger().info(
                'FRONTIER PUSH: candidate #%d at (%.3f, %.3f) score=%.2f '
                'unknown_vol=%d (%.1fm from exit, %.1fm from robot)'
                % (i + 1, c[0], c[1], c[2], c[3],
                   math.hypot(c[0] - self.exit_x, c[1] - self.exit_y),
                   math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]))
            )
        self.get_logger().info(
            'FRONTIER PUSH: %d candidates found (tried=%d, reachable=%d); '
            'best at (%.3f, %.3f) score=%.2f'
            % (len(candidates), len(self.frontier_push_tried_targets),
               len(reachable), best[0], best[1], best[2])
        )
        return (best[0], best[1])

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
