#!/usr/bin/env python3
"""Frontier exploration node for Tugbot SLAM + Nav2.

Phase 5 kept the Phase 3/4 launch contract intact while fixing premature
completion: raw frontier clusters and valid navigation candidates are tracked
separately, strict search falls back to relaxed search, and recovery scans are
required before final completion.

Phase 6 adds coverage cleanup mode for residual unknown regions: when normal
frontier exploration is nearly done, the node detects residual unknown cluster
regions near known free space and sends safe cleanup observation goals. Phase 14
defaults avoid spin-based supplemental scans to protect SLAM map consistency.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import subprocess
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin
import tf2_ros

Cell = Tuple[int, int]
Point = Tuple[float, float]
RobotPose = Tuple[float, float, float]
MapCounts = Tuple[int, int, int]


@dataclass
class SearchConfig:
    mode: str
    min_cluster_size: int
    min_obstacle_distance_m: float
    max_goal_distance_m: float


@dataclass
class FrontierCandidate:
    cells: List[Cell]
    target_cell: Cell
    target_xy: Point
    distance_m: float
    score: float
    search_mode: str
    clearance_cells: int
    free_neighbor_count: int


@dataclass
class UnknownCluster:
    cells: List[Cell]
    center_cell: Cell
    center_xy: Point
    boundary_free_neighbor_count: int


@dataclass
class CleanupCandidate:
    cluster: UnknownCluster
    target_cell: Cell
    target_xy: Point
    yaw: float
    distance_m: float
    score: float
    clearance_cells: int
    free_neighbor_count: int


@dataclass
class WallCluster:
    cells: List[Cell]
    bbox: Tuple[int, int, int, int]
    center_cell: Cell
    center_xy: Point
    length_m: float
    thickness_m: float
    aspect_ratio: float
    orientation: str


@dataclass
class PerimeterWaypoint:
    target_cell: Cell
    target_xy: Point
    yaw: float
    source_cluster_index: int
    clearance_cells: int
    score: float


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.action_name = self.declare_parameter('action_name', '/navigate_to_pose').value
        self.spin_action_name = self.declare_parameter('spin_action_name', '/spin').value

        self.min_cluster_size = int(self.declare_parameter('min_cluster_size', 5).value)
        self.min_obstacle_distance_m = float(self.declare_parameter('min_obstacle_distance_m', 0.45).value)
        self.min_goal_distance_m = float(self.declare_parameter('min_goal_distance_m', 0.5).value)
        self.max_goal_distance_m = float(self.declare_parameter('max_goal_distance_m', 4.0).value)
        self.relaxed_min_obstacle_distance_m = float(self.declare_parameter('relaxed_min_obstacle_distance_m', 0.35).value)
        self.relaxed_max_goal_distance_m = float(self.declare_parameter('relaxed_max_goal_distance_m', 8.0).value)
        self.relaxed_min_cluster_size = int(self.declare_parameter('relaxed_min_cluster_size', 3).value)
        self.max_candidates_per_cluster = int(self.declare_parameter('max_candidates_per_cluster', 3).value)

        self.goal_timeout_sec = float(self.declare_parameter('goal_timeout_sec', 60.0).value)
        self.blacklist_radius_m = float(self.declare_parameter('blacklist_radius_m', 0.5).value)
        self.exploration_rate_hz = float(self.declare_parameter('exploration_rate_hz', 0.5).value)
        self.finish_no_frontier_cycles = int(self.declare_parameter('finish_no_frontier_cycles', 5).value)
        self.max_goals = int(self.declare_parameter('max_goals', 20).value)
        self.min_goals_before_finish = int(self.declare_parameter('min_goals_before_finish', 8).value)
        self.min_frontier_clusters_before_finish = int(self.declare_parameter('min_frontier_clusters_before_finish', 5).value)
        self.recovery_scan_attempts = int(self.declare_parameter('recovery_scan_attempts', 3).value)
        self.relaxed_retry_attempts = int(self.declare_parameter('relaxed_retry_attempts', 3).value)
        self.map_stable_cycles_before_finish = int(self.declare_parameter('map_stable_cycles_before_finish', 3).value)
        self.map_change_free_threshold = int(self.declare_parameter('map_change_free_threshold', 50).value)
        self.map_change_unknown_threshold = int(self.declare_parameter('map_change_unknown_threshold', 50).value)
        self.enable_recovery_scan = bool(self.declare_parameter('enable_recovery_scan', False).value)
        self.recovery_spin_angle = float(self.declare_parameter('recovery_spin_angle', 1.57).value)
        self.recovery_wait_after_scan_sec = float(self.declare_parameter('recovery_wait_after_scan_sec', 2.0).value)

        self.enable_cleanup_mode = bool(self.declare_parameter('enable_cleanup_mode', True).value)
        self.cleanup_unknown_cluster_min_size = int(self.declare_parameter('cleanup_unknown_cluster_min_size', 30).value)
        self.cleanup_max_unknown_clusters = int(self.declare_parameter('cleanup_max_unknown_clusters', 5).value)
        self.cleanup_search_radius_min_m = float(self.declare_parameter('cleanup_search_radius_min_m', 0.5).value)
        self.cleanup_search_radius_max_m = float(self.declare_parameter('cleanup_search_radius_max_m', 2.0).value)
        self.cleanup_min_obstacle_distance_m = float(self.declare_parameter('cleanup_min_obstacle_distance_m', 0.35).value)
        self.cleanup_goal_timeout_sec = float(self.declare_parameter('cleanup_goal_timeout_sec', 60.0).value)
        self.cleanup_spin_after_goal = bool(self.declare_parameter('cleanup_spin_after_goal', False).value)
        self.cleanup_spin_angle = float(self.declare_parameter('cleanup_spin_angle', 1.57).value)
        self.cleanup_wait_after_spin_sec = float(self.declare_parameter('cleanup_wait_after_spin_sec', 2.0).value)
        self.target_unknown_ratio = float(self.declare_parameter('target_unknown_ratio', 0.03).value)
        self.max_cleanup_goals = int(self.declare_parameter('max_cleanup_goals', 5).value)

        self.exploration_strategy = str(self.declare_parameter('exploration_strategy', 'frontier').value)
        if self.exploration_strategy not in ('frontier', 'perimeter_then_frontier'):
            self.get_logger().warn(
                "Unsupported exploration_strategy=%s; falling back to frontier" % self.exploration_strategy
            )
            self.exploration_strategy = 'frontier'
        self.perimeter_enable_initial_spin = bool(self.declare_parameter('perimeter_enable_initial_spin', False).value)
        self.perimeter_spin_angle = float(self.declare_parameter('perimeter_spin_angle', 1.57).value)
        self.perimeter_wall_min_cluster_size = int(self.declare_parameter('perimeter_wall_min_cluster_size', 80).value)
        self.perimeter_wall_min_length_m = float(self.declare_parameter('perimeter_wall_min_length_m', 1.5).value)
        self.perimeter_wall_aspect_ratio_min = float(self.declare_parameter('perimeter_wall_aspect_ratio_min', 4.0).value)
        self.perimeter_wall_offset_m = float(self.declare_parameter('perimeter_wall_offset_m', 0.75).value)
        self.perimeter_waypoint_spacing_m = float(self.declare_parameter('perimeter_waypoint_spacing_m', 1.0).value)
        self.perimeter_max_waypoints = int(self.declare_parameter('perimeter_max_waypoints', 20).value)
        self.perimeter_direction = str(self.declare_parameter('perimeter_direction', 'ccw').value).lower()
        if self.perimeter_direction not in ('cw', 'ccw'):
            self.get_logger().warn('Unsupported perimeter_direction=%s; falling back to ccw' % self.perimeter_direction)
            self.perimeter_direction = 'ccw'
        self.perimeter_switch_to_frontier_after_done = bool(self.declare_parameter('perimeter_switch_to_frontier_after_done', True).value)
        self.perimeter_goal_timeout_sec = float(self.declare_parameter('perimeter_goal_timeout_sec', 60.0).value)

        self.goal_settle_sec = float(self.declare_parameter('goal_settle_sec', 2.0).value)
        self.save_map = bool(self.declare_parameter('save_map', False).value)
        self.map_save_path = str(self.declare_parameter(
            'map_save_path',
            '/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam',
        ).value)
        self.map_save_timeout_sec = float(self.declare_parameter('map_save_timeout_sec', 30.0).value)
        self.map_saved = False

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_active = False
        self.recovery_active = False
        self.goal_sent_time: Optional[Time] = None
        self.active_goal_kind = 'frontier'
        self.active_goal_timeout_sec = self.goal_timeout_sec
        self.goal_handle = None
        self.recovery_handle = None
        self.active_spin_kind = 'recovery'
        self.active_spin_wait_after_sec = self.recovery_wait_after_scan_sec
        self.goal_count = 0
        self.goal_success_count = 0
        self.goal_failure_count = 0
        self.cleanup_goals_started = 0
        self.cleanup_goals_completed = 0
        self.cleanup_goal_failure_count = 0
        self.cleanup_spins_started = 0
        self.cleanup_spins_completed = 0
        self.cleanup_spin_failure_count = 0
        self.no_frontier_cycles = 0
        self.recovery_no_candidate_cycles = 0
        self.relaxed_no_candidate_cycles = 0
        self.recovery_scans_completed = 0
        self.relaxed_search_executed = False
        self.exploration_done = False
        self.blacklist: List[Point] = []
        self.last_goal_xy: Optional[Point] = None
        self.wait_until: Optional[Time] = None
        self.last_map_counts: Optional[MapCounts] = None
        self.map_stable_cycles = 0
        self.last_map_delta: MapCounts = (0, 0, 0)
        self.last_map_counts_snapshot: Optional[MapCounts] = None
        self.last_unknown_total_count = 0
        self.last_unknown_ratio = 1.0
        self.last_unknown_cluster_count = 0
        self.last_cleanup_candidate_count = 0
        self.cleanup_mode = False
        self.recovery_mode = False
        self.perimeter_phase = 'pending_initial_spin'
        if self.exploration_strategy != 'perimeter_then_frontier':
            self.perimeter_phase = 'frontier'
        elif not self.perimeter_enable_initial_spin:
            self.perimeter_phase = 'detect_walls'
        self.perimeter_initial_spin_started = False
        self.perimeter_waypoints: List[PerimeterWaypoint] = []
        self.perimeter_waypoint_index = 0
        self.perimeter_success_count = 0
        self.perimeter_failure_count = 0
        self.perimeter_rejected_waypoint_count = 0
        self.perimeter_wall_cluster_count = 0
        self.perimeter_detection_attempted = False
        self.perimeter_initial_spin_skip_logged = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.spin_client = ActionClient(self, Spin, self.spin_action_name)

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, self._map_qos_profile())
        timer_period = 1.0 / max(self.exploration_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self._explore_once)

        self.get_logger().info(
            'frontier_explorer started: map_topic=%s action_name=%s spin_action_name=%s '
            'min_obstacle_distance_m=%.2f min_goal_distance_m=%.2f max_goal_distance_m=%.2f '
            'relaxed_min_obstacle_distance_m=%.2f relaxed_max_goal_distance_m=%.2f '
            'min_cluster_size=%d relaxed_min_cluster_size=%d max_goals=%d min_goals_before_finish=%d '
            'min_frontier_clusters_before_finish=%d recovery_scan_attempts=%d relaxed_retry_attempts=%d '
            'map_stable_cycles_before_finish=%d map_change_free_threshold=%d map_change_unknown_threshold=%d '
            'enable_recovery_scan=%s recovery_spin_angle=%.2f recovery_wait_after_scan_sec=%.1f '
            'enable_cleanup_mode=%s cleanup_unknown_cluster_min_size=%d cleanup_max_unknown_clusters=%d '
            'cleanup_search_radius_min_m=%.2f cleanup_search_radius_max_m=%.2f cleanup_min_obstacle_distance_m=%.2f '
            'cleanup_goal_timeout_sec=%.1f cleanup_spin_after_goal=%s cleanup_spin_angle=%.2f cleanup_wait_after_spin_sec=%.1f '
            'target_unknown_ratio=%.3f max_cleanup_goals=%d exploration_strategy=%s '
            'perimeter_enable_initial_spin=%s perimeter_spin_angle=%.2f perimeter_wall_min_cluster_size=%d '
            'perimeter_wall_min_length_m=%.2f perimeter_wall_aspect_ratio_min=%.2f perimeter_wall_offset_m=%.2f '
            'perimeter_waypoint_spacing_m=%.2f perimeter_max_waypoints=%d perimeter_direction=%s '
            'perimeter_switch_to_frontier_after_done=%s perimeter_goal_timeout_sec=%.1f '
            'max_vel_theta limited by Nav2 params_file (MPPI wz_max / velocity_smoother theta defaults) '
            'save_map=%s map_save_path=%s'
            % (
                self.map_topic,
                self.action_name,
                self.spin_action_name,
                self.min_obstacle_distance_m,
                self.min_goal_distance_m,
                self.max_goal_distance_m,
                self.relaxed_min_obstacle_distance_m,
                self.relaxed_max_goal_distance_m,
                self.min_cluster_size,
                self.relaxed_min_cluster_size,
                self.max_goals,
                self.min_goals_before_finish,
                self.min_frontier_clusters_before_finish,
                self.recovery_scan_attempts,
                self.relaxed_retry_attempts,
                self.map_stable_cycles_before_finish,
                self.map_change_free_threshold,
                self.map_change_unknown_threshold,
                self.enable_recovery_scan,
                self.recovery_spin_angle,
                self.recovery_wait_after_scan_sec,
                self.enable_cleanup_mode,
                self.cleanup_unknown_cluster_min_size,
                self.cleanup_max_unknown_clusters,
                self.cleanup_search_radius_min_m,
                self.cleanup_search_radius_max_m,
                self.cleanup_min_obstacle_distance_m,
                self.cleanup_goal_timeout_sec,
                self.cleanup_spin_after_goal,
                self.cleanup_spin_angle,
                self.cleanup_wait_after_spin_sec,
                self.target_unknown_ratio,
                self.max_cleanup_goals,
                self.exploration_strategy,
                self.perimeter_enable_initial_spin,
                self.perimeter_spin_angle,
                self.perimeter_wall_min_cluster_size,
                self.perimeter_wall_min_length_m,
                self.perimeter_wall_aspect_ratio_min,
                self.perimeter_wall_offset_m,
                self.perimeter_waypoint_spacing_m,
                self.perimeter_max_waypoints,
                self.perimeter_direction,
                self.perimeter_switch_to_frontier_after_done,
                self.perimeter_goal_timeout_sec,
                self.save_map,
                self.map_save_path,
            )
        )
        self.get_logger().info(
            'perimeter initial spin enabled/disabled: enabled=%s angle=%.2f'
            % (self.perimeter_enable_initial_spin, self.perimeter_spin_angle)
        )
        if self.exploration_strategy == 'perimeter_then_frontier' and not self.perimeter_enable_initial_spin:
            self.get_logger().warn(
                'initial spin disabled; skipping; spin skipped because disabled; perimeter_then_frontier will use current map and fallback to frontier if insufficient map'
            )


    @staticmethod
    def _map_qos_profile() -> QoSProfile:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _explore_once(self) -> None:
        if self.exploration_done:
            return
        if self.goal_active:
            self._check_goal_timeout()
            return
        if self.recovery_active:
            return
        now = self.get_clock().now()
        if self.wait_until is not None and now < self.wait_until:
            return
        self.wait_until = None
        if self.map_msg is None:
            self.get_logger().info('Waiting for occupancy grid on %s' % self.map_topic)
            return
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('Waiting for TF %s -> %s' % (self.map_frame, self.base_frame))
            return
        robot_xy = (robot_pose[0], robot_pose[1])
        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose action server %s' % self.action_name)
            self.action_client.wait_for_server(timeout_sec=0.1)
            return

        if self.exploration_strategy == 'perimeter_then_frontier' and self.perimeter_phase != 'frontier':
            if self._run_perimeter_then_frontier(robot_pose):
                return

        counts, deltas = self._update_map_stability(self.map_msg)
        strict_candidates, raw_cluster_count = self._find_frontier_candidates(self.map_msg, robot_xy, mode='strict')
        candidates = strict_candidates
        search_mode = 'strict'

        if not strict_candidates and raw_cluster_count > 0:
            self.relaxed_search_executed = True
            relaxed_candidates, relaxed_raw_cluster_count = self._find_frontier_candidates(self.map_msg, robot_xy, mode='relaxed')
            raw_cluster_count = max(raw_cluster_count, relaxed_raw_cluster_count)
            if relaxed_candidates:
                candidates = relaxed_candidates
                search_mode = 'relaxed'
            else:
                self.relaxed_no_candidate_cycles += 1

        cleanup_clusters = self._find_unknown_clusters(self.map_msg)
        cleanup_candidates = self._find_cleanup_candidates(self.map_msg, robot_xy, cleanup_clusters)
        unknown_total_count = counts[0]
        total_cells = max(1, int(self.map_msg.info.width) * int(self.map_msg.info.height))
        unknown_ratio = float(unknown_total_count) / float(total_cells)
        unknown_cluster_count = len(cleanup_clusters)
        cleanup_candidate_count = len(cleanup_candidates)
        self.last_unknown_total_count = unknown_total_count
        self.last_unknown_ratio = unknown_ratio
        self.last_unknown_cluster_count = unknown_cluster_count
        self.last_cleanup_candidate_count = cleanup_candidate_count

        self.recovery_mode = raw_cluster_count > self.min_frontier_clusters_before_finish and not candidates
        self.cleanup_mode = self._should_enter_cleanup_mode(
            valid_candidate_count=len(candidates),
            raw_cluster_count=raw_cluster_count,
            unknown_ratio=unknown_ratio,
            cleanup_candidate_count=cleanup_candidate_count,
            reached_max_goals=self._frontier_goal_budget_reached(),
        )
        self._log_frontier_state(
            raw_cluster_count=raw_cluster_count,
            valid_candidate_count=len(candidates),
            search_mode=search_mode,
            counts=counts,
            deltas=deltas,
            unknown_total_count=unknown_total_count,
            unknown_ratio=unknown_ratio,
            unknown_cluster_count=unknown_cluster_count,
            cleanup_candidate_count=cleanup_candidate_count,
        )

        if self.cleanup_mode and cleanup_candidates:
            self.no_frontier_cycles = 0 if candidates else self.no_frontier_cycles
            chosen_cleanup = cleanup_candidates[0]
            self.get_logger().warn(
                'Selected cleanup goal #%d: x=%.3f y=%.3f yaw=%.3f residual unknown cluster size=%d '
                'distance=%.3f score=%.3f clearance_cells=%d free_neighbor_count=%d unknown_ratio=%.4f cleanup_candidate_count=%d'
                % (
                    self.cleanup_goals_started + 1,
                    chosen_cleanup.target_xy[0],
                    chosen_cleanup.target_xy[1],
                    chosen_cleanup.yaw,
                    len(chosen_cleanup.cluster.cells),
                    chosen_cleanup.distance_m,
                    chosen_cleanup.score,
                    chosen_cleanup.clearance_cells,
                    chosen_cleanup.free_neighbor_count,
                    unknown_ratio,
                    cleanup_candidate_count,
                )
            )
            self._send_goal(chosen_cleanup.target_xy, yaw=chosen_cleanup.yaw, goal_kind='cleanup')
            return

        if self._frontier_goal_budget_reached():
            if cleanup_candidate_count > 0 and self.enable_cleanup_mode:
                self.get_logger().warn(
                    'Cleanup candidates remain; postponing final map save: cleanup_candidate_count > 0 (%d), '
                    'cleanup_goals_started=%d/%d unknown_ratio=%.4f target_unknown_ratio=%.4f'
                    % (
                        cleanup_candidate_count,
                        self.cleanup_goals_started,
                        self.max_cleanup_goals,
                        unknown_ratio,
                        self.target_unknown_ratio,
                    )
                )
                if self.cleanup_goals_started >= self.max_cleanup_goals:
                    self._mark_exploration_done('frontier goal budget reached and cleanup goal budget exhausted')
                return
            self._mark_exploration_done('frontier goal budget reached and cleanup gate satisfied')
            return

        if not candidates:
            self._handle_no_candidate(raw_cluster_count, cleanup_candidate_count, unknown_ratio)
            return

        self.no_frontier_cycles = 0
        self.recovery_no_candidate_cycles = 0
        self.relaxed_no_candidate_cycles = 0
        self.recovery_scans_completed = 0
        self.recovery_mode = False

        chosen = candidates[0]
        self.get_logger().info(
            'Selected frontier goal #%d: x=%.3f y=%.3f cluster_size=%d distance=%.3f score=%.3f '
            'search_mode=%s clearance_cells=%d free_neighbor_count=%d'
            % (
                self.goal_count + 1,
                chosen.target_xy[0],
                chosen.target_xy[1],
                len(chosen.cells),
                chosen.distance_m,
                chosen.score,
                chosen.search_mode,
                chosen.clearance_cells,
                chosen.free_neighbor_count,
            )
        )
        self._send_goal(chosen.target_xy, goal_kind='frontier')

    def _frontier_goal_budget_reached(self) -> bool:
        return self.max_goals > 0 and self.goal_count >= self.max_goals

    def _log_frontier_state(
        self,
        raw_cluster_count: int,
        valid_candidate_count: int,
        search_mode: str,
        counts: MapCounts,
        deltas: MapCounts,
        unknown_total_count: int,
        unknown_ratio: float,
        unknown_cluster_count: int,
        cleanup_candidate_count: int,
    ) -> None:
        # Phase 5 log contract examples: search_mode=strict, search_mode=relaxed.
        # Phase 5 behavior contract: relaxed search and recovery scan are separate gates.
        # Phase 6 log contract: unknown_total_count, unknown_ratio, unknown_cluster_count,
        # cleanup_candidate_count, cleanup mode, cleanup_goals_completed, cleanup_spins_completed.
        unknown, free, occupied = counts
        delta_unknown, delta_free, delta_occupied = deltas
        self.get_logger().info(
            'Map %dx%d res=%.3f, raw frontier cluster count=%d, valid candidate count=%d, '
            'frontier_clusters=%d, valid_candidates=%d, finish_no_frontier_cycles=%d/%d, '
            'recovery_no_candidate_cycles=%d/%d, relaxed_no_candidate_cycles=%d/%d, '
            'recovery_scans_completed=%d/%d, recovery_mode=%s, search_mode=%s, blacklist=%d, '
            'completed_goals=%d/%d, min_goals_before_finish=%d, map_stable_cycles=%d/%d, '
            'unknown_total_count=%d unknown_ratio=%.4f unknown_cluster_count=%d cleanup_candidate_count=%d, '
            'cleanup mode=%s cleanup_goals_completed=%d/%d cleanup_spins_completed=%d, '
            'map_stats unknown=%d free=%d occupied=%d delta_unknown=%d delta_free=%d delta_occupied=%d'
            % (
                self.map_msg.info.width,
                self.map_msg.info.height,
                self.map_msg.info.resolution,
                raw_cluster_count,
                valid_candidate_count,
                raw_cluster_count,
                valid_candidate_count,
                self.no_frontier_cycles,
                self.finish_no_frontier_cycles,
                self.recovery_no_candidate_cycles,
                self.recovery_scan_attempts,
                self.relaxed_no_candidate_cycles,
                self.relaxed_retry_attempts,
                self.recovery_scans_completed,
                self.recovery_scan_attempts,
                self.recovery_mode,
                search_mode,
                len(self.blacklist),
                self.goal_count,
                self.max_goals,
                self.min_goals_before_finish,
                self.map_stable_cycles,
                self.map_stable_cycles_before_finish,
                unknown_total_count,
                unknown_ratio,
                unknown_cluster_count,
                cleanup_candidate_count,
                self.cleanup_mode,
                self.cleanup_goals_completed,
                self.max_cleanup_goals,
                self.cleanup_spins_completed,
                unknown,
                free,
                occupied,
                delta_unknown,
                delta_free,
                delta_occupied,
            )
        )

    def _should_enter_cleanup_mode(
        self,
        valid_candidate_count: int,
        raw_cluster_count: int,
        unknown_ratio: float,
        cleanup_candidate_count: int,
        reached_max_goals: bool,
    ) -> bool:
        if not self.enable_cleanup_mode:
            return False
        if self.cleanup_goals_started >= self.max_cleanup_goals:
            return False
        if unknown_ratio <= self.target_unknown_ratio:
            return False
        if cleanup_candidate_count <= 0:
            return False
        frontier_sparse = raw_cluster_count <= max(self.min_frontier_clusters_before_finish * 3, self.min_frontier_clusters_before_finish + 1)
        candidates_sparse = valid_candidate_count <= 2
        enough_goals = self.goal_count >= self.min_goals_before_finish
        near_goal_budget = self.max_goals > 0 and self.goal_count >= max(self.min_goals_before_finish, self.max_goals - 2)
        return enough_goals and (frontier_sparse or candidates_sparse or near_goal_budget or reached_max_goals)

    def _handle_no_candidate(self, raw_cluster_count: int, cleanup_candidate_count: int = 0, unknown_ratio: float = 0.0) -> None:
        # Keep these exact phrases for Phase 5 contract readability:
        # valid_candidates == 0
        # frontier_clusters <= min_frontier_clusters_before_finish
        if raw_cluster_count <= self.min_frontier_clusters_before_finish:
            self.no_frontier_cycles += 1
            self.get_logger().info(
                'No valid candidate and raw frontier clusters are near finish threshold: '
                'frontier_clusters=%d <= min_frontier_clusters_before_finish=%d, '
                'finish_no_frontier_cycles=%d/%d'
                % (
                    raw_cluster_count,
                    self.min_frontier_clusters_before_finish,
                    self.no_frontier_cycles,
                    self.finish_no_frontier_cycles,
                )
            )
        else:
            self.recovery_no_candidate_cycles += 1
            self.get_logger().warn(
                'No valid candidate but raw frontier clusters remain: raw_cluster_count > self.min_frontier_clusters_before_finish '
                '(%d > %d). Entering recovery exploration state; recovery_no_candidate_cycles=%d/%d. '
                'Will not finish solely because valid_candidates == 0.'
                % (
                    raw_cluster_count,
                    self.min_frontier_clusters_before_finish,
                    self.recovery_no_candidate_cycles,
                    self.recovery_scan_attempts,
                )
            )
            if not self.enable_recovery_scan:
                self.get_logger().warn(
                    'recovery scan disabled; not spinning; valid_candidates == 0 will continue with map-stability/frontier gates'
                )
            elif self._should_start_recovery_scan(raw_cluster_count):
                self._start_recovery_scan()
                return

        if cleanup_candidate_count > 0 and self.enable_cleanup_mode and unknown_ratio > self.target_unknown_ratio:
            self.get_logger().warn(
                'Cleanup candidates remain; postponing final map save: cleanup_candidate_count > 0 (%d), unknown_ratio=%.4f target_unknown_ratio=%.4f'
                % (cleanup_candidate_count, unknown_ratio, self.target_unknown_ratio)
            )
            return
        if self._finish_conditions_met(raw_cluster_count):
            self._mark_exploration_done(self._finish_reason(raw_cluster_count))
        else:
            self.get_logger().info(
                'Exploration continues after no-candidate cycle: %s'
                % self._finish_gate_status(raw_cluster_count)
            )

    def _should_start_recovery_scan(self, raw_cluster_count: int) -> bool:
        if not self.enable_recovery_scan:
            return False
        if raw_cluster_count <= self.min_frontier_clusters_before_finish:
            return False
        if self.recovery_scans_completed >= self.recovery_scan_attempts:
            return False
        return True

    def _finish_conditions_met(self, raw_cluster_count: int) -> bool:
        valid_candidates = 0
        frontier_clusters = raw_cluster_count
        raw_frontiers_few = frontier_clusters <= self.min_frontier_clusters_before_finish
        map_stable = self.map_stable_cycles >= self.map_stable_cycles_before_finish
        few_frontier_exception = raw_frontiers_few and map_stable and self.no_frontier_cycles >= self.finish_no_frontier_cycles
        completed_goal_gate = self.goal_count >= self.min_goals_before_finish or few_frontier_exception
        no_candidate_gate = valid_candidates == 0
        frontier_gate = raw_frontiers_few or self.recovery_scans_completed >= self.recovery_scan_attempts or not self.enable_recovery_scan
        if raw_frontiers_few:
            cycle_gate = self.no_frontier_cycles >= self.finish_no_frontier_cycles
        else:
            cycle_gate = self.recovery_no_candidate_cycles >= self.recovery_scan_attempts
        relaxed_gate = raw_cluster_count == 0 or (
            self.relaxed_search_executed and self.relaxed_no_candidate_cycles >= self.relaxed_retry_attempts
        )
        recovery_gate = raw_frontiers_few or not self.enable_recovery_scan or self.recovery_scans_completed >= self.recovery_scan_attempts
        return (
            completed_goal_gate
            and no_candidate_gate
            and frontier_gate
            and map_stable
            and relaxed_gate
            and recovery_gate
            and cycle_gate
        )

    def _finish_gate_status(self, raw_cluster_count: int) -> str:
        raw_frontiers_few = raw_cluster_count <= self.min_frontier_clusters_before_finish
        map_stable = self.map_stable_cycles >= self.map_stable_cycles_before_finish
        few_frontier_exception = raw_frontiers_few and map_stable and self.no_frontier_cycles >= self.finish_no_frontier_cycles
        return (
            'completed_goals=%d min_goals_before_finish=%d completed_goals_gate=%s; '
            'frontier_clusters=%d min_frontier_clusters_before_finish=%d raw_frontiers_few=%s; '
            'finish_no_frontier_cycles=%d/%d recovery_no_candidate_cycles=%d/%d; '
            'relaxed_search_executed=%s relaxed_no_candidate_cycles=%d/%d; '
            'recovery_scans_completed=%d/%d enable_recovery_scan=%s; '
            'map_stable_cycles=%d/%d map_stable=%s'
            % (
                self.goal_count,
                self.min_goals_before_finish,
                self.goal_count >= self.min_goals_before_finish or few_frontier_exception,
                raw_cluster_count,
                self.min_frontier_clusters_before_finish,
                raw_frontiers_few,
                self.no_frontier_cycles,
                self.finish_no_frontier_cycles,
                self.recovery_no_candidate_cycles,
                self.recovery_scan_attempts,
                self.relaxed_search_executed,
                self.relaxed_no_candidate_cycles,
                self.relaxed_retry_attempts,
                self.recovery_scans_completed,
                self.recovery_scan_attempts,
                self.enable_recovery_scan,
                self.map_stable_cycles,
                self.map_stable_cycles_before_finish,
                map_stable,
            )
        )

    def _finish_reason(self, raw_cluster_count: int) -> str:
        return (
            'strict completion gates satisfied: valid_candidates=0 frontier_clusters=%d '
            'completed_goals=%d/%d finish_no_frontier_cycles=%d/%d recovery_no_candidate_cycles=%d/%d '
            'relaxed_search_executed=%s relaxed_no_candidate_cycles=%d/%d recovery_scans_completed=%d/%d '
            'map_stable_cycles=%d/%d'
            % (
                raw_cluster_count,
                self.goal_count,
                self.min_goals_before_finish,
                self.no_frontier_cycles,
                self.finish_no_frontier_cycles,
                self.recovery_no_candidate_cycles,
                self.recovery_scan_attempts,
                self.relaxed_search_executed,
                self.relaxed_no_candidate_cycles,
                self.relaxed_retry_attempts,
                self.recovery_scans_completed,
                self.recovery_scan_attempts,
                self.map_stable_cycles,
                self.map_stable_cycles_before_finish,
            )
        )

    def _lookup_robot_pose(self) -> Optional[RobotPose]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:  # tf2 exceptions vary by backend
            self.get_logger().debug('TF lookup failed: %s' % exc)
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (float(t.x), float(t.y), float(yaw))

    def _lookup_robot_xy(self) -> Optional[Point]:
        pose = self._lookup_robot_pose()
        if pose is None:
            return None
        return (pose[0], pose[1])

    def _send_goal(self, xy: Point, yaw: float = 0.0, goal_kind: str = 'frontier') -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(xy[0])
        goal_msg.pose.pose.position.y = float(xy[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)

        self.goal_active = True
        self.goal_sent_time = self.get_clock().now()
        self.active_goal_kind = goal_kind
        self.active_goal_timeout_sec = self.cleanup_goal_timeout_sec if goal_kind == 'cleanup' else self.goal_timeout_sec
        if goal_kind == 'cleanup':
            self.cleanup_goals_started += 1
        self.last_goal_xy = xy
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self._goal_feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            'NavigateToPose %s goal sent: x=%.3f y=%.3f yaw=%.3f timeout=%.1fs'
            % (goal_kind, xy[0], xy[1], yaw, self.active_goal_timeout_sec)
        )

    def _start_recovery_scan(self) -> None:
        self.recovery_scans_completed += 1
        self._start_spin(
            kind='recovery',
            target_yaw=self.recovery_spin_angle,
            wait_after_sec=self.recovery_wait_after_scan_sec,
            timeout_sec=self.goal_timeout_sec,
            attempt_index=self.recovery_scans_completed,
            attempt_limit=self.recovery_scan_attempts,
        )

    def _start_cleanup_spin(self) -> None:
        self.cleanup_spins_started += 1
        self._start_spin(
            kind='cleanup',
            target_yaw=self.cleanup_spin_angle,
            wait_after_sec=self.cleanup_wait_after_spin_sec,
            timeout_sec=self.cleanup_goal_timeout_sec,
            attempt_index=self.cleanup_spins_started,
            attempt_limit=self.max_cleanup_goals,
        )

    def _start_spin(
        self,
        kind: str,
        target_yaw: float,
        wait_after_sec: float,
        timeout_sec: float,
        attempt_index: int,
        attempt_limit: int,
    ) -> None:
        if not self.spin_client.server_is_ready():
            self.get_logger().warn(
                '%s spin requested but Spin action server %s is not ready; attempt=%d/%d, wait_after_spin=%.1fs'
                % (kind, self.spin_action_name, attempt_index, attempt_limit, wait_after_sec)
            )
            self.spin_client.wait_for_server(timeout_sec=0.1)
            self.wait_until = self.get_clock().now() + Duration(seconds=wait_after_sec)
            return

        goal_msg = Spin.Goal()
        goal_msg.target_yaw = float(target_yaw)
        allowance = max(timeout_sec, 1.0)
        goal_msg.time_allowance.sec = int(allowance)
        goal_msg.time_allowance.nanosec = int((allowance % 1.0) * 1e9)
        self.recovery_active = True
        self.active_spin_kind = kind
        self.active_spin_wait_after_sec = wait_after_sec
        future = self.spin_client.send_goal_async(goal_msg)
        future.add_done_callback(self._recovery_response_callback)
        self.get_logger().warn(
            'Starting %s spin: attempt=%d/%d target_yaw=%.2f wait_after_spin=%.1fs'
            % (kind, attempt_index, attempt_limit, target_yaw, wait_after_sec)
        )

    def _recovery_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error('%s spin send failed: %s' % (self.active_spin_kind, exc))
            self._finish_recovery_scan('send_exception')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('%s spin rejected' % self.active_spin_kind)
            self._finish_recovery_scan('rejected')
            return
        self.recovery_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._recovery_result_callback)
        self.get_logger().info('%s spin accepted' % self.active_spin_kind)

    def _recovery_result_callback(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
            spin_result = result.result
            error_code = int(getattr(spin_result, 'error_code', -1))
            error_msg = str(getattr(spin_result, 'error_msg', ''))
        except Exception as exc:
            self.get_logger().error('%s spin result exception: %s' % (self.active_spin_kind, exc))
            self._finish_recovery_scan('result_exception')
            return
        success = status == 4 and error_code == 0
        if self.active_spin_kind == 'cleanup':
            if success:
                self.cleanup_spins_completed += 1
            else:
                self.cleanup_spin_failure_count += 1
        self.get_logger().warn(
            '%s spin finished: status=%d error_code=%d error_msg=%s success=%s; waiting %.1fs for map update'
            % (self.active_spin_kind, status, error_code, error_msg, success, self.active_spin_wait_after_sec)
        )
        self._finish_recovery_scan('status_%d_error_%d' % (status, error_code))

    def _finish_recovery_scan(self, reason: str) -> None:
        self.get_logger().info('%s spin cleanup: reason=%s' % (self.active_spin_kind, reason))
        self.recovery_active = False
        self.recovery_handle = None
        self.wait_until = self.get_clock().now() + Duration(seconds=self.active_spin_wait_after_sec)
        self.active_spin_kind = 'recovery'
        self.active_spin_wait_after_sec = self.recovery_wait_after_scan_sec

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error('NavigateToPose goal send failed: %s' % exc)
            self._finish_goal(success=False, reason='send_exception')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            self._finish_goal(success=False, reason='rejected')
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)
        self.get_logger().info('NavigateToPose goal accepted')

    def _goal_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'NavigateToPose feedback: distance_remaining=%.3f recoveries=%d'
            % (float(feedback.distance_remaining), int(feedback.number_of_recoveries)),
            throttle_duration_sec=5.0,
        )

    def _goal_result_callback(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
            nav_result = result.result
            error_code = int(getattr(nav_result, 'error_code', -1))
            error_msg = str(getattr(nav_result, 'error_msg', ''))
        except Exception as exc:
            self.get_logger().error('NavigateToPose result exception: %s' % exc)
            self._finish_goal(success=False, reason='result_exception')
            return
        success = status == 4 and error_code == 0
        self.get_logger().info(
            'NavigateToPose result: status=%d error_code=%d error_msg=%s success=%s'
            % (status, error_code, error_msg, success)
        )
        self._finish_goal(success=success, reason='status_%d_error_%d' % (status, error_code))

    def _check_goal_timeout(self) -> None:
        if self.goal_sent_time is None:
            return
        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed < self.active_goal_timeout_sec:
            return
        self.get_logger().warn('NavigateToPose %s goal timed out after %.1fs' % (self.active_goal_kind, elapsed))
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._finish_goal(success=False, reason='timeout')

    def _finish_goal(self, success: bool, reason: str) -> None:
        goal_kind = self.active_goal_kind
        if goal_kind == 'cleanup':
            if success:
                self.cleanup_goals_completed += 1
                self.get_logger().warn(
                    'cleanup goal succeeded; cleanup_goals_completed=%d/%d unknown_ratio_before_recheck=%.4f'
                    % (self.cleanup_goals_completed, self.max_cleanup_goals, self.last_unknown_ratio)
                )
            else:
                self.cleanup_goal_failure_count += 1
                if self.last_goal_xy is not None:
                    self.blacklist.append(self.last_goal_xy)
                    self.get_logger().warn(
                        'cleanup goal failed (%s); blacklisting x=%.3f y=%.3f radius=%.2f'
                        % (reason, self.last_goal_xy[0], self.last_goal_xy[1], self.blacklist_radius_m)
                    )
        elif goal_kind == 'perimeter':
            # Perimeter goals are a Phase 13 perimeter-first pre-pass and must not consume
            # the existing frontier max_goals budget.
            pass
        elif success:
            self.goal_count += 1
            self.goal_success_count += 1
            self.get_logger().info('Goal succeeded; completed_goals=%d/%d' % (self.goal_count, self.max_goals))
        else:
            self.goal_failure_count += 1
            if self.last_goal_xy is not None:
                self.blacklist.append(self.last_goal_xy)
                self.get_logger().warn(
                    'Goal failed (%s); blacklisting x=%.3f y=%.3f radius=%.2f'
                    % (reason, self.last_goal_xy[0], self.last_goal_xy[1], self.blacklist_radius_m)
                )
        self.goal_active = False
        self.goal_handle = None
        self.goal_sent_time = None
        self.active_goal_timeout_sec = self.goal_timeout_sec
        self.active_goal_kind = 'frontier'
        self.last_goal_xy = None
        if goal_kind == 'perimeter':
            if success:
                self.perimeter_success_count += 1
                self.get_logger().info(
                    'perimeter waypoint succeeded: success=%d failure=%d current_index=%d/%d'
                    % (
                        self.perimeter_success_count,
                        self.perimeter_failure_count,
                        self.perimeter_waypoint_index,
                        len(self.perimeter_waypoints),
                    )
                )
            else:
                self.perimeter_failure_count += 1
                self.get_logger().warn(
                    'perimeter waypoint failed (%s); success=%d failure=%d current_index=%d/%d'
                    % (
                        reason,
                        self.perimeter_success_count,
                        self.perimeter_failure_count,
                        self.perimeter_waypoint_index,
                        len(self.perimeter_waypoints),
                    )
                )
            self.wait_until = self.get_clock().now() + Duration(seconds=self.goal_settle_sec)
            return
        if goal_kind == 'cleanup' and success and self.cleanup_spin_after_goal:
            self.wait_until = self.get_clock().now() + Duration(seconds=1.0)
            self.get_logger().warn('cleanup spin will start after cleanup goal settle: cleanup_spin_angle=%.2f' % self.cleanup_spin_angle)
            self._start_cleanup_spin()
        else:
            wait = self.cleanup_wait_after_spin_sec if goal_kind == 'cleanup' else self.goal_settle_sec
            if goal_kind == 'cleanup' and success and not self.cleanup_spin_after_goal:
                self.get_logger().warn(
                    'cleanup spin disabled; skipping; waiting %.1fs for map update from observation goal' % wait
                )
            self.wait_until = self.get_clock().now() + Duration(seconds=wait)

    def _run_perimeter_then_frontier(self, robot_pose: RobotPose) -> bool:
        # Phase 13 perimeter_then_frontier state machine:
        # initial spin -> perimeter wall cluster detection -> generated perimeter waypoints
        # -> NavigateToPose waypoints -> frontier/cleanup fallback.
        if self.perimeter_phase == 'pending_initial_spin':
            if self.perimeter_initial_spin_started:
                return True
            self.perimeter_initial_spin_started = True
            self.perimeter_phase = 'initial_spin'
            self.get_logger().warn(
                'perimeter_then_frontier initial spin starting: perimeter_spin_angle=%.2f; visual sensors remain outside navigation control'
                % self.perimeter_spin_angle
            )
            self._start_spin(
                kind='perimeter_initial',
                target_yaw=self.perimeter_spin_angle,
                wait_after_sec=self.recovery_wait_after_scan_sec,
                timeout_sec=self.perimeter_goal_timeout_sec,
                attempt_index=1,
                attempt_limit=1,
            )
            if not self.recovery_active:
                self.perimeter_phase = 'detect_walls'
            return True

        if self.perimeter_phase == 'initial_spin':
            # _finish_recovery_scan flips recovery_active off after Spin result. Once the
            # wait gate has elapsed, continue to wall extraction.
            if self.recovery_active:
                return True
            self.perimeter_phase = 'detect_walls'

        if self.perimeter_phase == 'detect_walls':
            if not self.perimeter_enable_initial_spin and not self.perimeter_initial_spin_skip_logged:
                self.perimeter_initial_spin_skip_logged = True
                self.get_logger().warn(
                    'initial spin disabled; skipping; spin skipped because disabled before wall cluster detection'
                )
            self.perimeter_detection_attempted = True
            wall_clusters = self._detect_perimeter_wall_clusters(self.map_msg)
            self.perimeter_wall_cluster_count = len(wall_clusters)
            self.get_logger().warn(
                'wall cluster detection result: detected=%d min_cluster_size=%d; fallback to frontier if insufficient map'
                % (len(wall_clusters), self.perimeter_wall_min_cluster_size)
            )
            waypoints, rejected = self._generate_perimeter_waypoints(self.map_msg, robot_pose, wall_clusters)
            self.perimeter_rejected_waypoint_count = rejected
            self.perimeter_waypoints = self._sort_perimeter_waypoints(waypoints, robot_pose)
            self.perimeter_waypoint_index = 0
            self.get_logger().warn(
                'generated perimeter waypoints: generated=%d accepted perimeter waypoints=%d rejected perimeter waypoints=%d direction=%s max=%d'
                % (
                    len(waypoints),
                    len(self.perimeter_waypoints),
                    rejected,
                    self.perimeter_direction,
                    self.perimeter_max_waypoints,
                )
            )
            if not self.perimeter_waypoints:
                self.get_logger().warn(
                    'Perimeter wall cluster detection produced no usable waypoints; fallback to frontier if insufficient map; switching to frontier fallback now'
                )
                self._switch_perimeter_to_frontier('no perimeter waypoints generated')
                return False
            self.perimeter_phase = 'execute_waypoints'

        if self.perimeter_phase == 'execute_waypoints':
            if self.perimeter_waypoint_index >= len(self.perimeter_waypoints):
                self._switch_perimeter_to_frontier('all perimeter waypoints processed')
                return False
            waypoint = self.perimeter_waypoints[self.perimeter_waypoint_index]
            self.perimeter_waypoint_index += 1
            self.active_goal_timeout_sec = self.perimeter_goal_timeout_sec
            self.get_logger().warn(
                'Executing perimeter waypoint %d/%d: x=%.3f y=%.3f yaw=%.3f source_cluster=%d clearance_cells=%d success=%d failure=%d'
                % (
                    self.perimeter_waypoint_index,
                    len(self.perimeter_waypoints),
                    waypoint.target_xy[0],
                    waypoint.target_xy[1],
                    waypoint.yaw,
                    waypoint.source_cluster_index,
                    waypoint.clearance_cells,
                    self.perimeter_success_count,
                    self.perimeter_failure_count,
                )
            )
            self._send_goal(waypoint.target_xy, yaw=waypoint.yaw, goal_kind='perimeter')
            self.active_goal_timeout_sec = self.perimeter_goal_timeout_sec
            return True

        return False

    def _switch_perimeter_to_frontier(self, reason: str) -> None:
        self.perimeter_phase = 'frontier'
        self.get_logger().warn(
            'Perimeter phase complete; switching to frontier: reason=%s detected wall clusters=%d generated perimeter waypoints=%d '
            'accepted perimeter waypoints=%d rejected perimeter waypoints=%d success=%d failure=%d switch_to_frontier=%s'
            % (
                reason,
                self.perimeter_wall_cluster_count,
                len(self.perimeter_waypoints) + self.perimeter_rejected_waypoint_count,
                len(self.perimeter_waypoints),
                self.perimeter_rejected_waypoint_count,
                self.perimeter_success_count,
                self.perimeter_failure_count,
                self.perimeter_switch_to_frontier_after_done,
            )
        )
        if not self.perimeter_switch_to_frontier_after_done:
            self._mark_exploration_done('perimeter phase finished and perimeter_switch_to_frontier_after_done=false')

    def _detect_perimeter_wall_clusters(self, msg: OccupancyGrid) -> List[WallCluster]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            self.get_logger().warn('perimeter wall cluster detection skipped: invalid map geometry')
            return []

        occupied_cells: Set[Cell] = set()
        data = msg.data
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                if self._is_occupied(data[row + x]):
                    occupied_cells.add((x, y))

        bridge_radius_cells = max(1, int(round(0.20 / max(resolution, 1e-6))))
        bridged_cells: Set[Cell] = set()
        for cell in occupied_cells:
            for expanded in self._cells_in_radius(cell, bridge_radius_cells, width, height):
                bridged_cells.add(expanded)
        raw_clusters = self._cluster_cells(bridged_cells, width, height)
        wall_clusters: List[WallCluster] = []
        for cluster in raw_clusters:
            if len(cluster) < self.perimeter_wall_min_cluster_size:
                continue
            xs = [cell[0] for cell in cluster]
            ys = [cell[1] for cell in cluster]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            span_x = (max_x - min_x + 1) * resolution
            span_y = (max_y - min_y + 1) * resolution
            length_m = max(span_x, span_y)
            thickness_m = max(resolution, min(span_x, span_y))
            aspect_ratio = length_m / max(thickness_m, resolution)
            if length_m < self.perimeter_wall_min_length_m:
                continue
            if aspect_ratio < self.perimeter_wall_aspect_ratio_min:
                continue
            cx = int(round(sum(xs) / len(cluster)))
            cy = int(round(sum(ys) / len(cluster)))
            orientation = 'vertical' if span_y >= span_x else 'horizontal'
            wall_clusters.append(
                WallCluster(
                    cells=cluster,
                    bbox=(min_x, min_y, max_x, max_y),
                    center_cell=(cx, cy),
                    center_xy=self._cell_to_world(cx, cy, msg.info.origin.position.x, msg.info.origin.position.y, resolution),
                    length_m=length_m,
                    thickness_m=thickness_m,
                    aspect_ratio=aspect_ratio,
                    orientation=orientation,
                )
            )

        wall_clusters.sort(key=lambda item: (item.length_m, len(item.cells)), reverse=True)
        top = wall_clusters[0] if wall_clusters else None
        self.get_logger().warn(
            'perimeter wall cluster detection: occupied_cells=%d bridged_cells=%d raw_clusters=%d detected wall clusters=%d min_cluster_size=%d '
            'min_length=%.2f aspect_min=%.2f top_length=%.2f top_aspect=%.2f'
            % (
                len(occupied_cells),
                len(bridged_cells),
                len(raw_clusters),
                len(wall_clusters),
                self.perimeter_wall_min_cluster_size,
                self.perimeter_wall_min_length_m,
                self.perimeter_wall_aspect_ratio_min,
                top.length_m if top else 0.0,
                top.aspect_ratio if top else 0.0,
            )
        )
        return wall_clusters

    def _generate_perimeter_waypoints(
        self,
        msg: OccupancyGrid,
        robot_pose: RobotPose,
        wall_clusters: List[WallCluster],
    ) -> Tuple[List[PerimeterWaypoint], int]:
        if not wall_clusters:
            return [], 0
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        data = msg.data
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        offset_cells = max(1, int(round(self.perimeter_wall_offset_m / max(resolution, 1e-6))))
        min_clearance_cells = max(1, int(math.ceil(0.45 / max(resolution, 1e-6))))
        spacing_cells = max(1, int(round(self.perimeter_waypoint_spacing_m / max(resolution, 1e-6))))
        accepted: List[PerimeterWaypoint] = []
        accepted_cells: List[Cell] = []
        rejected = 0
        map_center_x = width / 2.0
        map_center_y = height / 2.0

        for cluster_index, cluster in enumerate(wall_clusters):
            if len(accepted) >= self.perimeter_max_waypoints:
                break
            min_x, min_y, max_x, max_y = cluster.bbox
            samples: List[Cell]
            if cluster.orientation == 'vertical':
                step = max(1, spacing_cells)
                samples = [(cluster.center_cell[0], y) for y in range(min_y, max_y + 1, step)]
            else:
                step = max(1, spacing_cells)
                samples = [(x, cluster.center_cell[1]) for x in range(min_x, max_x + 1, step)]

            for sx, sy in samples:
                if len(accepted) >= self.perimeter_max_waypoints:
                    break
                if cluster.orientation == 'vertical':
                    direction = 1 if cluster.center_cell[0] < map_center_x else -1
                    target = (sx + direction * offset_cells, sy)
                else:
                    direction = 1 if cluster.center_cell[1] < map_center_y else -1
                    target = (sx, sy + direction * offset_cells)
                tx, ty = target
                if tx <= 0 or ty <= 0 or tx >= width - 1 or ty >= height - 1:
                    rejected += 1
                    continue
                if not self._is_free(data[ty * width + tx]):
                    target = self._nearest_free_offset_cell(data, width, height, target, min_clearance_cells, offset_cells)
                    if target is None:
                        rejected += 1
                        continue
                    tx, ty = target
                if not self._is_far_from_occupied(data, width, height, tx, ty, min_clearance_cells):
                    rejected += 1
                    continue
                if any(math.hypot(tx - ax, ty - ay) < max(1, spacing_cells * 0.65) for ax, ay in accepted_cells):
                    rejected += 1
                    continue
                wx, wy = self._cell_to_world(tx, ty, origin_x, origin_y, resolution)
                if self._is_blacklisted((wx, wy)):
                    rejected += 1
                    continue
                yaw = math.atan2(cluster.center_xy[1] - wy, cluster.center_xy[0] - wx)
                clearance_cells = self._nearest_occupied_distance_cells(
                    data,
                    width,
                    height,
                    tx,
                    ty,
                    max(min_clearance_cells + 6, min_clearance_cells),
                )
                dist = math.hypot(wx - robot_pose[0], wy - robot_pose[1])
                score = cluster.length_m + 0.05 * len(cluster.cells) + clearance_cells * resolution - 0.05 * dist
                accepted.append(
                    PerimeterWaypoint(
                        target_cell=(tx, ty),
                        target_xy=(wx, wy),
                        yaw=yaw,
                        source_cluster_index=cluster_index,
                        clearance_cells=clearance_cells,
                        score=score,
                    )
                )
                accepted_cells.append((tx, ty))

        self.get_logger().warn(
            'accepted perimeter waypoints=%d rejected perimeter waypoints=%d wall_clusters=%d offset_m=%.2f spacing_m=%.2f clearance_min_m=0.45'
            % (len(accepted), rejected, len(wall_clusters), self.perimeter_wall_offset_m, self.perimeter_waypoint_spacing_m)
        )
        return accepted, rejected

    def _nearest_free_offset_cell(
        self,
        data: Sequence[int],
        width: int,
        height: int,
        target: Cell,
        min_clearance_cells: int,
        search_radius_cells: int,
    ) -> Optional[Cell]:
        tx, ty = target
        best: Optional[Tuple[int, Cell]] = None
        max_radius = max(1, search_radius_cells)
        for radius in range(1, max_radius + 1):
            for y in range(max(1, ty - radius), min(height - 1, ty + radius + 1)):
                for x in range(max(1, tx - radius), min(width - 1, tx + radius + 1)):
                    d2 = (x - tx) * (x - tx) + (y - ty) * (y - ty)
                    if d2 > radius * radius:
                        continue
                    if not self._is_free(data[y * width + x]):
                        continue
                    if not self._is_far_from_occupied(data, width, height, x, y, min_clearance_cells):
                        continue
                    if best is None or d2 < best[0]:
                        best = (d2, (x, y))
            if best is not None:
                return best[1]
        return None

    def _sort_perimeter_waypoints(self, waypoints: List[PerimeterWaypoint], robot_pose: RobotPose) -> List[PerimeterWaypoint]:
        if not waypoints:
            return []
        cx = sum(item.target_xy[0] for item in waypoints) / len(waypoints)
        cy = sum(item.target_xy[1] for item in waypoints) / len(waypoints)
        reverse = self.perimeter_direction == 'cw'
        ordered = sorted(
            waypoints,
            key=lambda item: math.atan2(item.target_xy[1] - cy, item.target_xy[0] - cx),
            reverse=reverse,
        )
        robot_xy = (robot_pose[0], robot_pose[1])
        start_index = min(range(len(ordered)), key=lambda i: math.hypot(ordered[i].target_xy[0] - robot_xy[0], ordered[i].target_xy[1] - robot_xy[1]))
        ordered = ordered[start_index:] + ordered[:start_index]
        if len(ordered) > self.perimeter_max_waypoints:
            ordered = ordered[: self.perimeter_max_waypoints]
        self.get_logger().warn(
            'generated perimeter waypoints sorted: accepted=%d direction=%s start_index=%d center=(%.3f, %.3f)'
            % (len(ordered), self.perimeter_direction, start_index, cx, cy)
        )
        return ordered

    def _find_frontier_candidates(
        self,
        msg: OccupancyGrid,
        robot_xy: Point,
        mode: str = 'strict',
    ) -> Tuple[List[FrontierCandidate], int]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return [], 0

        config = self._search_config(mode)
        data = msg.data
        frontier_cells = self._find_frontier_cells(data, width, height)
        clusters = self._cluster_cells(frontier_cells, width, height)
        raw_cluster_count = len(clusters)
        min_obstacle_cells = max(1, int(math.ceil(config.min_obstacle_distance_m / resolution)))

        candidates: List[FrontierCandidate] = []
        for cluster in clusters:
            if len(cluster) < config.min_cluster_size:
                continue
            candidates.extend(
                self._candidates_from_cluster(
                    cluster,
                    data,
                    width,
                    height,
                    resolution,
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    robot_xy,
                    min_obstacle_cells,
                    config,
                )
            )

        candidates.sort(key=lambda item: item.score, reverse=True)
        return candidates, raw_cluster_count

    def _search_config(self, mode: str) -> SearchConfig:
        if mode == 'relaxed':
            return SearchConfig(
                mode='relaxed',
                min_cluster_size=self.relaxed_min_cluster_size,
                min_obstacle_distance_m=self.relaxed_min_obstacle_distance_m,
                max_goal_distance_m=self.relaxed_max_goal_distance_m,
            )
        return SearchConfig(
            mode='strict',
            min_cluster_size=self.min_cluster_size,
            min_obstacle_distance_m=self.min_obstacle_distance_m,
            max_goal_distance_m=self.max_goal_distance_m,
        )

    @staticmethod
    def _is_unknown(value: int) -> bool:
        return value == -1

    @staticmethod
    def _is_free(value: int) -> bool:
        return value == 0

    @staticmethod
    def _is_occupied(value: int) -> bool:
        return value > 50

    def _find_frontier_cells(self, data: Sequence[int], width: int, height: int) -> Set[Cell]:
        frontier_cells: Set[Cell] = set()
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                value = data[row + x]
                if not self._is_free(value):
                    continue
                for nx, ny in self._neighbors8(x, y):
                    value = data[ny * width + nx]
                    if self._is_unknown(value):
                        frontier_cells.add((x, y))
                        break
        return frontier_cells

    def _cluster_cells(self, frontier_cells: Set[Cell], width: int, height: int) -> List[List[Cell]]:
        clusters: List[List[Cell]] = []
        remaining = set(frontier_cells)
        while remaining:
            start = remaining.pop()
            cluster = [start]
            queue = deque([start])
            while queue:
                x, y = queue.popleft()
                for nx, ny in self._neighbors8(x, y):
                    if nx < 0 or ny < 0 or nx >= width or ny >= height:
                        continue
                    cell = (nx, ny)
                    if cell not in remaining:
                        continue
                    remaining.remove(cell)
                    queue.append(cell)
                    cluster.append(cell)
            clusters.append(cluster)
        return clusters

    def _candidates_from_cluster(
        self,
        cluster: List[Cell],
        data: Sequence[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        robot_xy: Point,
        min_obstacle_cells: int,
        config: SearchConfig,
    ) -> List[FrontierCandidate]:
        robot_cell = self._world_to_cell(robot_xy[0], robot_xy[1], origin_x, origin_y, resolution)
        representatives = self._representative_cells(cluster, robot_cell, resolution)
        min_search_cells = max(1, int(math.ceil(0.3 / resolution)))
        max_search_cells = max(min_search_cells, int(math.ceil(1.0 / resolution)))
        min_free_neighbors = 4 if config.mode == 'strict' else 3
        free_neighbor_radius = max(1, int(math.ceil(0.25 / resolution)))

        by_cell = {}
        for rep in representatives:
            checked_for_rep = 0
            for cell in self._candidate_search_cells(rep, min_search_cells, max_search_cells, width, height):
                if checked_for_rep >= 140:
                    break
                checked_for_rep += 1
                x, y = cell
                value = data[y * width + x]
                if not self._is_free(value):
                    continue
                if not self._is_far_from_occupied(data, width, height, x, y, min_obstacle_cells):
                    continue
                free_neighbors = self._free_neighbor_count(data, width, height, x, y, free_neighbor_radius)
                if free_neighbors < min_free_neighbors:
                    continue
                wx, wy = self._cell_to_world(x, y, origin_x, origin_y, resolution)
                if self._is_blacklisted((wx, wy)):
                    continue
                dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                if dist < self.min_goal_distance_m or dist > config.max_goal_distance_m:
                    continue
                clearance_cells = self._nearest_occupied_distance_cells(
                    data,
                    width,
                    height,
                    x,
                    y,
                    max(min_obstacle_cells + 4, min_obstacle_cells),
                )
                frontier_gap = math.hypot(x - rep[0], y - rep[1]) * resolution
                preferred = 0.5 * (self.min_goal_distance_m + config.max_goal_distance_m)
                distance_penalty = abs(dist - preferred)
                clearance_bonus = min(clearance_cells, min_obstacle_cells + 4) * resolution
                score = float(len(cluster)) + 6.0 * clearance_bonus - 2.0 * distance_penalty - 3.0 * frontier_gap
                candidate = FrontierCandidate(
                    cells=cluster,
                    target_cell=cell,
                    target_xy=(wx, wy),
                    distance_m=dist,
                    score=score,
                    search_mode=config.mode,
                    clearance_cells=clearance_cells,
                    free_neighbor_count=free_neighbors,
                )
                if cell not in by_cell or candidate.score > by_cell[cell].score:
                    by_cell[cell] = candidate

        candidates = sorted(by_cell.values(), key=lambda item: item.score, reverse=True)
        return candidates[: max(1, self.max_candidates_per_cluster)]


    def _find_unknown_clusters(self, msg: OccupancyGrid) -> List[UnknownCluster]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return []
        data = msg.data
        unknown_cells: Set[Cell] = set()
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                value = data[row + x]
                if value == -1:
                    unknown_cells.add((x, y))
        raw_clusters = self._cluster_cells(unknown_cells, width, height)
        clusters: List[UnknownCluster] = []
        for cluster in raw_clusters:
            if len(cluster) < self.cleanup_unknown_cluster_min_size:
                continue
            boundary_free = self._cluster_free_boundary_count(data, width, height, cluster)
            if boundary_free <= 0:
                continue
            cx = int(round(sum(c[0] for c in cluster) / len(cluster)))
            cy = int(round(sum(c[1] for c in cluster) / len(cluster)))
            center_xy = self._cell_to_world(cx, cy, msg.info.origin.position.x, msg.info.origin.position.y, resolution)
            clusters.append(
                UnknownCluster(
                    cells=cluster,
                    center_cell=(cx, cy),
                    center_xy=center_xy,
                    boundary_free_neighbor_count=boundary_free,
                )
            )
        clusters.sort(key=lambda item: (item.boundary_free_neighbor_count, len(item.cells)), reverse=True)
        if clusters:
            self.get_logger().info(
                'residual unknown cluster detection: unknown_cluster_count=%d largest_size=%d boundary_free_neighbors=%d'
                % (len(clusters), len(clusters[0].cells), clusters[0].boundary_free_neighbor_count),
                throttle_duration_sec=10.0,
            )
        return clusters[: max(1, self.cleanup_max_unknown_clusters)]

    def _cluster_free_boundary_count(self, data: Sequence[int], width: int, height: int, cluster: List[Cell]) -> int:
        boundary: Set[Cell] = set()
        cluster_set = set(cluster)
        for x, y in cluster:
            for nx, ny in self._neighbors8(x, y):
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if (nx, ny) in cluster_set:
                    continue
                if self._is_free(data[ny * width + nx]):
                    boundary.add((nx, ny))
        return len(boundary)

    def _find_cleanup_candidates(
        self,
        msg: OccupancyGrid,
        robot_xy: Point,
        clusters: List[UnknownCluster],
    ) -> List[CleanupCandidate]:
        if not self.enable_cleanup_mode or not clusters:
            return []
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return []
        data = msg.data
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        min_radius_cells = max(1, int(math.ceil(self.cleanup_search_radius_min_m / resolution)))
        max_radius_cells = max(min_radius_cells, int(math.ceil(self.cleanup_search_radius_max_m / resolution)))
        min_obstacle_cells = max(1, int(math.ceil(self.cleanup_min_obstacle_distance_m / resolution)))
        free_neighbor_radius = max(1, int(math.ceil(0.30 / resolution)))
        min_free_neighbors = 5
        candidates: List[CleanupCandidate] = []
        max_cleanup_distance = max(self.relaxed_max_goal_distance_m, self.max_goal_distance_m, 8.0)

        for cluster in clusters:
            checked = 0
            best_for_cluster: Optional[CleanupCandidate] = None
            for cell in self._candidate_search_cells(cluster.center_cell, min_radius_cells, max_radius_cells, width, height):
                if checked >= 320:
                    break
                checked += 1
                x, y = cell
                if not self._is_free(data[y * width + x]):
                    continue
                if not self._is_far_from_occupied(data, width, height, x, y, min_obstacle_cells):
                    continue
                free_neighbors = self._free_neighbor_count(data, width, height, x, y, free_neighbor_radius)
                if free_neighbors < min_free_neighbors:
                    continue
                wx, wy = self._cell_to_world(x, y, origin_x, origin_y, resolution)
                if self._is_blacklisted((wx, wy)):
                    continue
                dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                if dist < self.min_goal_distance_m or dist > max_cleanup_distance:
                    continue
                yaw = math.atan2(cluster.center_xy[1] - wy, cluster.center_xy[0] - wx)
                clearance_cells = self._nearest_occupied_distance_cells(
                    data,
                    width,
                    height,
                    x,
                    y,
                    max(min_obstacle_cells + 4, min_obstacle_cells),
                )
                observation_range_m = math.hypot(wx - cluster.center_xy[0], wy - cluster.center_xy[1])
                preferred_range = 0.5 * (self.cleanup_search_radius_min_m + self.cleanup_search_radius_max_m)
                score = (
                    0.35 * float(len(cluster.cells))
                    + 0.20 * float(cluster.boundary_free_neighbor_count)
                    + 4.0 * min(clearance_cells, min_obstacle_cells + 4) * resolution
                    + 0.12 * free_neighbors
                    - 1.6 * abs(observation_range_m - preferred_range)
                    - 0.35 * dist
                )
                candidate = CleanupCandidate(
                    cluster=cluster,
                    target_cell=cell,
                    target_xy=(wx, wy),
                    yaw=yaw,
                    distance_m=dist,
                    score=score,
                    clearance_cells=clearance_cells,
                    free_neighbor_count=free_neighbors,
                )
                if best_for_cluster is None or candidate.score > best_for_cluster.score:
                    best_for_cluster = candidate
            if best_for_cluster is not None:
                candidates.append(best_for_cluster)

        candidates.sort(key=lambda item: item.score, reverse=True)
        if candidates:
            top = candidates[0]
            self.get_logger().info(
                'cleanup candidate generation: cleanup_candidate_count=%d top_unknown_cluster_size=%d top_goal=(%.3f, %.3f) top_yaw=%.3f top_score=%.3f'
                % (len(candidates), len(top.cluster.cells), top.target_xy[0], top.target_xy[1], top.yaw, top.score),
                throttle_duration_sec=10.0,
            )
        return candidates[: max(1, self.cleanup_max_unknown_clusters)]

    def _candidate_search_cells(
        self,
        center: Cell,
        min_radius_cells: int,
        max_radius_cells: int,
        width: int,
        height: int,
    ) -> List[Cell]:
        cx, cy = center
        min2 = min_radius_cells * min_radius_cells
        max2 = max_radius_cells * max_radius_cells
        cells: List[Tuple[int, Cell]] = []
        for y in range(max(0, cy - max_radius_cells), min(height, cy + max_radius_cells + 1)):
            for x in range(max(0, cx - max_radius_cells), min(width, cx + max_radius_cells + 1)):
                d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy)
                if d2 < min2 or d2 > max2:
                    continue
                cells.append((d2, (x, y)))
        cells.sort(key=lambda item: item[0])
        return [cell for _, cell in cells]

    def _representative_cells(self, cluster: List[Cell], robot_cell: Optional[Cell], resolution: float) -> List[Cell]:
        if not cluster:
            return []
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        ordered: List[Cell] = []

        def add(cell: Cell) -> None:
            if cell not in ordered:
                ordered.append(cell)

        add(min(cluster, key=lambda cell: (cell[0] - cx) ** 2 + (cell[1] - cy) ** 2))
        add(min(cluster, key=lambda cell: cell[0]))
        add(max(cluster, key=lambda cell: cell[0]))
        add(min(cluster, key=lambda cell: cell[1]))
        add(max(cluster, key=lambda cell: cell[1]))
        if robot_cell is not None:
            add(min(cluster, key=lambda cell: (cell[0] - robot_cell[0]) ** 2 + (cell[1] - robot_cell[1]) ** 2))
            add(max(cluster, key=lambda cell: (cell[0] - robot_cell[0]) ** 2 + (cell[1] - robot_cell[1]) ** 2))
            # Back off from frontier edge toward robot to find a known-free, Nav2-friendlier goal.
            for base in list(ordered):
                vx = robot_cell[0] - base[0]
                vy = robot_cell[1] - base[1]
                norm = math.hypot(vx, vy)
                if norm <= 0.0:
                    continue
                for backoff_m in (0.3, 0.5, 0.8):
                    cells = int(round(backoff_m / max(resolution, 1e-6)))
                    add((int(round(base[0] + vx / norm * cells)), int(round(base[1] + vy / norm * cells))))
        if len(cluster) > 8:
            step = max(1, len(cluster) // 8)
            for cell in sorted(cluster)[::step][:8]:
                add(cell)
        return ordered[:18]

    def _cells_in_radius(self, center: Cell, radius_cells: int, width: int, height: int) -> Iterable[Cell]:
        cx, cy = center
        r2 = radius_cells * radius_cells
        for y in range(max(0, cy - radius_cells), min(height, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(width, cx + radius_cells + 1)):
                d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy)
                if d2 > r2:
                    continue
                yield (x, y)

    def _is_far_from_occupied(self, data: Sequence[int], width: int, height: int, x: int, y: int, radius_cells: int) -> bool:
        for ny in range(max(0, y - radius_cells), min(height, y + radius_cells + 1)):
            for nx in range(max(0, x - radius_cells), min(width, x + radius_cells + 1)):
                if (nx - x) * (nx - x) + (ny - y) * (ny - y) > radius_cells * radius_cells:
                    continue
                value = data[ny * width + nx]
                if self._is_occupied(value):
                    return False
        return True

    def _nearest_occupied_distance_cells(
        self,
        data: Sequence[int],
        width: int,
        height: int,
        x: int,
        y: int,
        max_radius_cells: int,
    ) -> int:
        for radius in range(1, max_radius_cells + 1):
            for ny in range(max(0, y - radius), min(height, y + radius + 1)):
                for nx in range(max(0, x - radius), min(width, x + radius + 1)):
                    if (nx - x) * (nx - x) + (ny - y) * (ny - y) > radius * radius:
                        continue
                    if self._is_occupied(data[ny * width + nx]):
                        return radius
        return max_radius_cells

    def _free_neighbor_count(self, data: Sequence[int], width: int, height: int, x: int, y: int, radius_cells: int) -> int:
        count = 0
        for ny in range(max(0, y - radius_cells), min(height, y + radius_cells + 1)):
            for nx in range(max(0, x - radius_cells), min(width, x + radius_cells + 1)):
                if (nx - x) * (nx - x) + (ny - y) * (ny - y) > radius_cells * radius_cells:
                    continue
                if self._is_free(data[ny * width + nx]):
                    count += 1
        return count

    def _is_blacklisted(self, xy: Point) -> bool:
        return any(math.hypot(xy[0] - bx, xy[1] - by) <= self.blacklist_radius_m for bx, by in self.blacklist)

    def _map_counts(self, msg: OccupancyGrid) -> MapCounts:
        unknown = 0
        free = 0
        occupied = 0
        for value in msg.data:
            if self._is_unknown(value):
                unknown += 1
            elif self._is_free(value):
                free += 1
            elif self._is_occupied(value):
                occupied += 1
        return (unknown, free, occupied)

    def _update_map_stability(self, msg: OccupancyGrid) -> Tuple[MapCounts, MapCounts]:
        counts = self._map_counts(msg)
        if self.last_map_counts is None:
            self.last_map_counts = counts
            self.last_map_counts_snapshot = counts
            self.last_map_delta = (0, 0, 0)
            self.map_stable_cycles = 0
            return counts, self.last_map_delta
        deltas = tuple(abs(counts[i] - self.last_map_counts[i]) for i in range(3))
        self.last_map_counts = counts
        self.last_map_counts_snapshot = counts
        self.last_map_delta = deltas
        occupied_threshold = max(self.map_change_free_threshold, self.map_change_unknown_threshold)
        stable = (
            deltas[0] <= self.map_change_unknown_threshold
            and deltas[1] <= self.map_change_free_threshold
            and deltas[2] <= occupied_threshold
        )
        if stable:
            self.map_stable_cycles += 1
        else:
            self.map_stable_cycles = 0
        return counts, deltas

    @staticmethod
    def _neighbors8(x: int, y: int) -> Iterable[Cell]:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield x + dx, y + dy

    @staticmethod
    def _cell_to_world(x: int, y: int, origin_x: float, origin_y: float, resolution: float) -> Point:
        return (origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution)

    @staticmethod
    def _world_to_cell(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> Optional[Cell]:
        if resolution <= 0.0:
            return None
        return (int(math.floor((x - origin_x) / resolution)), int(math.floor((y - origin_y) / resolution)))

    def _mark_exploration_done(self, reason: str) -> None:
        if self.exploration_done:
            return
        self.exploration_done = True
        self.get_logger().info(
            'Exploration complete: %s; completed_goals=%d success=%d failure=%d recovery_scans_completed=%d '
            'relaxed_search_executed=%s map_stable_cycles=%d cleanup_goals_completed=%d/%d cleanup_spins_completed=%d '
            'unknown_total_count=%d unknown_ratio=%.4f unknown_cluster_count=%d cleanup_candidate_count=%d'
            % (
                reason,
                self.goal_count,
                self.goal_success_count,
                self.goal_failure_count,
                self.recovery_scans_completed,
                self.relaxed_search_executed,
                self.map_stable_cycles,
                self.cleanup_goals_completed,
                self.max_cleanup_goals,
                self.cleanup_spins_completed,
                self.last_unknown_total_count,
                self.last_unknown_ratio,
                self.last_unknown_cluster_count,
                self.last_cleanup_candidate_count,
            )
        )
        self._save_map_if_requested(reason)

    def _save_map_if_requested(self, reason: str) -> None:
        if not self.save_map:
            self.get_logger().info('Map save skipped: save_map=false reason=%s' % reason)
            return
        if self.map_saved:
            self.get_logger().info('Map save skipped: already saved map_save_path=%s' % self.map_save_path)
            return

        output_prefix = Path(self.map_save_path).expanduser()
        output_prefix.parent.mkdir(parents=True, exist_ok=True)
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(output_prefix)]
        self.get_logger().info('Saving map with command: %s' % ' '.join(command))
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.map_save_timeout_sec,
                check=False,
            )
        except Exception as exc:
            self.get_logger().error('Map save failed before completion: %s' % exc)
            return

        stdout = (completed.stdout or '').strip()
        stderr = (completed.stderr or '').strip()
        self.get_logger().info('Map save returncode=%d' % completed.returncode)
        if stdout:
            self.get_logger().info('Map save stdout: %s' % stdout)
        if stderr:
            self.get_logger().warn('Map save stderr: %s' % stderr)

        yaml_path = output_prefix.with_suffix('.yaml')
        pgm_path = output_prefix.with_suffix('.pgm')
        yaml_ok = yaml_path.exists() and yaml_path.stat().st_size > 0
        pgm_ok = pgm_path.exists() and pgm_path.stat().st_size > 0
        if completed.returncode == 0 and yaml_ok and pgm_ok:
            self.map_saved = True
            self.get_logger().info(
                'Map save succeeded: yaml=%s bytes=%d pgm=%s bytes=%d'
                % (yaml_path, yaml_path.stat().st_size, pgm_path, pgm_path.stat().st_size)
            )
        else:
            self.get_logger().error(
                'Map save failed validation: returncode=%d yaml_exists=%s yaml_bytes=%d pgm_exists=%s pgm_bytes=%d'
                % (
                    completed.returncode,
                    yaml_path.exists(),
                    yaml_path.stat().st_size if yaml_path.exists() else 0,
                    pgm_path.exists(),
                    pgm_path.stat().st_size if pgm_path.exists() else 0,
                )
            )



def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
