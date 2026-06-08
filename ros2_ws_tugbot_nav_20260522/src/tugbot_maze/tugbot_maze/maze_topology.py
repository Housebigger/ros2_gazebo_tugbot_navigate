"""Pure Python topology memory for maze exploration.

This module intentionally has no ROS dependencies. The ROS explorer can use it to
remember junctions, branch states, dead ends, and backtracking targets while unit
tests exercise the maze search policy cheaply without Gazebo/Nav2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Tuple

Point = Tuple[float, float]

UNTRIED = 'untried'
IN_PROGRESS = 'in_progress'
EXPLORED = 'explored'
DEAD_END = 'dead_end'
BLOCKED = 'blocked'
BLACKLISTED = 'blacklisted'

BLOCKED_NAV2 = 'blocked_nav2'
TRUE_DEAD_END = 'true_dead_end'
GOAL_REJECTED = 'goal_rejected'
GOAL_TIMEOUT = 'goal_timeout'
GOAL_PREEMPTED = 'goal_preempted'
GOAL_CANCELED_AFTER_TIMEOUT = 'goal_canceled_after_timeout'
GOAL_CANCELED_AFTER_EXIT = 'goal_canceled_after_exit'
BACKTRACK_FAILED = 'backtrack_failed'

TERMINAL_BRANCH_STATES = {EXPLORED, DEAD_END, BLOCKED, BLACKLISTED}
SELECTABLE_BRANCH_STATES = {UNTRIED}


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass
class BranchOption:
    angle_rad: float
    target_xy: Point
    state: str = UNTRIED
    edge_id: Optional[int] = None
    failures: int = 0
    failure_reason: Optional[str] = None

    def score_for_exit(self, node_xy: Point, exit_xy: Point, exit_bias_weight: float) -> float:
        """Higher score is better; nearby-to-exit targets win among untried branches."""
        target_distance_to_exit = _distance(self.target_xy, exit_xy)
        heading_alignment = math.cos(_angle_delta(self.angle_rad, math.atan2(exit_xy[1] - node_xy[1], exit_xy[0] - node_xy[0])))
        return -target_distance_to_exit + exit_bias_weight * heading_alignment


@dataclass
class BlacklistedGoal:
    xy: Point
    reason: str


@dataclass
class TopoNode:
    node_id: int
    x: float
    y: float
    node_type: str = 'junction'
    branches: List[BranchOption] = field(default_factory=list)
    backtrack_failures: int = 0

    @property
    def xy(self) -> Point:
        return (self.x, self.y)

    def has_untried_branches(self) -> bool:
        return any(branch.state in SELECTABLE_BRANCH_STATES for branch in self.branches)


@dataclass
class TopoEdge:
    edge_id: int
    start_node_id: int
    end_node_id: int
    state: str = IN_PROGRESS


class MazeTopology:
    """Mutable topological memory for DFS-style maze exploration."""

    def __init__(
        self,
        junction_merge_radius_m: float = 0.75,
        exit_bias_weight: float = 0.5,
        branch_angle_merge_tolerance_rad: float = math.radians(20.0),
        branch_target_merge_radius_m: float = 0.45,
        blacklist_radius_m: float = 0.5,
        max_failures_per_branch: int = 2,
        max_backtrack_failures_per_node: int = 2,
        backtrack_goal_tolerance_m: float = 0.35,
    ) -> None:
        self.junction_merge_radius_m = float(junction_merge_radius_m)
        self.exit_bias_weight = float(exit_bias_weight)
        self.branch_angle_merge_tolerance_rad = float(branch_angle_merge_tolerance_rad)
        self.branch_target_merge_radius_m = float(branch_target_merge_radius_m)
        self.blacklist_radius_m = float(blacklist_radius_m)
        self.max_failures_per_branch = int(max_failures_per_branch)
        self.max_backtrack_failures_per_node = int(max_backtrack_failures_per_node)
        self.backtrack_goal_tolerance_m = float(backtrack_goal_tolerance_m)
        self.nodes: Dict[int, TopoNode] = {}
        self.edges: Dict[int, TopoEdge] = {}
        self.visit_stack: List[int] = []
        self.blacklist: List[BlacklistedGoal] = []
        self._next_node_id = 1
        self._next_edge_id = 1

    def find_or_create_node(self, x: float, y: float, node_type: str = 'junction') -> TopoNode:
        xy = (float(x), float(y))
        nearest: Optional[TopoNode] = None
        nearest_distance = float('inf')
        for node in self.nodes.values():
            distance = _distance(node.xy, xy)
            if distance <= self.junction_merge_radius_m and distance < nearest_distance:
                nearest = node
                nearest_distance = distance
        if nearest is not None:
            if node_type == 'dead_end':
                nearest.node_type = 'dead_end'
            elif nearest.node_type != 'dead_end':
                nearest.node_type = node_type
            return nearest

        node = TopoNode(node_id=self._next_node_id, x=xy[0], y=xy[1], node_type=node_type)
        self.nodes[node.node_id] = node
        self._next_node_id += 1
        return node

    def set_branch_options(self, node_id: int, branch_options: Iterable[BranchOption]) -> None:
        node = self.nodes[node_id]
        merged: List[BranchOption] = []
        old_branches = list(node.branches)
        for option in branch_options:
            previous = self._find_matching_branch(old_branches, option)
            if previous is not None:
                option.state = previous.state
                option.edge_id = previous.edge_id
                option.failures = previous.failures
                option.failure_reason = previous.failure_reason
            if self.is_goal_blacklisted(option.target_xy):
                option.state = BLACKLISTED
                option.failure_reason = self.blacklist_reason(option.target_xy)
            merged.append(option)
        node.branches = merged

    def connect_nodes(self, start_node_id: int, end_node_id: int, state: str = IN_PROGRESS) -> TopoEdge:
        edge = TopoEdge(
            edge_id=self._next_edge_id,
            start_node_id=start_node_id,
            end_node_id=end_node_id,
            state=state,
        )
        self.edges[edge.edge_id] = edge
        self._next_edge_id += 1
        self._attach_edge_to_matching_branch(edge)
        return edge

    def visit_node(self, node_id: int) -> None:
        if node_id not in self.nodes:
            raise KeyError(f'unknown node_id={node_id}')
        if not self.visit_stack or self.visit_stack[-1] != node_id:
            self.visit_stack.append(node_id)

    def mark_dead_end(self, node_id: int, incoming_edge_id: Optional[int] = None) -> None:
        node = self.nodes[node_id]
        node.node_type = 'dead_end'
        for branch in node.branches:
            if branch.state == IN_PROGRESS:
                branch.state = DEAD_END

        if incoming_edge_id is not None:
            edge = self.edges[incoming_edge_id]
            edge.state = DEAD_END
            start_node = self.nodes[edge.start_node_id]
            for branch in start_node.branches:
                if branch.edge_id == incoming_edge_id or _distance(branch.target_xy, node.xy) <= self.branch_target_merge_radius_m:
                    branch.state = DEAD_END
                    branch.edge_id = incoming_edge_id

    def mark_branch_state(self, node_id: int, branch: BranchOption, state: str) -> None:
        node = self.nodes[node_id]
        for existing in node.branches:
            if existing is branch or self._branches_match(existing, branch):
                existing.state = state
                return
        raise ValueError('branch does not belong to node')

    def next_backtrack_target(self, current_node_id: int) -> Optional[TopoNode]:
        seen = {current_node_id}
        for node_id in reversed(self.visit_stack):
            if node_id in seen:
                continue
            seen.add(node_id)
            node = self.nodes[node_id]
            if node.node_type == 'junction' and node.has_untried_branches() and node.backtrack_failures < self.max_backtrack_failures_per_node:
                return node
        return None

    def choose_next_branch(self, node_id: int, exit_xy: Point) -> Optional[BranchOption]:
        node = self.nodes[node_id]
        self._apply_blacklist_to_node(node)
        candidates = [branch for branch in node.branches if branch.state in SELECTABLE_BRANCH_STATES]
        if not candidates:
            return None
        return max(candidates, key=lambda branch: branch.score_for_exit(node.xy, exit_xy, self.exit_bias_weight))

    def blacklist_goal(self, xy: Point, reason: str = BLOCKED_NAV2) -> None:
        self.blacklist.append(BlacklistedGoal(xy=(float(xy[0]), float(xy[1])), reason=reason))
        for node in self.nodes.values():
            self._apply_blacklist_to_node(node)

    def is_goal_blacklisted(self, xy: Point) -> bool:
        return self.blacklist_reason(xy) is not None

    def blacklist_reason(self, xy: Point) -> Optional[str]:
        for item in self.blacklist:
            if _distance(item.xy, xy) <= self.blacklist_radius_m:
                return item.reason
        return None

    def record_branch_failure(
        self,
        node_id: int,
        branch: BranchOption,
        reason: str,
        end_node_id: Optional[int] = None,
        edge_id: Optional[int] = None,
    ) -> str:
        node = self.nodes[node_id]
        target_branch = self._find_matching_branch(node.branches, branch)
        if target_branch is None:
            raise ValueError('branch does not belong to node')
        target_branch.failures += 1
        target_branch.failure_reason = reason
        if reason == TRUE_DEAD_END:
            target_branch.state = DEAD_END
            if end_node_id is not None:
                self.mark_dead_end(end_node_id, incoming_edge_id=edge_id)
            elif edge_id is not None:
                self.edges[edge_id].state = DEAD_END
            return DEAD_END
        if target_branch.failures >= self.max_failures_per_branch:
            target_branch.state = BLOCKED
            self.blacklist_goal(target_branch.target_xy, reason=reason)
            return BLOCKED
        target_branch.state = UNTRIED
        return UNTRIED

    def record_backtrack_failure(self, node_id: int) -> None:
        self.nodes[node_id].backtrack_failures += 1

    def backtrack_target_reached(self, node_id: int, robot_xy: Point) -> bool:
        return _distance(self.nodes[node_id].xy, robot_xy) <= self.backtrack_goal_tolerance_m

    def blocked_branch_count(self) -> int:
        return sum(1 for node in self.nodes.values() for branch in node.branches if branch.state == BLOCKED)

    def blacklisted_branch_count(self) -> int:
        return sum(1 for node in self.nodes.values() for branch in node.branches if branch.state == BLACKLISTED)

    def _apply_blacklist_to_node(self, node: TopoNode) -> None:
        for branch in node.branches:
            reason = self.blacklist_reason(branch.target_xy)
            if reason is not None and branch.state in SELECTABLE_BRANCH_STATES:
                branch.state = BLACKLISTED
                branch.failure_reason = reason

    def exhausted(self) -> bool:
        return not any(node.has_untried_branches() for node in self.nodes.values())

    def _find_matching_branch(self, branches: Iterable[BranchOption], option: BranchOption) -> Optional[BranchOption]:
        for branch in branches:
            if self._branches_match(branch, option):
                return branch
        return None

    def _branches_match(self, a: BranchOption, b: BranchOption) -> bool:
        return (
            abs(_angle_delta(a.angle_rad, b.angle_rad)) <= self.branch_angle_merge_tolerance_rad
            or _distance(a.target_xy, b.target_xy) <= self.branch_target_merge_radius_m
        )

    def _attach_edge_to_matching_branch(self, edge: TopoEdge) -> None:
        start = self.nodes[edge.start_node_id]
        end = self.nodes[edge.end_node_id]
        best: Optional[BranchOption] = None
        best_distance = float('inf')
        for branch in start.branches:
            distance = _distance(branch.target_xy, end.xy)
            if distance < best_distance:
                best = branch
                best_distance = distance
        if best is not None and best_distance <= max(self.branch_target_merge_radius_m, self.junction_merge_radius_m):
            best.edge_id = edge.edge_id
            best.state = edge.state
