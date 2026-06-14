"""Pure Python topology memory for maze exploration.

This module intentionally has no ROS dependencies. The ROS explorer can use it to
remember junctions, branch states, dead ends, and backtracking targets while unit
tests exercise the maze search policy cheaply without Gazebo/Nav2.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import heapq
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

    def score_for_exit(
        self,
        node_xy: Point,
        exit_xy: Point,
        exit_bias_weight: float,
        exploration_bonus_weight: float = 0.0,
        known_positions: Optional[List[Point]] = None,
        distance_to_exit_weight: float = 1.0,
    ) -> float:
        """Higher score is better.

        Components:
        - -distance_to_exit_weight * distance_to_exit: prefer branches closer to exit
        - exit_bias_weight * heading_alignment: prefer branches pointing toward exit
        - exploration_bonus_weight * novelty: prefer branches in unexplored territory

        distance_to_exit_weight scales the raw distance penalty.  Default 1.0 is
        legacy behaviour; values ~0.1 bring it to comparable magnitude with the
        other terms so that novelty and heading can actually influence selection.
        """
        target_distance_to_exit = _distance(self.target_xy, exit_xy)
        heading_alignment = math.cos(_angle_delta(
            self.angle_rad,
            math.atan2(exit_xy[1] - node_xy[1], exit_xy[0] - node_xy[0]),
        ))
        score = -distance_to_exit_weight * target_distance_to_exit + exit_bias_weight * heading_alignment

        if exploration_bonus_weight > 0 and known_positions:
            min_dist = min(_distance(self.target_xy, p) for p in known_positions)
            # Cap novelty at 5.0m to prevent extreme bonuses
            novelty = min(min_dist, 5.0)
            score += exploration_bonus_weight * novelty

        return score


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

    def all_branches_terminal(self) -> bool:
        """Return True if every branch state is terminal (EXPLORED, DEAD_END, BLOCKED, BLACKLISTED).

        Returns False for nodes with no branches set (unvisited/corridor nodes).
        Used by level-2 backtracking to detect junctions whose entire subtree is dead.
        """
        return bool(self.branches) and all(
            branch.state in TERMINAL_BRANCH_STATES for branch in self.branches
        )


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
        exploration_bonus_weight: float = 0.0,
        distance_to_exit_weight: float = 1.0,
    ) -> None:
        self.junction_merge_radius_m = float(junction_merge_radius_m)
        self.exit_bias_weight = float(exit_bias_weight)
        self.branch_angle_merge_tolerance_rad = float(branch_angle_merge_tolerance_rad)
        self.branch_target_merge_radius_m = float(branch_target_merge_radius_m)
        self.blacklist_radius_m = float(blacklist_radius_m)
        self.max_failures_per_branch = int(max_failures_per_branch)
        self.max_backtrack_failures_per_node = int(max_backtrack_failures_per_node)
        self.backtrack_goal_tolerance_m = float(backtrack_goal_tolerance_m)
        self.exploration_bonus_weight = float(exploration_bonus_weight)
        self.distance_to_exit_weight = float(distance_to_exit_weight)
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
            # Level-2 backtracking: explicitly skip junctions where ALL branches
            # are terminal (DEAD_END/BLOCKED/BLACKLISTED/EXPLORED). These have no
            # useful work left — the dead-subtree signal should cascade upward so
            # that higher-level junctions get a chance to use their remaining branches.
            if node.all_branches_terminal():
                continue
        return None

    def farthest_stack_backtrack_target(self, current_node_id: int, current_xy: Point) -> Optional[TopoNode]:
        """Find the best junction on the visit stack for stagnation backtracking.

        Scores by untried_count * min(distance, 15m) to prefer moderately distant
        junctions with many untried branches. Caps distance at 15m to avoid
        overshooting back to the entrance (which can be 20+m away). Falls back
        to the best available candidate if none exceed the minimum distance.
        """
        MIN_STAGNATION_DISTANCE_M = 3.0
        DISTANCE_CAP_M = 15.0
        candidates: list[tuple[float, int, float, 'TopoNode']] = []
        seen_ids: set[int] = {current_node_id}
        for node_id in self.visit_stack:
            if node_id in seen_ids:
                continue
            seen_ids.add(node_id)
            node = self.nodes[node_id]
            if (node.node_type == 'junction'
                    and node.has_untried_branches()
                    and node.backtrack_failures < self.max_backtrack_failures_per_node):
                dist = _distance(node.xy, current_xy)
                untried = sum(1 for b in node.branches if b.state in SELECTABLE_BRANCH_STATES)
                effective_dist = min(dist, DISTANCE_CAP_M)
                score = untried * effective_dist
                candidates.append((score, untried, dist, node))
        if not candidates:
            return None
        # Prefer candidates at least 3m away to break out of local loops
        distant = [(s, u, d, n) for s, u, d, n in candidates if d >= MIN_STAGNATION_DISTANCE_M]
        pool = distant if distant else candidates
        # Sort by score descending, then by untried count, then by distance
        pool.sort(key=lambda item: (item[0], item[1], item[2]), reverse=True)
        return pool[0][3]

    def choose_next_branch(self, node_id: int, exit_xy: Point) -> Optional[BranchOption]:
        node = self.nodes[node_id]
        self._apply_blacklist_to_node(node)
        candidates = [branch for branch in node.branches if branch.state in SELECTABLE_BRANCH_STATES]
        if not candidates:
            return None
        known_positions = self.get_known_positions() if self.exploration_bonus_weight > 0 else None
        return max(candidates, key=lambda branch: branch.score_for_exit(
            node.xy, exit_xy, self.exit_bias_weight,
            self.exploration_bonus_weight, known_positions,
            self.distance_to_exit_weight,
        ))

    def get_known_positions(self) -> List[Point]:
        """Return all node positions for exploration novelty calculation."""
        return [node.xy for node in self.nodes.values()]

    def _build_dijkstra_adjacency(self, blocked_penalty: float = 2.0) -> Dict[int, List[Tuple[int, float]]]:
        """Build undirected adjacency for Dijkstra pathfinding.

        EXPLORED and IN_PROGRESS edges use Euclidean distance.
        BLOCKED edges are included with a penalty multiplier — the edge was
        blocked due to Nav2 failure at the branch target, but the topological
        connection still exists and junctions beyond may have valid untried branches.
        DEAD_END edges are excluded entirely (nothing useful beyond them).

        Args:
            blocked_penalty: Distance multiplier for BLOCKED edges (default 2.0).
        """
        adj: Dict[int, List[Tuple[int, float]]] = defaultdict(list)
        for edge in self.edges.values():
            s, e = edge.start_node_id, edge.end_node_id
            d = _distance(self.nodes[s].xy, self.nodes[e].xy)
            if edge.state in (EXPLORED, IN_PROGRESS):
                adj[s].append((e, d))
                adj[e].append((s, d))
            elif edge.state == BLOCKED:
                adj[s].append((e, d * blocked_penalty))
                adj[e].append((s, d * blocked_penalty))
        return adj

    def dijkstra_nearest_unexplored(self, start_node_id: int) -> Optional[Tuple[List[int], TopoNode]]:
        """Find the nearest junction with untried branches via shortest explored-edge path.

        Uses Dijkstra with Euclidean edge weights on the known topology graph.
        Transits through fully-explored junctions (all branches terminal) but
        never stops at them — the target must have at least one untried branch.
        BLOCKED edges are included with a penalty to keep the graph connected.

        Returns (path_node_ids, target_node) or None if no unexplored junction is reachable.
        """
        adj = self._build_dijkstra_adjacency()

        # Dijkstra search
        dist: Dict[int, float] = {start_node_id: 0.0}
        prev: Dict[int, Optional[int]] = {start_node_id: None}
        heap: List[Tuple[float, int]] = [(0.0, start_node_id)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, float('inf')):
                continue
            node = self.nodes[u]
            # Target: junction with untried branches, not the start, within failure limit
            if (u != start_node_id
                    and node.node_type == 'junction'
                    and node.has_untried_branches()
                    and node.backtrack_failures < self.max_backtrack_failures_per_node):
                path: List[int] = []
                curr: Optional[int] = u
                while curr is not None:
                    path.append(curr)
                    curr = prev.get(curr)
                path.reverse()
                return (path, node)
            for v, w in adj.get(u, []):
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        return None

    def dijkstra_farthest_unexplored(
        self,
        start_node_id: int,
        exit_xy: Optional[Point] = None,
        current_xy: Optional[Point] = None,
    ) -> Optional[Tuple[List[int], TopoNode]]:
        """Find the best junction with untried branches for stagnation recovery.

        When exit_xy and current_xy are provided, uses exit-directed scoring that
        prefers junctions CLOSER to the exit with untried branches.  This prevents
        stagnation recovery from sending the robot backward toward the entrance
        (which the legacy distance-based scoring often does).

        Score formula (exit-directed):
            score = untried_count * (1.0 + max(0, exit_closeness_improvement))
        Where exit_closeness_improvement = dist(current, exit) - dist(junction, exit).
        A junction 3m closer to exit with 2 untried: 2*(1+3) = 8.  A junction 3m
        further with 4 untried: 4*1 = 4.  Exit-closer junctions win unless a
        far-junction has substantially more untried branches.

        When exit_xy/current_xy are not provided, falls back to legacy scoring:
            score = untried_count * dijkstra_distance
        BLOCKED edges are included with a penalty to keep the graph connected.
        """
        adj = self._build_dijkstra_adjacency()

        # Full Dijkstra to compute shortest distances to all reachable nodes
        dist: Dict[int, float] = {start_node_id: 0.0}
        prev: Dict[int, Optional[int]] = {start_node_id: None}
        heap: List[Tuple[float, int]] = [(0.0, start_node_id)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, float('inf')):
                continue
            for v, w in adj.get(u, []):
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        # Collect candidates: junctions with untried branches, reachable, not start
        candidates: List[Tuple[int, TopoNode]] = []
        for uid, d in dist.items():
            if uid == start_node_id:
                continue
            node = self.nodes[uid]
            if (node.node_type == 'junction'
                    and node.has_untried_branches()
                    and node.backtrack_failures < self.max_backtrack_failures_per_node):
                candidates.append((uid, node))

        if not candidates:
            return None

        if exit_xy is not None and current_xy is not None:
            # Exit-directed scoring: prefer junctions closer to the exit
            current_exit_dist = _distance(current_xy, exit_xy)

            def _score(item: Tuple[int, TopoNode]) -> float:
                uid, node = item
                untried = sum(1 for b in node.branches if b.state in SELECTABLE_BRANCH_STATES)
                exit_dist = _distance(node.xy, exit_xy)
                exit_closeness_improvement = current_exit_dist - exit_dist
                return untried * (1.0 + max(0.0, exit_closeness_improvement))
        else:
            # Legacy scoring: prefer far junctions with many untried branches
            def _score(item: Tuple[int, TopoNode]) -> float:
                uid, node = item
                d = dist[uid]
                untried = sum(1 for b in node.branches if b.state in SELECTABLE_BRANCH_STATES)
                return untried * d

        best_uid, best_node = max(candidates, key=_score)

        # Reconstruct path
        path: List[int] = []
        curr: Optional[int] = best_uid
        while curr is not None:
            path.append(curr)
            curr = prev.get(curr)
        path.reverse()
        return (path, best_node)

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

    def reset_blocked_branches(self) -> int:
        """Reset BLOCKED branches to UNTRIED so DFS can retry them.

        Called after frontier push exhaustion to give DFS a second chance.
        Also clears failure counts on the reset branches so they can fail
        again without immediately re-blocking.  Returns the number of
        branches that were reset.
        """
        reset_count = 0
        for node in self.nodes.values():
            for branch in node.branches:
                if branch.state == BLOCKED:
                    branch.state = UNTRIED
                    branch.failures = 0
                    branch.failure_reason = None
                    reset_count += 1
        return reset_count

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
