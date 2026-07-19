import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.maze_topology import (
    BLOCKED,
    DEAD_END,
    EXPLORED,
    IN_PROGRESS,
    UNTRIED,
    BranchOption,
    MazeTopology,
    BLACKLISTED,
    BLOCKED_NAV2,
    TRUE_DEAD_END,
    TERMINAL_BRANCH_STATES,
)


def test_new_junction_tracks_untried_branch_directions():
    topology = MazeTopology(junction_merge_radius_m=0.5)

    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(0.0, -1.0)),
        ],
    )

    assert len(node.branches) == 3
    assert {branch.state for branch in node.branches} == {UNTRIED}


def test_find_or_create_node_reuses_nearby_junctions_and_creates_distant_nodes():
    topology = MazeTopology(junction_merge_radius_m=0.75)

    first = topology.find_or_create_node(1.0, 2.0, node_type='junction')
    nearby = topology.find_or_create_node(1.3, 2.2, node_type='junction')
    distant = topology.find_or_create_node(2.5, 2.0, node_type='junction')

    assert nearby.node_id == first.node_id
    assert distant.node_id != first.node_id
    assert len(topology.nodes) == 2


def test_dead_end_marks_incoming_edge_and_backtracks_to_recent_untried_junction():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    topology.set_branch_options(
        a.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),
        ],
    )
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0)),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0)),
        ],
    )
    edge_ab = topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)
    edge_bc = topology.connect_nodes(b.node_id, c.node_id, state=IN_PROGRESS)
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    topology.mark_dead_end(c.node_id, incoming_edge_id=edge_bc.edge_id)
    backtrack = topology.next_backtrack_target(c.node_id)

    assert topology.nodes[c.node_id].node_type == 'dead_end'
    assert topology.edges[edge_bc.edge_id].state == DEAD_END
    assert topology.edges[edge_ab.edge_id].state == EXPLORED
    assert backtrack is not None
    assert backtrack.node_id == b.node_id


def test_backtracking_skips_exhausted_junctions():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    topology.set_branch_options(b.node_id, [BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0))])
    for branch in b.branches:
        branch.state = DEAD_END
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    backtrack = topology.next_backtrack_target(c.node_id)

    assert backtrack is not None
    assert backtrack.node_id == a.node_id


def test_choose_next_branch_prefers_untried_branch_biased_toward_exit():
    topology = MazeTopology(junction_merge_radius_m=0.5, exit_bias_weight=1.0)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=math.pi, target_xy=(-1.0, 0.0)),
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0), state=DEAD_END),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(0.0, -1.0), state=BLOCKED),
        ],
    )

    chosen = topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0))

    assert chosen is not None
    assert chosen.target_xy == (1.0, 0.0)
    assert chosen.state == UNTRIED


def test_choose_next_branch_returns_none_when_all_branches_terminal():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=math.pi, target_xy=(-1.0, 0.0), state=DEAD_END),
        ],
    )

    assert topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0)) is None


# ── Level-2 backtracking tests ──────────────────────────────────────────────


def test_all_branches_terminal_true_when_all_dead_end():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0), state=DEAD_END),
        ],
    )

    assert node.all_branches_terminal() is True


def test_all_branches_terminal_true_with_mixed_terminal_states():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0), state=BLOCKED),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(0.0, -1.0), state=DEAD_END),
        ],
    )

    assert node.all_branches_terminal() is True


def test_all_branches_terminal_false_when_untried_remains():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),  # UNTRIED
        ],
    )

    assert node.all_branches_terminal() is False


def test_all_branches_terminal_false_for_empty_branches():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    # No branches set

    assert node.all_branches_terminal() is False


def test_level2_backtrack_skips_fully_explored_junction():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    # A has one UNTRIED branch (not fully explored)
    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    # B has all branches terminal (fully explored)
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0), state=BLOCKED),
        ],
    )
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    backtrack = topology.next_backtrack_target(c.node_id)

    # Should skip B (all terminal) and return A
    assert backtrack is not None
    assert backtrack.node_id == a.node_id


def test_multi_level_backtrack_cascades_through_two_fully_explored_junctions():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='junction')
    d = topology.find_or_create_node(3.0, 0.0, node_type='dead_end')

    # A has one UNTRIED branch
    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    # B fully explored
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0), state=DEAD_END),
        ],
    )
    # C fully explored
    topology.set_branch_options(
        c.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(3.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(2.0, 1.0), state=BLOCKED),
        ],
    )
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)
    topology.visit_node(d.node_id)

    backtrack = topology.next_backtrack_target(d.node_id)

    assert backtrack is not None
    assert backtrack.node_id == a.node_id


def test_partially_explored_junction_is_not_skipped_by_level2_backtrack():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    # A has UNTRIED branch
    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    # B has one DEAD_END and one UNTRIED — partially explored, NOT fully-explored
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0)),  # UNTRIED
        ],
    )
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    backtrack = topology.next_backtrack_target(c.node_id)

    # B still has untried branches, so it is the target (not skipped)
    assert backtrack is not None
    assert backtrack.node_id == b.node_id


# ── Dijkstra shortest-path backtracking tests ────────────────────────────────


def test_dijkstra_finds_nearest_unexplored():
    """Linear graph: A - B - C.  B has an untried branch.  From A, Dijkstra
    should find B via the shortest explored-edge path."""
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    # A: both branches explored
    topology.set_branch_options(
        a.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0), state=DEAD_END),
        ],
    )
    # B: one dead-end, one untried
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0)),  # UNTRIED
        ],
    )
    topology.set_branch_options(c.node_id, [BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0))])

    edge_ab = topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)
    topology.connect_nodes(b.node_id, c.node_id, state=IN_PROGRESS)

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is not None
    path, target = result
    assert target.node_id == b.node_id
    assert path[0] == a.node_id
    assert path[-1] == b.node_id


def test_dijkstra_skips_dead_subtrees():
    """A - B(all terminal) - C(has untried).  From A, Dijkstra should skip
    B (dead subtree) and find C via the through path."""
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(3.0, 0.0, node_type='junction')

    topology.set_branch_options(
        a.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
        ],
    )
    # B: all branches terminal → dead subtree
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(1.0, 1.0), state=BLOCKED),
        ],
    )
    # C: has an untried branch
    topology.set_branch_options(
        c.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(4.0, 0.0), state=DEAD_END),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(3.0, 1.0)),  # UNTRIED
        ],
    )
    topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)
    topology.connect_nodes(b.node_id, c.node_id, state=EXPLORED)

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is not None
    _, target = result
    assert target.node_id == c.node_id


def test_dijkstra_returns_none_when_exhausted():
    """All junctions have no untried branches → returns None."""
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')

    topology.set_branch_options(
        a.node_id,
        [BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED)],
    )
    topology.set_branch_options(
        b.node_id,
        [BranchOption(angle_rad=math.pi / 2.0, target_xy=(1.0, 1.0), state=DEAD_END)],
    )
    topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is None


def test_dijkstra_respects_backtrack_failure_limit():
    """Junction with backtrack_failures >= max is skipped."""
    topology = MazeTopology(
        junction_merge_radius_m=0.5,
        max_backtrack_failures_per_node=2,
    )
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')

    topology.set_branch_options(
        a.node_id,
        [BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED)],
    )
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(1.0, 1.0)),  # UNTRIED
        ],
    )
    topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)

    # Exhaust B's backtrack failures
    topology.record_backtrack_failure(b.node_id)
    topology.record_backtrack_failure(b.node_id)

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is None


def test_dijkstra_picks_nearest_when_multiple_unexplored():
    """Diamond graph: A -> B (near, untried) and A -> C -> D (far, untried).
    Dijkstra should find B as nearest."""
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(0.0, 5.0, node_type='junction')
    d = topology.find_or_create_node(0.0, 10.0, node_type='junction')

    topology.set_branch_options(
        a.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 5.0), state=EXPLORED),
        ],
    )
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0)),  # UNTRIED
        ],
    )
    topology.set_branch_options(
        c.node_id,
        [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 10.0), state=EXPLORED)],
    )
    topology.set_branch_options(
        d.node_id,
        [
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 11.0)),  # UNTRIED
        ],
    )
    topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)
    topology.connect_nodes(a.node_id, c.node_id, state=EXPLORED)
    topology.connect_nodes(c.node_id, d.node_id, state=EXPLORED)

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is not None
    _, target = result
    assert target.node_id == b.node_id


def test_dijkstra_returns_none_from_disconnected_component():
    """No explored edges connect start to the junction with untried branches."""
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(10.0, 0.0, node_type='junction')

    topology.set_branch_options(
        a.node_id,
        [BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=DEAD_END)],
    )
    topology.set_branch_options(
        b.node_id,
        [BranchOption(angle_rad=0.0, target_xy=(11.0, 0.0))],  # UNTRIED
    )
    # No edges → graph is disconnected

    result = topology.dijkstra_nearest_unexplored(a.node_id)

    assert result is None


# ── Exploration novelty bonus tests ──────────────────────────────────────────


def test_exploration_bonus_favors_novel_targets():
    """Two branches from the same node: one close to a known junction, one far.
    With exploration_bonus_weight > 0, the far branch should score higher for
    equivalent exit-distance."""
    node_xy = (0.0, 0.0)
    exit_xy = (20.0, 20.0)

    near_branch = BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0))  # near known
    far_branch = BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 10.0))  # far from known

    known_positions = [(1.0, 0.0)]  # the near branch target is already known

    score_near = near_branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=known_positions,
    )
    score_far = far_branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=known_positions,
    )

    # With weight=0, compare raw scores (no exploration bonus)
    bonus_near = near_branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.5, known_positions=known_positions,
    )
    bonus_far = far_branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.5, known_positions=known_positions,
    )

    # The far branch should gain more from the exploration bonus
    gain_near = bonus_near - score_near
    gain_far = bonus_far - score_far
    assert gain_far > gain_near


def test_exploration_bonus_zero_is_backward_compatible():
    """With exploration_bonus_weight=0, scoring ignores known_positions entirely."""
    node_xy = (0.0, 0.0)
    exit_xy = (10.0, 0.0)
    branch = BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0))

    score_no_positions = branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=None,
    )
    score_with_positions = branch.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=[(0.0, 0.0)],
    )

    assert score_no_positions == score_with_positions


def test_exploration_bonus_capped_at_5m():
    """Novelty > 5m is capped, preventing extreme score values."""
    node_xy = (0.0, 0.0)
    exit_xy = (10.0, 0.0)
    branch_far = BranchOption(angle_rad=math.pi, target_xy=(-100.0, 0.0))  # 100m from known
    branch_5m = BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 5.0))  # exactly 5m from known

    known_positions = [(0.0, 0.0)]
    weight = 0.3

    bonus_100m = branch_far.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=weight, known_positions=known_positions,
    )
    bonus_5m = branch_5m.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=weight, known_positions=known_positions,
    )

    # Both should get the same capped bonus contribution (0.3 * 5.0 = 1.5)
    # The only difference should be from the base score (distance + heading)
    base_far = branch_far.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=None,
    )
    base_5m = branch_5m.score_for_exit(
        node_xy, exit_xy, exit_bias_weight=1.0,
        exploration_bonus_weight=0.0, known_positions=None,
    )

    exploration_bonus_far = bonus_100m - base_far
    exploration_bonus_5m = bonus_5m - base_5m

    assert abs(exploration_bonus_far - 1.5) < 1e-9  # 0.3 * min(100, 5.0) = 1.5
    assert abs(exploration_bonus_5m - 1.5) < 1e-9   # 0.3 * min(5.0, 5.0) = 1.5


def test_choose_next_branch_uses_exploration_bonus():
    """MazeTopology.choose_next_branch with exploration_bonus_weight > 0 should
    prefer a novel branch over one near known junctions when distances are comparable."""
    topology = MazeTopology(
        junction_merge_radius_m=0.5,
        exit_bias_weight=0.5,
        exploration_bonus_weight=0.5,
    )
    # Create a known junction nearby
    topology.find_or_create_node(1.0, 0.0, node_type='junction')

    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),    # near known junction
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 5.0)),  # novel territory
        ],
    )

    chosen = topology.choose_next_branch(node.node_id, exit_xy=(0.0, 20.0))

    # Both branches point roughly toward exit (north). The novel one should win
    # because it gets a large exploration bonus.
    assert chosen is not None
    assert chosen.target_xy == (0.0, 5.0)
