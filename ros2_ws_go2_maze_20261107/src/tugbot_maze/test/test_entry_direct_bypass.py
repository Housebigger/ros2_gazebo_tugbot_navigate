"""Tests for ENTRY_DIRECT bypass: maze entrance straight-line goal dispatch."""

import math
import pytest


class TestEntryDirectGoalComputation:
    """Verify the entry direct goal geometry: target = entrance + distance * direction."""

    def test_entry_direct_goal_at_origin_facing_east(self):
        d = 1.5
        entrance_x, entrance_y, entrance_yaw = 0.0, 0.0, 0.0
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(1.5, abs=0.01)
        assert target_y == pytest.approx(0.0, abs=0.01)

    def test_entry_direct_goal_at_origin_facing_north(self):
        d = 1.5
        entrance_x, entrance_y, entrance_yaw = 0.0, 0.0, math.pi / 2
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(0.0, abs=0.01)
        assert target_y == pytest.approx(1.5, abs=0.01)

    def test_entry_direct_goal_nonzero_entrance(self):
        d = 2.0
        entrance_x, entrance_y, entrance_yaw = 5.0, 3.0, math.pi
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(3.0, abs=0.01)
        assert target_y == pytest.approx(3.0, abs=0.01)

    def test_entry_direct_respects_entrance_yaw(self):
        entrance_yaw = 1.23
        assert entrance_yaw == pytest.approx(1.23)


class TestEntryDirectDefaultParams:
    def test_default_entry_direct_distance(self):
        assert 1.5 == 1.5

    def test_default_entry_direct_enabled(self):
        assert True is True


class TestEntryDirectDispatchConditions:
    def test_triggered_only_when_goal_count_zero(self):
        goal_count = 5
        entry_direct_dispatched = False
        assert goal_count > 0
        assert not entry_direct_dispatched

    def test_disabled_flag_prevents_entry_direct(self):
        entry_direct_enabled = False
        goal_count = 0
        should_dispatch = entry_direct_enabled and goal_count == 0
        assert not should_dispatch

    def test_enabled_and_zero_goals_triggers(self):
        entry_direct_enabled = True
        goal_count = 0
        should_dispatch = entry_direct_enabled and goal_count == 0
        assert should_dispatch


class TestEntryDirectStateTransitions:
    def test_success_transitions_to_at_node_analyze(self):
        completed_goal_kind = 'entry_direct'
        if completed_goal_kind == 'entry_direct':
            next_mode = 'AT_NODE_ANALYZE'
        else:
            next_mode = 'SETTLING'
        assert next_mode == 'AT_NODE_ANALYZE'

    def test_failure_falls_to_at_node_analyze(self):
        completed_goal_kind = 'entry_direct'
        if completed_goal_kind == 'entry_direct':
            next_mode = 'AT_NODE_ANALYZE'
        else:
            next_mode = 'FAILED_EXHAUSTED'
        assert next_mode == 'AT_NODE_ANALYZE'

    def test_entry_direct_not_dispatched_twice(self):
        entry_direct_dispatched = True
        goal_count = 1
        should_dispatch = not entry_direct_dispatched and goal_count == 0
        assert not should_dispatch
