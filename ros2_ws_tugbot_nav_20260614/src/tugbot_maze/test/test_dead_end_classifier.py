import math
from tugbot_maze.dead_end_classifier import is_true_dead_end


def test_wall_ahead_with_single_opening_is_true_dead_end():
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=0.6,
                            corridor_length_m=1.76) is True


def test_open_ahead_is_not_dead_end_even_if_kind_says_so():
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=2.5,
                            corridor_length_m=1.76) is False


def test_junction_kind_is_never_dead_end():
    assert is_true_dead_end(perception_kind='junction',
                            forward_min_range_m=0.3,
                            corridor_length_m=1.76) is False


def test_missing_scan_defers_to_not_dead_end():
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=None,
                            corridor_length_m=1.76) is False
