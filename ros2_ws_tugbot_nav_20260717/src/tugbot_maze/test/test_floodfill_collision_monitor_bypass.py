import copy
from pathlib import Path

import yaml

_CONFIG = Path(__file__).resolve().parents[2] / 'tugbot_navigation' / 'config'
_BASE = _CONFIG / 'nav2_slam_params.yaml'
_VARIANT = _CONFIG / 'nav2_slam_floodfill_params.yaml'


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _cm(doc):
    return doc['collision_monitor']['ros__parameters']


def test_base_has_both_collision_monitor_gates_enabled():
    cm = _cm(_load(_BASE))
    assert cm['FootprintApproach']['enabled'] is True
    assert cm['scan']['enabled'] is True


def test_variant_disables_both_collision_monitor_gates():
    cm = _cm(_load(_VARIANT))
    assert cm['FootprintApproach']['enabled'] is False
    assert cm['scan']['enabled'] is False


def test_variant_differs_from_base_only_by_the_two_enabled_flags():
    base, variant = _load(_BASE), _load(_VARIANT)
    patched = copy.deepcopy(variant)
    cm = patched['collision_monitor']['ros__parameters']
    cm['FootprintApproach']['enabled'] = True
    cm['scan']['enabled'] = True
    assert patched == base, "variant drifted from base beyond the two intended enabled flags"


import os


def _assembled_nav2_params(explorer_type, nav_share='/NAV'):
    """Mirror tugbot_maze_explore.launch.py's nav2_params_file PythonExpression assembly (flood_fill
    clause + abbreviated existing chain) and eval it. Guards the quoting seam. profiles all 'false'."""
    ff = os.path.join(nav_share, 'config', 'nav2_slam_floodfill_params.yaml')
    p26pc = os.path.join(nav_share, 'config', 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml')
    default = os.path.join(nav_share, 'config', 'nav2_slam_params.yaml')
    parts = [
        "'", ff, "' if '", explorer_type, "' == 'flood_fill' else ",   # new clause: ends 'else ' (NO quote)
        "'", p26pc, "' if '", 'false', "' == 'true' else '",            # existing chain's leading "'" follows
        default, "'",
    ]
    return eval(''.join(parts))


def test_launch_expr_selects_floodfill_variant_for_flood_fill():
    assert _assembled_nav2_params('flood_fill').endswith('nav2_slam_floodfill_params.yaml')


def test_launch_expr_falls_through_for_other_explorers():
    assert _assembled_nav2_params('maze_dfs').endswith('nav2_slam_params.yaml')
