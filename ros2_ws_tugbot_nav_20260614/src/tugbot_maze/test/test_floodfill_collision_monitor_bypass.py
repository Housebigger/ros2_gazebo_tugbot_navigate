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
