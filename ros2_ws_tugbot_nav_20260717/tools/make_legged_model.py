#!/usr/bin/env python3
"""One-shot surgery: turn the kinematic-ghost anymal_c model.sdf into the
physical one. Reads the CURRENT workspace model.sdf + the ORIGINAL CERBERUS
model.sdf, writes the workspace file in place. Asserts every count so a
partial application cannot slip through. Run from the workspace root:

    python3 tools/make_legged_model.py
"""
import re
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
CUR = WS / 'src/tugbot_description/models/anymal_c/model.sdf'
ORIG = WS.parent / 'tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf'

# initial_position = STAND_POSE (legged/params.py); order matches the 12 plugin blocks
STAND = {
    'LF_HAA': 0.0, 'LF_HFE': 0.705, 'LF_KFE': -0.9608,
    'RF_HAA': 0.0, 'RF_HFE': 0.705, 'RF_KFE': -0.9608,
    'LH_HAA': 0.0, 'LH_HFE': -0.705, 'LH_KFE': 0.9608,
    'RH_HAA': 0.0, 'RH_HFE': -0.705, 'RH_KFE': 0.9608,
}
LINKS = ['base'] + [f'{l}_{part}' for l in ('LF', 'RF', 'LH', 'RH')
                    for part in ('HIP', 'THIGH', 'SHANK')]
FRICTION = ('<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
            '</surface>')


def link_block(text, name):
    m = re.search(rf'(<link name="{name}">.*?</link>)', text, re.S)
    assert m, f'link {name} not found'
    return m.group(1)


def main():
    cur = CUR.read_text()
    orig = ORIG.read_text()

    # 1. collisions: copy every <collision> block of each link from the original
    total = 0
    for name in LINKS:
        ob = link_block(orig, name)
        cols = re.findall(r'<collision name=.*?</collision>', ob, re.S)
        assert cols, f'no collisions in original link {name}'
        total += len(cols)
        block = '\n      '.join(cols)
        cb = link_block(cur, name)
        assert '<collision' not in cb, f'link {name} already has collisions'
        cur = cur.replace(cb, cb.replace('</link>', f'  {block}\n    </link>'))
    assert total == 54, f'expected 54 collisions, copied {total}'

    # 2. foot friction: FOOT sphere + cylinder collisions get explicit mu
    n_friction = 0
    def add_friction(m):
        nonlocal n_friction
        n_friction += 1
        return m.group(0).replace('</collision>', f'  {FRICTION}\n        </collision>')
    cur = re.sub(r'<collision name="[^"]*FOOT[^"]*">.*?</collision>', add_friction, cur, flags=re.S)
    assert n_friction == 8, f'expected 8 FOOT collisions to get friction, got {n_friction}'

    # 3. gravity on
    cur, n = re.subn(r'\s*<gravity>false</gravity>', '', cur)
    assert n == 13, f'expected 13 gravity tags, removed {n}'

    # 4. delete VelocityControl plugin
    cur, n = re.subn(
        r'\s*<plugin filename="gz-sim-velocity-control-system".*?</plugin>', '', cur, flags=re.S)
    assert n == 1, 'VelocityControl block not found'

    # 5. OdometryPublisher dimensions 2 -> 3 (stabilizer + fall detection need z/roll/pitch)
    cur, n = re.subn(r'<dimensions>2</dimensions>', '<dimensions>3</dimensions>', cur)
    assert n == 1, 'odometry dimensions tag not found'

    # 6. controllers: velocity-command mode -> torque PID with gains
    def controller(joint):
        return (
            '<plugin filename="gz-sim-joint-position-controller-system" '
            'name="gz::sim::systems::JointPositionController">\n'
            f'      <joint_name>{joint}</joint_name>\n'
            '      <p_gain>250.0</p_gain>\n'
            '      <i_gain>0.0</i_gain>\n'
            '      <d_gain>5.0</d_gain>\n'
            '      <cmd_max>80.0</cmd_max>\n'
            '      <cmd_min>-80.0</cmd_min>\n'
            f'      <initial_position>{STAND[joint]}</initial_position>\n'
            '    </plugin>')
    n_ctl = 0
    def replace_ctl(m):
        nonlocal n_ctl
        n_ctl += 1
        joint = re.search(r'<joint_name>([^<]+)</joint_name>', m.group(0)).group(1)
        return controller(joint)
    cur = re.sub(
        r'<plugin filename="gz-sim-joint-position-controller-system".*?</plugin>',
        replace_ctl, cur, flags=re.S)
    assert n_ctl == 12, f'expected 12 controller blocks, rewrote {n_ctl}'
    for joint, val in STAND.items():
        assert f'<joint_name>{joint}</joint_name>' in cur, joint

    # final sanity (friction was added INSIDE existing blocks, so still 54 tags)
    assert cur.count('<collision') == 54
    assert 'use_velocity_commands' not in cur
    assert 'VelocityControl' not in cur
    CUR.write_text(cur)
    print(f'OK: 54 collisions, 8 foot frictions, gravity on, 12 torque-PID controllers, dims=3')


if __name__ == '__main__':
    sys.exit(main())
