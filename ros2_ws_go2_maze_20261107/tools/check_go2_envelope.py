#!/usr/bin/env python3
"""Read-only projected envelope sample of all official Go2 SDF collisions.

Uses 5 x 5 command samples, 3 x 3 stabilizer-input samples, 360 gait phases,
and 5 x 5 independent body roll/pitch orientations.  Each primitive's planar
support is analytic; only the gait/attitude domain is sampled.  No ROS is used.
The independent body tilt grid is +/-0.1 rad in each axis, which is broader
than a 0.1 rad total-tilt bound.  It does not model joint tracking error.
"""
import argparse
import itertools
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('workspace', type=Path)
args = parser.parse_args()
sys.path.insert(0, str(args.workspace / 'src/tugbot_maze'))
from tugbot_maze.legged.params import LeggedParams, NEUTRAL_FOOT

p = LeggedParams()
configs = np.array(list(itertools.product(
    np.linspace(-p.vx_max, p.vx_max, 5),
    np.linspace(-p.wz_max, p.wz_max, 5),
    (-0.06, 0.0, 0.06), (-0.06, 0.0, 0.06),
    np.linspace(0, math.tau, 360, endpoint=False),
)))
n = len(configs)
vx, wz, rs, ps, phase = configs.T


def rx(q):
    r = np.zeros((len(q), 3, 3))
    r[:, 0, 0] = 1
    r[:, 1, 1] = r[:, 2, 2] = np.cos(q)
    r[:, 2, 1] = np.sin(q)
    r[:, 1, 2] = -np.sin(q)
    return r


def ry(q):
    r = np.zeros((len(q), 3, 3))
    r[:, 1, 1] = 1
    r[:, 0, 0] = r[:, 2, 2] = np.cos(q)
    r[:, 0, 2] = np.sin(q)
    r[:, 2, 0] = -np.sin(q)
    return r


def rz(q):
    r = np.zeros((len(q), 3, 3))
    r[:, 2, 2] = 1
    r[:, 0, 0] = r[:, 1, 1] = np.cos(q)
    r[:, 1, 0] = np.sin(q)
    r[:, 0, 1] = -np.sin(q)
    return r


def pose6(value):
    x, y, z, roll, pitch, yaw = map(float, value.split())
    r = (rz(np.array([yaw])) @ ry(np.array([pitch])) @ rx(np.array([roll])))[0]
    return r, np.array([x, y, z])


transforms = {
    'base_link': (np.broadcast_to(np.eye(3), (n, 3, 3)), np.zeros((n, 3))),
}
for leg, prefix, phase_offset in (
    ('LF', 'fl', 0), ('RF', 'fr', math.pi),
    ('LH', 'rl', math.pi), ('RH', 'rr', 0),
):
    nx, ny, nz = NEUTRAL_FOOT[leg]
    s = ((phase + phase_offset) % math.tau) / math.tau
    k = np.where(s < p.duty, 1 - 2*s/p.duty,
                 2*(s-p.duty)/(1-p.duty) - 1)
    dz = np.where(s < p.duty, 0,
                  p.swing_lift*np.sin(math.pi*(s-p.duty)/(1-p.duty)))
    dz += np.clip(p.kp_att*(rs*ny - ps*nx), -p.dz_max, p.dz_max)
    foot = np.column_stack([
        nx + (vx-wz*ny)*p.duty/(2*p.f_trot)*k,
        ny + wz*nx*p.duty/(2*p.f_trot)*k,
        nz + dz,
    ])
    hx = 0.1934 if leg in ('LF', 'RF') else -0.1934
    hy = 0.0465 if leg in ('LF', 'LH') else -0.0465
    lateral = 0.0955 if leg in ('LF', 'LH') else -0.0955
    x, y, z = (foot - np.array([hx, hy, 0])).T
    sagittal = -np.sqrt(np.maximum(1e-9, y*y + z*z - lateral*lateral))
    extension = np.minimum(1-1e-9, np.hypot(x, sagittal)/0.426)
    roll = (np.arctan2(z, y) - np.arctan2(sagittal, lateral) + np.pi) % math.tau - np.pi
    knee = -2*np.arccos(extension)
    thigh = np.arctan2(-x, -sagittal) - knee/2
    hip_r = rx(roll)
    hip_t = np.broadcast_to(np.array([hx, hy, 0]), (n, 3))
    thigh_r = hip_r @ ry(thigh)
    thigh_t = hip_t + hip_r @ np.array([0, lateral, 0])
    shank_r = thigh_r @ ry(knee)
    shank_t = thigh_t + thigh_r @ np.array([0, 0, -0.213])
    transforms[prefix+'_hip_link'] = hip_r, hip_t
    transforms[prefix+'_thigh_link'] = thigh_r, thigh_t
    transforms[prefix+'_shank_link'] = shank_r, shank_t

model = ET.parse(args.workspace / 'src/tugbot_description/models/go2/model.sdf').getroot().find('model')
records = []
for link in model.findall('link'):
    r, t = transforms[link.get('name')]
    for collision in link.findall('collision'):
        cr, ct = pose6(collision.findtext('pose', '0 0 0 0 0 0'))
        center, rotation = t + r @ ct, r @ cr
        geometry = collision.find('geometry')
        kind = list(geometry)[0].tag
        primitive = geometry.find(kind)
        if kind == 'box':
            dimensions = np.array(list(map(float, primitive.findtext('size').split())))/2
        else:
            dimensions = (float(primitive.findtext('radius')),
                          float(primitive.findtext('length', '0'))/2)
        records.append((collision.get('name'), center, rotation, kind, dimensions))

mins = [(math.inf, None), (math.inf, None)]
maxs = [(-math.inf, None), (-math.inf, None)]
for body_roll, body_pitch in itertools.product(np.linspace(-0.1, 0.1, 5), repeat=2):
    tilt = (ry(np.array([body_pitch])) @ rx(np.array([body_roll])))[0]
    for name, center, rotation, kind, dimensions in records:
        for axis in range(2):
            v = tilt[axis]
            c = np.einsum('ij,j->i', center, v)
            rv = np.einsum('nij,i->nj', rotation, v)
            if kind == 'box':
                extent = np.sum(np.abs(rv)*dimensions, axis=1)
            elif kind == 'sphere':
                extent = dimensions[0]
            elif kind == 'cylinder':
                extent = dimensions[0]*np.hypot(rv[:, 0], rv[:, 1]) + dimensions[1]*np.abs(rv[:, 2])
            else:
                raise ValueError(kind)
            lo, hi = c-extent, c+extent
            i, j = int(np.argmin(lo)), int(np.argmax(hi))
            if lo[i] < mins[axis][0]:
                mins[axis] = float(lo[i]), (name, configs[i].tolist(), float(body_roll), float(body_pitch))
            if hi[j] > maxs[axis][0]:
                maxs[axis] = float(hi[j]), (name, configs[j].tolist(), float(body_roll), float(body_pitch))
print('Configurations:', n, 'body orientations:', 25, 'primitive count:', len(records))
print('Configuration order: vx, wz, stabilizer roll, stabilizer pitch, phase')
print('min x:', mins[0])
print('max x:', maxs[0])
print('min y:', mins[1])
print('max y:', maxs[1])
print('Bounds including 1 cm tracking allowance:',
      mins[0][0]-0.01, maxs[0][0]+0.01,
      max(abs(mins[1][0]), abs(maxs[1][0]))+0.01)
