#!/usr/bin/env python3
"""Generate a small, visual-only green direction arrow for the Go2 back."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

NS = 'http://www.collada.org/2005/11/COLLADASchema'
ET.register_namespace('', NS)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, '{'+NS+'}'+tag, attributes)
    if text is not None:
        node.text = text
    return node


def prism(polygon, lower, upper):
    points = [(x, y, z) for z in (lower, upper) for x, y in polygon]
    top = [(0, 1, 6), (1, 5, 6), (1, 2, 3), (1, 3, 5), (3, 4, 5)]
    faces = [(c, b, a) for a, b, c in top]
    faces += [tuple(i+7 for i in face) for face in top]
    for i in range(7):
        j = (i+1) % 7
        faces += [(i, j, j+7), (i, j+7, i+7)]
    positions, normals = [], []
    for face in faces:
        a, b, c = (points[i] for i in face)
        u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
        normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(n*n for n in normal))
        assert length > 0
        for point in (a, b, c):
            positions.extend(point)
            normals.extend(n/length for n in normal)
    return positions, normals


def source(mesh, identifier, values):
    src = element(mesh, 'source', id=identifier)
    element(src, 'float_array', ' '.join(f'{v:.9g}' for v in values),
            id=identifier+'-array', count=str(len(values)))
    accessor = element(element(src, 'technique_common'), 'accessor',
                       source='#'+identifier+'-array', count=str(len(values)//3), stride='3')
    for name in ('X', 'Y', 'Z'):
        element(accessor, 'param', name=name, type='float')


def build():
    root = ET.Element('{'+NS+'}COLLADA', version='1.4.1')
    asset = element(root, 'asset')
    element(asset, 'created', '2026-11-07T00:00:00Z')
    element(asset, 'modified', '2026-11-07T00:00:00Z')
    element(asset, 'unit', name='meter', meter='1')
    element(asset, 'up_axis', 'Z_UP')
    effects, materials = element(root, 'library_effects'), element(root, 'library_materials')
    geometries = element(root, 'library_geometries')
    scene = element(element(root, 'library_visual_scenes'), 'visual_scene', id='Scene')
    layers = [
        ('outline', [(-.125, -.033), (.065, -.033), (.065, -.084), (.215, 0),
                     (.065, .084), (.065, .033), (-.125, .033)], 0., .003, '.008 .008 .008 1', '0 0 0 1'),
        ('green', [(-.115, -.025), (.070, -.025), (.070, -.075), (.205, 0),
                  (.070, .075), (.070, .025), (-.115, .025)], .0031, .0061, '.02 1 0 1', '0 .7 0 1')]
    for name, polygon, lower, upper, color, emission in layers:
        effect = element(effects, 'effect', id=name+'-effect')
        lambert = element(element(element(effect, 'profile_COMMON'), 'technique', sid='common'), 'lambert')
        for tag, value in [('emission', emission), ('ambient', color), ('diffuse', color)]:
            element(element(lambert, tag), 'color', value)
        element(element(materials, 'material', id=name+'-material'), 'instance_effect', url='#'+name+'-effect')
        mesh = element(element(geometries, 'geometry', id=name+'-geometry'), 'mesh')
        positions, normals = prism(polygon, lower, upper)
        source(mesh, name+'-positions', positions)
        source(mesh, name+'-normals', normals)
        element(element(mesh, 'vertices', id=name+'-vertices'), 'input',
                semantic='POSITION', source='#'+name+'-positions')
        triangles = element(mesh, 'triangles', count=str(len(positions)//9), material=name)
        element(triangles, 'input', semantic='VERTEX', source='#'+name+'-vertices', offset='0')
        element(triangles, 'input', semantic='NORMAL', source='#'+name+'-normals', offset='1')
        element(triangles, 'p', ' '.join(f'{i} {i}' for i in range(len(positions)//3)))
        instance = element(element(scene, 'node', id=name+'-node'), 'instance_geometry', url='#'+name+'-geometry')
        element(element(element(instance, 'bind_material'), 'technique_common'),
                'instance_material', symbol=name, target='#'+name+'-material')
    element(element(root, 'scene'), 'instance_visual_scene', url='#Scene')
    tree = ET.ElementTree(root)
    ET.indent(tree)
    return tree


if __name__ == '__main__':
    workspace = Path(__file__).resolve().parents[1]
    tree = build()
    for relative in ('src/go2_description/dae', 'src/tugbot_description/models/go2/dae'):
        path = workspace / relative / 'back_direction_arrow.dae'
        tree.write(path, encoding='utf-8', xml_declaration=True)
        print(path)
