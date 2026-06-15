#!/usr/bin/env python3
"""Generate a Phase29 image-faithful candidate Tugbot maze world.

Inputs are manually reviewed hybrid wall segments in normalized image-percent
coordinates. This script intentionally writes a candidate SDF path only and
refuses to overwrite the existing scaffold world unless the caller explicitly
points elsewhere and the YAML policy allows it.
"""
import argparse
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import xml.sax.saxutils as xml_escape

import cv2
import numpy as np
import yaml
from PIL import Image


@dataclass(frozen=True)
class ImageWorldTransform:
    image_width_px: int
    image_height_px: int
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> 'ImageWorldTransform':
        bounds = cfg['world_bounds_m']
        return cls(
            image_width_px=int(cfg['image_width_px']),
            image_height_px=int(cfg['image_height_px']),
            xmin=float(bounds['xmin']),
            xmax=float(bounds['xmax']),
            ymin=float(bounds['ymin']),
            ymax=float(bounds['ymax']),
        )

    def pixel_pct_to_world_xy(self, point_pct: tuple[float, float]) -> tuple[float, float]:
        x_pct, y_pct = float(point_pct[0]), float(point_pct[1])
        x = self.xmin + x_pct / 100.0 * (self.xmax - self.xmin)
        y = self.ymax - y_pct / 100.0 * (self.ymax - self.ymin)
        return (round(x, 12), round(y, 12))

    def world_xy_to_pixel_pct(self, xy: tuple[float, float]) -> tuple[float, float]:
        x, y = float(xy[0]), float(xy[1])
        x_pct = (x - self.xmin) / (self.xmax - self.xmin) * 100.0
        y_pct = (self.ymax - y) / (self.ymax - self.ymin) * 100.0
        return (x_pct, y_pct)

    def pixel_pct_to_image_px(self, point_pct: tuple[float, float]) -> tuple[int, int]:
        x_pct, y_pct = float(point_pct[0]), float(point_pct[1])
        return (
            int(round(x_pct / 100.0 * (self.image_width_px - 1))),
            int(round(y_pct / 100.0 * (self.image_height_px - 1))),
        )

    def world_xy_to_image_px(self, xy: tuple[float, float]) -> tuple[int, int]:
        return self.pixel_pct_to_image_px(self.world_xy_to_pixel_pct(xy))


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'YAML root must be mapping: {path}')
    return data


def validate_yaml(data: dict[str, Any]) -> None:
    if data.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    if data.get('review_status') != 'hybrid_manual_reviewed_candidate':
        raise ValueError('review_status must be hybrid_manual_reviewed_candidate')
    policy = data.get('generation_policy') or {}
    if policy.get('automatic_detector_dump_used_directly') is not False:
        raise ValueError('Phase29 requires automatic_detector_dump_used_directly: false')
    if policy.get('do_not_overwrite_scaffold_world') is not True:
        raise ValueError('Phase29 requires do_not_overwrite_scaffold_world: true')
    segments = data.get('segments')
    if not isinstance(segments, list) or len(segments) < 10:
        raise ValueError('segments must contain at least 10 entries')
    ids = set()
    for seg in segments:
        sid = seg.get('id')
        if not sid or sid in ids:
            raise ValueError(f'invalid/duplicate segment id: {sid}')
        ids.add(sid)
        if seg.get('orientation') not in ('horizontal', 'vertical'):
            raise ValueError(f'{sid}: orientation must be horizontal/vertical')
        for key in ('p0_pct', 'p1_pct'):
            point = seg.get(key)
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError(f'{sid}: {key} must be a 2-value list')
            if any(float(v) < 0.0 or float(v) > 100.0 for v in point):
                raise ValueError(f'{sid}: {key} out of percent range')


def segment_to_wall(seg: dict[str, Any], transform: ImageWorldTransform, defaults: dict[str, Any], index: int) -> dict[str, Any]:
    p0 = transform.pixel_pct_to_world_xy(tuple(seg['p0_pct']))
    p1 = transform.pixel_pct_to_world_xy(tuple(seg['p1_pct']))
    x0, y0 = p0
    x1, y1 = p1
    thickness = float(seg.get('thickness_m', defaults['thickness_m']))
    height = float(seg.get('height_m', defaults['height_m']))
    z = float(defaults.get('z_center_m', height / 2.0))
    orientation = seg['orientation']
    if orientation == 'horizontal':
        length = abs(x1 - x0)
        sx, sy = max(length, thickness), thickness
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    else:
        length = abs(y1 - y0)
        sx, sy = thickness, max(length, thickness)
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    if length <= 0:
        raise ValueError(f'{seg["id"]}: zero length')
    prefix = 'maze_wall_outer' if seg.get('outer', False) else 'maze_wall_seg'
    return {
        'source_id': seg['id'],
        'name': f'{prefix}_{index:03d}_{seg["id"]}',
        'x': cx,
        'y': cy,
        'z': z,
        'sx': sx,
        'sy': sy,
        'sz': height,
        'outer': bool(seg.get('outer', False)),
        'orientation': orientation,
        'p0_world': p0,
        'p1_world': p1,
        'p0_pct': seg['p0_pct'],
        'p1_pct': seg['p1_pct'],
    }


def sdf_wall_model(wall: dict[str, Any], rgba: list[float]) -> str:
    name = xml_escape.escape(wall['name'])
    ambient = ' '.join(f'{float(v):.3f}' for v in rgba)
    diffuse = ambient
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{wall['x']:.3f} {wall['y']:.3f} {wall['z']:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{wall['sx']:.3f} {wall['sy']:.3f} {wall['sz']:.3f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{wall['sx']:.3f} {wall['sy']:.3f} {wall['sz']:.3f}</size></box></geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def build_sdf(data: dict[str, Any], walls: list[dict[str, Any]]) -> str:
    world_name = data['generation_policy'].get('candidate_world_name', 'tugbot_maze_world_image_faithful')
    entrance = data['entrance']
    exit_cfg = data['exit']
    rgba = data['wall_defaults'].get('material_rgba', [0.92, 0.92, 0.90, 1.0])
    models = '\n'.join(sdf_wall_model(wall, rgba) for wall in walls)
    return f"""<sdf version='1.10'>
  <world name=\"{xml_escape.escape(world_name)}\">
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <scene>
      <shadows>false</shadows>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.72 0.74 1</background>
    </scene>
    <physics name='3ms' type='ode'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <model name='maze_exit_marker'>
      <static>true</static>
      <pose>{float(exit_cfg['x_m']):.3f} {float(exit_cfg['y_m']):.3f} 0.01 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><cylinder><radius>0.30</radius><length>0.02</length></cylinder></geometry>
          <material>
            <ambient>0 0.8 0.2 0.7</ambient>
            <diffuse>0 0.8 0.2 0.7</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <!-- Phase29 candidate: hybrid manually reviewed vectorization of maze_20260522.jpg. -->
    <!-- Do not overwrite tugbot_maze_world.sdf until separately promoted. -->
{models}
    <light name='sun' type='directional'>
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <include>
      <uri>model://tugbot</uri>
      <name>tugbot</name>
      <pose>{float(entrance['x_m']):.3f} {float(entrance['y_m']):.3f} 0 0 0 {float(entrance.get('yaw_rad', 0.0)):.3f}</pose>
    </include>
  </world>
</sdf>
"""


def read_rgb(path: Path) -> np.ndarray:
    return np.array(Image.open(path).convert('RGB'))


def draw_segments_on_image(img: np.ndarray, data: dict[str, Any], transform: ImageWorldTransform, *, draw_labels: bool = False) -> np.ndarray:
    out = img.copy()
    for seg in data['segments']:
        p0 = transform.pixel_pct_to_image_px(tuple(seg['p0_pct']))
        p1 = transform.pixel_pct_to_image_px(tuple(seg['p1_pct']))
        color = (40, 80, 255) if seg.get('outer', False) else (255, 30, 30)
        cv2.line(out, p0, p1, color, 4 if seg.get('outer', False) else 3)
        if draw_labels:
            mid = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
            cv2.putText(out, str(seg['id'])[:8], mid, cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
    return out


def draw_sdf_plan(data: dict[str, Any], walls: list[dict[str, Any]], transform: ImageWorldTransform) -> np.ndarray:
    h = transform.image_height_px
    w = transform.image_width_px
    canvas = np.full((h, w, 3), 245, dtype=np.uint8)
    for wall in walls:
        p0 = transform.world_xy_to_image_px(wall['p0_world'])
        p1 = transform.world_xy_to_image_px(wall['p1_world'])
        color = (40, 80, 255) if wall['outer'] else (255, 30, 30)
        thickness_px = max(3, int(round(float(data['wall_defaults']['thickness_m']) / (transform.xmax - transform.xmin) * w)))
        cv2.line(canvas, p0, p1, color, thickness_px)
    return canvas


def draw_entrance_exit(img: np.ndarray, data: dict[str, Any], transform: ImageWorldTransform) -> np.ndarray:
    out = img.copy()
    ent_px = transform.pixel_pct_to_image_px(tuple(data['entrance']['pixel_pct']))
    exit_px = transform.pixel_pct_to_image_px(tuple(data['exit']['pixel_pct']))
    cv2.circle(out, ent_px, 16, (0, 180, 255), -1)
    cv2.putText(out, 'entrance (-4,-3)', (ent_px[0] + 12, ent_px[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2, cv2.LINE_AA)
    cv2.circle(out, exit_px, 16, (0, 200, 0), -1)
    cv2.putText(out, 'exit (4,3)', (exit_px[0] - 130, exit_px[1] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 160, 0), 2, cv2.LINE_AA)
    return out


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(segments_yaml: Path, output_world: Path, artifact_dir: Path) -> dict[str, Any]:
    data = load_yaml(segments_yaml)
    validate_yaml(data)
    baseline = Path(data['generation_policy']['baseline_scaffold_world'])
    if not baseline.is_absolute():
        baseline = segments_yaml.parents[3] / baseline
    if output_world.resolve() == baseline.resolve():
        raise ValueError(f'refusing to overwrite scaffold world: {baseline}')
    before_baseline_hash = sha256(baseline)
    transform = ImageWorldTransform.from_config(data['transform'])
    walls = [segment_to_wall(seg, transform, data['wall_defaults'], i) for i, seg in enumerate(data['segments'], start=1)]
    output_world.parent.mkdir(parents=True, exist_ok=True)
    output_world.write_text(build_sdf(data, walls), encoding='utf-8')

    artifact_dir.mkdir(parents=True, exist_ok=True)
    source_image = Path(data['source_image'])
    if not source_image.is_absolute():
        source_image = segments_yaml.parents[3] / source_image
    source = read_rgb(source_image)
    segment_overlay = draw_segments_on_image(source, data, transform)
    save_rgb(artifact_dir / 'phase29_source_with_wall_segment_overlay.png', segment_overlay)
    plan = draw_sdf_plan(data, walls, transform)
    save_rgb(artifact_dir / 'phase29_generated_sdf_plan_view.png', plan)
    blended = cv2.addWeighted(source, 0.68, segment_overlay, 0.32, 0)
    save_rgb(artifact_dir / 'phase29_sdf_overlay_on_source_image.png', blended)
    entrance_exit = draw_entrance_exit(segment_overlay, data, transform)
    save_rgb(artifact_dir / 'phase29_entrance_exit_overlay.png', entrance_exit)

    inner_count = sum(1 for w in walls if not w['outer'])
    outer_count = len(walls) - inner_count
    total_length = sum(max(w['sx'], w['sy']) for w in walls)
    inner_length = sum(max(w['sx'], w['sy']) for w in walls if not w['outer'])
    summary = {
        'phase': 'Phase29',
        'status': 'candidate_generated_not_promoted',
        'segments_yaml': str(segments_yaml),
        'output_world': str(output_world),
        'artifact_dir': str(artifact_dir),
        'source_image': str(source_image),
        'wall_model_count_total': len(walls),
        'inner_wall_count': inner_count,
        'outer_wall_count': outer_count,
        'total_wall_centerline_length_m': total_length,
        'inner_wall_centerline_length_m': inner_length,
        'scaffold_inner_wall_baseline_count': 5,
        'improvement_inner_wall_count_delta': inner_count - 5,
        'world_bounds_m': data['transform']['world_bounds_m'],
        'entrance': data['entrance'],
        'exit': data['exit'],
        'baseline_scaffold_world': str(baseline),
        'baseline_scaffold_sha256_before': before_baseline_hash,
        'baseline_scaffold_sha256_after': sha256(baseline),
        'baseline_scaffold_preserved': before_baseline_hash == sha256(baseline),
        'guardrails': {
            'nav2_mppi_params_modified': False,
            'fallback_terminal_acceptance_modified': False,
            'navigation_strategy_modified': False,
            'runtime_navigation_started': False,
            'scaffold_world_overwritten': False,
            'candidate_not_promoted': True,
        },
        'artifacts': [
            'phase29_source_with_wall_segment_overlay.png',
            'phase29_generated_sdf_plan_view.png',
            'phase29_sdf_overlay_on_source_image.png',
            'phase29_entrance_exit_overlay.png',
        ],
    }
    (artifact_dir / 'phase29_generation_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--segments-yaml', type=Path, required=True)
    parser.add_argument('--output-world', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = generate(args.segments_yaml, args.output_world, args.artifact_dir)
    print(json.dumps({
        'output_world': summary['output_world'],
        'inner_wall_count': summary['inner_wall_count'],
        'artifact_dir': summary['artifact_dir'],
        'baseline_scaffold_preserved': summary['baseline_scaffold_preserved'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
