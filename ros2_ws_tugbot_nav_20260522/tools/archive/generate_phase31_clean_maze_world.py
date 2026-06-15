#!/usr/bin/env python3
"""Phase31 clean 2D binary maze image to candidate Gazebo world generator.

Diagnostics/static-generation only. This script imports a clean black/white maze
PNG, extracts editable orthogonal wall segments, writes a candidate SDF beside
existing worlds, and creates overlays for human acceptance. It never promotes the
world or starts navigation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.sax.saxutils as xml_escape
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from PIL import Image


@dataclass(frozen=True)
class Transform:
    width_px: int
    height_px: int
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def px_to_world(self, x_px: float, y_px: float) -> tuple[float, float]:
        x = self.xmin + float(x_px) / max(1, self.width_px - 1) * (self.xmax - self.xmin)
        y = self.ymax - float(y_px) / max(1, self.height_px - 1) * (self.ymax - self.ymin)
        return (round(x, 12), round(y, 12))

    def world_to_px(self, x_m: float, y_m: float) -> tuple[int, int]:
        x_px = (float(x_m) - self.xmin) / (self.xmax - self.xmin) * max(1, self.width_px - 1)
        y_px = (self.ymax - float(y_m)) / (self.ymax - self.ymin) * max(1, self.height_px - 1)
        return (int(round(x_px)), int(round(y_px)))


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'metadata root must be mapping: {path}')
    return data


def repo_root_from_metadata(path: Path) -> Path:
    # metadata lives under src/tugbot_maze/config in this workspace.
    return path.resolve().parents[3]


def resolve_repo_path(root: Path, value: str) -> Path:
    p = Path(value)
    return p if p.is_absolute() else root / p


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_source_image(root: Path, metadata: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, Path]:
    img_path = resolve_repo_path(root, metadata.get('source_image_path', ''))
    if not img_path.exists():
        raise FileNotFoundError(img_path)
    rgb = np.array(Image.open(img_path).convert('RGB'))
    gray = np.array(Image.open(img_path).convert('L'))
    return rgb, gray, img_path


def make_wall_mask(gray: np.ndarray, threshold: int) -> np.ndarray:
    # Clean image has anti-aliased black walls; threshold 128 includes dark/gray wall pixels.
    mask = (gray < int(threshold)).astype(np.uint8) * 255
    # Close small anti-alias gaps while preserving orthogonal line geometry.
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)


def bbox_from_mask(mask: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        raise ValueError('wall mask is empty')
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def merge_intervals(intervals: list[tuple[int, int]], gap: int) -> list[tuple[int, int]]:
    if not intervals:
        return []
    intervals = sorted(intervals)
    merged = [intervals[0]]
    for start, end in intervals[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end + gap:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def runs_in_line(values: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    in_run = False
    start = 0
    for i, v in enumerate(values):
        if v and not in_run:
            in_run = True
            start = i
        elif not v and in_run:
            runs.append((start, i - 1))
            in_run = False
    if in_run:
        runs.append((start, len(values) - 1))
    return runs


def extract_axis_segments(mask: np.ndarray, min_len: int, merge_gap_px: int = 5) -> list[dict[str, Any]]:
    binary = mask > 0
    horizontal_candidates: list[tuple[int, int, int]] = []
    for y in range(binary.shape[0]):
        for x0, x1 in runs_in_line(binary[y, :]):
            if x1 - x0 + 1 >= min_len:
                horizontal_candidates.append((y, x0, x1))

    vertical_candidates: list[tuple[int, int, int]] = []
    for x in range(binary.shape[1]):
        for y0, y1 in runs_in_line(binary[:, x]):
            if y1 - y0 + 1 >= min_len:
                vertical_candidates.append((x, y0, y1))

    # Collapse multi-pixel-thick strokes into centerline rows/columns. Group rows/cols that
    # are adjacent and have overlapping spans; then merge collinear intervals.
    horizontals_by_y: dict[int, list[tuple[int, int]]] = {}
    used = [False] * len(horizontal_candidates)
    for i, (y, x0, x1) in enumerate(horizontal_candidates):
        if used[i]:
            continue
        group = [(y, x0, x1)]
        used[i] = True
        changed = True
        while changed:
            changed = False
            gy0, gy1 = min(g[0] for g in group), max(g[0] for g in group)
            gx0, gx1 = min(g[1] for g in group), max(g[2] for g in group)
            for j, (yy, xx0, xx1) in enumerate(horizontal_candidates):
                if used[j]:
                    continue
                overlap = not (xx1 < gx0 - merge_gap_px or xx0 > gx1 + merge_gap_px)
                if abs(yy - gy0) <= merge_gap_px or abs(yy - gy1) <= merge_gap_px:
                    if overlap:
                        group.append((yy, xx0, xx1))
                        used[j] = True
                        changed = True
        center_y = int(round(float(np.median([g[0] for g in group]))))
        intervals = merge_intervals([(g[1], g[2]) for g in group], merge_gap_px)
        horizontals_by_y.setdefault(center_y, []).extend(intervals)

    verticals_by_x: dict[int, list[tuple[int, int]]] = {}
    used = [False] * len(vertical_candidates)
    for i, (x, y0, y1) in enumerate(vertical_candidates):
        if used[i]:
            continue
        group = [(x, y0, y1)]
        used[i] = True
        changed = True
        while changed:
            changed = False
            gx0, gx1 = min(g[0] for g in group), max(g[0] for g in group)
            gy0, gy1 = min(g[1] for g in group), max(g[2] for g in group)
            for j, (xx, yy0, yy1) in enumerate(vertical_candidates):
                if used[j]:
                    continue
                overlap = not (yy1 < gy0 - merge_gap_px or yy0 > gy1 + merge_gap_px)
                if abs(xx - gx0) <= merge_gap_px or abs(xx - gx1) <= merge_gap_px:
                    if overlap:
                        group.append((xx, yy0, yy1))
                        used[j] = True
                        changed = True
        center_x = int(round(float(np.median([g[0] for g in group]))))
        intervals = merge_intervals([(g[1], g[2]) for g in group], merge_gap_px)
        verticals_by_x.setdefault(center_x, []).extend(intervals)

    segments: list[dict[str, Any]] = []
    for y, intervals in sorted(horizontals_by_y.items()):
        for x0, x1 in merge_intervals(intervals, merge_gap_px):
            if x1 - x0 + 1 >= min_len:
                segments.append({'orientation': 'horizontal', 'p0_px': [int(x0), int(y)], 'p1_px': [int(x1), int(y)]})
    for x, intervals in sorted(verticals_by_x.items()):
        for y0, y1 in merge_intervals(intervals, merge_gap_px):
            if y1 - y0 + 1 >= min_len:
                segments.append({'orientation': 'vertical', 'p0_px': [int(x), int(y0)], 'p1_px': [int(x), int(y1)]})
    return classify_and_id_segments(segments, mask)


def classify_and_id_segments(segments: list[dict[str, Any]], mask: np.ndarray) -> list[dict[str, Any]]:
    x0, y0, x1, y1 = bbox_from_mask(mask)
    tol = 8
    out: list[dict[str, Any]] = []
    h_idx = v_idx = outer_idx = 0
    for seg in segments:
        p0 = seg['p0_px']; p1 = seg['p1_px']
        is_outer = False
        if seg['orientation'] == 'horizontal':
            y = p0[1]
            length = abs(p1[0] - p0[0])
            if (abs(y - y0) <= tol or abs(y - y1) <= tol) and length > (x1 - x0) * 0.45:
                is_outer = True
        else:
            x = p0[0]
            length = abs(p1[1] - p0[1])
            if (abs(x - x0) <= tol or abs(x - x1) <= tol) and length > (y1 - y0) * 0.45:
                is_outer = True
        seg = dict(seg)
        seg['outer'] = bool(is_outer)
        if is_outer:
            outer_idx += 1
            seg['id'] = f'outer_{outer_idx:03d}'
        elif seg['orientation'] == 'horizontal':
            h_idx += 1
            seg['id'] = f'h_{h_idx:03d}'
        else:
            v_idx += 1
            seg['id'] = f'v_{v_idx:03d}'
        out.append(seg)
    return out


def build_transform(metadata: dict[str, Any], width: int, height: int) -> Transform:
    b = metadata['world_bounds_m']
    return Transform(width, height, float(b['xmin']), float(b['xmax']), float(b['ymin']), float(b['ymax']))


def segment_to_wall(seg: dict[str, Any], transform: Transform, defaults: dict[str, Any], index: int) -> dict[str, Any]:
    x0, y0 = seg['p0_px']; x1, y1 = seg['p1_px']
    wx0, wy0 = transform.px_to_world(x0, y0)
    wx1, wy1 = transform.px_to_world(x1, y1)
    thickness = float(defaults['wall_thickness_m'])
    height = float(defaults['wall_height_m'])
    if seg['orientation'] == 'horizontal':
        length = abs(wx1 - wx0)
        sx, sy = max(length, thickness), thickness
        cx, cy = (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0
    else:
        length = abs(wy1 - wy0)
        sx, sy = thickness, max(length, thickness)
        cx, cy = (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0
    prefix = 'maze_wall_outer' if seg.get('outer') else 'maze_wall_seg'
    return {
        'name': f'{prefix}_{index:03d}_{seg["id"]}',
        'source_id': seg['id'],
        'orientation': seg['orientation'],
        'outer': bool(seg.get('outer')),
        'x': cx,
        'y': cy,
        'z': height / 2.0,
        'sx': sx,
        'sy': sy,
        'sz': height,
        'p0_world': [wx0, wy0],
        'p1_world': [wx1, wy1],
    }


def sdf_wall_model(wall: dict[str, Any]) -> str:
    name = xml_escape.escape(wall['name'])
    rgba = '0.92 0.92 0.90 1'
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{wall['x']:.3f} {wall['y']:.3f} {wall['z']:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{wall['sx']:.3f} {wall['sy']:.3f} {wall['sz']:.3f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{wall['sx']:.3f} {wall['sy']:.3f} {wall['sz']:.3f}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
        </visual>
      </link>
    </model>"""


def build_sdf(metadata: dict[str, Any], walls: list[dict[str, Any]]) -> str:
    entrance = metadata['entrance']
    exit_cfg = metadata['exit']
    models = '\n'.join(sdf_wall_model(w) for w in walls)
    return f"""<sdf version='1.10'>
  <world name="tugbot_maze_world_20260528_clean">
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'><render_engine>ogre2</render_engine></plugin>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <scene><shadows>false</shadows><ambient>0.55 0.55 0.55 1</ambient><background>0.70 0.72 0.74 1</background></scene>
    <physics name='3ms' type='ode'><max_step_size>0.003</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <atmosphere type='adiabatic'/>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry></collision>
        <visual name='visual'><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material></visual>
      </link>
    </model>
    <model name='maze_exit_marker'>
      <static>true</static>
      <pose>{float(exit_cfg['x_m']):.3f} {float(exit_cfg['y_m']):.3f} 0.01 0 0 0</pose>
      <link name='link'><visual name='visual'><geometry><cylinder><radius>{float(exit_cfg.get('radius_m', 0.6)):.3f}</radius><length>0.02</length></cylinder></geometry><material><ambient>0 0.8 0.2 0.7</ambient><diffuse>0 0.8 0.2 0.7</diffuse></material></visual></link>
    </model>
    <!-- Phase31 candidate clean 2D maze import. Not promoted; do not overwrite scaffold. -->
{models}
    <light name='sun' type='directional'><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation><direction>-0.5 0.1 -0.9</direction></light>
    <include><uri>model://tugbot</uri><name>tugbot</name><pose>{float(entrance['x_m']):.3f} {float(entrance['y_m']):.3f} 0 0 0 {float(entrance.get('yaw_rad', 0.0)):.3f}</pose></include>
  </world>
</sdf>
"""


def draw_segments(rgb: np.ndarray, segments: list[dict[str, Any]], transform: Transform | None = None, blank: bool = False) -> np.ndarray:
    out = np.full_like(rgb, 245) if blank else rgb.copy()
    for seg in segments:
        if transform is None:
            p0 = tuple(int(v) for v in seg['p0_px']); p1 = tuple(int(v) for v in seg['p1_px'])
        else:
            p0w = seg.get('p0_world')
            p1w = seg.get('p1_world')
            if not p0w or not p1w:
                raise ValueError(f"segment missing world points: {seg.get('name') or seg.get('id')}")
            p0 = transform.world_to_px(p0w[0], p0w[1])
            p1 = transform.world_to_px(p1w[0], p1w[1])
        color = (30, 80, 255) if seg.get('outer') else (255, 30, 30)
        cv2.line(out, p0, p1, color, 3 if not seg.get('outer') else 4)
    return out


def draw_entrance_exit(rgb: np.ndarray, metadata: dict[str, Any], transform: Transform) -> np.ndarray:
    out = rgb.copy()
    ex, ey = transform.world_to_px(float(metadata['entrance']['x_m']), float(metadata['entrance']['y_m']))
    gx, gy = transform.world_to_px(float(metadata['exit']['x_m']), float(metadata['exit']['y_m']))
    cv2.circle(out, (ex, ey), 10, (0, 180, 255), -1)
    cv2.putText(out, 'entrance candidate', (min(ex + 8, out.shape[1] - 160), max(20, ey - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 130, 255), 1, cv2.LINE_AA)
    cv2.circle(out, (gx, gy), 10, (0, 200, 0), -1)
    cv2.putText(out, 'exit candidate', (max(0, gx - 120), min(out.shape[0] - 8, gy + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 150, 0), 1, cv2.LINE_AA)
    return out


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def save_gray(path: Path, gray: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), gray)


def write_segments_yaml(path: Path, metadata: dict[str, Any], segments: list[dict[str, Any]], transform: Transform, walls: list[dict[str, Any]], img_path: Path, mask: np.ndarray) -> None:
    extraction = metadata['conversion']
    data = {
        'schema_version': 1,
        'source_image': metadata['source_image'],
        'source_image_path': metadata['source_image_path'],
        'source_image_kind': metadata['source_image_kind'],
        'source_image_sha256': sha256(img_path),
        'conversion': {'mode': metadata['conversion']['mode'], 'candidate_not_promoted': True},
        'world_bounds_m': metadata['world_bounds_m'],
        'transform': {
            'image_width_px': transform.width_px,
            'image_height_px': transform.height_px,
            'coordinate_origin': 'image_top_left',
            'x_m': metadata['transform']['x_m'],
            'y_m': metadata['transform']['y_m'],
        },
        'extraction': {
            'binary_threshold': int(extraction['binary_threshold']),
            'min_segment_length_px': int(extraction['min_segment_length_px']),
            'wall_mask_black_pixel_count': int((mask > 0).sum()),
            'wall_mask_black_pixel_ratio': float((mask > 0).mean()),
        },
        'wall_defaults': {
            'wall_thickness_m': float(extraction['wall_thickness_m']),
            'wall_height_m': float(extraction['wall_height_m']),
        },
        'generation_policy': {
            'do_not_overwrite_scaffold_world': True,
            'candidate_output': metadata['generation_outputs']['candidate_world_sdf'],
            'baseline_scaffold_world': metadata['baseline_preservation']['scaffold_world'],
            'phase29_candidate_world_preserved': metadata['baseline_preservation']['phase29_candidate_world'],
        },
        'entrance': metadata['entrance'],
        'exit': metadata['exit'],
        'segments': segments,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def generate(metadata_path: Path, output_segments: Path, output_world: Path, artifact_dir: Path) -> dict[str, Any]:
    metadata = load_yaml(metadata_path)
    root = repo_root_from_metadata(metadata_path)
    rgb, gray, img_path = load_source_image(root, metadata)
    threshold = int(metadata['conversion']['binary_threshold'])
    min_len = int(metadata['conversion']['min_segment_length_px'])
    mask = make_wall_mask(gray, threshold)
    transform = build_transform(metadata, gray.shape[1], gray.shape[0])
    segments = extract_axis_segments(mask, min_len)
    defaults = {
        'wall_thickness_m': float(metadata['conversion']['wall_thickness_m']),
        'wall_height_m': float(metadata['conversion']['wall_height_m']),
    }
    walls = [segment_to_wall(seg, transform, defaults, i) for i, seg in enumerate(segments, start=1)]

    scaffold = resolve_repo_path(root, metadata['baseline_preservation']['scaffold_world'])
    phase29 = resolve_repo_path(root, metadata['baseline_preservation']['phase29_candidate_world'])
    if output_world.resolve() == scaffold.resolve():
        raise ValueError('refusing to overwrite scaffold world')
    scaffold_before = sha256(scaffold)
    phase29_before = sha256(phase29)

    output_world.parent.mkdir(parents=True, exist_ok=True)
    output_world.write_text(build_sdf(metadata, walls), encoding='utf-8')
    write_segments_yaml(output_segments, metadata, segments, transform, walls, img_path, mask)

    artifact_dir.mkdir(parents=True, exist_ok=True)
    save_rgb(artifact_dir / '00_source_image.png', rgb)
    save_gray(artifact_dir / '01_binary_wall_mask.png', mask)
    overlay = draw_segments(rgb, segments)
    save_rgb(artifact_dir / '02_detected_wall_segments_overlay.png', overlay)
    wall_dicts_for_plan = [{**w, 'outer': w['outer']} for w in walls]
    plan = draw_segments(rgb, wall_dicts_for_plan, transform=transform, blank=True)
    save_rgb(artifact_dir / '03_generated_sdf_plan_view.png', plan)
    blended = cv2.addWeighted(rgb, 0.70, overlay, 0.30, 0)
    save_rgb(artifact_dir / '04_sdf_overlay_on_source_image.png', blended)
    entrance_exit = draw_entrance_exit(overlay, metadata, transform)
    save_rgb(artifact_dir / '05_entrance_exit_overlay.png', entrance_exit)

    inner_count = sum(1 for s in segments if not s.get('outer'))
    outer_count = len(segments) - inner_count
    summary = {
        'phase': 'Phase31',
        'status': 'candidate_generated_not_promoted',
        'source_image': str(img_path),
        'source_image_kind': metadata['source_image_kind'],
        'metadata': str(metadata_path),
        'output_segments': str(output_segments),
        'output_world': str(output_world),
        'artifact_dir': str(artifact_dir),
        'image_width_px': int(gray.shape[1]),
        'image_height_px': int(gray.shape[0]),
        'binary_threshold': threshold,
        'wall_mask_black_pixel_ratio': float((mask > 0).mean()),
        'wall_model_count_total': len(segments),
        'inner_wall_count': inner_count,
        'outer_wall_count': outer_count,
        'world_bounds_m': metadata['world_bounds_m'],
        'entrance': metadata['entrance'],
        'exit': metadata['exit'],
        'baseline_scaffold_world': str(scaffold),
        'baseline_scaffold_sha256_before': scaffold_before,
        'baseline_scaffold_sha256_after': sha256(scaffold),
        'baseline_scaffold_preserved': scaffold_before == sha256(scaffold),
        'phase29_candidate_world': str(phase29),
        'phase29_candidate_sha256_before': phase29_before,
        'phase29_candidate_sha256_after': sha256(phase29),
        'phase29_candidate_preserved': phase29_before == sha256(phase29),
        'guardrails': {
            'nav2_mppi_params_modified': False,
            'fallback_terminal_acceptance_modified': False,
            'navigation_strategy_modified': False,
            'runtime_navigation_started': False,
            'scaffold_world_overwritten': False,
            'candidate_not_promoted': True,
        },
        'artifacts': [
            '00_source_image.png',
            '01_binary_wall_mask.png',
            '02_detected_wall_segments_overlay.png',
            '03_generated_sdf_plan_view.png',
            '04_sdf_overlay_on_source_image.png',
            '05_entrance_exit_overlay.png',
            'phase31_generation_summary.json',
        ],
    }
    (artifact_dir / 'phase31_generation_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output-segments', type=Path, required=True)
    parser.add_argument('--output-world', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = generate(args.metadata, args.output_segments, args.output_world, args.artifact_dir)
    print(json.dumps({
        'output_world': summary['output_world'],
        'output_segments': summary['output_segments'],
        'wall_model_count_total': summary['wall_model_count_total'],
        'inner_wall_count': summary['inner_wall_count'],
        'outer_wall_count': summary['outer_wall_count'],
        'baseline_scaffold_preserved': summary['baseline_scaffold_preserved'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
