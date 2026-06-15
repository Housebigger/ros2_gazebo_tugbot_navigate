#!/usr/bin/env python3
"""Phase28 static fidelity audit for the 20260522 maze image vs current Gazebo world.

Diagnostics-only: generates image artifacts and JSON metrics. It does not rewrite
worlds, navigation configs, or controller parameters.
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def save(path: Path, array: np.ndarray) -> None:
    ensure_dir(path.parent)
    cv2.imwrite(str(path), array)


def parse_walls(world_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(world_path).getroot()
    walls = []
    for model in root.findall('.//world/model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        pose_vals = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        size_text = model.findtext('.//collision/geometry/box/size') or model.findtext('.//visual/geometry/box/size')
        if not size_text:
            continue
        size_vals = [float(v) for v in size_text.split()]
        x, y = pose_vals[0], pose_vals[1]
        sx, sy = size_vals[0], size_vals[1]
        walls.append({
            'name': name,
            'x': x,
            'y': y,
            'sx': sx,
            'sy': sy,
            'xmin': x - sx / 2,
            'xmax': x + sx / 2,
            'ymin': y - sy / 2,
            'ymax': y + sy / 2,
            'area_m2': sx * sy,
            'is_outer': name in {'maze_wall_bottom', 'maze_wall_top', 'maze_wall_left', 'maze_wall_right'},
        })
    return walls


def world_to_px(x: float, y: float, width: int, height: int, extent: float = 6.0) -> tuple[int, int]:
    px = int(round((x + extent) / (2 * extent) * (width - 1)))
    py = int(round((extent - y) / (2 * extent) * (height - 1)))
    return px, py


def draw_world_overlay(base_rgb: np.ndarray, walls: list[dict[str, Any]], extent: float = 6.0) -> np.ndarray:
    out = base_rgb.copy()
    h, w = out.shape[:2]
    overlay = out.copy()
    for wall in walls:
        x0, y0 = world_to_px(wall['xmin'], wall['ymax'], w, h, extent)
        x1, y1 = world_to_px(wall['xmax'], wall['ymin'], w, h, extent)
        color = (30, 60, 255) if wall['is_outer'] else (255, 40, 40)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), color, thickness=-1)
        cv2.rectangle(out, (x0, y0), (x1, y1), color, thickness=2)
    return cv2.addWeighted(overlay, 0.35, out, 0.65, 0)


def crop_maze_region(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    # Conservative crop for this square 1000x1000 reference: remove only outer
    # blank margin/watermark area while preserving full maze body. Also compute a
    # simple content bbox from non-background pixels and use the larger/safer of
    # the two.
    h, w = rgb.shape[:2]
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    bg = np.median(np.concatenate([gray[:40, :].ravel(), gray[-40:, :].ravel(), gray[:, :40].ravel(), gray[:, -40:].ravel()]))
    diff = cv2.absdiff(gray, np.full_like(gray, int(bg)))
    _, content = cv2.threshold(diff, 12, 255, cv2.THRESH_BINARY)
    ys, xs = np.where(content > 0)
    if len(xs) == 0:
        return rgb, (0, 0, w, h)
    x0, x1 = max(0, int(xs.min()) - 25), min(w, int(xs.max()) + 26)
    y0, y1 = max(0, int(ys.min()) - 25), min(h, int(ys.max()) + 26)
    # Avoid accidental over-crop from watermark/noise by keeping a centered square.
    side = max(x1 - x0, y1 - y0, int(min(w, h) * 0.82))
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    x0 = max(0, cx - side // 2)
    y0 = max(0, cy - side // 2)
    x1 = min(w, x0 + side)
    y1 = min(h, y0 + side)
    x0 = max(0, x1 - side)
    y0 = max(0, y1 - side)
    return rgb[y0:y1, x0:x1], (x0, y0, x1, y1)


def connected_stats(mask: np.ndarray) -> dict[str, Any]:
    n, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype('uint8'), 8)
    areas = [int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, n)]
    return {
        'component_count': max(0, n - 1),
        'component_area_max_px': max(areas) if areas else 0,
        'component_area_median_px': float(np.median(areas)) if areas else 0.0,
        'component_area_sum_px': int(sum(areas)),
    }


def line_segments(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    min_len = max(30, min(mask.shape[:2]) // 18)
    raw = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40, minLineLength=min_len, maxLineGap=8)
    if raw is None:
        return []
    segs = []
    for line in raw[:, 0, :]:
        x1, y1, x2, y2 = [int(v) for v in line]
        angle = abs(math.atan2(y2 - y1, x2 - x1))
        # Keep mainly orthogonal maze-like segments.
        if min(abs(angle), abs(angle - math.pi), abs(angle - math.pi / 2)) < math.radians(12):
            segs.append((x1, y1, x2, y2))
    return segs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-image', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    outdir = args.output_dir
    ensure_dir(outdir)

    pil = Image.open(args.source_image).convert('RGB')
    rgb = np.array(pil)
    h, w = rgb.shape[:2]
    save(outdir / '00_source_image.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    cropped, crop = crop_maze_region(rgb)
    save(outdir / '01_cropped_maze_region.png', cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR))
    gray = cv2.cvtColor(cropped, cv2.COLOR_RGB2GRAY)
    save(outdir / '02_grayscale.png', gray)

    # Multiple static masks for audit visibility, not for world generation.
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, otsu_dark = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    _, otsu_light = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    adaptive_dark = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 41, 4)
    save(outdir / '03_threshold_binary_otsu_dark.png', otsu_dark)
    save(outdir / '04_threshold_binary_otsu_light.png', otsu_light)
    save(outdir / '05_threshold_binary_adaptive_dark.png', adaptive_dark)

    # For this white-wall illustration, light maze material is the best static
    # proxy for raised walls; dark/adaptive masks mainly capture shadows.
    wall_mask = cv2.morphologyEx(otsu_light, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    save(outdir / '06_wall_mask_light_proxy.png', wall_mask)

    contours, _ = cv2.findContours(wall_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_overlay = cropped.copy()
    cv2.drawContours(contour_overlay, contours, -1, (255, 0, 0), 2)
    save(outdir / '07_contour_overlay.png', cv2.cvtColor(contour_overlay, cv2.COLOR_RGB2BGR))

    segs = line_segments(wall_mask)
    segment_overlay = cropped.copy()
    for x1, y1, x2, y2 in segs:
        cv2.line(segment_overlay, (x1, y1), (x2, y2), (0, 0, 255), 2)
    save(outdir / '08_generated_wall_segments_overlay_static_detected.png', cv2.cvtColor(segment_overlay, cv2.COLOR_RGB2BGR))

    walls = parse_walls(args.world)
    world_overlay = draw_world_overlay(cropped, walls)
    save(outdir / '09_final_world_footprint_overlay_on_source.png', cv2.cvtColor(world_overlay, cv2.COLOR_RGB2BGR))

    # Separate SDF-only plan view for clarity.
    canvas = np.full_like(cropped, 245)
    sdf_overlay = draw_world_overlay(canvas, walls)
    save(outdir / '10_current_sdf_wall_plan_view.png', cv2.cvtColor(sdf_overlay, cv2.COLOR_RGB2BGR))

    crop_h, crop_w = cropped.shape[:2]
    maze_area_px = crop_h * crop_w
    wall_area_px = int(np.count_nonzero(wall_mask))
    border = int(max(5, min(crop_w, crop_h) * 0.08))
    interior = wall_mask[border:crop_h-border, border:crop_w-border]
    interior_area_px = int(interior.size)
    interior_wall_px = int(np.count_nonzero(interior))
    outer_walls = [wall for wall in walls if wall['is_outer']]
    inner_walls = [wall for wall in walls if not wall['is_outer']]
    total_world_area = 12.0 * 12.0
    sdf_wall_area = sum(wall['area_m2'] for wall in walls)
    sdf_inner_area = sum(wall['area_m2'] for wall in inner_walls)
    sdf_outer_area = sum(wall['area_m2'] for wall in outer_walls)

    # Estimate source-side orthogonal segment count using contours and Hough lines.
    contour_areas = [cv2.contourArea(c) for c in contours]
    large_contours = [a for a in contour_areas if a >= 100]
    metrics = {
        'phase': 'Phase28',
        'source_image': str(args.source_image),
        'world': str(args.world),
        'source_dimensions_px': [w, h],
        'crop_xyxy_px': list(crop),
        'crop_dimensions_px': [crop_w, crop_h],
        'image_processing_chain_status': 'diagnostic_static_only_no_existing_automated_generation_chain',
        'existing_generation_chain': {
            'maze_image_to_world': 'placeholder_only',
            'binarization_threshold_in_existing_chain': None,
            'mask_in_existing_chain': None,
            'contour_or_skeleton_in_existing_chain': None,
            'wall_segment_generation_in_existing_chain': None,
            'world_authoring_mode': 'manual_simplified_first_pass',
        },
        'diagnostic_image_metrics': {
            'wall_mask_light_proxy_area_ratio': wall_area_px / maze_area_px,
            'interior_wall_mask_area_ratio': interior_wall_px / interior_area_px if interior_area_px else None,
            'contour_count_total': len(contours),
            'contour_count_area_ge_100_px': len(large_contours),
            'orthogonal_hough_segment_count': len(segs),
            'wall_mask_connected_components': connected_stats(wall_mask),
            'otsu_dark_connected_components': connected_stats(otsu_dark),
            'adaptive_dark_connected_components': connected_stats(adaptive_dark),
        },
        'sdf_world_metrics': {
            'maze_wall_model_count_total': len(walls),
            'outer_wall_count': len(outer_walls),
            'inner_wall_count': len(inner_walls),
            'sdf_wall_area_m2': sdf_wall_area,
            'sdf_inner_wall_area_m2': sdf_inner_area,
            'sdf_outer_wall_area_m2': sdf_outer_area,
            'sdf_wall_area_ratio_vs_12x12': sdf_wall_area / total_world_area,
            'sdf_inner_wall_area_ratio_vs_12x12': sdf_inner_area / total_world_area,
            'walls': walls,
        },
        'fidelity_findings': {
            'current_world_is_manual_scaffold': True,
            'current_world_has_only_five_inner_walls': len(inner_walls) == 5,
            'source_image_is_complex_maze': True,
            'automated_extraction_missing': True,
            'likely_reason_for_missing_walls': 'current SDF was manually authored as simplified first-pass scaffold; maze_image_to_world is placeholder and never generated walls from image masks/contours',
            'threshold_or_contour_filter_bug_found_in_existing_chain': False,
        },
        'coordinate_metadata': {
            'metadata_resolution_m_per_pixel': 0.02,
            'metadata_implied_full_image_width_m': 20.0,
            'current_world_extent_m': [-6.0, 6.0, -6.0, 6.0],
            'current_world_width_m': 12.0,
            'source_to_world_scale_mismatch_note': 'metadata 1000px at 0.02m/px implies 20m width, while current SDF maze walls span 12m; scaffold used manual scale rather than image-derived full-width scale',
            'entrance_m': [-4.0, -3.0],
            'exit_m': [4.0, 3.0],
        },
        'artifacts': [str(p.relative_to(outdir)) for p in sorted(outdir.glob('*.png'))],
    }
    (outdir / 'phase28_metrics.json').write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({'output_dir': str(outdir), 'metrics': str(outdir / 'phase28_metrics.json'), 'inner_walls': len(inner_walls), 'hough_segments': len(segs)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
