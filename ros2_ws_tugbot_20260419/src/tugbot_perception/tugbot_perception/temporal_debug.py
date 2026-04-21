from __future__ import annotations

from importlib import util
from pathlib import Path
from typing import Any

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


def _load_lane_detector_module():
    module_path = Path(__file__).with_name('lane_detector_node.py')
    spec = util.spec_from_file_location('tugbot_perception_lane_detector_for_temporal_debug', module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Unable to load lane detector module from {module_path}')
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_LANE_DETECTOR_MODULE = _load_lane_detector_module()
detect_blue_lane_center = _LANE_DETECTOR_MODULE.detect_blue_lane_center


def _to_rgb_array(image):
    if np is not None and hasattr(image, 'shape'):
        arr = image[:, :, :3]
        if arr.dtype != np.int16:
            return arr.astype(np.int16, copy=False)
        return arr
    if np is None:
        raise RuntimeError('numpy is required for temporal frame summaries')
    return np.asarray(image, dtype=np.int16)[:, :, :3]


def summarize_frame(image, blue_threshold: int, blue_margin: int, crop_top_ratio: float) -> dict[str, Any]:
    arr = _to_rgb_array(image)
    height = int(arr.shape[0])
    width = int(arr.shape[1])
    start_row = min(height - 1, max(0, int(height * crop_top_ratio)))
    roi = arr[start_row:, :, :3]
    blue_mask = (
        (arr[:, :, 2] > blue_threshold)
        & (arr[:, :, 2] > arr[:, :, 0] + blue_margin)
        & (arr[:, :, 2] > arr[:, :, 1] + blue_margin)
    )
    roi_mask = blue_mask[start_row:, :]
    lane_center = detect_blue_lane_center(
        arr,
        blue_threshold=blue_threshold,
        blue_margin=blue_margin,
        crop_top_ratio=crop_top_ratio,
    )
    return {
        'shape': [height, width, 3],
        'lane_center': lane_center,
        'detected': lane_center is not None,
        'mean_rgb': [float(x) for x in arr.mean(axis=(0, 1))],
        'roi_mean_rgb': [float(x) for x in roi.mean(axis=(0, 1))],
        'blue_mask_pixels': int(blue_mask.sum()),
        'roi_blue_mask_pixels': int(roi_mask.sum()),
        'blue_mask_ratio': float(blue_mask.mean()),
        'roi_blue_mask_ratio': float(roi_mask.mean()),
    }


def extract_transition_windows(samples: list[dict[str, Any]], before: int = 2, after: int = 2) -> list[dict[str, Any]]:
    windows = []
    for idx in range(1, len(samples)):
        prev_detected = bool(samples[idx - 1].get('detected', False))
        current_detected = bool(samples[idx].get('detected', False))
        if prev_detected == current_detected:
            continue
        start = max(0, idx - before)
        end = min(len(samples), idx + after + 1)
        transition = 'missing_to_detected' if current_detected else 'detected_to_missing'
        window_samples = samples[start:end]
        windows.append({
            'transition': transition,
            'indices': [int(sample['frame_index']) for sample in window_samples],
            'samples': window_samples,
        })
    return windows
