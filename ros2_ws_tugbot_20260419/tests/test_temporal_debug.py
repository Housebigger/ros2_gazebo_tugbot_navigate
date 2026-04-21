from importlib import util
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
TEMPORAL_MODULE_PATH = WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'tugbot_perception' / 'temporal_debug.py'


def load_module(path: Path, module_name: str):
    assert path.exists(), f'{path} should exist'
    spec = util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_test_image(width: int, height: int, blue_column: int | None):
    image = []
    for row_index in range(height):
        row = []
        for column_index in range(width):
            if blue_column is not None and row_index >= height // 2 and column_index == blue_column:
                row.append([10, 10, 220])
            else:
                row.append([30, 30, 30])
        image.append(row)
    return image


def test_summarize_frame_marks_detected_when_blue_lane_is_present():
    module = load_module(TEMPORAL_MODULE_PATH, 'temporal_debug_detected')
    summary = module.summarize_frame(
        make_test_image(width=9, height=6, blue_column=4),
        blue_threshold=80,
        blue_margin=20,
        crop_top_ratio=0.45,
    )
    assert summary['lane_center'] == 4
    assert summary['detected'] is True
    assert summary['blue_mask_pixels'] > 0
    assert summary['roi_blue_mask_pixels'] > 0


def test_summarize_frame_marks_missing_for_uniform_gray_image():
    module = load_module(TEMPORAL_MODULE_PATH, 'temporal_debug_missing')
    numpy = pytest.importorskip('numpy')
    image = numpy.full((6, 9, 3), 91, dtype=numpy.uint8)
    summary = module.summarize_frame(
        image,
        blue_threshold=80,
        blue_margin=20,
        crop_top_ratio=0.45,
    )
    assert summary['lane_center'] is None
    assert summary['detected'] is False
    assert summary['blue_mask_pixels'] == 0
    assert summary['roi_blue_mask_pixels'] == 0


def test_extract_transition_windows_returns_detected_missing_boundaries():
    module = load_module(TEMPORAL_MODULE_PATH, 'temporal_debug_transitions')
    samples = [
        {'frame_index': 0, 'detected': False},
        {'frame_index': 1, 'detected': False},
        {'frame_index': 2, 'detected': True},
        {'frame_index': 3, 'detected': True},
        {'frame_index': 4, 'detected': False},
        {'frame_index': 5, 'detected': False},
    ]
    windows = module.extract_transition_windows(samples, before=1, after=1)
    assert [window['transition'] for window in windows] == ['missing_to_detected', 'detected_to_missing']
    assert windows[0]['indices'] == [1, 2, 3]
    assert windows[1]['indices'] == [3, 4, 5]
