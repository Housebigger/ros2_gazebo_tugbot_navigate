from importlib import util
from pathlib import Path

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_MODULE_PATH = WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'tugbot_perception' / 'lane_detector_node.py'
CONTROL_MODULE_PATH = WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'tugbot_control' / 'lane_controller_node.py'



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



def test_detect_blue_lane_center_returns_middle_for_centered_track():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test')
    center = module.detect_blue_lane_center(make_test_image(width=9, height=6, blue_column=4))
    assert center == 4



def test_detect_blue_lane_center_returns_none_when_track_is_missing():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_missing')
    center = module.detect_blue_lane_center(make_test_image(width=9, height=6, blue_column=None))
    assert center is None



def test_detect_blue_lane_center_does_not_misclassify_uniform_gray_numpy_image_as_blue():
    numpy = pytest.importorskip('numpy')
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_gray_numpy')
    image = numpy.full((6, 9, 3), 225, dtype=numpy.uint8)
    center = module.detect_blue_lane_center(image, blue_threshold=100, blue_margin=40, crop_top_ratio=0.5)
    assert center is None



def test_compute_lane_error_positive_when_lane_is_left_of_center():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_error_left')
    error = module.compute_lane_error(image_width=9, lane_center=1)
    assert error > 0.0



def test_compute_lane_error_negative_when_lane_is_right_of_center():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_error_right')
    error = module.compute_lane_error(image_width=9, lane_center=7)
    assert error < 0.0



def test_update_detection_state_holds_previous_lane_for_short_missing_burst():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_hold_logic')
    lane_center, missing_frames = module.update_detection_state(
        detected_lane_center=None,
        prev_lane_center=319,
        prev_missing_frames=0,
        detection_hold_frames=3,
    )
    assert lane_center == 319
    assert missing_frames == 1



def test_update_detection_state_drops_lane_after_hold_budget_is_exhausted():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_hold_drop')
    lane_center, missing_frames = module.update_detection_state(
        detected_lane_center=None,
        prev_lane_center=319,
        prev_missing_frames=3,
        detection_hold_frames=3,
    )
    assert lane_center is None
    assert missing_frames == 4



def test_update_detection_state_resets_missing_counter_when_detection_returns():
    module = load_module(PERCEPTION_MODULE_PATH, 'lane_detector_under_test_hold_reset')
    lane_center, missing_frames = module.update_detection_state(
        detected_lane_center=320,
        prev_lane_center=319,
        prev_missing_frames=2,
        detection_hold_frames=3,
    )
    assert lane_center == 320
    assert missing_frames == 0



def test_compute_control_command_turns_left_for_positive_error():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_left')
    linear_x, angular_z, integral = module.compute_control_command(
        error=0.5,
        prev_error=0.0,
        integral=0.0,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
        },
    )
    assert linear_x > 0.0
    assert angular_z > 0.0
    assert integral > 0.0



def test_compute_control_command_turns_right_for_negative_error():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_right')
    linear_x, angular_z, _ = module.compute_control_command(
        error=-0.5,
        prev_error=0.0,
        integral=0.0,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
        },
    )
    assert linear_x > 0.0
    assert angular_z < 0.0



def test_compute_control_command_stops_when_error_is_missing():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_stop')
    linear_x, angular_z, integral = module.compute_control_command(
        error=None,
        prev_error=0.0,
        integral=0.3,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
        },
    )
    assert linear_x == 0.0
    assert angular_z == 0.0
    assert integral == 0.0


def test_compute_control_output_enters_counterclockwise_spin_search_when_error_is_missing_and_search_enabled():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_search_spin')
    linear_x, angular_z, integral, search_state = module.compute_control_output(
        error=None,
        prev_error=0.0,
        integral=0.4,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
            'search_enabled': True,
            'search_spin_angular_speed': 0.3,
            'search_spin_revolution_target': 6.28318,
            'search_arc_linear_speed': 0.08,
            'search_arc_angular_speed': 0.2,
            'search_arc_revolution_target': 6.28318,
            'search_default_turn_direction': 1.0,
        },
        search_state=None,
    )
    assert linear_x == 0.0
    assert angular_z > 0.0
    assert integral == 0.0
    assert search_state.mode == 'spin'
    assert search_state.progress > 0.0


def test_compute_control_output_transitions_from_spin_to_arc_after_full_rotation_without_detection():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_search_arc_transition')
    linear_x, angular_z, integral, search_state = module.compute_control_output(
        error=None,
        prev_error=0.0,
        integral=0.2,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
            'search_enabled': True,
            'search_spin_angular_speed': 0.3,
            'search_spin_revolution_target': 0.2,
            'search_arc_linear_speed': 0.08,
            'search_arc_angular_speed': 0.2,
            'search_arc_revolution_target': 6.28318,
            'search_default_turn_direction': 1.0,
        },
        search_state=module.SearchState(mode='spin', progress=0.19, arc_direction=1.0),
    )
    assert linear_x > 0.0
    assert angular_z > 0.0
    assert integral == 0.0
    assert search_state.mode == 'arc'
    assert search_state.progress == 0.0


def test_compute_control_output_flips_arc_direction_after_full_loop_without_detection():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_search_arc_flip')
    linear_x, angular_z, integral, search_state = module.compute_control_output(
        error=None,
        prev_error=0.0,
        integral=0.2,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
            'search_enabled': True,
            'search_spin_angular_speed': 0.3,
            'search_spin_revolution_target': 6.28318,
            'search_arc_linear_speed': 0.08,
            'search_arc_angular_speed': 0.2,
            'search_arc_revolution_target': 0.25,
            'search_default_turn_direction': 1.0,
        },
        search_state=module.SearchState(mode='arc', progress=0.24, arc_direction=1.0),
    )
    assert linear_x > 0.0
    assert angular_z < 0.0
    assert integral == 0.0
    assert search_state.mode == 'arc'
    assert search_state.arc_direction == -1.0
    assert search_state.progress == 0.0


def test_compute_control_output_resets_search_and_returns_pid_when_detection_reappears():
    module = load_module(CONTROL_MODULE_PATH, 'lane_controller_under_test_search_reset')
    linear_x, angular_z, integral, search_state = module.compute_control_output(
        error=0.2,
        prev_error=0.0,
        integral=0.0,
        dt=0.1,
        config={
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.05,
            'integral_limit': 1.5,
            'base_linear_speed': 0.25,
            'min_linear_speed': 0.05,
            'max_angular_speed': 1.2,
            'speed_reduction_gain': 0.6,
            'search_enabled': True,
            'search_spin_angular_speed': 0.3,
            'search_spin_revolution_target': 6.28318,
            'search_arc_linear_speed': 0.08,
            'search_arc_angular_speed': 0.2,
            'search_arc_revolution_target': 6.28318,
            'search_default_turn_direction': 1.0,
        },
        search_state=module.SearchState(mode='arc', progress=1.0, arc_direction=-1.0),
    )
    assert linear_x > 0.0
    assert angular_z > 0.0
    assert integral > 0.0
    assert search_state.mode == 'idle'
    assert search_state.arc_direction == 1.0
