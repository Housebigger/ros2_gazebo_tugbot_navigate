from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[3]
NODE = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'phase67_goal1_timeout_visual_overlay.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase67_goal1_timeout_visual_replay.py'
LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'phase67_goal1_timeout_visual_replay.launch.py'
SCRIPT = ROOT / 'tools' / 'run_phase67_goal1_timeout_visual_replay.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase67_goal1_timeout_visual_replay_report.md'
PHASE66 = ROOT / 'log' / 'phase66_bounded_autonomous_rerun_from_inner_ingress' / 'phase66_bounded_autonomous_rerun_from_inner_ingress.json'


def _load_node_module():
    spec = importlib.util.spec_from_file_location('phase67_goal1_timeout_visual_overlay', NODE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_analyzer_module():
    spec = importlib.util.spec_from_file_location('analyze_phase67_goal1_timeout_visual_replay', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase67_overlay_node_contract_and_guardrails():
    assert NODE.exists(), 'Phase67 visual overlay node must exist'
    text = NODE.read_text()
    assert 'phase67_goal1_timeout_visual_overlay' in text
    assert '/phase67/goal1_timeout_visual_markers' in text
    assert 'visualization_msgs.msg' in text
    for marker_name in [
        'goal1_dispatch_target',
        'goal1_dispatch_pose',
        'goal1_timeout_terminal_pose',
        'goal1_trajectory_path',
        'terminal_robot_footprint_circle',
        'terminal_front_wedge',
        'target_tolerance_circle',
        'nearest_wall_or_costmap_risk_marker',
        'timeout_diagnostics_text',
    ]:
        assert marker_name in text
    assert 'no Nav2/MPPI/controller parameter edits' in text
    assert 'no branch scoring change' in text
    assert 'no target projection integration' in text
    assert 'not autonomous exploration success' in text
    assert 'not exit success' in text


def test_phase67_payload_and_marker_specs_from_phase66_artifact():
    analyzer = _load_analyzer_module()
    node = _load_node_module()
    payload = analyzer.build_phase67_payload(PHASE66)
    assert payload['run_id'] == 'phase67_goal1_timeout_visual_replay'
    assert payload['classification'] in {
        'VISUALLY_TARGET_TOO_CLOSE_TO_WALL',
        'VISUALLY_TURNING_FOOTPRINT_COLLIDES',
        'VISUALLY_GOAL_TOLERANCE_ORIENTATION_BLOCKED',
        'VISUALLY_COSTMAP_RECOVERY_LOOP',
        'VISUAL_EVIDENCE_INCONCLUSIVE',
    }
    assert payload['phase66_classification_preserved'] == 'INNER_INGRESS_TIMEOUT_REMAINS'
    assert payload['goal1']['dispatch_observed'] is True
    assert payload['goal1']['timeout'] is True
    assert payload['goal1']['target'] == [2.083600873682933, 1.022924811693503]
    assert payload['goal1']['terminal_pose'] == [2.4175180046806055, 1.0220449273557242, 1.5843474169880594]
    assert payload['goal1']['cmd_vel']['nonzero_command_count'] == 1056
    assert payload['goal1']['nav2_feedback']['number_of_recoveries_max'] == 4
    assert payload['goal1']['local_cost']['timeout_front_wedge_cost_max'] == 99
    assert payload['goal1']['local_cost']['timeout_footprint_lethal_cell_count'] == 48
    specs = node.build_marker_specs(payload)
    names = {spec['name'] for spec in specs}
    assert {
        'goal1_dispatch_target',
        'goal1_dispatch_pose',
        'goal1_timeout_terminal_pose',
        'goal1_trajectory_path',
        'terminal_robot_footprint_circle',
        'terminal_front_wedge',
        'target_tolerance_circle',
        'nearest_wall_or_costmap_risk_marker',
        'timeout_diagnostics_text',
    }.issubset(names)
    trajectory = next(spec for spec in specs if spec['name'] == 'goal1_trajectory_path')
    assert len(trajectory['points']) >= 2
    footprint = next(spec for spec in specs if spec['name'] == 'terminal_robot_footprint_circle')
    assert footprint['radius_m'] == payload['robot_radius_m']


def test_phase67_launch_and_script_are_visible_bounded_replay_only():
    assert LAUNCH.exists(), 'Phase67 visible launch must exist'
    launch_text = LAUNCH.read_text()
    assert 'phase67_goal1_timeout_visual_replay' in launch_text
    assert 'tugbot_maze_slam_nav.launch.py' in launch_text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in launch_text
    assert 'use_rviz' in launch_text and 'true' in launch_text
    assert 'headless' in launch_text and 'false' in launch_text
    assert 'phase67_goal1_timeout_visual_overlay' in launch_text
    assert 'nav2_slam_params.yaml' in launch_text
    assert 'target_projection' not in launch_text

    assert SCRIPT.exists(), 'Phase67 run script must exist'
    script_text = SCRIPT.read_text()
    assert 'PHASE67_MAX_GOALS="${PHASE67_MAX_GOALS:-1}"' in script_text
    assert 'max_goals:="$PHASE67_MAX_GOALS"' in script_text
    assert 'INNER_INGRESS_X="2.0"' in script_text
    assert '--send-inner-ingress-goal' in script_text
    assert '--record-runtime' in script_text
    assert '/maze/goal_events' in script_text
    assert '/maze/explorer_state' in script_text
    assert 'PHASE67_READY_FOR_HUMAN_OBSERVATION' in script_text
    assert 'no Nav2/MPPI/controller parameter edits' in script_text
    assert 'no target projection integration' in script_text


def test_phase67_report_contract_and_observation_steps():
    assert REPORT.exists(), 'Phase67 report must exist'
    text = REPORT.read_text()
    assert 'Phase67: Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis' in text
    assert 'VISUAL_EVIDENCE_INCONCLUSIVE' in text
    assert 'INNER_INGRESS_TIMEOUT_REMAINS' in text
    assert '/phase67/goal1_timeout_visual_markers' in text
    assert 'Suggested screenshot moments' in text
    assert 'dispatch target' in text
    assert 'terminal pose' in text
    assert 'trajectory path' in text
    assert 'front wedge' in text
    assert 'local costmap high-cost/lethal' in text
    assert '不调 Nav2/MPPI/controller' in text
    assert '不接入 target projection' in text
    assert '不声明 autonomous exploration success' in text
    assert '不声明 exit success' in text
