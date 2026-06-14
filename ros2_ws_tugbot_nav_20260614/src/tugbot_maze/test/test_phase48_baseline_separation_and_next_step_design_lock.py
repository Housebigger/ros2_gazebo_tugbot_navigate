from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
REPORT = ROOT / 'doc' / 'doc_report' / 'phase48_baseline_separation_and_next_step_design_lock_report.md'


def test_phase48_report_locks_manual_baseline_and_auto_failure_scope():
    text = REPORT.read_text()
    assert 'BASELINE_SEPARATION_AND_NEXT_STEP_DESIGN_LOCKED' in text
    assert 'MANUAL_NAV2_GOAL_BASELINE_OK' in text
    assert 'NAV2_LIFECYCLE_ACTIVE_RECOVERED' in text
    assert 'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING' in text
    assert 'TOPOLOGY_REJECTION_CAUSE_IDENTIFIED' in text
    assert '手动 Nav2 baseline 已经确认可用' in text
    assert '自动探索失败点限定在 `maze_explorer` 首次 topology sampling' in text


def test_phase48_report_requires_future_autonomous_preconditions():
    text = REPORT.read_text()
    required = [
        '等待 Nav2 lifecycle active',
        'scan/map sufficient',
        '不破坏手动基线',
        '/controller_server active [3]',
        '/planner_server active [3]',
        '/bt_navigator active [3]',
        '/navigate_to_pose Action servers: 1',
        '/goal_pose Subscription count: 1',
        'map/scan/TF/local costmap sufficiency evidence',
    ]
    for token in required:
        assert token in text


def test_phase48_report_forbids_mixing_distinct_problem_classes():
    text = REPORT.read_text()
    forbidden_mix = '禁止再把 lifecycle 未 active、手动 Goal 无响应、maze_explorer no-dispatch 混成一个问题'
    assert forbidden_mix in text
    assert 'Lifecycle activation problem' in text
    assert 'Manual Nav2 Goal baseline problem' in text
    assert 'maze_explorer topology sampling/dispatch entry problem' in text
    assert '不得用 Nav2 lifecycle 或手动 Goal 问题解释 Phase42/43 no-dispatch' in text
    assert '不得用 Phase42/43 no-dispatch 否定 Phase47 手动基线' in text


def test_phase48_report_has_guardrails_and_no_runtime_experiment():
    text = REPORT.read_text()
    assert '本阶段不跑实验' in text
    assert '未修改 Nav2/MPPI/controller 参数' in text
    assert '未修改 `maze_explorer`' in text
    assert '未运行 Gazebo/SLAM/Nav2/maze_explorer runtime' in text
    assert '不声明自主探索成功' in text
    assert 'git diff -- src/tugbot_navigation/config: empty' in text
    assert 'cleanup check: empty' in text
