# Contributing

Thank you for considering a contribution to this workspace.

This repository is a ROS 2 Jazzy + Gazebo Harmonic engineering workspace focused on a minimal but extensible tugbot visual cruise stack. The project values clear architecture boundaries, reproducible validation, and experience capture.

## 1. Before You Change Anything

Please read first:

- `README.md`
- `CHANGELOG.md`
- `../doc/doc_experience/zeroerr_outer_search_recovery_cold_start.md`
- `../doc/doc_experience/tugbot_model_visual_detail_restoration_20260422.md`

If your change touches tugbot model visuals, world files, perception thresholds, spawn pose, or recovery behavior, update the relevant experience document or add a new one under `../doc/doc_experience/`.

## 2. Development Principles

### Preserve the layered architecture

Keep responsibilities separated:

- `tugbot_description`: model and packaged assets
- `tugbot_gazebo`: worlds and bridge config
- `tugbot_perception`: image-to-error logic
- `tugbot_control`: error-to-cmd_vel logic
- `tugbot_bringup`: launch orchestration

Do not hide major behavior changes inside unrelated packages.

### Prefer minimal-variable validation

When debugging a suspected runtime issue, prefer changing as little as possible:

- keep working architecture intact
- change only the disturbance or single layer under test
- verify with live evidence instead of relying only on static reasoning

### Preserve validated baselines

If a world or flow is already validated:

- avoid overwriting the formal baseline world
- add a separate world variant for harder scenes or robustness experiments
- prefer dedicated wrapper launch files over repeated long command-line overrides

### Model restoration boundary

For tugbot model detail work, the current default policy is:

- restore visual detail if needed
- do not reintroduce legacy sensor chains unless the task explicitly requires it

## 3. Validation Requirements

Before opening a PR or sharing a patch, run:

```bash
python3 -m pytest tests -q
```

If your change affects packaging or model assets, also run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_description
```

If your change affects bringup, perception, control, or world selection, also do a live run with one of:

```bash
ros2 launch tugbot_bringup sim_minimal.launch.py
ros2 launch tugbot_bringup full_system.launch.py
ros2 launch tugbot_bringup perception_debug.launch.py
ros2 launch tugbot_bringup full_system_zeroerr_outer.launch.py
```

## 4. Expected Evidence in Contributions

Good contributions include:

- what changed
- why it changed
- what files were modified
- what validation was run
- what was observed in live evidence, tests, or logs

For non-trivial engineering changes, prefer reporting in this structure:

- 完成内容
- 新建或修改文件
- 设计理由
- 下一步计划

## 5. Commit Scope

Please keep commits focused.

Examples of good scopes:

- one world-design change
- one perception bugfix
- one model-asset restoration batch
- one documentation update tied to a single engineering topic

Avoid mixing unrelated launch, model, and controller changes in one patch unless they are inseparable.

## 6. Documentation Rules

Update documentation whenever behavior changes.

Typical expectations:

- update `README.md` when public usage or current status changes
- update `CHANGELOG.md` for notable engineering changes
- add or update an experience document for tricky debugging or integration work

## 7. Pull Request Checklist

Before submitting, verify:

- tests pass
- documentation matches the current implementation
- paths and filenames in docs are correct
- no temporary debug code is left behind without explanation
- new assets required at runtime are actually packaged and installable

## 8. Questions

If you are unsure whether to preserve a baseline or create a variant, prefer the safer option:

- keep the validated baseline untouched
- add a new world/launch variant
- document the reason clearly
