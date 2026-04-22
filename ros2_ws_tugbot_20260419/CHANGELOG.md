# Changelog

All notable changes to this workspace will be documented in this file.

The format is inspired by Keep a Changelog and this project follows a date-driven engineering iteration style rather than strict semantic version tags.

## [2026-04-22]

### Added
- Restored tugbot visual detail meshes for beacon, top lidar, rear gripper, and logo-related model visuals.
- Added regression coverage for restored model visual tokens and required mesh assets.
- Added open-source repository support files: `LICENSE`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, and `.gitignore`.
- Added an experience document for the tugbot model visual detail restoration workflow under `../doc/doc_experience/`.

### Changed
- Updated `README.md` to reflect the current five-package architecture, launch/world layout, search recovery status, testing status, and model restoration boundary.
- Clarified that the tugbot model restoration is visual-only and does not reintroduce legacy sensor chains.

### Verified
- `python3 -m pytest tests -q` -> `30 passed`
- `colcon build --symlink-install --packages-select tugbot_description`
- Live Gazebo world expansion confirmed restored mesh URIs are present in runtime SDF resolution.
- Manual Gazebo inspection confirmed the tugbot visual details are visible again.

## [2026-04-21]

### Added
- Added `full_system_zeroerr_outer.launch.py` as a dedicated zeroerr outer-loop bringup wrapper.
- Added search recovery support and temporal debugging utilities for lane-following experiments.

### Changed
- Refined zeroerr outer-loop world usage into a dedicated validation flow instead of relying on long `world_sdf` overrides.
