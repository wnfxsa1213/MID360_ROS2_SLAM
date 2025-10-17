# Repository Guidelines

## Project Structure & Module Organization
- `ws_livox/` holds the ROS2 workspace with core packages: SLAM nodes under `fastlio2`, localization in `localizer`, optimization in `pgo`, plus `hba`, drivers (`livox_ros_driver2`), and shared `interface/`.
- Configuration YAML lives in `config/`; timestamped backups sit in `config/backups/` for rollbacks.
- Utility scripts are under `tools/`, while repo-level smoke tests start with `test_*.sh`.
- Generated artifacts and datasets stay outside the code path in `data/`, `test_data/`, `recorded_data/`, `saved_maps/`, and logs in `log/`.

## Build, Test, and Development Commands
- `cd ws_livox && colcon build --symlink-install` — build the entire workspace with symlinked installs for faster iteration.
- `source ws_livox/install/setup.bash` — load the overlay before launching any ROS2 nodes.
- `ros2 launch fastlio2 cooperative_slam_system.launch.py` — run the end-to-end SLAM stack.
- `bash tools/check_slam.sh` — quick integration smoke test for tooling sanity.
- `python3 tools/pcd_to_gridmap.py --pcd saved_maps/example.pcd` — convert a saved PCD map to a gridmap for validation.

## Coding Style & Naming Conventions
- C++ nodes follow ROS2 ament defaults: 2-space indent, snake_case files, CamelCase types, lower_snake_case functions and variables.
- Python utilities use 4-space indent, lowercase module names, and prefer `argparse` for CLIs; format with `black --line-length 88`.
- Launch/config YAML keep lower_snake_case keys; mirror sample configs into `config/backups/` when adjusted.

## Testing Guidelines
- Prefer scenario runners in `tools/` and repo-level `test_*.sh` scripts; add new scripts as `tools/test_<feature>.py` to stay consistent.
- Validate SLAM pipelines with `bash tools/check_slam.sh` and `python3 tools/replay_validation.py`; inspect outputs under `validation_results/`.
- Keep regression coverage by replaying relevant bags in `test_data/` whenever sensor models change.

## Commit & Pull Request Guidelines
- Write commits in imperative mood scoped by package, e.g., `localizer: fix voxel downsampling overflow`; group config updates with matching backups.
- PRs need problem statements, approach summaries, linked issues, and explicit validation commands; attach RViz captures or metrics for SLAM-focused changes.
- Document config edits in `docs/` when behavior shifts, and never push large bags or credentials.

## Security & Configuration Tips
- Store sensitive maps and recordings locally under ignored directories; verify `.gitignore` before adding data.
- Back up any modified YAML to `config/backups/<timestamp>.yaml` and reference it in review notes for traceability.
