# Repository Guidelines

## Project Structure & Module Organization
The ROS2 overlay lives in `ws_livox/`, where SLAM logic is grouped under `fastlio2`, localization under `localizer`, and backend optimization in `pgo`. Shared interfaces and drivers (including `livox_ros_driver2`) stay alongside `hba` utilities. Runtime configuration files sit in `config/`, with every change mirrored to `config/backups/` for rollbacks. Long-running assets such as datasets, bag files, and generated maps belong in `data/`, `test_data/`, `recorded_data/`, `saved_maps/`, while logs collect in `log/` and `validation_results/`. Repo-level helper scripts and smoke tests live in `tools/`.

## Build, Test, and Development Commands
Run `cd ws_livox && colcon build --symlink-install` to build the full workspace with editable installs. Source `ws_livox/install/setup.bash` before launching anything so ROS2 can resolve packages. Execute `ros2 launch fastlio2 cooperative_slam_system.launch.py` for the end-to-end SLAM stack. Use `bash tools/check_slam.sh` for a quick integration check, and `python3 tools/pcd_to_gridmap.py --pcd saved_maps/example.pcd` when converting LiDAR maps into grid maps for downstream validation.

## Coding Style & Naming Conventions
C++ nodes follow ament defaults: 2-space indentation, snake_case filenames, CamelCase classes, and lower_snake_case members and functions. Python utilities keep 4-space indentation, lowercase module names, and rely on `argparse`; format them with `black --line-length 88`. Launch and configuration YAML files prefer lower_snake_case keys, and any tweaks must be copied to `config/backups/` before review.

## Testing Guidelines
Scenario runners and smoke tests start with `test_*.sh` at the repository root; add new checks as `tools/test_<feature>.py`. Validate primary SLAM flows via `bash tools/check_slam.sh`, inspect `validation_results/`, and replay representative bags from `test_data/` after sensor-model changes. Failed runs should capture logs under `log/` for triage.

## Commit & Pull Request Guidelines
Write commits in the format `package: imperative action`, e.g., `localizer: fix voxel downsampling overflow`, and bundle config edits with their timestamped backups. Pull requests must link issues, describe the problem and approach, list validation commands, and attach metrics or RViz screenshots for SLAM-impacting changes. Never include large datasets or credentials; instead, document their storage location in `docs/`.

## Security & Configuration Tips
Keep sensitive maps and recordings outside version control under ignored directories, and confirm `.gitignore` catches new artifacts. Before merging, verify configuration diffs reference their backup copies and note any environmental requirements in `docs/` for future operators.
