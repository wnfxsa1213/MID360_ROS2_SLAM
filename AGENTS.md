# Repository Guidelines

## Project Structure & Module Organization
- Core ROS2 workspace: `ws_livox/` with packages `fastlio2`, `localizer`, `pgo`, `hba`, drivers under `livox_ros_driver2`, and custom interfaces in `interface/`.
- Configuration: `config/` (primary YAMLs and backups), launch presets in package folders.
- Tools & scripts: `tools/` (Python utilities, shell runners) and top-level `test_*.sh` helpers.
- Data & outputs: `data/`, `test_data/`, `recorded_data/`, `saved_maps/`, `validation_results/`, logs in `log/`.
- Docs: `docs/` with development guides.

## Build, Test, and Development Commands
- Build workspace: `cd ws_livox && colcon build --symlink-install`
- Source overlay: `source ws_livox/install/setup.bash`
- Run full SLAM (example): `ros2 launch fastlio2 cooperative_slam_system.launch.py`
- Quick tools smoke test: `bash tools/check_slam.sh`
- Validate dynamic filter: `bash test_dynamic_filter.sh` or `bash test_bag_dynamic_filter.sh`
- Convert PCD→gridmap: `python3 tools/pcd_to_gridmap.py --pcd saved_maps/example.pcd`

## Coding Style & Naming Conventions
- C++ (ROS2 nodes): follow ROS2/ament conventions; 2 spaces indent; snake_case for files, CamelCase for types, lower_snake_case for functions/vars.
- Python tools: 4 spaces indent; lowercase_snake_case modules and functions. Prefer `argparse` for CLIs.
- Launch/config: YAML keys lower_snake_case; keep sample configs in `config/` and backups under `config/backups/`.
- Formatting: use `ament_uncrustify`/`ament_clang_format` where available; Python formatted with `black` (line length 88).

## Testing Guidelines
- Scenario scripts live in `tools/` and repo root `test_*.sh`.
- Name tests descriptively: `tools/test_<feature>.py` or `test_<feature>.sh`.
- Run validations: `python3 tools/replay_validation.py` and inspect `validation_results/`.
- Target: keep changes from breaking `tools/check_slam.sh` and core launches; add minimal reproduction scripts for new features.

## Commit & Pull Request Guidelines
- Commits: use imperative mood and scope, e.g., `localizer: fix voxel downsampling overflow`.
- Include: problem, approach, and any config changes; update `docs/` or `config/` examples when relevant.
- PRs must include: clear description, linked issues, repro/validation steps (commands), and before/after metrics or screenshots from RViz when applicable.

## Security & Configuration Tips
- Do not commit large bags or proprietary maps; place them in `data/` locally and add to `.gitignore`.
- Keep credentials and network info out of configs; use env vars or local overrides.
- Back up modified YAML via `config/backups/` and reference the timestamped copy in PRs.
