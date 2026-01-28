# Repository Guidelines

This repository is a ROS 2 workspace containing message definitions, simulation, planning, control, visualization, and launch packages for the Nautomatic stack.

## Project Structure & Module Organization

- `src/` holds ROS 2 packages (Python and CMake-based). Core packages include `na_msg` (msgs/srvs/actions), `na_sim`, `na_planner`, `na_controller`, `na_viz`, `na_utils`, and `na_launch`.
- Package-specific tests live under `src/<package>/test`.
- Launch and configuration assets are in `src/na_launch/launch`, `src/na_launch/config`, `src/na_launch/rviz`, and `src/na_launch/plot_juggler`.
- Build outputs land in `build/`, `install/`, and `log/` after running `colcon`.

## Build, Test, and Development Commands

- `./build_docker.sh`: build the `nautomatica-base` Docker image.
- `./run_docker.sh`: run the container with X11 and mount the repo at `/root/code`.
- `./test_docker.sh`: run `ct` inside the Docker container.
- `./build_overview_docker.sh`: build the LaTeX overview PDF (outputs `latex/output/main.pdf`).
- `colcon build --symlink-install`: build the workspace from the repo root.
- `source install/setup.bash`: load the workspace overlay after building.
- `ros2 launch na_launch controller_launch.py`: run the controller stack.
- `ros2 launch na_launch sim_controller_launch.py`: run the simulation + controller stack.
- `colcon test`: run all package tests (including linters).

## Developer Workflow Notes

- Primary workflow runs inside Docker via `./run_docker.sh`.
- Common aliases in `common_scripts.sh`: `cb` for full `colcon build` + source, `lsc` for `ros2 launch na_launch sim_controller_launch.py`.

## Coding Style & Naming Conventions

- Python code follows PEP 8 with 4-space indentation; docstrings are enforced via PEP 257.
- Linting is handled by `ament_flake8` and `ament_pep257` tests.
- Use `na_*` for ROS 2 package names and snake_case for Python modules.
- Node entrypoints follow the existing `*_node.py` pattern (for example, `sim_node.py`).

## Testing Guidelines

- Tests are `pytest`-based and collected from `src/<package>/test` or `src/<package>/tests`.
- Lint tests include `flake8`, `pep257`, and `xmllint`; keep XML/YAML configs well-formed.
- Prefer `ct` for verification.

## Commit & Pull Request Guidelines

- Git history uses short, sentence-case, imperative summaries (no prefixes).
- Keep commits focused and include the key change in the first line.
- PRs should include a brief description, linked issues if applicable, and testing notes.
- For visualization or config changes, include a screenshot or example output when useful.

## Planning

- Use `.agent_md/Plans.md` to track multi-step work, goals, and decisions.
- Start a new plan when a task spans packages or requires coordination.

## Environment Notes

- `common_scripts.sh` defines helpful aliases for `colcon` builds and ROS 2 launches.
- Clean workspaces with `rm -rf build install log` when you need a full rebuild.

## Sim Sign Conventions

- Simulation yaw sign mismatch likely indicates actuator sign differences; confirm the convention and align simulation to match controller and 3DOF setup.
- Recommended convention: world ENU (+x east, +y north, +z up), body FLU (+x forward, +y left, +z up), yaw positive counterclockwise (left turn) in `[-pi, pi]`, rotor delta positive port (pointing left).
