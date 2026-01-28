# Project Guidelines

These guidelines keep the SjarkSimulator ROS 2 workspace consistent, testable, and easy to evolve.

## Workspace Expectations

- Work from the repo root and build with `colcon build --symlink-install`.
- Source the overlay before running nodes: `source install/setup.bash`.
- Do not commit generated artifacts in `build/`, `install/`, or `log/`.
- Run and test the system inside the Docker container defined by the root `Dockerfile`.
- Build LaTeX documentation in a separate Docker container via `./build_overview_docker.sh`.

## Package Boundaries

- Keep shared utilities in `src/sj_utils`; avoid duplicating helpers across packages.
- Put ROS interfaces only in `src/sj_msg` (msgs/srvs/actions).
- Launch and config assets belong in `src/sj_launch/launch` and `src/sj_launch/config`.
- Put reusable computational procedures (splines, geometry, matrix ops, ODE integrators) in `src/sj_utils` and cover them with tests.

## ROS Node Organization

- Keep ROS-specific logic in `*_node.py` (message parsing, callbacks, timers, publishers/subscribers).
- Move domain logic into non-node modules (for example, `control.py`, `planner.py`, `simulation.py`) that handle math, projections, and controller selection.
- Apply this split consistently across control, simulation, planner, and state estimation packages.
- The only visualization node should live in `sj_viz` and act as a listener-only client (no control or pipeline dependencies).
- Core logic must be ROS-agnostic; only node files and visualization code should import or depend on ROS APIs.

## Modeling & Control Changes

- Keep units in SI and document frames and sign conventions for dynamics.
- When changing dynamics, update related control assumptions and tests.
- Prefer parameters in YAML (for example, `src/sj_launch/config/sim_controller_params.yaml`).

## Testing & Linting

- Run `ct` (from `common_scripts.sh`) after implementing features to execute `colcon test` and print results.
- Maintain linter coverage (`flake8`, `pep257`, `xmllint`) when adding files.
- Docker test runs (`test_docker.sh` or `testd`) write CSV logs to `log/sj_test_results` by default; override with `SJ_TEST_LOG_DIR`.

## Planning & Documentation

- Use `.agent_md/Plans.md` to track multi-step changes and decisions.
- Update `.agent_md/AGENTS.md` if workflows or commands change.
- Only keep a `README.md` at the repo root; prefer inline documentation in
  each file over package-level READMEs.
- Keep the LaTeX overview as a single file in `latex/main.tex`; update
  inline TikZ blocks when regenerating figures.

## Collaboration Expectations

- When tasks are vague or risky, pause and ask clarifying questions before making changes.
- Do not create commits without explicit user approval.
- Offer alternatives and call out potential issues when a request looks unsafe or suboptimal.
