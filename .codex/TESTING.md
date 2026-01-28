# Testing

## Conventions and frames
The codebase follows ENU/FLU conventions throughout:

- World frame: +x east, +y north, +z up (ENU).
- Body frame: +x forward, +y left, +z up (FLU).
- Yaw: positive counterclockwise (left turn).
- B-spline Frenet frame uses the left normal `(-ty, tx)` so `cte > 0` means the
  vessel is to port/left of the path tangent.
- Path yaw from a tangent uses `psi_path = atan2(t_y, t_x)`.

When changing sign conventions or frame mappings, update the simulator,
controller, utilities, and tests together.

## Regression tests for sign conventions
These tests guard against "turns the wrong way" regressions:

- `src/sj_utils/tests/test_bspline.py`:
  validates the B-spline Frenet frame and that `cte` matches the left-normal
  convention on a circular path.
- `src/sj_utils/tests/test_enu_projection.py`:
  checks that ProjectionTracker kinematic prediction uses ENU conventions.
- `src/sj_controller/test/test_los_enu.py`:
  validates LOS heading math and ENU yaw conversions.
- `src/sj_sim/test/test_enu_conventions.py`:
  validates ENU kinematics for 3DOF and quaternion yaw rotations.
- `src/sj_launch/test/test_path_following_3dof_launch.py`:
  launches the 3DOF sim + controller and asserts that the yaw-rate sign matches
  the local path direction inferred from the spline tangent.
- `src/sj_launch/test/test_path_following_6dof_launch.py`:
  same as above for the 6DOF sim.

These tests are intended to catch mismatches between path geometry and the
controller/simulator under ENU/FLU conventions.

## Running tests
Use the Docker harness to build and test everything:

```bash
./test_docker.sh
```

This runs the LaTeX build and all package tests (including the launch tests).
