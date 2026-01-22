# Testing

## Conventions and frames
The codebase follows Fossen NED/FRD conventions throughout:

- World frame: +z down (NED).
- Body frame: +x forward, +y starboard, +z down (FRD).
- Yaw: positive clockwise (right turn).
- B-spline Frenet frame uses the right normal `(-ty, tx)` so `cte > 0` means the
  vessel is to starboard of the path tangent.
- Path yaw from a tangent uses `psi_path = atan2(t_y, t_x)`.

When changing sign conventions or frame mappings, update the simulator,
controller, utilities, and tests together.

## Regression tests for sign conventions
These tests guard against "turns the wrong way" regressions:

- `src/na_utils/tests/test_bspline.py`:
  validates the B-spline Frenet frame and that `cte` matches the right-normal
  convention on a circular path.
- `src/na_utils/tests/test_ned_projection.py`:
  checks that ProjectionTracker kinematic prediction uses NED conventions.
- `src/na_controller/test/test_los_ned.py`:
  validates LOS heading math and NED yaw conversions.
- `src/na_sim/test/test_ned_conventions.py`:
  validates NED kinematics for 3DOF and quaternion yaw rotations.
- `src/na_launch/test/test_path_following_3dof_launch.py`:
  launches the 3DOF sim + controller and asserts that the yaw-rate sign matches
  the local path direction inferred from the spline tangent.
- `src/na_launch/test/test_path_following_6dof_launch.py`:
  same as above for the 6DOF sim.

These tests are intended to catch mismatches between path geometry and the
controller/simulator under NED/FRD conventions.

## Running tests
Use the Docker harness to build and test everything:

```bash
./test_docker.sh
```

This runs the LaTeX build and all package tests (including the launch tests).
