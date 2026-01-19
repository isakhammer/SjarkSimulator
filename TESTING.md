# Testing

## Conventions and frames
The codebase mixes two layers of conventions:

- `na_utils` geometry (B-splines, projection, Frenet frame) uses an abstract
  math frame where headings are counterclockwise from +x and the unit normal is
  the left normal `(-ty, tx)`.
- The simulator and controller use Fossen NED/FRD conventions: world +z down,
  body +y starboard, yaw positive clockwise. The controller converts the spline
  tangent into a NED yaw with `psi_path = -atan2(t_y, t_x)`.

When changing sign conventions or frame mappings, update both the math utilities
and the controller/simulator together, and adjust the tests below.

## Regression tests for sign conventions
These tests guard against "turns the wrong way" regressions:

- `src/na_utils/tests/test_bspline.py`:
  validates the B-spline Frenet frame and that `cte` matches the left-normal
  convention on a circular path.
- `src/na_launch/test/test_path_following_3dof_launch.py`:
  launches the 3DOF sim + controller and asserts that the yaw-rate sign matches
  the local path direction inferred from the spline tangent.
- `src/na_launch/test/test_path_following_6dof_launch.py`:
  same as above for the 6DOF sim.

These tests are intended to catch mismatches between spline geometry (math-frame)
and the controller/simulator (NED/FRD).

## Running tests
Use the Docker harness to build and test everything:

```bash
./test_docker.sh
```

This runs the LaTeX build and all package tests (including the launch tests).
