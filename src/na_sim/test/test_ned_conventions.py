import math

import numpy as np
import pytest

from na_sim.Boat3DOF import Boat3DOF
from na_sim.fossen import rotation_matrix_from_quaternion_wxyz


def _default_params():
    return {
        "m": 70.0,
        "Iz": 10.0,
        "Xu": 5.0,
        "Xuu": 1.0,
        "Yv": 40.0,
        "Yr": 5.0,
        "Nv": 5.0,
        "Nr": 40.0,
        "l": 0.5,
    }


@pytest.mark.parametrize(
    ("psi", "u", "v", "expected_dx", "expected_dy"),
    [
        (0.0, 1.0, 0.5, 1.0, 0.5),
        (math.pi / 2.0, 1.0, 0.0, 0.0, 1.0),
        (-math.pi / 2.0, 1.0, 0.0, 0.0, -1.0),
        (math.pi, 1.0, 0.0, -1.0, 0.0),
        (math.pi / 2.0, 0.0, 1.0, -1.0, 0.0),
    ],
)
def test_boat3dof_kinematics_ned(psi, u, v, expected_dx, expected_dy):
    boat = Boat3DOF(_default_params())
    state = np.array([0.0, 0.0, psi, u, v, 0.0], dtype=float)
    dx = boat.dynamics(state, thrust=0.0, delta=0.0)

    assert math.isclose(dx[0], expected_dx, abs_tol=1e-6)
    assert math.isclose(dx[1], expected_dy, abs_tol=1e-6)


@pytest.mark.parametrize("psi", [0.0, math.pi / 2.0, -math.pi / 2.0])
def test_rotation_matrix_ned_yaw(psi):
    q = np.array(
        [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)],
        dtype=float,
    )
    r = rotation_matrix_from_quaternion_wxyz(q)
    forward_world = r @ np.array([1.0, 0.0, 0.0], dtype=float)
    expected = np.array([math.cos(psi), math.sin(psi), 0.0], dtype=float)
    np.testing.assert_allclose(forward_world, expected, atol=1e-6)
