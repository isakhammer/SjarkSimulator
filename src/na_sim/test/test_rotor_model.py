import math

import numpy as np
import pytest

from na_sim.Boat3DOF import Boat3DOF
from na_sim.fossen import Fossen6DOF


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


def _default_fossen_model():
    params = _default_params()
    d_linear = np.zeros((6, 6), dtype=float)
    d_linear[0, 0] = params["Xu"]
    d_linear[1, 1] = params["Yv"]
    d_linear[1, 5] = params["Yr"]
    d_linear[5, 1] = params["Nv"]
    d_linear[5, 5] = params["Nr"]
    d_quad = np.zeros((6, 6), dtype=float)
    d_quad[0, 0] = params["Xuu"]

    model = Fossen6DOF(
        mass_kg=params["m"],
        inertia_cg_kgm2=np.diag([1.0, 1.0, params["Iz"]]),
        linear_damping=d_linear,
        quadratic_damping=d_quad,
    )
    return model, params


def _simulate_fossen_yaw_rate(model, params, thrust, delta, dt, steps):
    state = np.zeros(13, dtype=float)
    state[3] = 1.0
    for _ in range(steps):
        x_force = thrust * math.cos(delta)
        y_force = thrust * math.sin(delta)
        tau = np.array(
            [x_force, y_force, 0.0, 0.0, 0.0, -params["l"] * y_force],
            dtype=float,
        )
        state = model.step(state, tau, dt, method="rk4")
    u = state[7]
    r = state[12]
    return u, r


@pytest.mark.parametrize(
    ("delta", "expected_sign"),
    [
        (0.0, 0),
        (-0.3, 1),
        (0.3, -1),
    ],
)
def test_stern_rotor_yaw_sign(delta, expected_sign):
    boat = Boat3DOF(_default_params())
    dt = 0.02
    thrust = 20.0

    for _ in range(250):
        boat.step(thrust, delta, dt)

    _, _, _, u, _, r = boat.state
    assert u > 0.0
    if expected_sign == 0:
        assert abs(r) < 0.2
    elif expected_sign > 0:
        assert r > 0.0
    else:
        assert r < 0.0


@pytest.mark.parametrize(
    ("delta", "expected_sign"),
    [
        (0.0, 0),
        (-0.3, 1),
        (0.3, -1),
    ],
)
def test_fossen6dof_rotor_yaw_sign(delta, expected_sign):
    model, params = _default_fossen_model()
    dt = 0.02
    thrust = 20.0

    u, r = _simulate_fossen_yaw_rate(model, params, thrust, delta, dt, steps=250)
    assert u > 0.0
    if expected_sign == 0:
        assert abs(r) < 0.2
    elif expected_sign > 0:
        assert r > 0.0
    else:
        assert r < 0.0


def test_rotor_yaw_symmetry_3dof():
    boat = Boat3DOF(_default_params())
    dt = 0.02
    thrust = 20.0

    for _ in range(250):
        boat.step(thrust, -0.3, dt)
    _, _, _, _, _, left_r = boat.state

    boat = Boat3DOF(_default_params())
    for _ in range(250):
        boat.step(thrust, 0.3, dt)
    _, _, _, _, _, right_r = boat.state

    assert left_r > 0.0
    assert right_r < 0.0
    max_abs = max(abs(left_r), abs(right_r))
    assert max_abs > 0.01
    assert abs(abs(left_r) - abs(right_r)) / max_abs < 0.2


def test_rotor_yaw_symmetry_6dof():
    model, params = _default_fossen_model()
    dt = 0.02
    thrust = 20.0

    _, left_r = _simulate_fossen_yaw_rate(
        model, params, thrust, -0.3, dt, steps=250
    )
    model, params = _default_fossen_model()
    _, right_r = _simulate_fossen_yaw_rate(
        model, params, thrust, 0.3, dt, steps=250
    )

    assert left_r > 0.0
    assert right_r < 0.0
    max_abs = max(abs(left_r), abs(right_r))
    assert max_abs > 0.01
    assert abs(abs(left_r) - abs(right_r)) / max_abs < 0.2


def test_rotor_angle_changes_lateral_velocity():
    boat = Boat3DOF(_default_params())
    dt = 0.02
    thrust = 20.0
    delta = 0.3

    for _ in range(250):
        boat.step(thrust, delta, dt)

    _, _, _, _, v, r = boat.state
    assert math.isfinite(v)
    assert abs(v) > 0.01
    assert math.isfinite(r)
