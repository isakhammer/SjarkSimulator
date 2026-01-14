import math

import pytest

from na_sim.Boat3DOF import Boat3DOF


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
