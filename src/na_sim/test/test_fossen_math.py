import math

import numpy as np

from na_sim.fossen import (
    coriolis_from_mass_matrix,
    normalize_quaternion_wxyz,
    quaternion_derivative_wxyz,
    rigid_body_mass_matrix,
    rotation_matrix_from_quaternion_wxyz,
    skew,
)
from na_sim.Boat3DOF import Boat3DOF


def _yaw_from_quaternion_wxyz(quaternion_wxyz):
    qw, qx, qy, qz = quaternion_wxyz
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def test_skew_is_skew_symmetric():
    s = skew([1.0, -2.0, 3.5])
    np.testing.assert_allclose(s + s.T, 0.0, atol=1e-12)


def test_rigid_body_mass_matrix_is_symmetric():
    m_rb = rigid_body_mass_matrix(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        r_g_m=[0.1, -0.2, 0.05],
    )
    np.testing.assert_allclose(m_rb, m_rb.T, atol=1e-12)


def test_coriolis_from_mass_matrix_is_skew_symmetric():
    m_rb = rigid_body_mass_matrix(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        r_g_m=[0.0, 0.0, 0.0],
    )
    nu = np.array([1.0, -0.2, 0.3, 0.1, -0.4, 0.8], dtype=float)
    c_rb = coriolis_from_mass_matrix(m_rb, nu)
    np.testing.assert_allclose(c_rb + c_rb.T, 0.0, atol=1e-12)


def test_coriolis_power_is_zero():
    m_rb = rigid_body_mass_matrix(
        mass_kg=20.0,
        inertia_cg_kgm2=np.diag([4.0, 5.0, 6.0]),
    )
    nu = np.array([0.5, -0.7, 0.2, 0.1, 0.3, -0.4], dtype=float)
    c_rb = coriolis_from_mass_matrix(m_rb, nu)
    power = float(nu.T @ (c_rb @ nu))
    assert abs(power) < 1e-12


def test_rotation_matrix_from_quaternion_is_orthonormal():
    q = normalize_quaternion_wxyz([0.9, 0.1, -0.3, 0.2])
    r = rotation_matrix_from_quaternion_wxyz(q)
    np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert abs(float(np.linalg.det(r)) - 1.0) < 1e-12


def test_quaternion_kinematics_matches_constant_yaw_rate():
    dt = 0.001
    steps = 2000
    r_yaw = 0.3
    omega = np.array([0.0, 0.0, r_yaw], dtype=float)

    q = normalize_quaternion_wxyz([1.0, 0.0, 0.0, 0.0])
    for _ in range(steps):
        q = q + dt * quaternion_derivative_wxyz(q, omega)
        q = normalize_quaternion_wxyz(q)

    yaw = _yaw_from_quaternion_wxyz(q)
    expected = r_yaw * steps * dt
    assert abs(yaw - expected) < 5e-4


def test_planar_matrix_form_matches_boat3dof_dynamics():
    params = {
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
    boat = Boat3DOF(params)

    state = np.array([1.2, -0.7, 0.3, 2.0, -0.4, 0.25], dtype=float)
    thrust = 15.0
    delta = -0.2

    dx_ref = boat.dynamics(state, thrust, delta)

    x, y, psi, u, v, r = state
    dx = u * math.cos(psi) + v * math.sin(psi)
    dy = -u * math.sin(psi) + v * math.cos(psi)
    dpsi = r

    x_force = thrust * math.cos(delta)
    y_force = thrust * math.sin(delta)
    n_moment = -params["l"] * y_force

    m = params["m"]
    iz = params["Iz"]
    d_vec = np.array(
        [
            params["Xu"] * u + params["Xuu"] * abs(u) * u,
            params["Yv"] * v + params["Yr"] * r,
            params["Nv"] * v + params["Nr"] * r,
        ],
        dtype=float,
    )
    c_nu = np.array([-m * v * r, m * u * r, 0.0], dtype=float)
    tau = np.array([x_force, y_force, n_moment], dtype=float)
    nu_dot = np.array([1.0 / m, 1.0 / m, 1.0 / iz], dtype=float) * (tau - c_nu - d_vec)
    dx_mat = np.array([dx, dy, dpsi, nu_dot[0], nu_dot[1], nu_dot[2]], dtype=float)

    np.testing.assert_allclose(dx_mat, dx_ref, atol=1e-12)
