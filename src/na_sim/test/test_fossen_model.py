import numpy as np

from na_sim.fossen import (
    Fossen6DOF,
    coriolis_from_mass_matrix,
    rigid_body_mass_matrix,
    restoring_forces_body,
)
from na_sim.Boat3DOF import Boat3DOF


def test_fossen6dof_zero_state_zero_input():
    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
    )
    state = np.zeros(13, dtype=float)
    state[3] = 1.0
    tau = np.zeros(6, dtype=float)

    dx = model.state_derivative(state, tau)
    np.testing.assert_allclose(dx, 0.0, atol=1e-12)


def test_fossen6dof_forward_kinematics_identity_orientation():
    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
    )
    state = np.zeros(13, dtype=float)
    state[3] = 1.0
    state[7] = 1.5
    tau = np.zeros(6, dtype=float)

    dx = model.state_derivative(state, tau)
    np.testing.assert_allclose(dx[:3], [1.5, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(dx[3:7], 0.0, atol=1e-12)


def test_fossen6dof_surge_acceleration():
    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        linear_damping=np.zeros(6, dtype=float),
        quadratic_damping=np.zeros(6, dtype=float),
    )
    nu = np.zeros(6, dtype=float)
    tau = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    nu_dot = model.dynamics(nu, tau)
    np.testing.assert_allclose(nu_dot, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_restoring_forces_neutral_buoyancy():
    r = np.eye(3, dtype=float)
    g_vec = restoring_forces_body(
        rotation_body_to_world=r,
        weight_n=100.0,
        buoyancy_n=100.0,
    )
    np.testing.assert_allclose(g_vec, 0.0, atol=1e-12)


def test_restoring_forces_offsets_and_buoyancy():
    r = np.eye(3, dtype=float)
    g_vec = restoring_forces_body(
        rotation_body_to_world=r,
        weight_n=100.0,
        buoyancy_n=120.0,
        r_g_m=[0.2, 0.0, 0.0],
        r_b_m=[0.0, 0.0, 0.0],
    )
    np.testing.assert_allclose(g_vec[:2], 0.0, atol=1e-12)
    assert g_vec[2] == 20.0
    np.testing.assert_allclose(g_vec[3], 0.0, atol=1e-12)
    assert g_vec[4] == 20.0
    np.testing.assert_allclose(g_vec[5], 0.0, atol=1e-12)


def test_restoring_forces_lateral_offsets_create_roll_moment():
    r = np.eye(3, dtype=float)
    g_vec = restoring_forces_body(
        rotation_body_to_world=r,
        weight_n=100.0,
        buoyancy_n=100.0,
        r_g_m=[0.0, 0.2, 0.0],
        r_b_m=[0.0, -0.2, 0.0],
    )
    assert g_vec[0] == 0.0
    assert g_vec[1] == 0.0
    assert g_vec[2] == 0.0
    assert g_vec[3] < 0.0
    np.testing.assert_allclose(g_vec[4:], [0.0, 0.0], atol=1e-12)


def test_current_damping_opposes_relative_velocity():
    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        linear_damping=np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    )
    nu = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    nu_c = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    tau = np.zeros(6, dtype=float)

    nu_dot = model.dynamics(nu, tau, nu_c=nu_c)
    assert nu_dot[0] < 0.0
    np.testing.assert_allclose(nu_dot[1:], 0.0, atol=1e-12)


def test_damping_stability_energy_decreases():
    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        linear_damping=np.array([5.0, 6.0, 7.0, 0.2, 0.3, 0.4]),
        quadratic_damping=np.zeros(6, dtype=float),
    )
    nu = np.array([1.5, -0.8, 0.4, 0.2, -0.1, 0.3], dtype=float)
    tau = np.zeros(6, dtype=float)

    energy = []
    dt = 0.01
    for _ in range(500):
        energy.append(0.5 * float(nu.T @ (model.m @ nu)))
        nu_dot = model.dynamics(nu, tau)
        nu = nu + dt * nu_dot

    assert energy[-1] < energy[0]


def test_mass_matrix_positive_definite():
    model = Fossen6DOF(
        mass_kg=12.0,
        inertia_cg_kgm2=np.diag([2.0, 3.0, 4.0]),
    )
    eigenvalues = np.linalg.eigvals(model.m)
    assert np.all(np.real(eigenvalues) > 0.0)


def test_coriolis_power_is_zero_with_offset_cg():
    m_rb = rigid_body_mass_matrix(
        mass_kg=15.0,
        inertia_cg_kgm2=np.diag([1.5, 2.5, 3.5]),
        r_g_m=[0.2, -0.1, 0.05],
    )
    nu = np.array([0.8, -0.6, 0.2, 0.1, -0.2, 0.4], dtype=float)
    c_rb = coriolis_from_mass_matrix(m_rb, nu)
    power = float(nu.T @ (c_rb @ nu))
    assert abs(power) < 1e-12


def test_added_mass_coriolis_is_skew_symmetric():
    m_a = np.diag([3.0, 4.0, 5.0, 0.2, 0.3, 0.4])
    nu_r = np.array([0.4, -0.3, 0.2, 0.05, -0.02, 0.1], dtype=float)
    c_a = coriolis_from_mass_matrix(m_a, nu_r)
    np.testing.assert_allclose(c_a + c_a.T, 0.0, atol=1e-12)


def test_fossen6dof_matches_boat3dof_planar_dynamics():
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

    state = np.array([1.2, -0.7, 0.3, 2.0, -0.4, 0.25], dtype=float)
    thrust = 15.0
    delta = -0.2
    dx_ref = boat.dynamics(state, thrust, delta)

    u, v, r = state[3:]
    nu = np.array([u, v, 0.0, 0.0, 0.0, r], dtype=float)
    tau = np.array(
        [
            thrust * np.cos(delta),
            thrust * np.sin(delta),
            0.0,
            0.0,
            0.0,
            -params["l"] * thrust * np.sin(delta),
        ],
        dtype=float,
    )
    nu_dot = model.dynamics(nu, tau)
    np.testing.assert_allclose(nu_dot[0], dx_ref[3], atol=1e-12)
    np.testing.assert_allclose(nu_dot[1], dx_ref[4], atol=1e-12)
    np.testing.assert_allclose(nu_dot[5], dx_ref[5], atol=1e-12)
