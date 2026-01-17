import numpy as np

from na_sim.fossen import Fossen6DOF, restoring_forces_body


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
