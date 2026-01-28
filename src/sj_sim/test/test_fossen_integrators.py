import math

import numpy as np

from sj_sim.fossen import (
    integrate_quaternion_euler,
    integrate_quaternion_expmap,
    integrate_quaternion_rk4,
    integrate_rotation_matrix_euler,
    integrate_rotation_matrix_expm,
    normalize_quaternion_wxyz,
)


def _yaw_from_quaternion_wxyz(quaternion_wxyz):
    qw, qx, qy, qz = quaternion_wxyz
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _wrap_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def test_quaternion_integrators_accuracy_order():
    dt = 0.05
    steps = 200
    omega = np.array([0.0, 0.0, 1.2], dtype=float)

    q_euler = normalize_quaternion_wxyz([1.0, 0.0, 0.0, 0.0])
    q_rk4 = q_euler.copy()
    q_exp = q_euler.copy()

    for _ in range(steps):
        q_euler = integrate_quaternion_euler(q_euler, omega, dt, normalize=True)
        q_rk4 = integrate_quaternion_rk4(q_rk4, omega, dt)
        q_exp = integrate_quaternion_expmap(q_exp, omega, dt)

    expected = omega[2] * dt * steps
    err_euler = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_euler) - expected))
    err_rk4 = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_rk4) - expected))
    err_exp = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_exp) - expected))

    assert err_exp <= err_rk4 + 1e-10
    assert err_rk4 < err_euler


def test_rotation_matrix_expm_preserves_orthonormality():
    dt = 0.02
    steps = 500
    omega = np.array([0.2, -0.1, 0.3], dtype=float)

    r_euler = np.eye(3, dtype=float)
    r_expm = np.eye(3, dtype=float)
    for _ in range(steps):
        r_euler = integrate_rotation_matrix_euler(r_euler, omega, dt)
        r_expm = integrate_rotation_matrix_expm(r_expm, omega, dt)

    euler_err = np.linalg.norm(r_euler.T @ r_euler - np.eye(3))
    expm_err = np.linalg.norm(r_expm.T @ r_expm - np.eye(3))

    assert expm_err < 1e-10
    assert expm_err < euler_err
