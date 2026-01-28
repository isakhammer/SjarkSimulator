#!/usr/bin/env python3
import time

import numpy as np

from sj_sim.fossen import (
    Fossen6DOF,
    integrate_quaternion_euler,
    integrate_quaternion_expmap,
    integrate_quaternion_rk4,
    integrate_quaternion_scipy_expmap,
    integrate_rotation_matrix_euler,
    integrate_rotation_matrix_expm,
    normalize_quaternion_wxyz,
)


def _yaw_from_quaternion_wxyz(quaternion_wxyz):
    qw, qx, qy, qz = quaternion_wxyz
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _wrap_angle(angle_rad):
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def _time_fn(label, fn, steps):
    start = time.perf_counter()
    for _ in range(steps):
        fn()
    end = time.perf_counter()
    return label, (end - start)


def main():
    dt = 0.01
    steps = 20000
    omega = np.array([0.2, -0.1, 0.3], dtype=float)
    omega_yaw = np.array([0.0, 0.0, 1.2], dtype=float)

    q0 = normalize_quaternion_wxyz([1.0, 0.0, 0.0, 0.0])

    def run_euler():
        q = q0.copy()
        for _ in range(steps):
            q = integrate_quaternion_euler(q, omega_yaw, dt, normalize=True)
        return q

    def run_rk4():
        q = q0.copy()
        for _ in range(steps):
            q = integrate_quaternion_rk4(q, omega_yaw, dt)
        return q

    def run_expmap():
        q = q0.copy()
        for _ in range(steps):
            q = integrate_quaternion_expmap(q, omega_yaw, dt)
        return q

    def run_scipy():
        q = q0.copy()
        for _ in range(steps):
            q = integrate_quaternion_scipy_expmap(q, omega_yaw, dt)
        return q

    results = []
    results.append(_time_fn("euler", run_euler, 1))
    results.append(_time_fn("rk4", run_rk4, 1))
    results.append(_time_fn("expmap", run_expmap, 1))
    try:
        results.append(_time_fn("scipy_expmap", run_scipy, 1))
    except RuntimeError:
        pass

    expected_yaw = omega_yaw[2] * dt * steps
    q_euler = run_euler()
    q_rk4 = run_rk4()
    q_exp = run_expmap()

    err_euler = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_euler) - expected_yaw))
    err_rk4 = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_rk4) - expected_yaw))
    err_exp = abs(_wrap_angle(_yaw_from_quaternion_wxyz(q_exp) - expected_yaw))

    r_euler = np.eye(3, dtype=float)
    r_expm = np.eye(3, dtype=float)
    for _ in range(steps):
        r_euler = integrate_rotation_matrix_euler(r_euler, omega, dt)
        r_expm = integrate_rotation_matrix_expm(r_expm, omega, dt)
    ortho_euler = np.linalg.norm(r_euler.T @ r_euler - np.eye(3))
    ortho_expm = np.linalg.norm(r_expm.T @ r_expm - np.eye(3))

    print("Integrator timing (s):")
    for label, elapsed in results:
        print(f"  {label:12s}: {elapsed:.3f}")
    print("Yaw error (rad):")
    print(f"  euler:  {err_euler:.6f}")
    print(f"  rk4:    {err_rk4:.6f}")
    print(f"  expmap: {err_exp:.6f}")
    print("SO(3) orthonormality error:")
    print(f"  R_euler: {ortho_euler:.6e}")
    print(f"  R_expm:  {ortho_expm:.6e}")

    model = Fossen6DOF(
        mass_kg=10.0,
        inertia_cg_kgm2=np.diag([1.0, 2.0, 3.0]),
        linear_damping=np.array([5.0, 6.0, 7.0, 0.2, 0.3, 0.4]),
    )
    state = np.zeros(13, dtype=float)
    state[3] = 1.0
    state[7:13] = np.array([1.2, -0.8, 0.4, 0.2, -0.1, 0.3], dtype=float)
    tau = np.zeros(6, dtype=float)
    for method in ("euler", "rk4", "expmap"):
        s = state.copy()
        energy = []
        for _ in range(2000):
            nu = s[7:]
            energy.append(0.5 * float(nu.T @ (model.m @ nu)))
            s = model.step(s, tau, dt, method=method)
        print(f"Energy decay ({method}): {energy[0]:.6f} -> {energy[-1]:.6f}")


if __name__ == "__main__":
    main()
