"""
Fossen-style marine craft math helpers.

This module provides small, testable building blocks for 6-DOF marine craft
simulation using ROS-style ENU/FLU conventions:

- World frame {W}: right-handed, +x east, +y north, +z up (ENU).
- Body frame {B}: right-handed, +x forward, +y port/left, +z up (FLU).

The functions are written to support:
  M * nu_dot + C(nu) * nu + d(nu_r) + g(eta) = tau
and quaternion-based attitude kinematics.

State ordering (used by Fossen6DOF):
  [x, y, z, qw, qx, qy, qz, u, v, w, p, q, r]

Generalized velocity and force ordering:
  nu  = [u, v, w, p, q, r]
  tau = [X, Y, Z, K, M, N]

Yaw/pitch/roll notes (ENU/FLU, ZYX convention):
  - yaw ψ: rotation about world +z (up), positive CCW when viewed from above.
  - pitch θ: rotation about intermediate +y (left) after yaw.
  - roll ϕ: rotation about body +x (forward) after yaw+pitch.
  - Rotation from body to world is R = Rz(ψ) * Ry(θ) * Rx(ϕ).
  - The quaternion in this module is q_WB = [w, x, y, z] and represents the
    same body-to-world rotation. Use it directly in the ODEs for kinematics:
      p_dot = R(q_WB) * v_body
      q_dot = 0.5 * Omega(omega_body) * q_WB
  - The state stores quaternions only; there is no automatic Euler conversion
    during integration. Convert yaw/pitch/roll to q_WB once before integration
    (e.g., for initialization), and back to yaw/pitch/roll after integration
    if you need human-readable angles for logging or UI.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np

try:
    from scipy.spatial.transform import Rotation as _SciPyRotation
except Exception:
    _SciPyRotation = None

GRAVITY_MPS2 = 9.81


def _as_vec(value: Iterable[float], size: int, name: str) -> np.ndarray:
    """Return a float vector of a specific size or raise on mismatch."""
    try:
        return np.asarray(value, dtype=float).reshape(size,)
    except ValueError as exc:
        raise ValueError(f"{name} must be a length-{size} vector.") from exc


def _as_mat(value: np.ndarray, shape: tuple[int, int], name: str) -> np.ndarray:
    """Return a float matrix of a specific shape or raise on mismatch."""
    try:
        return np.asarray(value, dtype=float).reshape(shape)
    except ValueError as exc:
        raise ValueError(f"{name} must have shape {shape}.") from exc


def skew(vector_xyz: Iterable[float]) -> np.ndarray:
    """
    Return the cross-product matrix [a]_x for a 3-vector a.

    For vectors a and b, a x b = [a]_x b.
    """
    vector = _as_vec(vector_xyz, 3, "vector_xyz")
    ax, ay, az = (float(vector[0]), float(vector[1]), float(vector[2]))
    return np.array(
        [
            [0.0, -az, ay],
            [az, 0.0, -ax],
            [-ay, ax, 0.0],
        ],
        dtype=float,
    )


def rigid_body_mass_matrix(
    mass_kg: float,
    inertia_cg_kgm2: np.ndarray,
    r_g_m: Optional[Iterable[float]] = None,
) -> np.ndarray:
    """
    Return the rigid-body 6x6 inertia matrix M_RB.

    The inertia is expressed in the body frame and assumes the origin is at the
    CG unless r_g_m is provided.
    """
    mass = float(mass_kg)
    inertia = _as_mat(inertia_cg_kgm2, (3, 3), "inertia_cg_kgm2")
    r_g = np.zeros(3, dtype=float) if r_g_m is None else _as_vec(r_g_m, 3, "r_g_m")
    s_rg = skew(r_g)

    m_rb = np.zeros((6, 6), dtype=float)
    m_rb[:3, :3] = mass * np.eye(3)
    m_rb[:3, 3:] = -mass * s_rg
    m_rb[3:, :3] = mass * s_rg
    m_rb[3:, 3:] = inertia
    return m_rb


def coriolis_from_mass_matrix(mass_matrix: np.ndarray, nu: Iterable[float]) -> np.ndarray:
    """
    Return an energy-preserving Coriolis/centripetal matrix for constant M.

    This constructs the standard 6-DOF structure:
      C = [[0, -S(p)],
           [-S(p), -S(h)]]
    where [p; h] = M * nu is the generalized momentum and S(·) is the skew
    operator. The resulting matrix is skew-symmetric in the sense that
    nu^T C(nu) nu = 0 for any nu.
    """
    m = _as_mat(mass_matrix, (6, 6), "mass_matrix")
    nu_vec = _as_vec(nu, 6, "nu")
    momentum = m @ nu_vec
    p = momentum[:3]
    h = momentum[3:]

    c = np.zeros((6, 6), dtype=float)
    c[:3, 3:] = -skew(p)
    c[3:, :3] = -skew(p)
    c[3:, 3:] = -skew(h)
    return c


def omega_matrix_body_rates(omega_pqr: Iterable[float]) -> np.ndarray:
    """
    Return the 4x4 Omega(omega) matrix for quaternion kinematics.

    The quaternion is assumed to be q = [w, x, y, z]^T and omega is expressed in
    the body frame.
    """
    omega = _as_vec(omega_pqr, 3, "omega_pqr")
    p, q, r = (float(omega[0]), float(omega[1]), float(omega[2]))
    return np.array(
        [
            [0.0, -p, -q, -r],
            [p, 0.0, r, -q],
            [q, -r, 0.0, p],
            [r, q, -p, 0.0],
        ],
        dtype=float,
    )


def quaternion_derivative_wxyz(
    quaternion_wxyz: Iterable[float], omega_pqr: Iterable[float]
) -> np.ndarray:
    """
    Compute q_dot for a body-to-world unit quaternion q_WB.

    Uses:
      q_dot = 0.5 * Omega(omega) * q
    with omega in body coordinates and q = [w, x, y, z]^T.

    Here omega = [p, q, r] is the physical body-frame angular velocity,
    while Omega(omega) is the 4x4 matrix built from omega (see
    omega_matrix_body_rates). This is why we do not integrate yaw/pitch/roll
    directly in the ODE: Euler rates must first be converted to omega.

    This matches the standard ZYX (yaw-pitch-roll) kinematics when you convert
    Euler angles to q_WB and back, but avoids singularities in the ODE.
    """
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    return 0.5 * (omega_matrix_body_rates(omega_pqr) @ q)


def normalize_quaternion_wxyz(quaternion_wxyz: Iterable[float]) -> np.ndarray:
    """Normalize a quaternion to unit length (w,x,y,z order)."""
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("Cannot normalize a zero-norm quaternion.")
    return q / norm


def rotation_matrix_from_quaternion_wxyz(quaternion_wxyz: Iterable[float]) -> np.ndarray:
    """
    Return R(q) mapping body vectors to world vectors for q=[w,x,y,z].

    This rotation is equivalent to the yaw-pitch-roll (ZYX) sequence:
      R = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    q = normalize_quaternion_wxyz(quaternion_wxyz)
    qw, qx, qy, qz = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def quaternion_multiply_wxyz(lhs_wxyz: Iterable[float], rhs_wxyz: Iterable[float]) -> np.ndarray:
    """Return the Hamilton product lhs ⊗ rhs for wxyz quaternions."""
    left = _as_vec(lhs_wxyz, 4, "lhs_wxyz")
    right = _as_vec(rhs_wxyz, 4, "rhs_wxyz")
    lw, lx, ly, lz = (float(left[0]), float(left[1]), float(left[2]), float(left[3]))
    rw, rx, ry, rz = (float(right[0]), float(right[1]), float(right[2]), float(right[3]))
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def quaternion_expmap_wxyz(omega_pqr: Iterable[float], dt: float) -> np.ndarray:
    """
    Return a delta quaternion for constant body rate omega over dt.

    This is the SO(3) exponential map: it gives the exact rotation for a
    constant angular velocity over the step, keeping the update on-manifold.
    """
    omega = _as_vec(omega_pqr, 3, "omega_pqr")
    theta = float(np.linalg.norm(omega)) * float(dt)
    if theta < 1e-12:
        return np.array(
            [1.0, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt],
            dtype=float,
        )
    axis = omega / float(np.linalg.norm(omega))
    half = 0.5 * theta
    return np.array(
        [math.cos(half), *(math.sin(half) * axis)],
        dtype=float,
    )


def integrate_quaternion_euler(
    quaternion_wxyz: Iterable[float], omega_pqr: Iterable[float], dt: float, normalize: bool = True
) -> np.ndarray:
    """Integrate quaternion using forward Euler on q_dot."""
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    q_next = q + float(dt) * quaternion_derivative_wxyz(q, omega_pqr)
    return normalize_quaternion_wxyz(q_next) if normalize else q_next


def integrate_quaternion_rk4(
    quaternion_wxyz: Iterable[float], omega_pqr: Iterable[float], dt: float
) -> np.ndarray:
    """Integrate quaternion using RK4 with normalization."""
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    k1 = quaternion_derivative_wxyz(q, omega_pqr)
    k2 = quaternion_derivative_wxyz(q + 0.5 * float(dt) * k1, omega_pqr)
    k3 = quaternion_derivative_wxyz(q + 0.5 * float(dt) * k2, omega_pqr)
    k4 = quaternion_derivative_wxyz(q + float(dt) * k3, omega_pqr)
    q_next = q + float(dt) * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    return normalize_quaternion_wxyz(q_next)


def integrate_quaternion_expmap(
    quaternion_wxyz: Iterable[float], omega_pqr: Iterable[float], dt: float
) -> np.ndarray:
    """Integrate quaternion using the exponential map (exact for constant omega)."""
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    dq = quaternion_expmap_wxyz(omega_pqr, dt)
    q_next = quaternion_multiply_wxyz(dq, q)
    return normalize_quaternion_wxyz(q_next)


def integrate_quaternion_scipy_expmap(
    quaternion_wxyz: Iterable[float], omega_pqr: Iterable[float], dt: float
) -> np.ndarray:
    """Integrate quaternion using SciPy Rotation (requires scipy)."""
    if _SciPyRotation is None:
        raise RuntimeError("SciPy not available for expmap integration.")
    q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
    omega = _as_vec(omega_pqr, 3, "omega_pqr")
    delta = _SciPyRotation.from_rotvec(omega * float(dt))
    rot = _SciPyRotation.from_quat([q[1], q[2], q[3], q[0]])
    q_next = (delta * rot).as_quat()
    return normalize_quaternion_wxyz([q_next[3], q_next[0], q_next[1], q_next[2]])


def so3_exp(omega_pqr: Iterable[float], dt: float) -> np.ndarray:
    """Return exp([omega]_x dt) using Rodrigues' formula."""
    omega = _as_vec(omega_pqr, 3, "omega_pqr")
    theta = float(np.linalg.norm(omega)) * float(dt)
    if theta < 1e-12:
        return np.eye(3, dtype=float) + skew(omega * dt)
    k = omega / float(np.linalg.norm(omega))
    kx = skew(k)
    return np.eye(3, dtype=float) + math.sin(theta) * kx + (1.0 - math.cos(theta)) * (kx @ kx)


def integrate_rotation_matrix_euler(
    rotation_body_to_world: np.ndarray, omega_pqr: Iterable[float], dt: float
) -> np.ndarray:
    """Integrate rotation matrix using forward Euler on R_dot = R[omega]_x."""
    r = _as_mat(rotation_body_to_world, (3, 3), "rotation_body_to_world")
    return r + float(dt) * (r @ skew(omega_pqr))


def integrate_rotation_matrix_expm(
    rotation_body_to_world: np.ndarray, omega_pqr: Iterable[float], dt: float
) -> np.ndarray:
    """Integrate rotation matrix using the exponential map (SO(3) exact)."""
    r = _as_mat(rotation_body_to_world, (3, 3), "rotation_body_to_world")
    return r @ so3_exp(omega_pqr, dt)


def restoring_forces_body(
    rotation_body_to_world: np.ndarray,
    weight_n: float,
    buoyancy_n: float,
    r_g_m: Optional[Iterable[float]] = None,
    r_b_m: Optional[Iterable[float]] = None,
) -> np.ndarray:
    """
    Return the 6x1 restoring force/moment vector g(eta) in the body frame.

    Uses the standard Fossen-style expression with forces expressed in the body
    frame and the CG/CB offsets defined in the body frame.
    """
    r_g = np.zeros(3, dtype=float) if r_g_m is None else _as_vec(r_g_m, 3, "r_g_m")
    r_b = np.zeros(3, dtype=float) if r_b_m is None else _as_vec(r_b_m, 3, "r_b_m")
    r = _as_mat(rotation_body_to_world, (3, 3), "rotation_body_to_world")
    r_t = r.T

    f_g_world = np.array([0.0, 0.0, -float(weight_n)], dtype=float)
    f_b_world = np.array([0.0, 0.0, float(buoyancy_n)], dtype=float)
    f_g_body = r_t @ f_g_world
    f_b_body = r_t @ f_b_world

    g_force = f_g_body + f_b_body
    g_moment = np.cross(r_g, f_g_body) + np.cross(r_b, f_b_body)
    return np.hstack((g_force, g_moment)).astype(float)


def _diag_or_matrix(value: Optional[Iterable[float] | np.ndarray]) -> np.ndarray:
    """Return a 6x6 matrix from a 6-vector (diagonal) or a 6x6 matrix."""
    if value is None:
        return np.zeros((6, 6), dtype=float)
    arr = np.asarray(value, dtype=float)
    if arr.shape == (6,):
        return np.diag(arr)
    if arr.shape == (6, 6):
        return arr
    raise ValueError("Expected a 6-vector or 6x6 matrix.")


class Fossen6DOF:
    """
    Minimal 6-DOF marine craft model using Fossen-style equations (ENU/FLU).

    State uses world-frame position and a body-to-world quaternion with
    body-frame velocities: [x, y, z, qw, qx, qy, qz, u, v, w, p, q, r].

    All units are SI (meters, seconds, radians, newtons).
    """

    def __init__(
        self,
        mass_kg: float,
        inertia_cg_kgm2: np.ndarray,
        added_mass_kg: Optional[np.ndarray] = None,
        linear_damping: Optional[Iterable[float] | np.ndarray] = None,
        quadratic_damping: Optional[Iterable[float] | np.ndarray] = None,
        r_g_m: Optional[Iterable[float]] = None,
        r_b_m: Optional[Iterable[float]] = None,
        weight_n: Optional[float] = None,
        buoyancy_n: Optional[float] = None,
    ) -> None:
        self.mass = float(mass_kg)
        self.inertia = np.asarray(inertia_cg_kgm2, dtype=float).reshape(3, 3)
        self.r_g = (
            np.zeros(3, dtype=float)
            if r_g_m is None
            else np.asarray(r_g_m, dtype=float).reshape(3,)
        )
        self.r_b = (
            np.zeros(3, dtype=float)
            if r_b_m is None
            else np.asarray(r_b_m, dtype=float).reshape(3,)
        )

        self.m_rb = rigid_body_mass_matrix(self.mass, self.inertia, self.r_g)
        self.m_a = _diag_or_matrix(added_mass_kg)
        self.m = self.m_rb + self.m_a

        self.d_linear = _diag_or_matrix(linear_damping)
        self.d_quad = _diag_or_matrix(quadratic_damping)

        default_weight = self.mass * GRAVITY_MPS2
        self.weight = default_weight if weight_n is None else float(weight_n)
        self.buoyancy = self.weight if buoyancy_n is None else float(buoyancy_n)

    def kinematics(
        self, position_xyz: np.ndarray, quaternion_wxyz: np.ndarray, nu: np.ndarray
    ) -> np.ndarray:
        """Return [p_dot, q_dot] for the given state (7x1)."""
        _ = _as_vec(position_xyz, 3, "position_xyz")
        q = _as_vec(quaternion_wxyz, 4, "quaternion_wxyz")
        nu_vec = _as_vec(nu, 6, "nu")
        v = nu_vec[:3]
        omega = nu_vec[3:]

        r = rotation_matrix_from_quaternion_wxyz(q)
        p_dot = r @ v
        q_dot = quaternion_derivative_wxyz(q, omega)
        return np.hstack((p_dot, q_dot)).astype(float)

    def damping(self, nu_r: np.ndarray) -> np.ndarray:
        """Return the 6x1 damping vector for a relative velocity nu_r."""
        nu_vec = _as_vec(nu_r, 6, "nu_r")
        linear = self.d_linear @ nu_vec
        quad = self.d_quad @ (np.abs(nu_vec) * nu_vec)
        return (linear + quad).astype(float)

    def dynamics(
        self,
        nu: np.ndarray,
        tau: np.ndarray,
        quaternion_wxyz: Optional[np.ndarray] = None,
        nu_c: Optional[np.ndarray] = None,
        nu_c_dot: Optional[np.ndarray] = None,
        tau_env: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Return nu_dot for the current velocity nu and generalized forces tau.

        - nu: body-frame velocity [u, v, w, p, q, r].
        - tau: body-frame generalized force [X, Y, Z, K, M, N].
        - quaternion_wxyz: if provided, restoring forces are included.
        - nu_c / nu_c_dot: body-frame current velocity and acceleration.
        - tau_env: additional external generalized forces.
        """
        nu_vec = _as_vec(nu, 6, "nu")
        tau_vec = _as_vec(tau, 6, "tau")
        tau_env_vec = (
            np.zeros(6, dtype=float)
            if tau_env is None
            else _as_vec(tau_env, 6, "tau_env")
        )

        nu_c_vec = (
            np.zeros(6, dtype=float)
            if nu_c is None
            else _as_vec(nu_c, 6, "nu_c")
        )
        nu_c_dot_vec = (
            np.zeros(6, dtype=float)
            if nu_c_dot is None
            else _as_vec(nu_c_dot, 6, "nu_c_dot")
        )
        nu_r = nu_vec - nu_c_vec

        c_rb = coriolis_from_mass_matrix(self.m_rb, nu_vec)
        c_a = coriolis_from_mass_matrix(self.m_a, nu_r)
        d_vec = self.damping(nu_r)

        if quaternion_wxyz is None:
            g_vec = np.zeros(6, dtype=float)
        else:
            r = rotation_matrix_from_quaternion_wxyz(quaternion_wxyz)
            g_vec = restoring_forces_body(r, self.weight, self.buoyancy, self.r_g, self.r_b)

        rhs = (
            tau_vec
            + tau_env_vec
            - (c_rb @ nu_vec)
            - (c_a @ nu_r)
            - d_vec
            - g_vec
            + (self.m_a @ nu_c_dot_vec)
        )
        return np.linalg.solve(self.m, rhs).astype(float)

    def state_derivative(
        self,
        state: np.ndarray,
        tau: np.ndarray,
        nu_c: Optional[np.ndarray] = None,
        nu_c_dot: Optional[np.ndarray] = None,
        tau_env: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Return the 13x1 derivative for [p, q, nu] given generalized forces."""
        x = _as_vec(state, 13, "state")
        position = x[:3]
        quat = x[3:7]
        nu = x[7:]

        kin = self.kinematics(position, quat, nu)
        nu_dot = self.dynamics(
            nu,
            tau,
            quaternion_wxyz=quat,
            nu_c=nu_c,
            nu_c_dot=nu_c_dot,
            tau_env=tau_env,
        )
        return np.hstack((kin, nu_dot)).astype(float)

    def _rk4_state_step(
        self,
        state: np.ndarray,
        tau: np.ndarray,
        dt: float,
        nu_c: Optional[np.ndarray],
        nu_c_dot: Optional[np.ndarray],
        tau_env: Optional[np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (x_next, omega_avg) from an RK4 step of the full state."""
        x = _as_vec(state, 13, "state")
        k1 = self.state_derivative(x, tau, nu_c=nu_c, nu_c_dot=nu_c_dot, tau_env=tau_env)
        k2 = self.state_derivative(
            x + 0.5 * float(dt) * k1, tau, nu_c=nu_c, nu_c_dot=nu_c_dot, tau_env=tau_env
        )
        k3 = self.state_derivative(
            x + 0.5 * float(dt) * k2, tau, nu_c=nu_c, nu_c_dot=nu_c_dot, tau_env=tau_env
        )
        k4 = self.state_derivative(
            x + float(dt) * k3, tau, nu_c=nu_c, nu_c_dot=nu_c_dot, tau_env=tau_env
        )
        x_next = x + float(dt) * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        omega_avg = (k1[10:] + 2 * k2[10:] + 2 * k3[10:] + k4[10:]) / 6.0
        return x_next, omega_avg

    def step(
        self,
        state: np.ndarray,
        tau: np.ndarray,
        dt: float,
        method: str = "rk4",
        nu_c: Optional[np.ndarray] = None,
        nu_c_dot: Optional[np.ndarray] = None,
        tau_env: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Advance the 6-DOF state using the selected integration method.

        Methods
        -------
        - "euler": forward Euler for full state.
        - "rk4": RK4 for full state with quaternion normalization.
        - "expmap": RK4 for position/velocity + expmap for attitude.

        """
        x = _as_vec(state, 13, "state")
        if method == "euler":
            dx = self.state_derivative(x, tau, nu_c=nu_c, nu_c_dot=nu_c_dot, tau_env=tau_env)
            x_next = x + float(dt) * dx
            x_next[3:7] = normalize_quaternion_wxyz(x_next[3:7])
            return x_next
        if method == "rk4":
            x_next, _ = self._rk4_state_step(x, tau, dt, nu_c, nu_c_dot, tau_env)
            x_next[3:7] = normalize_quaternion_wxyz(x_next[3:7])
            return x_next
        if method == "expmap":
            x_next, omega_avg = self._rk4_state_step(x, tau, dt, nu_c, nu_c_dot, tau_env)
            x_next[3:7] = integrate_quaternion_expmap(x[3:7], omega_avg, dt)
            return x_next
        raise ValueError(f"Unknown integration method: {method}")
