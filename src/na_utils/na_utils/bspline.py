"""
Minimal uniform cubic B-spline utilities.

Key idea:
- The spline uses a native parameter `u` (knot index + local offset). `u` is
  unitless, not in meters, and treated as a private internal parameter.
- The spline is sampled into points with an arc-length parameter `t` in meters.
- `BSplinePath` keeps both `u` and `t` samples and provides projection and
  lookahead helpers.

Typical usage:
    path = BSplinePath(ctrl_pts, start_u=0.0, samples=400, closed=True)
    proj = path.project(x, y)
    target_t = path.advance_t(proj.t, lookahead_m)
    target_x, target_y = path.point_at_t(target_t)

Projection example:
    control = [
        (0.0, 0.0),
        (4.0, 1.0),
        (6.0, 4.0),
        (4.0, 7.0),
        (0.0, 6.0),
        (-2.0, 3.0),
    ]
    path = BSplinePath(control, start_u=0.0, samples=400, closed=True)
    proj = path.project(2.0, 1.5)
    if proj is not None:
        print("projection:", proj.point)
        print("cte:", proj.cte)
        print("tangent:", proj.tangent)

B-spline utilities:
    `BSplinePath` builds a sampled, arc-length parameterized spline and provides
    projection and lookahead helpers. The spline is stored as:
    - points: sampled (x, y) points.
    - u: unitless spline parameter values for each sample (private internal
      parameter, not meters).
    - t: arc-length parameter values for each sample (meters).

Projection and Frenet frame:
    `BSplinePath.project(x, y)` maps a point onto the spline and returns a
    `SplineProjection` with the projection point and a local Frenet frame:
    - tangent: unit tangent along the spline.
    - normal: unit normal (perpendicular to the tangent), pointing port/left.
    - cte: signed lateral error = dot((x - proj_x, y - proj_y), normal).

    `BSplinePath.sample_at_t(t)` evaluates the spline at arc-length t and
    returns position, tangent, normal, and curvature (second derivative with
    respect to arc length).

    The implementation keeps a sampled u ↔ t lookup table so it can:
    - map a projection's u to t (meters), and
    - map a desired t back to u for evaluation.

    The projection uses a two-step approach:
    1) Coarse nearest-sample search to get an initial spline parameter u.
    2) A short Gauss-Newton style refinement along the tangent direction:
       du = dot((x - px, y - py), dC/du) / dot(dC/du, dC/du)
       with configurable iteration count, tolerance, and max step size.

    This keeps the projection fast while remaining accurate at the centimeter
    scale. The nearest-sample lookup acts as the initial guess; for sequential
    queries you can reuse the last projection t to reduce work further.

Tuning knobs (constructor args):
    - refine_iters: number of refinement iterations (default 5).
    - refine_tol: convergence threshold for du (default 1e-4).
    - refine_max_step: clamps each iteration (default 0.5).

Tests:
    `src/na_utils/tests/test_bspline.py` includes:
    - Accuracy: project points offset by 1 m along the normal and assert the
      projection point and cte are within 0.01 m.
    - Performance: average projection time stays below 0.5 ms per call in a
      "worst-case" 1600-sample path.

    Run via:
    - ct for full workspace verification.
    - colcon test --packages-select na_utils for this package only.
"""

import bisect
import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class SplineProjection:
    index: int
    point: Tuple[float, float]
    t: float
    tangent: Tuple[float, float]
    normal: Tuple[float, float]
    cte: float


@dataclass(frozen=True)
class SplineSample:
    t: float
    point: Tuple[float, float]
    tangent: Tuple[float, float]
    normal: Tuple[float, float]
    curvature: Tuple[float, float]


class BSplinePath:
    """Sampled spline with arc-length `t` (meters) and native parameter `u`."""

    def __init__(
        self,
        control_points: Sequence[Tuple[float, float]],
        start_u: float,
        samples: int,
        closed: bool = True,
        refine_iters: int = 5,
        refine_tol: float = 1e-4,
        refine_max_step: float = 0.5,
    ) -> None:
        self.control_points = list(control_points)
        self.start_u = start_u
        self.closed = closed
        # `u` is the spline parameter; `t` is the arc-length (meters) at each `u`.
        self.points, self.u, self.t = build_spline_samples(
            self.control_points, start_u, samples, closed=closed
        )
        self.length = self.t[-1] if self.t else 0.0
        self.t_start = self.t[0] if self.t else 0.0
        self.t_end = self.t[-1] if self.t else 0.0
        self.u_start = self.u[0] if self.u else 0.0
        self.u_end = self.u[-1] if self.u else 0.0
        self.refine_iters = max(0, int(refine_iters))
        self.refine_tol = float(refine_tol)
        self.refine_max_step = float(refine_max_step)

    @classmethod
    def from_control_points(
        cls,
        control_points: Sequence[Tuple[float, float]],
        samples: int = 400,
        closed: bool = True,
        start_u: float | None = None,
        refine_iters: int = 5,
        refine_tol: float = 1e-4,
        refine_max_step: float = 0.5,
    ) -> "BSplinePath":
        """Build a path and choose a default start_u when none is provided."""
        if start_u is None:
            if closed:
                start_u = find_start_u(control_points, samples=samples)
            else:
                start_u = 0.0
        return cls(
            control_points,
            start_u,
            samples,
            closed=closed,
            refine_iters=refine_iters,
            refine_tol=refine_tol,
            refine_max_step=refine_max_step,
        )

    def empty(self) -> bool:
        """Return True if no samples were generated."""
        return not self.points

    def _tangent_normal(
        self, index: int
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        m = len(self.points)
        if m == 0:
            return (0.0, 0.0), (0.0, 0.0)
        if self.closed:
            prev_idx = index - 1 if index > 0 else m - 1
            next_idx = index + 1 if index < m - 1 else 0
        else:
            prev_idx = max(0, index - 1)
            next_idx = min(m - 1, index + 1)
        tx = self.points[next_idx][0] - self.points[prev_idx][0]
        ty = self.points[next_idx][1] - self.points[prev_idx][1]
        t_norm = math.hypot(tx, ty)
        if t_norm < 1e-6:
            return (0.0, 0.0), (0.0, 0.0)
        tx /= t_norm
        ty /= t_norm
        return (tx, ty), (-ty, tx)

    def _wrap_u(self, u: float) -> float:
        if not self.u:
            return u
        if self.closed:
            span = self.u_end - self.u_start
            if span <= 0.0:
                return u
            return ((u - self.u_start) % span) + self.u_start
        return max(self.u_start, min(self.u_end, u))

    def _wrap_t(self, t: float) -> float:
        if self.length <= 0.0:
            return t
        if self.closed:
            return t % self.length
        return max(self.t_start, min(self.t_end, t))

    def _u_at_t(self, t: float) -> float:
        if not self.t or not self.u:
            return 0.0
        t = self._wrap_t(t)
        idx = bisect.bisect_left(self.t, t)
        if idx <= 0:
            return self.u[0]
        if idx >= len(self.t):
            return self.u[-1]
        t0 = self.t[idx - 1]
        t1 = self.t[idx]
        u0 = self.u[idx - 1]
        u1 = self.u[idx]
        if abs(t1 - t0) < 1e-9:
            return u0
        alpha = (t - t0) / (t1 - t0)
        return u0 + alpha * (u1 - u0)

    def _arc_length_at_u(self, u: float) -> float:
        if not self.t or not self.u:
            return 0.0
        idx = bisect.bisect_left(self.u, u)
        if idx <= 0:
            return self.t[0]
        if idx >= len(self.u):
            return self.t[-1]
        u0 = self.u[idx - 1]
        u1 = self.u[idx]
        t0 = self.t[idx - 1]
        t1 = self.t[idx]
        if abs(u1 - u0) < 1e-9:
            return t0
        alpha = (u - u0) / (u1 - u0)
        return t0 + alpha * (t1 - t0)

    def _tangent_normal_at_u(
        self, u: float, fallback_idx: int
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        dx, dy = eval_bspline_derivative(self.control_points, u, closed=self.closed)
        t_norm = math.hypot(dx, dy)
        if t_norm < 1e-6:
            return self._tangent_normal(fallback_idx)
        tx = dx / t_norm
        ty = dy / t_norm
        return (tx, ty), (-ty, tx)

    def _refine_u(self, x: float, y: float, u: float) -> float:
        u = self._wrap_u(u)
        for _ in range(self.refine_iters):
            px, py = eval_bspline(self.control_points, u, closed=self.closed)
            dx, dy = eval_bspline_derivative(self.control_points, u, closed=self.closed)
            denom = dx * dx + dy * dy
            if denom < 1e-12:
                break
            du = ((x - px) * dx + (y - py) * dy) / denom
            if abs(du) < self.refine_tol:
                break
            if self.refine_max_step > 0.0:
                du = max(-self.refine_max_step, min(self.refine_max_step, du))
            u = self._wrap_u(u + du)
        return u

    def _frame_at_u(
        self, u: float, fallback_idx: int | None = None
    ) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        dx, dy = eval_bspline_derivative(self.control_points, u, closed=self.closed)
        speed = math.hypot(dx, dy)
        if speed < 1e-6:
            if fallback_idx is None:
                return (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)
            tangent, normal = self._tangent_normal(fallback_idx)
            return tangent, normal, (0.0, 0.0)
        tangent = (dx / speed, dy / speed)
        normal = (-tangent[1], tangent[0])
        ddx, ddy = eval_bspline_second_derivative(
            self.control_points, u, closed=self.closed
        )
        dot = dx * ddx + dy * ddy
        speed2 = speed * speed
        speed4 = speed2 * speed2
        curvature = (
            ddx / speed2 - dx * dot / speed4,
            ddy / speed2 - dy * dot / speed4,
        )
        return tangent, normal, curvature

    def _nearest_sample_index(
        self, x: float, y: float, hint_t: float | None, max_hint_distance: float | None
    ) -> int:
        m = len(self.points)
        if m == 0:
            return 0
        if hint_t is None or not self.t:
            best_idx = 0
            best_d2 = float("inf")
            for i, (px, py) in enumerate(self.points):
                dx = x - px
                dy = y - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = i
            return best_idx

        t_hint = self._wrap_t(hint_t)
        idx = bisect.bisect_left(self.t, t_hint)
        if idx >= m:
            idx = m - 1
        if max_hint_distance is not None and not self.closed and self.t:
            if max_hint_distance <= 0.0:
                lo = idx
                hi = idx
            else:
                t_low = t_hint - max_hint_distance
                t_high = t_hint + max_hint_distance
                lo = bisect.bisect_left(self.t, t_low)
                hi = bisect.bisect_right(self.t, t_high) - 1
                lo = max(0, min(lo, m - 1))
                hi = max(0, min(hi, m - 1))
                if lo > hi:
                    lo = idx
                    hi = idx
            best_idx = idx
            best_d2 = float("inf")
            for j in range(lo, hi + 1):
                px, py = self.points[j]
                dx = x - px
                dy = y - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = j
            return best_idx
        search = max(10, m // 50)
        if max_hint_distance is not None:
            if max_hint_distance <= 0.0:
                search = 0
            elif m > 1 and self.length > 0.0:
                avg_step = self.length / (m - 1)
                if avg_step > 0.0:
                    max_samples = int(math.ceil(max_hint_distance / avg_step))
                    search = min(search, max(1, max_samples))
                else:
                    search = 0
            else:
                search = 0
        search = min(search, m - 1)
        best_idx = idx
        best_d2 = float("inf")
        for offset in range(-search, search + 1):
            j = idx + offset
            if self.closed:
                j %= m
            else:
                if j < 0 or j >= m:
                    continue
            px, py = self.points[j]
            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = j
        return best_idx

    def project(
        self,
        x: float,
        y: float,
        hint_t: float | None = None,
        max_hint_distance: float | None = None,
    ) -> SplineProjection | None:
        """Project a point onto the spline (nearest sample + refine)."""
        if not self.points:
            return None
        best_idx = self._nearest_sample_index(x, y, hint_t, max_hint_distance)
        u = self.u[best_idx] if self.u else 0.0
        if self.refine_iters > 0:
            u = self._refine_u(x, y, u)
        proj_x, proj_y = eval_bspline(self.control_points, u, closed=self.closed)
        tangent, normal = self._tangent_normal_at_u(u, best_idx)
        cte = (x - proj_x) * normal[0] + (y - proj_y) * normal[1]
        t = self._arc_length_at_u(u)
        return SplineProjection(
            index=best_idx,
            point=(proj_x, proj_y),
            t=t,
            tangent=tangent,
            normal=normal,
            cte=cte,
        )

    def sample_at_t(self, t: float) -> SplineSample:
        if not self.points:
            return SplineSample(
                t, (0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)
            )
        t = self._wrap_t(t)
        u = self._u_at_t(t)
        x, y = eval_bspline(self.control_points, u, closed=self.closed)
        tangent, normal, curvature = self._frame_at_u(u)
        return SplineSample(t, (x, y), tangent, normal, curvature)

    def advance_t(self, t: float, distance: float) -> float:
        """
        Advance along arc-length `t` by `distance` meters.

        Closed splines wrap (loop back to start). Open splines clamp (stop at
        ends).

        Examples (length = 100):
        - closed: t=95, distance=10 -> t=5
        - open:   t=95, distance=10 -> t=100
        """
        if self.length <= 0.0:
            return t
        return self._wrap_t(t + distance)

    def point_at_t(self, t: float) -> Tuple[float, float]:
        """Evaluate the spline point at arc-length `t`."""
        if not self.points:
            return (0.0, 0.0)
        t = self._wrap_t(t)
        u = self._u_at_t(t)
        return eval_bspline(self.control_points, u, closed=self.closed)


class ProjectionTracker:
    """
    Track stable projections along a spline with hinting and jump limits.

    This wraps the stateless `BSplinePath.project` with a stateful `last_t`
    history, optional kinematic prediction, and clamp logic so sequential
    projections stay smooth. That matters for controllers that project every
    tick: `BSplinePath.project` alone can jump to a different local minimum
    when the vessel is far from the path or the path loops back on itself.
    """

    def __init__(self) -> None:
        self.last_t: float | None = None

    def reset(self) -> None:
        self.last_t = None

    def _predict_step(
        self,
        path: "BSplinePath",
        psi: float | None,
        u: float | None,
        v: float | None,
        dt: float | None,
    ) -> tuple[float, float | None]:
        if (
            self.last_t is None
            or dt is None
            or dt <= 0.0
            or psi is None
            or u is None
            or v is None
        ):
            return 0.0, None
        last_sample = path.sample_at_t(self.last_t)
        tx, ty = last_sample.tangent
        if abs(tx) < 1e-6 and abs(ty) < 1e-6:
            return 0.0, None
        vx = u * math.cos(psi) - v * math.sin(psi)
        vy = u * math.sin(psi) + v * math.cos(psi)
        along_speed = vx * tx + vy * ty
        return along_speed * dt, along_speed

    def _projection_delta(
        self, path: "BSplinePath", last_t: float, new_t: float
    ) -> float:
        delta = new_t - last_t
        if not path.closed or path.length <= 0.0:
            return delta
        length = path.length
        delta = (delta + length) % length
        if delta > 0.5 * length:
            delta -= length
        return delta

    def project_t(
        self,
        path: "BSplinePath",
        x: float,
        y: float,
        *,
        max_jump: float = 0.0,
        pred_step: float | None = None,
        along_speed: float | None = None,
        psi: float | None = None,
        u: float | None = None,
        v: float | None = None,
        dt: float | None = None,
    ) -> float | None:
        """
        Project (x, y) onto `path`, returning a stable arc-length `t`.

        If `pred_step` is omitted and kinematics are provided, a predicted
        step is computed from body velocities projected onto the path tangent.
        That predicted step is used to seed the hint and to enforce a minimum
        forward progress when `max_jump` is active.

        Unlike `BSplinePath.project`, this method is stateful (uses `last_t`),
        clamps large jumps, and returns only the tracked `t` value.
        """
        if path is None or path.empty():
            return None

        if pred_step is None:
            pred_step, along_speed = self._predict_step(path, psi, u, v, dt)
        else:
            pred_step = float(pred_step)
            if along_speed is not None:
                along_speed = float(along_speed)

        if max_jump > 0.0:
            pred_step = max(-max_jump, min(max_jump, pred_step))

        hint_t = self.last_t
        if self.last_t is not None and pred_step != 0.0:
            hint_t = path.advance_t(self.last_t, pred_step)

        if self.last_t is not None and max_jump > 0.0:
            projection = path.project(x, y, hint_t=hint_t, max_hint_distance=max_jump)
        else:
            projection = path.project(x, y, hint_t=hint_t)
        if projection is None:
            return None

        proj_t = projection.t
        if self.last_t is not None and max_jump > 0.0:
            delta_t = self._projection_delta(path, self.last_t, proj_t)
            delta_t = max(-max_jump, min(max_jump, delta_t))
            if along_speed is not None and along_speed > 0.05:
                min_progress = max(0.0, min(max_jump, pred_step * 0.5))
                if delta_t < min_progress:
                    delta_t = min_progress
            proj_t = path.advance_t(self.last_t, delta_t)

        self.last_t = proj_t
        return proj_t


def _control_point(control_points, idx, closed):
    if closed:
        return control_points[idx % len(control_points)]
    if idx < 0:
        return control_points[0]
    if idx >= len(control_points):
        return control_points[-1]
    return control_points[idx]


def eval_bspline(control_points, u, closed=True):
    n = len(control_points)
    if n < 4:
        return control_points[0]

    i = int(math.floor(u))
    t = u - math.floor(u)
    p0 = _control_point(control_points, i - 1, closed)
    p1 = _control_point(control_points, i, closed)
    p2 = _control_point(control_points, i + 1, closed)
    p3 = _control_point(control_points, i + 2, closed)

    b0 = (-t**3 + 3.0 * t**2 - 3.0 * t + 1.0) / 6.0
    b1 = (3.0 * t**3 - 6.0 * t**2 + 4.0) / 6.0
    b2 = (-3.0 * t**3 + 3.0 * t**2 + 3.0 * t + 1.0) / 6.0
    b3 = t**3 / 6.0

    x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
    y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
    return (x, y)


def eval_bspline_derivative(control_points, u, closed=True):
    n = len(control_points)
    if n < 4:
        return (0.0, 0.0)

    i = int(math.floor(u))
    t = u - math.floor(u)
    p0 = _control_point(control_points, i - 1, closed)
    p1 = _control_point(control_points, i, closed)
    p2 = _control_point(control_points, i + 1, closed)
    p3 = _control_point(control_points, i + 2, closed)

    db0 = (-3.0 * t * t + 6.0 * t - 3.0) / 6.0
    db1 = (9.0 * t * t - 12.0 * t) / 6.0
    db2 = (-9.0 * t * t + 6.0 * t + 3.0) / 6.0
    db3 = (3.0 * t * t) / 6.0

    x = db0 * p0[0] + db1 * p1[0] + db2 * p2[0] + db3 * p3[0]
    y = db0 * p0[1] + db1 * p1[1] + db2 * p2[1] + db3 * p3[1]
    return (x, y)


def eval_bspline_second_derivative(control_points, u, closed=True):
    n = len(control_points)
    if n < 4:
        return (0.0, 0.0)

    i = int(math.floor(u))
    t = u - math.floor(u)
    p0 = _control_point(control_points, i - 1, closed)
    p1 = _control_point(control_points, i, closed)
    p2 = _control_point(control_points, i + 1, closed)
    p3 = _control_point(control_points, i + 2, closed)

    ddb0 = 1.0 - t
    ddb1 = 3.0 * t - 2.0
    ddb2 = -3.0 * t + 1.0
    ddb3 = t

    x = ddb0 * p0[0] + ddb1 * p1[0] + ddb2 * p2[0] + ddb3 * p3[0]
    y = ddb0 * p0[1] + ddb1 * p1[1] + ddb2 * p2[1] + ddb3 * p3[1]
    return (x, y)

def build_spline_samples(control_points, start_u, samples, closed=True):
    n = len(control_points)
    if n < 4 or samples < 2:
        return [], [], []

    u_start = start_u
    u_end = start_u + n
    if not closed:
        u_start = 0.0
        u_end = max(1.0, n - 3.0)

    du = (u_end - u_start) / (samples - 1)
    points = []
    params = []
    for k in range(samples):
        u = u_start + du * k
        params.append(u)
        points.append(eval_bspline(control_points, u, closed=closed))

    arc = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        arc.append(arc[-1] + math.hypot(dx, dy))

    return points, params, arc


def estimate_spline_length(control_points, closed=True, samples=100):
    """Estimate arc length with a coarse spline sampling."""
    n = len(control_points)
    if n < 4 or samples < 2:
        return 0.0

    u_start = 0.0
    u_end = float(n) if closed else max(1.0, n - 3.0)
    du = (u_end - u_start) / (samples - 1)

    prev_x, prev_y = eval_bspline(control_points, u_start, closed=closed)
    length = 0.0
    for k in range(1, samples):
        u = u_start + du * k
        x, y = eval_bspline(control_points, u, closed=closed)
        length += math.hypot(x - prev_x, y - prev_y)
        prev_x, prev_y = x, y
    return length


def samples_from_density(
    control_points,
    samples_per_meter,
    closed=True,
    min_samples=50,
    estimate_samples=100,
):
    """Convert samples-per-meter into a sample count."""
    if samples_per_meter <= 0.0:
        return max(min_samples, 2)
    length = estimate_spline_length(
        control_points, closed=closed, samples=estimate_samples
    )
    if length <= 0.0:
        return max(min_samples, 2)
    target = int(math.ceil(length * samples_per_meter)) + 1
    return max(min_samples, target)


def find_start_u(control_points, samples=400):
    n = len(control_points)
    if n < 4:
        return 0.0

    best_u = 0.0
    best_d2 = float("inf")
    for k in range(samples):
        u = (n * k) / samples
        x, y = eval_bspline(control_points, u, closed=True)
        d2 = x * x + y * y
        if d2 < best_d2:
            best_d2 = d2
            best_u = u
    return best_u
