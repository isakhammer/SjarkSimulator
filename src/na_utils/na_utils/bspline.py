"""
Minimal uniform cubic B-spline utilities.

Key idea:
- The spline is sampled into points with an arc-length parameter `t` in meters.
- `BSplinePath` keeps those samples and provides projection and lookahead helpers.

Typical usage:
    path = BSplinePath(ctrl_pts, start_u=0.0, samples=400, closed=True)
    proj = path.project(x, y)
    target_t = path.advance_t(proj.t, lookahead_m)
    target_x, target_y = path.point_at_t(target_t)
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


class BSplinePath:
    """Sampled spline representation with arc-length parameter `t` in meters."""

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
        self.points, self.u, self.t = build_spline_samples(
            self.control_points, start_u, samples, closed=closed
        )
        self.length = self.t[-1] if self.t else 0.0
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
            start_u = find_start_u(control_points, samples=samples) if closed else 0.0
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

    def _tangent_normal(self, index: int) -> Tuple[Tuple[float, float], Tuple[float, float]]:
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
        dx, dy = eval_bspline_derivative(self.control_points, u)
        t_norm = math.hypot(dx, dy)
        if t_norm < 1e-6:
            return self._tangent_normal(fallback_idx)
        tx = dx / t_norm
        ty = dy / t_norm
        return (tx, ty), (-ty, tx)

    def _refine_u(self, x: float, y: float, u: float) -> float:
        u = self._wrap_u(u)
        for _ in range(self.refine_iters):
            px, py = eval_bspline(self.control_points, u)
            dx, dy = eval_bspline_derivative(self.control_points, u)
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

    def project(self, x: float, y: float) -> SplineProjection | None:
        """Project a point onto the sampled spline (nearest sample + refine)."""
        if not self.points:
            return None
        best_idx = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(self.points):
            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        u = self.u[best_idx] if self.u else 0.0
        if self.refine_iters > 0:
            u = self._refine_u(x, y, u)
        proj_x, proj_y = eval_bspline(self.control_points, u)
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

    def advance_t(self, t: float, distance: float) -> float:
        """
        Advance along arc-length `t` by `distance` meters.

        Closed splines wrap (loop back to start). Open splines clamp (stop at ends).

        Examples (length = 100):
        - closed: t=95, distance=10 -> t=5
        - open:   t=95, distance=10 -> t=100
        """
        if self.length <= 0.0:
            return t
        target = t + distance
        if self.closed:
            target = target % self.length
        else:
            target = max(0.0, min(self.length, target))
        return target

    def point_at_t(self, t: float) -> Tuple[float, float]:
        """Interpolate the sampled spline point at arc-length `t`."""
        if not self.points:
            return (0.0, 0.0)
        if self.length <= 0.0:
            return self.points[0]
        if self.closed:
            t = t % self.length
        else:
            t = max(0.0, min(self.length, t))
        idx = bisect.bisect_left(self.t, t)
        if idx <= 0:
            return self.points[0]
        if idx >= len(self.points):
            return self.points[-1]
        t0 = self.t[idx - 1]
        t1 = self.t[idx]
        if t1 - t0 < 1e-6:
            alpha = 0.0
        else:
            alpha = (t - t0) / (t1 - t0)
        x0, y0 = self.points[idx - 1]
        x1, y1 = self.points[idx]
        return (x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0))


def eval_bspline(control_points, u):
    n = len(control_points)
    if n < 4:
        return control_points[0]

    i = int(math.floor(u)) % n
    t = u - math.floor(u)
    p0 = control_points[(i - 1) % n]
    p1 = control_points[i % n]
    p2 = control_points[(i + 1) % n]
    p3 = control_points[(i + 2) % n]

    b0 = (-t**3 + 3.0 * t**2 - 3.0 * t + 1.0) / 6.0
    b1 = (3.0 * t**3 - 6.0 * t**2 + 4.0) / 6.0
    b2 = (-3.0 * t**3 + 3.0 * t**2 + 3.0 * t + 1.0) / 6.0
    b3 = t**3 / 6.0

    x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
    y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
    return (x, y)


def eval_bspline_derivative(control_points, u):
    n = len(control_points)
    if n < 4:
        return (0.0, 0.0)

    i = int(math.floor(u)) % n
    t = u - math.floor(u)
    p0 = control_points[(i - 1) % n]
    p1 = control_points[i % n]
    p2 = control_points[(i + 1) % n]
    p3 = control_points[(i + 2) % n]

    db0 = (-3.0 * t * t + 6.0 * t - 3.0) / 6.0
    db1 = (9.0 * t * t - 12.0 * t) / 6.0
    db2 = (-9.0 * t * t + 6.0 * t + 3.0) / 6.0
    db3 = (3.0 * t * t) / 6.0

    x = db0 * p0[0] + db1 * p1[0] + db2 * p2[0] + db3 * p3[0]
    y = db0 * p0[1] + db1 * p1[1] + db2 * p2[1] + db3 * p3[1]
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
        points.append(eval_bspline(control_points, u))

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

    prev_x, prev_y = eval_bspline(control_points, u_start)
    length = 0.0
    for k in range(1, samples):
        u = u_start + du * k
        x, y = eval_bspline(control_points, u)
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
        x, y = eval_bspline(control_points, u)
        d2 = x * x + y * y
        if d2 < best_d2:
            best_d2 = d2
            best_u = u
    return best_u
