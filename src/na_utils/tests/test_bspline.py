import math
import time

from na_utils.bspline import (
    BSplinePath,
    build_spline_samples,
    eval_bspline,
    eval_bspline_derivative,
)


def test_eval_bspline_returns_point():
    control = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    x, y = eval_bspline(control, 0.5)
    assert isinstance(x, float)
    assert isinstance(y, float)


def test_build_spline_samples_monotonic_arc():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    points, params, arc = build_spline_samples(
        control, start_u=0.0, samples=50, closed=True
    )

    assert len(points) == 50
    assert len(params) == 50
    assert len(arc) == 50
    assert math.isclose(arc[0], 0.0, abs_tol=1e-6)
    assert arc[-1] > arc[0]
    assert all(arc[i] <= arc[i + 1] for i in range(len(arc) - 1))


def test_bspline_path_project_on_sample():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    path = BSplinePath(control, start_u=0.0, samples=50, closed=True)

    idx = 10
    px, py = path.points[idx]
    proj = path.project(px, py)

    assert proj is not None
    assert proj.index == idx
    assert math.isclose(proj.point[0], px, abs_tol=1e-6)
    assert math.isclose(proj.point[1], py, abs_tol=1e-6)
    assert math.isclose(proj.cte, 0.0, abs_tol=1e-6)
    assert math.isclose(math.hypot(*proj.tangent), 1.0, rel_tol=1e-3)
    assert math.isclose(math.hypot(*proj.normal), 1.0, rel_tol=1e-3)


def test_bspline_path_point_at_t_sampled():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    path = BSplinePath(control, start_u=0.0, samples=50, closed=True)

    idx = 20
    t = path.t[idx]
    px, py = path.points[idx]
    x, y = path.point_at_t(t)

    assert math.isclose(x, px, abs_tol=1e-6)
    assert math.isclose(y, py, abs_tol=1e-6)


def test_bspline_path_advance_t_wraps_and_clamps():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    closed_path = BSplinePath(control, start_u=0.0, samples=50, closed=True)
    open_path = BSplinePath(control, start_u=0.0, samples=50, closed=False)

    t0 = closed_path.length - 0.1
    wrapped = closed_path.advance_t(t0, 0.2)
    assert wrapped < 0.2

    clamped = open_path.advance_t(open_path.length - 0.1, 0.2)
    assert math.isclose(clamped, open_path.length, abs_tol=1e-6)


def test_bspline_sample_at_t_unit_tangent():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    path = BSplinePath(control, start_u=0.0, samples=50, closed=True)

    sample = path.sample_at_t(path.t[10])
    assert math.isclose(math.hypot(*sample.tangent), 1.0, rel_tol=1e-3)


def test_bspline_projection_accuracy_cm():
    control = [
        (0.0, 0.0),
        (4.0, 1.0),
        (6.0, 4.0),
        (4.0, 7.0),
        (0.0, 6.0),
        (-2.0, 3.0),
    ]
    path = BSplinePath(control, start_u=0.0, samples=1600, closed=True)
    offset = 1.0
    checked = 0
    max_err = 0.0
    max_cte = 0.0

    n = len(control)
    for k in range(40):
        u = (n * k) / 40.0
        x, y = eval_bspline(control, u)
        dx, dy = eval_bspline_derivative(control, u)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        nx, ny = -dy / norm, dx / norm
        qx, qy = x + nx * offset, y + ny * offset
        proj = path.project(qx, qy)
        assert proj is not None
        err = math.hypot(proj.point[0] - x, proj.point[1] - y)
        max_err = max(max_err, err)
        max_cte = max(max_cte, abs(abs(proj.cte) - offset))
        checked += 1

    assert checked > 0
    assert max_err <= 0.01
    assert max_cte <= 0.01


def test_bspline_cte_sign_matches_left_normal():
    radius = 6.0
    angles = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi]
    control = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
    path = BSplinePath(control, start_u=0.0, samples=400, closed=True)

    sample = path.sample_at_t(path.t[len(path.t) // 4])
    nx, ny = sample.normal
    offset = 1.0

    proj_left = path.project(
        sample.point[0] + nx * offset,
        sample.point[1] + ny * offset,
    )
    assert proj_left is not None
    assert proj_left.cte > 0.0
    assert math.isclose(proj_left.cte, offset, abs_tol=0.05)

    proj_right = path.project(
        sample.point[0] - nx * offset,
        sample.point[1] - ny * offset,
    )
    assert proj_right is not None
    assert proj_right.cte < 0.0
    assert math.isclose(proj_right.cte, -offset, abs_tol=0.05)


def test_bspline_projection_hint_matches_full_search():
    control = [
        (0.0, 0.0),
        (4.0, 1.0),
        (6.0, 4.0),
        (4.0, 7.0),
        (0.0, 6.0),
        (-2.0, 3.0),
    ]
    path = BSplinePath(control, start_u=0.0, samples=600, closed=True)
    idx = len(path.points) // 3
    px, py = path.points[idx]
    _, normal = path._tangent_normal(idx)
    x = px + normal[0] * 1.0
    y = py + normal[1] * 1.0

    proj_full = path.project(x, y)
    proj_hint = path.project(x, y, hint_t=path.t[idx])

    assert proj_full is not None
    assert proj_hint is not None
    step = path.length / max(1, len(path.points) - 1)
    assert math.isclose(
        proj_full.t,
        proj_hint.t,
        abs_tol=step * 5.0,
    )


def test_bspline_projection_max_hint_distance_limits_search():
    control = [
        (0.0, 0.0),
        (5.0, 0.0),
        (10.0, 0.0),
        (15.0, 0.0),
        (20.0, 0.0),
        (25.0, 0.0),
        (30.0, 0.0),
        (35.0, 0.0),
    ]
    path = BSplinePath(
        control, start_u=0.0, samples=300, closed=False, refine_iters=0
    )
    x, y = path.points[-1]
    y += 2.0
    max_hint_distance = 0.5
    avg_step = path.length / max(1, len(path.points) - 1)
    max_samples = int(math.ceil(max_hint_distance / avg_step))
    hint_idx = len(path.points) // 2
    assert hint_idx > max_samples
    assert len(path.points) - hint_idx > max_samples
    hint_t = path.t[hint_idx]

    proj_full = path.project(x, y)
    assert proj_full is not None
    assert proj_full.t - hint_t > max_hint_distance * 2.0
    proj_hint = path.project(
        x,
        y,
        hint_t=hint_t,
        max_hint_distance=max_hint_distance,
    )
    assert proj_hint is not None
    assert abs(proj_hint.t - hint_t) <= max_hint_distance + avg_step


def test_bspline_projection_performance_budget():
    control = [
        (0.0, 0.0),
        (4.0, 1.0),
        (6.0, 4.0),
        (4.0, 7.0),
        (0.0, 6.0),
        (-2.0, 3.0),
    ]
    path = BSplinePath(control, start_u=0.0, samples=800, closed=True)
    n = len(control)
    points = []
    for k in range(100):
        u = (n * k) / 100.0
        x, y = eval_bspline(control, u)
        dx, dy = eval_bspline_derivative(control, u)
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        nx, ny = -dy / norm, dx / norm
        points.append((x + nx * 2.0, y + ny * 2.0))

    assert points
    for x, y in points[:10]:
        path.project(x, y)

    loops = 10
    start = time.perf_counter()
    for _ in range(loops):
        for x, y in points:
            path.project(x, y)
    elapsed = time.perf_counter() - start
    per_call = elapsed / (len(points) * loops)

    assert per_call <= 0.0005, f"avg {per_call * 1000:.3f}ms"
