import math

from na_utils.bspline import BSplinePath, build_spline_samples, eval_bspline


def test_eval_bspline_returns_point():
    control = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    x, y = eval_bspline(control, 0.5)
    assert isinstance(x, float)
    assert isinstance(y, float)


def test_build_spline_samples_monotonic_arc():
    control = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    points, params, arc = build_spline_samples(control, start_u=0.0, samples=50, closed=True)

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
    assert proj.point == (px, py)
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
