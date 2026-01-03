import math

from na_utils.bspline import build_spline_samples, eval_bspline


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
