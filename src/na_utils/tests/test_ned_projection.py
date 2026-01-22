import math

from na_utils.bspline import BSplinePath, ProjectionTracker


def test_projection_tracker_predict_step_uses_ned_kinematics():
    control = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)]
    path = BSplinePath(control, start_u=0.0, samples=200, closed=False)

    tracker = ProjectionTracker()
    tracker.last_t = path.t[len(path.t) // 2]

    psi = math.pi / 2.0
    u = 1.0
    v = 0.0
    dt = 1.0
    pred_step, along_speed = tracker._predict_step(path, psi, u, v, dt)

    assert along_speed is not None
    assert along_speed > 0.5
    assert pred_step > 0.5
