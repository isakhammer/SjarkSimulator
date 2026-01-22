import math

from na_controller.los import (
    desired_heading_ned,
    path_yaw_from_tangent,
    wrap_to_pi,
)


def test_path_yaw_from_tangent_ned():
    assert math.isclose(path_yaw_from_tangent(1.0, 0.0), 0.0, abs_tol=1e-6)
    assert math.isclose(
        path_yaw_from_tangent(0.0, 1.0), math.pi / 2.0, abs_tol=1e-6
    )
    assert math.isclose(
        path_yaw_from_tangent(0.0, -1.0), -math.pi / 2.0, abs_tol=1e-6
    )
    yaw = path_yaw_from_tangent(-1.0, 0.0)
    assert math.isclose(abs(yaw), math.pi, abs_tol=1e-6)


def test_desired_heading_ned_cte_sign():
    proj_yaw = 0.0
    lookahead = 2.0

    desired_right = desired_heading_ned(proj_yaw, cte=1.0, lookahead=lookahead)
    desired_left = desired_heading_ned(proj_yaw, cte=-1.0, lookahead=lookahead)

    assert desired_right < 0.0
    assert desired_left > 0.0


def test_wrap_to_pi_range():
    for angle in (
        -4.0 * math.pi,
        -1.5 * math.pi,
        0.0,
        1.5 * math.pi,
        4.0 * math.pi,
    ):
        wrapped = wrap_to_pi(angle)
        assert -math.pi <= wrapped < math.pi
