import math


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def path_yaw_from_tangent(tx: float, ty: float) -> float:
    """Return NED yaw (clockwise-positive) from a path tangent."""
    if abs(tx) < 1e-6 and abs(ty) < 1e-6:
        return 0.0
    return math.atan2(ty, tx)


def desired_heading_ned(proj_yaw: float, cte: float, lookahead: float) -> float:
    """Return desired NED yaw given path yaw and signed cross-track error."""
    return wrap_to_pi(proj_yaw - math.atan2(cte, lookahead))
