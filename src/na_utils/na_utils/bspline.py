import math


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
