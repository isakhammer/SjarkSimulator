import math
import random

from na_utils.bspline import BSplinePath, samples_from_density


def _scircle_control(side=30.0, exponent=8.0, count=32):
    if side <= 0.0:
        side = 30.0
    exponent = max(2.0, float(exponent))
    count = max(8, int(count))
    half = side / 2.0
    points = []
    for i in range(count):
        theta = 2.0 * math.pi * i / count
        c = math.cos(theta)
        s = math.sin(theta)
        x = half * math.copysign(abs(c) ** (2.0 / exponent), c)
        y = half * math.copysign(abs(s) ** (2.0 / exponent), s)
        points.append((x, y))
    return points


def _align_path_to_origin(control, samples=400):
    preview = BSplinePath.from_control_points(
        control, samples=samples, closed=True, start_u=0.0
    )
    if preview.empty():
        return list(control), 0.0

    min_y = min(p[1] for p in preview.points)
    tol = max(1e-6, (max(p[1] for p in preview.points) - min_y) * 1e-3)
    candidates = [i for i, p in enumerate(preview.points) if p[1] <= min_y + tol]
    if candidates:
        anchor_idx = min(
            candidates,
            key=lambda i: preview.points[i][0] ** 2 + preview.points[i][1] ** 2,
        )
    else:
        anchor_idx = 0
    anchor = preview.points[anchor_idx]
    tangent, _ = preview._tangent_normal(anchor_idx)
    tan_x, tan_y = tangent
    if abs(tan_x) < 1e-6 and abs(tan_y) < 1e-6:
        next_idx = anchor_idx + 1 if anchor_idx < len(preview.points) - 1 else 0
        tan_x = preview.points[next_idx][0] - anchor[0]
        tan_y = preview.points[next_idx][1] - anchor[1]

    angle = math.atan2(tan_y, tan_x)
    cos_a = math.cos(-angle)
    sin_a = math.sin(-angle)

    def rotate(pt):
        x, y = pt
        return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)

    anchor_rot = rotate(anchor)
    offset_x = -anchor_rot[0]
    offset_y = -anchor_rot[1]
    aligned = []
    for pt in control:
        rx, ry = rotate(pt)
        aligned.append((rx + offset_x, ry + offset_y))

    return aligned, 0.0


def _wrap_delta(delta: float, length: float) -> float:
    if length <= 0.0:
        return delta
    delta = (delta + length) % length
    if delta > 0.5 * length:
        delta -= length
    return delta


def test_projection_tracks_scircle_path():
    control = _scircle_control()
    samples_per_meter = 4.0
    samples = samples_from_density(control, samples_per_meter, closed=True)
    control, start_u = _align_path_to_origin(control, samples=samples)
    path = BSplinePath.from_control_points(
        control, samples=samples, closed=True, start_u=start_u
    )

    rng = random.Random(0)
    max_proj_jump = 0.2
    target_step = 0.18
    points = max(400, int(math.ceil(path.length / target_step)))
    max_hint_distance = max_proj_jump
    expected_step = path.length / points
    max_error = 0.0
    last_proj_t = None

    for i in range(points):
        t = path.length * i / points
        sample = path.sample_at_t(t)
        offset = rng.uniform(-5.0, 5.0)
        x = sample.point[0] + sample.normal[0] * offset
        y = sample.point[1] + sample.normal[1] * offset
        min_progress = 0.0
        if last_proj_t is None:
            proj = path.project(x, y)
            assert proj is not None
            proj_t = proj.t
        else:
            hint_t = path.advance_t(last_proj_t, expected_step)
            proj = path.project(
                x, y, hint_t=hint_t, max_hint_distance=max_hint_distance
            )
            assert proj is not None
            delta_t = _wrap_delta(proj.t - last_proj_t, path.length)
            delta_t = max(-max_proj_jump, min(max_proj_jump, delta_t))
            min_progress = min(max_proj_jump, expected_step * 0.5)
            if delta_t < min_progress:
                delta_t = min_progress
            proj_t = path.advance_t(last_proj_t, delta_t)
        delta = _wrap_delta(proj_t - t, path.length)
        max_error = max(max_error, abs(delta))
        if last_proj_t is not None:
            jump = _wrap_delta(proj_t - last_proj_t, path.length)
            assert jump >= min_progress - 1e-6
            assert jump <= max_proj_jump + 1e-6
        last_proj_t = proj_t

    max_error_limit = max_proj_jump * 2.5
    assert max_error <= max_error_limit
