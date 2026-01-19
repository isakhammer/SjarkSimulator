import math

from na_msg.msg import BsplinePath
from na_utils.bspline import BSplinePath, samples_from_density
import rclpy
from rclpy.node import Node


class PlannerPublisher(Node):
    """Publish a selectable B-spline path."""

    def __init__(self):
        super().__init__('planner_publisher')
        self.declare_parameter("path_type", "SQUARE_SINUS")
        self.declare_parameter("samples_per_meter", 4.0)
        self.publisher_ = self.create_publisher(BsplinePath, 'path', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_circle(
        self,
        radius: float = 15.0,
        control_points: int = 12,
    ):
        if radius <= 0.0:
            radius = 15.0
        count = max(6, int(control_points))
        angles = [2.0 * math.pi * i / count for i in range(count)]
        cx = 0.0
        cy = radius
        control = [
            (cx + radius * math.cos(a), cy + radius * math.sin(a))
            for a in angles
        ]
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        preview_samples = samples_from_density(
            control, samples_per_meter, closed=True
        )
        control, start_u = self._align_path_to_origin(
            control, samples=preview_samples
        )
        path = BSplinePath.from_control_points(
            control, samples=preview_samples, closed=True, start_u=start_u
        )
        return path.control_points, path.start_u, True

    def get_square_sinus(
        self,
        side: float = 30.0,
        zigzags_per_edge: int = 2,
        amplitude: float = 1.0,
    ):
        if side <= 0.0:
            side = 30.0
        points_per_edge = max(1, int(zigzags_per_edge))
        corner_weight = max(0.0, min(float(amplitude), 1.0))
        corner_repeats = 1 + int(round(corner_weight))
        half = side / 2.0
        corners = [
            (half, -half),
            (half, half),
            (-half, half),
            (-half, -half),
        ]
        control = []
        for idx, start in enumerate(corners):
            end = corners[(idx + 1) % len(corners)]
            for i in range(points_per_edge):
                t = i / points_per_edge
                x = start[0] + (end[0] - start[0]) * t
                y = start[1] + (end[1] - start[1]) * t
                if i == 0:
                    control.extend([(x, y)] * corner_repeats)
                else:
                    control.append((x, y))
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        preview_samples = samples_from_density(
            control, samples_per_meter, closed=True
        )
        control, start_u = self._align_path_to_origin(
            control, samples=preview_samples
        )
        path = BSplinePath.from_control_points(
            control, samples=preview_samples, closed=True, start_u=start_u
        )
        return path.control_points, path.start_u, True

    def get_straight(
        self,
        length: float = 30.0,
    ):
        if length <= 0.0:
            length = 30.0
        half = length / 2.0
        control = [(-half, 0.0), (-half / 3.0, 0.0), (half / 3.0, 0.0), (half, 0.0)]
        return control, 0.0, False

    def get_squircle(
        self,
        radius: float = 12.0,
        exponent: float = 4.0,
        control_points: int = 16,
    ):
        if radius <= 0.0:
            radius = 12.0
        n = max(4.0, float(exponent))
        count = max(8, int(control_points))
        angles = [2.0 * math.pi * i / count for i in range(count)]

        def superellipse(coord):
            if coord == 0.0:
                return 0.0
            return math.copysign(abs(coord) ** (2.0 / n), coord)

        control = [
            (
                radius * superellipse(math.cos(a)),
                radius * superellipse(math.sin(a)),
            )
            for a in angles
        ]
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        preview_samples = samples_from_density(control, samples_per_meter, closed=True)
        control, start_u = self._align_path_to_origin(control, samples=preview_samples)
        path = BSplinePath.from_control_points(
            control, samples=preview_samples, closed=True, start_u=start_u
        )
        return path.control_points, path.start_u, True

    def get_complex(
        self,
        scale: float = 12.0,
        control_points: int = 20,
    ):
        if scale <= 0.0:
            scale = 12.0
        count = max(10, int(control_points))
        angles = [2.0 * math.pi * i / count for i in range(count)]
        control = [(scale * math.sin(a), scale * math.sin(a) * math.cos(a)) for a in angles]
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        preview_samples = samples_from_density(control, samples_per_meter, closed=True)
        control, start_u = self._align_path_to_origin(control, samples=preview_samples)
        path = BSplinePath.from_control_points(
            control, samples=preview_samples, closed=True, start_u=start_u
        )
        return path.control_points, path.start_u, True

    def _align_path_to_origin(self, control, samples: int = 400):
        # Anchor the lowest-y sample at the origin and align its heading with +x.
        preview = BSplinePath.from_control_points(
            control, samples=samples, closed=True, start_u=0.0
        )
        if preview.empty():
            return list(control), 0.0

        min_y = min(p[1] for p in preview.points)
        tol = max(1e-6, (max(p[1] for p in preview.points) - min_y) * 1e-3)
        candidates = [
            i for i, p in enumerate(preview.points) if p[1] <= min_y + tol
        ]
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
            next_idx = (
                anchor_idx + 1 if anchor_idx < len(preview.points) - 1 else 0
            )
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

    def timer_callback(self):
        msg = BsplinePath()
        path_type = str(self.get_parameter("path_type").value).strip().upper()
        path_type = path_type.replace("-", "_")
        if path_type == "CIRCLE":
            control, start_u, closed = self.get_circle()
        elif path_type == "SQUARE_SINUS":
            control, start_u, closed = self.get_square_sinus()
        elif path_type == "STRAIGHT":
            control, start_u, closed = self.get_straight()
        elif path_type == "SQUIRCLE":
            control, start_u, closed = self.get_squircle()
        elif path_type == "COMPLEX":
            control, start_u, closed = self.get_complex()
        else:
            self.get_logger().warn(
                f"Unknown path_type '{path_type}', using SQUARE_SINUS"
            )
            control, start_u, closed = self.get_square_sinus()

        msg.ctrl_x = [p[0] for p in control]
        msg.ctrl_y = [p[1] for p in control]
        msg.ctrl_z = [0.0 for _ in control]
        msg.degree = 3
        msg.closed = bool(closed)
        msg.start_u = float(start_u)

        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = PlannerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
