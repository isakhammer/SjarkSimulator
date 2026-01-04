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
        path = BSplinePath.from_control_points(control, samples=400, closed=True)
        return path.control_points, path.start_u

    def get_square_sinus(
        self,
        side: float = 30.0,
        zigzags_per_edge: int = 5,
        amplitude: float = 4.0,
    ):
        oscillations = max(1, int(zigzags_per_edge))
        if side <= 0.0:
            side = 30.0
        max_amp = max(0.0, side * 0.3)
        amp = max(0.0, min(float(amplitude), max_amp))
        points_per_edge = max(8, oscillations * 8)
        if points_per_edge % 2 != 0:
            points_per_edge += 1

        def edge_points(start, end, normal):
            pts = []
            sx, sy = start
            ex, ey = end
            nx, ny = normal
            for k in range(points_per_edge + 1):
                t = k / points_per_edge
                wave = math.sin(2.0 * math.pi * oscillations * t) ** 3
                x = sx + (ex - sx) * t + nx * amp * wave
                y = sy + (ey - sy) * t + ny * amp * wave
                pts.append((x, y))
            return pts

        half = side / 2.0
        bottom = edge_points((-half, 0.0), (half, 0.0), (0.0, 1.0))
        right = edge_points((half, 0.0), (half, side), (-1.0, 0.0))
        top = edge_points((half, side), (-half, side), (0.0, -1.0))
        left = edge_points((-half, side), (-half, 0.0), (1.0, 0.0))
        control = bottom + right[1:] + top[1:] + left[1:-1]
        # Start at the bottom edge midpoint so origin sits on the boundary.
        origin_idx = len(bottom) // 2
        control = control[origin_idx:] + control[:origin_idx]

        path = BSplinePath.from_control_points(
            control, samples=400, closed=True, start_u=0.0
        )
        return path.control_points, path.start_u

    def timer_callback(self):
        msg = BsplinePath()
        path_type = str(self.get_parameter("path_type").value).strip().upper()
        path_type = path_type.replace("-", "_")
        if path_type == "CIRCLE":
            control, start_u = self.get_circle()
        elif path_type == "SQUARE_SINUS":
            control, start_u = self.get_square_sinus()
        else:
            self.get_logger().warn(
                f"Unknown path_type '{path_type}', using SQUARE_SINUS"
            )
            control, start_u = self.get_square_sinus()

        msg.ctrl_x = [p[0] for p in control]
        msg.ctrl_y = [p[1] for p in control]
        msg.ctrl_z = [0.0 for _ in control]
        msg.degree = 3
        msg.closed = True
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
