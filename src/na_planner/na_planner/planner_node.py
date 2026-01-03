import rclpy
import numpy as np
from rclpy.node import Node

from na_utils.bspline import find_start_u
from na_msg.msg import BsplinePath


class PlannerPublisher(Node):
    def __init__(self):
        super().__init__('planner_publisher')
        self.publisher_ = self.create_publisher(BsplinePath, 'path', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def build_circle_path(
        self,
        radius: float = 15.0,
        control_points: int = 12,
    ):
        # Control points around a circle centered at (0, R).
        angles = np.linspace(0.0, 2.0 * np.pi, control_points, endpoint=False)
        cx = 0.0
        cy = radius
        control = [
            (cx + radius * np.cos(a), cy + radius * np.sin(a))
            for a in angles
        ]
        start_u = find_start_u(control)
        return control, start_u

    def timer_callback(self):
        msg = BsplinePath()
        control, start_u = self.build_circle_path()

        msg.ctrl_x = [p[0] for p in control]
        msg.ctrl_y = [p[1] for p in control]
        msg.ctrl_z = [0.0 for _ in control]
        msg.degree = 3
        msg.closed = True
        msg.start_u = float(start_u)

        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):

    rclpy.init(args=args)
    node = PlannerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
