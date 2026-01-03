import rclpy
import numpy as np
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from na_msg.msg import Path 


class PlannerPublisher(Node):
    def __init__(self):
        super().__init__('planner_publisher')
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def build_square_sine_path(
        self,
        side_length: float = 20.0,
        samples_per_edge: int = 60,
        amplitude: float = 1.5,
        cycles_per_edge: float = 2.0,
    ):
        half = side_length / 2.0
        corners = [
            (-half, -half),
            (half, -half),
            (half, half),
            (-half, half),
        ]

        xs = []
        ys = []
        for idx in range(4):
            x0, y0 = corners[idx]
            x1, y1 = corners[(idx + 1) % 4]
            dx = x1 - x0
            dy = y1 - y0
            length = (dx**2 + dy**2) ** 0.5
            if length == 0.0:
                continue
            ux = dx / length
            uy = dy / length
            # Left-hand normal
            nx = -uy
            ny = ux

            for j in range(samples_per_edge):
                s = j / samples_per_edge
                phase = 2.0 * np.pi * cycles_per_edge * s
                offset = amplitude * np.sin(phase)
                xs.append(x0 + dx * s + nx * offset)
                ys.append(y0 + dy * s + ny * offset)

        # Close the loop
        xs.append(xs[0])
        ys.append(ys[0])

        zs = np.zeros(len(xs))
        return np.array(xs), np.array(ys), zs

    def timer_callback(self):
        wpts = Path()

        x, y, z = self.build_square_sine_path()
        wpts.wpts_x = x.tolist()
        wpts.wpts_y = y.tolist()
        wpts.wpts_z = z.tolist()

        self.publisher_.publish(wpts)
        self.i += 1


def main(args=None):

    rclpy.init(args=args)
    node = PlannerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
