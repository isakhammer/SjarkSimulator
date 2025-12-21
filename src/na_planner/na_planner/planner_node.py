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

    def timer_callback(self):
        wpts = Path()
    
        # Forward progression
        x = np.linspace(0.0, 10.0, 50)
    
        # S-shape in y
        amplitude = 2.0     # lateral excursion
        wavelength = 10.0   # length of the S
        y = amplitude * np.sin(2.0 * np.pi * x / wavelength)
    
        # Flat in z
        z = np.zeros_like(x)
    
        wpts.wpts_x = x
        wpts.wpts_y = y
        wpts.wpts_z = z
    
        self.publisher_.publish(wpts)
        print("Publishing", self.i, wpts)
        self.i += 1


def main(args=None):

    print("HELLOWOWOWOW")
    rclpy.init(args=args)
    node = PlannerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
