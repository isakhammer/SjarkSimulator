

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

from na_sim.Boat3DOF import Boat3DOF


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")


        # Thruster controller outputs (TODO: generelize this)
        self.T_L = 0.0
        self.T_R = 0.0

        # Subscribers
        self.sub_state = self.create_subscriber(Float32MultiArray, "/boat_state",self., 10)
            

    def state_callback(self, msg):
        state = msg



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
