

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Publisher: thrust command [T_L, T_R]
        self.thrust_pub = self.create_publisher(Float32MultiArray, "/cmd_thrust", 10)

        # Subscriber: boat state [x, y, psi, u, v, r]
        self.state_sub = self.create_subscription(
            Float32MultiArray, "/boat_state", self.state_callback, 10
        )

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,psi,u,v,r
        self.start_time = self.get_clock().now()
        self.get_logger().info("Boat controller started")

    def state_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.state = list(msg.data)

    def control_loop(self) -> None:
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Initializing left and right trust
        t_l = 0.0
        t_r = 0.0

        # Simple control logic:
        #  - Drive forward for 20 seconds
        #  - Then turn right by increasing starboard thrust
        if t < 20.0:
            t_l = 20.0
            t_r = 20.0
        else:
            t_l = 15.0
            t_r = 25.0

        msg = Float32MultiArray()
        msg.data = [float(t_l), float(t_r)]
        self.thrust_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
