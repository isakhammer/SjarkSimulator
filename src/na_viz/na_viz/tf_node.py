import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from na_msg.msg import ControllerState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BoatTfBroadcaster(Node):
    def __init__(self):
        super().__init__("boat_tf_broadcaster")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("frenet_frame", "path_frenet")
        self.declare_parameter("flip_tf_yaw", False)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.frenet_frame = str(self.get_parameter("frenet_frame").value)
        self.flip_tf_yaw = bool(self.get_parameter("flip_tf_yaw").value)

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 20
        )
        self.sub_controller = self.create_subscription(
            ControllerState, "/controller_ns/controller_state", self.controller_state_cb, 20
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Boat TF broadcaster started (ENU)")

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def controller_state_cb(self, msg):
        yaw = float(msg.proj_yaw)
        if self.flip_tf_yaw:
            yaw = -yaw
        qz = float(math.sin(yaw / 2.0))
        qw = float(math.cos(yaw / 2.0))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.frenet_frame
        t.transform.translation.x = float(msg.proj_x)
        t.transform.translation.y = float(msg.proj_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BoatTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
