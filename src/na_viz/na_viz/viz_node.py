import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

import numpy as np


class BoatVisualizer(Node):
    def __init__(self):
        super().__init__("boat_visualizer")

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 20)

        # Publishers
        self.pub_path = self.create_publisher(Path, "/viz/path", 10)
        self.pub_boat = self.create_publisher(Marker, "/viz/boat_marker", 10)
        self.pub_heading = self.create_publisher(Marker, "/viz/heading_marker", 10)

        # Internal path storage
        self.path = Path()
        self.path.header.frame_id = "map"

        self.get_logger().info("BoatVisualizer started (no thrust arrows)")

    # --------------------------------------------------------
    # PATH TRACE
    # --------------------------------------------------------
    def update_path(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        self.pub_path.publish(self.path)

    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def odom_cb(self, msg):
        self.update_path(msg)
        self.publish_boat_marker(msg)
        self.publish_heading_arrow(msg)

    # --------------------------------------------------------
    # BOAT MARKER (cube hull)
    # --------------------------------------------------------
    def publish_boat_marker(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = msg.header.stamp
        m.ns = "boat"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Boat dimensions
        m.scale.x = 1.2  # length
        m.scale.y = 0.6  # width
        m.scale.z = 0.2  # height

        m.color.r = 0.1
        m.color.g = 0.2
        m.color.b = 0.9
        m.color.a = 1.0

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation = q

        self.pub_boat.publish(m)

    # --------------------------------------------------------
    # HEADING ARROW
    # --------------------------------------------------------
    def publish_heading_arrow(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        psi = 2.0 * np.arctan2(qz, qw)

        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = msg.header.stamp
        arrow.ns = "heading"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        arrow.scale.x = 1.2  # arrow length
        arrow.scale.y = 0.15
        arrow.scale.z = 0.15

        arrow.color.r = 1.0
        arrow.color.g = 0.2
        arrow.color.b = 0.2
        arrow.color.a = 1.0

        arrow.points.append(Point(x, y, 0.15))
        arrow.points.append(Point(x + np.cos(psi), y + np.sin(psi), 0.15))

        self.pub_heading.publish(arrow)


def main(args=None):
    rclpy.init(args=args)
    node = BoatVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
