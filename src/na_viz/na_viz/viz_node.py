import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from na_msg.msg import Path as NaPath
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

import numpy as np


class BoatVisualizer(Node):
    def __init__(self):
        super().__init__("boat_visualizer")

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 20)
        self.sub_path = self.create_subscription(
            NaPath, "/planner_ns/path", self.path_cb, 20)

        # Publishers
        self.pub_path_trace = self.create_publisher(Path, "/viz/path_trace", 10)
        self.pub_boat = self.create_publisher(Marker, "/viz/boat_marker", 10)
        self.pub_heading = self.create_publisher(Marker, "/viz/heading_marker", 10)
        self.pub_planner_path = self.create_publisher(Marker, "/viz/path", 10)

        # Internal path storage
        self.path_trace = Path()
        self.path_trace.header.frame_id = "map"

        self.get_logger().info("BoatVisualizer started (no thrust arrows)")


    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def path_cb(self, msg):
        n = len(msg.wpts_x)
        if n < 2 or n != len(msg.wpts_y) or n != len(msg.wpts_z):
            self.get_logger().error("Invalid planner path")
            return
    
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
    
        marker.ns = "path_viz"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
    
        marker.scale.x = 0.05
    
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0
    
        for i in range(n):
            p = Point()
            p.x = msg.wpts_x[i]
            p.y = msg.wpts_y[i]
            p.z = msg.wpts_z[i]
            marker.points.append(p)

        self.pub_planner_path.publish(marker)


    # --------------------------------------------------------
    # PATH TRACE
    # --------------------------------------------------------
    def update_path_trace(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path_trace.header.stamp = msg.header.stamp
        self.path_trace.poses.append(pose)
        self.pub_path_trace.publish(self.path_trace)

    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def odom_cb(self, msg):
        self.update_path_trace(msg)
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

        p1 = Point()
        p1.x = x
        p1.y = y
        p1.z = 0.15
        
        p2 = Point()
        p2.x = x + np.cos(psi)
        p2.y = y + np.sin(psi)
        p2.z = 0.15


        arrow.points.append(p1)
        arrow.points.append(p2)

        self.pub_heading.publish(arrow)


def main(args=None):
    rclpy.init(args=args)
    node = BoatVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
