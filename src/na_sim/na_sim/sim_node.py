import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from ament_index_python.packages import get_package_share_directory

from na_sim.Boat3DOF import Boat3DOF
from na_utils.ros_params import load_ros_params


class BoatSimNode(Node):
    def __init__(self):
        super().__init__("boat_sim")

        # Parameters
        default_config = os.path.join(
            get_package_share_directory("na_launch"),
            "config",
            "sim_controller_params.yaml",
        )
        self.declare_parameter("config_path", default_config)
        config_path = (
            self.get_parameter("config_path")
            .get_parameter_value()
            .string_value
        )

        defaults = {
            "m": 70.0,
            "Iz": 10.0,
            "Xu": 5.0,
            "Xuu": 1.0,
            "Yv": 40.0,
            "Yr": 5.0,
            "Nv": 5.0,
            "Nr": 40.0,
            "l": 0.5,
            "dt": 0.05,
            "max_thrust": 40.0,
            "max_delta": 1.57079632679,
            "max_delta_rate": 0.0,
        }
        defaults = load_ros_params(
            config_path, "sim_node", defaults, logger=self.get_logger()
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        param_names = ("m", "Iz", "Xu", "Xuu", "Yv", "Yr", "Nv", "Nr", "l")
        params = {
            name: float(self.get_parameter(name).value)
            for name in param_names
        }

        self.sim = Boat3DOF(params)

        # Rotor inputs (thrust magnitude + steering angle)
        self.thrust_cmd = 0.0
        self.delta_cmd = 0.0
        self.thrust = 0.0
        self.delta = 0.0

        # Subscribers
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            "/cmd_thrust",
            self.cmd_callback,
            10
        )

        # Publishers
        self.pub_state = self.create_publisher(
            Float32MultiArray, "/boat_state", 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Time step
        self.dt = float(self.get_parameter("dt").value)
        timer_period = self.dt if self.dt > 0.0 else 0.02
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info("Boat 3DOF simulator with Odometry started.")

    def cmd_callback(self, msg):
        if len(msg.data) >= 2:
            self.thrust_cmd = float(msg.data[0])
            self.delta_cmd = float(msg.data[1])

    def update(self):
        max_thrust = float(self.get_parameter("max_thrust").value)
        max_delta = float(self.get_parameter("max_delta").value)
        max_delta_rate = float(self.get_parameter("max_delta_rate").value)

        thrust_cmd = max(-max_thrust, min(max_thrust, self.thrust_cmd))
        delta_cmd = max(-max_delta, min(max_delta, self.delta_cmd))
        if max_delta_rate > 0.0 and self.dt > 0.0:
            max_step = max_delta_rate * self.dt
            delta_err = delta_cmd - self.delta
            delta_step = max(-max_step, min(max_step, delta_err))
            self.delta = self.delta + delta_step
        else:
            self.delta = delta_cmd
        self.thrust = thrust_cmd

        # Run physics
        self.sim.step(self.thrust, self.delta, self.dt)
        x, y, psi, u, v, r = self.sim.state

        # Publish /boat_state
        state_msg = Float32MultiArray()
        state_msg.data = [x, y, psi, u, v, r]
        self.pub_state.publish(state_msg)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion
        qz = np.sin(psi / 2.0)
        qw = np.cos(psi / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = u
        odom.twist.twist.linear.y = v
        odom.twist.twist.angular.z = r

        self.pub_odom.publish(odom)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BoatSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
