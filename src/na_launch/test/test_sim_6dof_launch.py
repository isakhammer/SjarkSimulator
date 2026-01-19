import math
import os
import time
import unittest

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from na_msg.msg import BoatState
from tf2_msgs.msg import TFMessage


def _wait_for_message(node, topic, msg_type, timeout_sec, predicate=None):
    received = {"msg": None}

    def callback(msg):
        received["msg"] = msg

    subscription = node.create_subscription(msg_type, topic, callback, 10)
    end_time = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            msg = received["msg"]
            if msg is not None and (predicate is None or predicate(msg)):
                return msg
    finally:
        node.destroy_subscription(subscription)
    return None


def _quat_norm(orientation):
    return math.sqrt(
        orientation.w * orientation.w
        + orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
    )


def _has_base_link_tf(msg):
    for transform in msg.transforms:
        if transform.child_frame_id == "base_link":
            return True
    return False


@pytest.mark.launch_test
def generate_test_description():
    config_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "sim_controller_params_6dof.yaml",
        )
    )

    sim_node = launch_ros.actions.Node(
        package="na_sim",
        executable="sim_node",
        name="sim_node",
        namespace="sim_ns",
        parameters=[{"config_path": config_path}],
    )

    return (
        launch.LaunchDescription(
            [
                sim_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestSim6DofLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sim_6dof_launch_test_listener")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_simulator_publishes_state(self):
        msg = _wait_for_message(
            self.node, "/boat_state", BoatState, timeout_sec=5.0
        )
        self.assertIsNotNone(msg, "No simulator state received")
        self.assertTrue(
            all(
                math.isfinite(value)
                for value in (msg.x, msg.y, msg.yaw, msg.u, msg.v, msg.r)
            ),
            "Simulator state contains non-finite values",
        )

    def test_odometry_has_unit_quaternion(self):
        msg = _wait_for_message(
            self.node, "/odom", Odometry, timeout_sec=5.0
        )
        self.assertIsNotNone(msg, "No odometry received")
        norm = _quat_norm(msg.pose.pose.orientation)
        self.assertTrue(math.isfinite(norm), "Odometry quaternion norm is NaN")
        self.assertAlmostEqual(norm, 1.0, delta=1e-3)

    def test_odometry_twist_is_finite(self):
        msg = _wait_for_message(
            self.node, "/odom", Odometry, timeout_sec=5.0
        )
        self.assertIsNotNone(msg, "No odometry received")
        values = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )
        self.assertTrue(
            all(math.isfinite(value) for value in values),
            "Odometry twist contains non-finite values",
        )

    def test_robot_state_publisher_tf(self):
        msg = _wait_for_message(
            self.node, "/tf", TFMessage, timeout_sec=5.0, predicate=_has_base_link_tf
        )
        self.assertIsNotNone(msg, "No TF with base_link received")
