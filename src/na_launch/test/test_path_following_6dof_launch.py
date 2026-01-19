import math
import os
import time
import unittest

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from na_msg.msg import BoatState, BsplinePath


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


def _publish_path_for(node, msg, duration_sec):
    publisher = node.create_publisher(BsplinePath, "/test_path", 10)
    end_time = time.monotonic() + duration_sec
    try:
        while time.monotonic() < end_time and rclpy.ok():
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_publisher(publisher)


def _straight_path():
    msg = BsplinePath()
    msg.ctrl_x = [0.0, 5.0, 10.0, 15.0]
    msg.ctrl_y = [0.0, 0.0, 0.0, 0.0]
    msg.ctrl_z = [0.0, 0.0, 0.0, 0.0]
    msg.degree = 3
    msg.closed = False
    msg.start_u = 0.0
    return msg


def _circular_path(radius=6.0):
    angles = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi]
    msg = BsplinePath()
    msg.ctrl_x = [radius * math.cos(a) for a in angles]
    msg.ctrl_y = [radius * math.sin(a) for a in angles]
    msg.ctrl_z = [0.0 for _ in angles]
    msg.degree = 3
    msg.closed = True
    msg.start_u = 0.0
    return msg


def _has_forward_speed(msg):
    return msg.u > 0.01


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

    controller_node = launch_ros.actions.Node(
        package="na_controller",
        executable="controller_node",
        name="controller_node",
        namespace="controller_ns",
        parameters=[
            {"config_path": config_path},
            {"path_topic": "/test_path"},
        ],
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
                controller_node,
                sim_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestPathFollowing6DofLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("path_following_6dof_test_listener")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_path_following_response(self):
        _publish_path_for(self.node, _straight_path(), duration_sec=1.0)
        straight_state = _wait_for_message(
            self.node,
            "/boat_state",
            BoatState,
            timeout_sec=8.0,
            predicate=_has_forward_speed,
        )
        self.assertIsNotNone(straight_state, "No forward motion on straight path")
        self.assertLess(
            abs(straight_state.r),
            0.08,
            "Yaw rate too large on straight path",
        )

        _publish_path_for(self.node, _circular_path(), duration_sec=1.0)
        turn_state = _wait_for_message(
            self.node,
            "/boat_state",
            BoatState,
            timeout_sec=8.0,
            predicate=lambda msg: abs(msg.r) > 0.02,
        )
        self.assertIsNotNone(turn_state, "No turning response on circular path")
