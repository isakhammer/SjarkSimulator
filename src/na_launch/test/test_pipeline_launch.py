import math
import os
import time
import unittest

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from na_msg.msg import BoatState, BsplinePath, ControllerState
from std_msgs.msg import Float32MultiArray


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


def _count_messages(
    node, topic, msg_type, timeout_sec, min_count=2, predicate=None
):
    counter = {"count": 0}

    def callback(msg):
        if predicate is None or predicate(msg):
            counter["count"] += 1

    subscription = node.create_subscription(msg_type, topic, callback, 10)
    end_time = time.monotonic() + timeout_sec
    try:
        while (
            time.monotonic() < end_time
            and rclpy.ok()
            and counter["count"] < min_count
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
    return counter["count"]


def _has_thrust(msg):
    if len(msg.data) < 2:
        return False
    return abs(msg.data[0]) > 0.1 or abs(msg.data[1]) > 0.1


def _has_forward_speed(msg):
    u = msg.u
    return u > 0.02


def _path_length(msg):
    count = min(len(msg.ctrl_x), len(msg.ctrl_y))
    if count < 2:
        return 0.0
    length = 0.0
    for i in range(1, count):
        dx = msg.ctrl_x[i] - msg.ctrl_x[i - 1]
        dy = msg.ctrl_y[i] - msg.ctrl_y[i - 1]
        length += math.hypot(dx, dy)
    if msg.closed and count > 2:
        dx = msg.ctrl_x[0] - msg.ctrl_x[-1]
        dy = msg.ctrl_y[0] - msg.ctrl_y[-1]
        length += math.hypot(dx, dy)
    return length


@pytest.mark.launch_test
def generate_test_description():
    config_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "sim_controller_params.yaml",
        )
    )

    planner_node = launch_ros.actions.Node(
        package="na_planner",
        executable="planner_node",
        name="planner_node",
        namespace="planner_ns",
    )
    controller_node = launch_ros.actions.Node(
        package="na_controller",
        executable="controller_node",
        name="controller_node",
        namespace="controller_ns",
        parameters=[{"config_path": config_path}],
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
                planner_node,
                controller_node,
                sim_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestPipelineLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("pipeline_launch_test_listener")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_path_is_published(self):
        msg = _wait_for_message(
            self.node, "/planner_ns/path", BsplinePath, timeout_sec=5.0
        )
        self.assertIsNotNone(msg, "No planner path received")
        self.assertGreaterEqual(
            len(msg.ctrl_x), 4, "Planner path has too few points"
        )
        self.assertEqual(
            len(msg.ctrl_x),
            len(msg.ctrl_y),
            "Planner path has mismatched control arrays",
        )
        self.assertEqual(
            len(msg.ctrl_x),
            len(msg.ctrl_z),
            "Planner path has mismatched control arrays",
        )
        self.assertEqual(msg.degree, 3, "Planner path degree is unexpected")
        self.assertTrue(msg.closed, "Planner path is not marked closed")
        self.assertTrue(
            math.isfinite(msg.start_u),
            "Planner path has non-finite start_u",
        )
        self.assertGreater(
            _path_length(msg), 0.1, "Planner path length is too short"
        )

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

    def test_controller_outputs_thrust(self):
        msg = _wait_for_message(
            self.node,
            "/cmd_thrust",
            Float32MultiArray,
            timeout_sec=5.0,
            predicate=_has_thrust,
        )
        self.assertIsNotNone(msg, "Controller did not output thrust")
        self.assertTrue(
            all(math.isfinite(value) for value in msg.data[:2]),
            "Controller thrust contains non-finite values",
        )

    def test_controller_state_published(self):
        msg = _wait_for_message(
            self.node,
            "/controller_ns/controller_state",
            ControllerState,
            timeout_sec=5.0,
        )
        self.assertIsNotNone(msg, "No controller state received")
        self.assertTrue(
            all(
                math.isfinite(value)
                for value in (
                    msg.cte,
                    msg.heading_error,
                    msg.target_x,
                    msg.target_y,
                    msg.proj_x,
                    msg.proj_y,
                    msg.proj_yaw,
                )
            ),
            "Controller state contains non-finite values",
        )
        self.assertLessEqual(
            abs(msg.heading_error),
            math.pi + 1e-3,
            "Controller heading error is out of bounds",
        )

    def test_topics_publish_multiple_messages(self):
        path_count = _count_messages(
            self.node,
            "/planner_ns/path",
            BsplinePath,
            timeout_sec=3.0,
            min_count=2,
        )
        self.assertGreaterEqual(
            path_count, 2, "Planner path did not update"
        )
        state_count = _count_messages(
            self.node,
            "/boat_state",
            BoatState,
            timeout_sec=3.0,
            min_count=2,
        )
        self.assertGreaterEqual(
            state_count, 2, "Simulator state did not update"
        )
        thrust_count = _count_messages(
            self.node,
            "/cmd_thrust",
            Float32MultiArray,
            timeout_sec=3.0,
            min_count=2,
        )
        self.assertGreaterEqual(
            thrust_count, 2, "Controller thrust did not update"
        )

    def test_vehicle_moves_under_thrust(self):
        msg = _wait_for_message(
            self.node,
            "/cmd_thrust",
            Float32MultiArray,
            timeout_sec=5.0,
            predicate=_has_thrust,
        )
        self.assertIsNotNone(msg, "Controller did not output thrust")
        state = _wait_for_message(
            self.node,
            "/boat_state",
            BoatState,
            timeout_sec=5.0,
            predicate=_has_forward_speed,
        )
        self.assertIsNotNone(state, "Simulator state did not show vx > 0")
