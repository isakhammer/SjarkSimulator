import math
import os
import time
import unittest

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from na_msg.msg import BoatState, BsplinePath, ControllerState, RotorCommand
from na_utils.bspline import BSplinePath


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


def _circular_path(radius=6.0, center_x=0.0, center_y=None):
    if center_y is None:
        center_y = radius
    angles = [-0.5 * math.pi, 0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]
    msg = BsplinePath()
    msg.ctrl_x = [center_x + radius * math.cos(a) for a in angles]
    msg.ctrl_y = [center_y + radius * math.sin(a) for a in angles]
    msg.ctrl_z = [0.0 for _ in angles]
    msg.degree = 3
    msg.closed = True
    msg.start_u = 0.0
    return msg


def _has_forward_speed(msg):
    return msg.u > 0.01


def _path_yaw_sign(path, proj_x, proj_y):
    proj = path.project(proj_x, proj_y)
    if proj is None or path.length <= 0.0:
        return 0
    step = max(0.05, 0.01 * path.length)
    t0 = proj.t
    t1 = path.advance_t(t0, step)
    s0 = path.sample_at_t(t0)
    s1 = path.sample_at_t(t1)
    yaw0 = -math.atan2(s0.tangent[1], s0.tangent[0])
    yaw1 = -math.atan2(s1.tangent[1], s1.tangent[0])
    dyaw = (yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
    if abs(dyaw) < 1e-6:
        return 0
    return 1 if dyaw > 0.0 else -1


def _wait_for_tracking_turn(
    node,
    timeout_sec=8.0,
    cte_tol=0.5,
    r_min=0.02,
    delta_min=0.02,
):
    last_state = {"boat": None, "ctrl": None, "cmd": None}

    def boat_cb(msg):
        last_state["boat"] = msg

    def ctrl_cb(msg):
        last_state["ctrl"] = msg

    def cmd_cb(msg):
        last_state["cmd"] = msg

    sub_boat = node.create_subscription(BoatState, "/boat_state", boat_cb, 10)
    sub_ctrl = node.create_subscription(
        ControllerState, "/controller_ns/controller_state", ctrl_cb, 10
    )
    sub_cmd = node.create_subscription(RotorCommand, "/cmd_rotor", cmd_cb, 10)
    end_time = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            boat = last_state["boat"]
            ctrl = last_state["ctrl"]
            cmd = last_state["cmd"]
            if boat is None or ctrl is None or cmd is None:
                continue
            if abs(ctrl.cte) > cte_tol:
                continue
            if abs(boat.r) <= r_min:
                continue
            if abs(boat.delta) <= delta_min:
                continue
            return boat, ctrl
    finally:
        node.destroy_subscription(sub_boat)
        node.destroy_subscription(sub_ctrl)
        node.destroy_subscription(sub_cmd)
    return None, None


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

        circle_msg = _circular_path()
        _publish_path_for(self.node, circle_msg, duration_sec=1.0)
        boat_state, ctrl_state = _wait_for_tracking_turn(self.node)
        self.assertIsNotNone(boat_state, "No tracking turn state on circular path")
        self.assertIsNotNone(ctrl_state, "No controller state during circular path")
        path = BSplinePath(
            list(zip(circle_msg.ctrl_x, circle_msg.ctrl_y)),
            start_u=circle_msg.start_u,
            samples=400,
            closed=circle_msg.closed,
        )
        expected_sign = _path_yaw_sign(path, ctrl_state.proj_x, ctrl_state.proj_y)
        self.assertNotEqual(
            expected_sign,
            0,
            "Could not infer path yaw sign on circular path",
        )
        actual_sign = 1 if boat_state.r > 0.0 else -1
        self.assertEqual(
            actual_sign,
            expected_sign,
            "Yaw rate sign is inconsistent with circular path direction",
        )
        self.assertGreater(
            boat_state.r * boat_state.delta,
            0.0,
            "Yaw rate sign should match steering angle under ENU/FLU convention",
        )
