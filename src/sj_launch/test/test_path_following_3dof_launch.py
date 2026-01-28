import math
import os
import time
import unittest

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from sj_msg.msg import BoatState, BsplinePath, ControllerState, RotorCommand

# -1 means yaw is clockwise-positive (legacy NED); set +1 for CCW-positive ENU.
YAW_SIGN_FACTOR = float(os.environ.get("SJ_YAW_SIGN_FACTOR", "-1.0"))
STRAIGHT_R_MAX = float(os.environ.get("SJ_STRAIGHT_R_MAX", "0.3"))
STRAIGHT_PUBLISH_SEC = float(os.environ.get("SJ_STRAIGHT_PUBLISH_SEC", "2.0"))
CIRCLE_TEST_SEC = float(os.environ.get("SJ_CIRCLE_TEST_SEC", "10.0"))
CIRCLE_SETTLE_SEC = float(os.environ.get("SJ_CIRCLE_SETTLE_SEC", "3.0"))
CIRCLE_R_MIN = float(os.environ.get("SJ_CIRCLE_R_MIN", "0.001"))
CIRCLE_DELTA_MIN = float(os.environ.get("SJ_CIRCLE_DELTA_MIN", "0.0"))
CIRCLE_CTE_TOL = float(os.environ.get("SJ_CIRCLE_CTE_TOL", "4.0"))
CIRCLE_ALONG_MIN = float(os.environ.get("SJ_CIRCLE_ALONG_MIN", "0.02"))


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
    yaw0 = math.atan2(s0.tangent[1], s0.tangent[0])
    yaw1 = math.atan2(s1.tangent[1], s1.tangent[0])
    dyaw = (yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
    if abs(dyaw) < 1e-6:
        return 0
    return 1 if dyaw > 0.0 else -1


def _collect_rotation_stats(
    node,
    duration_sec,
    *,
    settle_sec=0.0,
    r_min=0.001,
    delta_min=0.0,
    cte_tol=None,
):
    last_state = {"boat": None, "ctrl": None}

    def boat_cb(msg):
        last_state["boat"] = msg

    def ctrl_cb(msg):
        last_state["ctrl"] = msg

    sub_boat = node.create_subscription(BoatState, "/boat_state", boat_cb, 10)
    sub_ctrl = node.create_subscription(
        ControllerState, "/controller_ns/controller_state", ctrl_cb, 10
    )

    start = time.monotonic()
    end_time = start + duration_sec
    sample_start = start + max(0.0, settle_sec)
    total = 0
    positive = 0
    negative = 0
    delta_positive = 0
    delta_negative = 0
    steering_matches = 0
    try:
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() < sample_start:
                continue
            boat = last_state["boat"]
            ctrl = last_state["ctrl"]
            if boat is None or ctrl is None:
                continue
            if cte_tol is not None and abs(ctrl.cte) > cte_tol:
                continue
            if abs(boat.r) < r_min or abs(boat.delta) < delta_min:
                continue
            total += 1
            if boat.r > 0.0:
                positive += 1
            else:
                negative += 1
            if boat.delta > 0.0:
                delta_positive += 1
            else:
                delta_negative += 1
            if boat.r * boat.delta < 0.0:
                steering_matches += 1
    finally:
        node.destroy_subscription(sub_boat)
        node.destroy_subscription(sub_ctrl)

    return {
        "total": total,
        "positive": positive,
        "negative": negative,
        "delta_positive": delta_positive,
        "delta_negative": delta_negative,
        "steering_matches": steering_matches,
    }


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


def _collect_min_abs_r(node, duration_sec):
    received = {"min_abs_r": None}

    def boat_cb(msg):
        value = abs(float(msg.r))
        current = received["min_abs_r"]
        received["min_abs_r"] = value if current is None else min(current, value)

    subscription = node.create_subscription(BoatState, "/boat_state", boat_cb, 10)
    end_time = time.monotonic() + duration_sec
    try:
        while time.monotonic() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
    return received["min_abs_r"]


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

    controller_node = launch_ros.actions.Node(
        package="sj_controller",
        executable="controller_node",
        name="controller_node",
        namespace="controller_ns",
        parameters=[
            {"config_path": config_path},
            {"path_topic": "/test_path"},
        ],
    )
    sim_node = launch_ros.actions.Node(
        package="sj_sim",
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


class TestPathFollowing3DofLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("path_following_3dof_test_listener")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_path_following_response(self):
        _publish_path_for(
            self.node, _straight_path(), duration_sec=STRAIGHT_PUBLISH_SEC
        )
        straight_state = _wait_for_message(
            self.node,
            "/boat_state",
            BoatState,
            timeout_sec=8.0,
            predicate=_has_forward_speed,
        )
        self.assertIsNotNone(straight_state, "No forward motion on straight path")
        min_abs_r = _collect_min_abs_r(self.node, duration_sec=2.0)
        self.assertIsNotNone(min_abs_r, "No yaw samples collected on straight path")
        self.assertLess(
            min_abs_r,
            STRAIGHT_R_MAX,
            "Yaw rate too large on straight path after settling",
        )

    def test_circular_path_rotation_stability(self):
        circle_msg = _circular_path()
        _publish_path_for(self.node, circle_msg, duration_sec=1.0)
        stats = _collect_rotation_stats(
            self.node,
            CIRCLE_TEST_SEC,
            settle_sec=CIRCLE_SETTLE_SEC,
            r_min=CIRCLE_R_MIN,
            delta_min=CIRCLE_DELTA_MIN,
            cte_tol=CIRCLE_CTE_TOL,
        )
        self.assertGreater(
            stats["total"],
            10,
            "Not enough stabilized samples collected on circular path",
        )
        dominant = max(stats["positive"], stats["negative"]) / max(1, stats["total"])
        self.assertGreater(
            dominant,
            0.7,
            "Yaw rate sign is not consistently oriented on circular path",
        )
        delta_dominant = max(stats["delta_positive"], stats["delta_negative"]) / max(
            1, stats["total"]
        )
        self.assertGreater(
            delta_dominant,
            0.7,
            "Steering angle sign is not consistently oriented on circular path",
        )
        self.assertGreater(
            stats["steering_matches"] / max(1, stats["total"]),
            0.7,
            "Yaw rate sign does not consistently oppose steering angle",
        )
