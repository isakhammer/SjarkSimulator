#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

from na_utils.bspline import (
    BSplinePath,
    ProjectionTracker,
    samples_from_density,
)
from na_utils.ros_params import load_ros_params
from na_msg.msg import BoatState, BsplinePath, ControllerState


class ControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        default_config = os.path.join(
            get_package_share_directory("na_launch"),
            "config",
            "sim_controller_params.yaml",
        )
        self.declare_parameter("config_path", default_config)
        config_path = self.get_parameter("config_path").value

        defaults = {
            "path_topic": "/planner_ns/path",
            "lookahead": 3.0,
            "base_thrust": 25.0,
            "heading_kp": 2.0,
            "heading_kd": 0.5,
            "max_thrust": 40.0,
            "max_delta": math.pi / 2.0,
            "samples_per_meter": 4.0,
            "max_proj_jump": 0.2,
        }
        defaults = load_ros_params(
            config_path, "controller_node", defaults, logger=self.get_logger()
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # Publisher: rotor command [thrust, delta]
        self.thrust_pub = self.create_publisher(
            Float32MultiArray, "/cmd_thrust", 10
        )

        # Subscriber: boat state
        self.state_sub = self.create_subscription(
            BoatState, "/boat_state", self.state_callback, 10
        )

        # Subscriber: planner spline path
        path_topic = self.get_parameter("path_topic").value
        self.path_sub = self.create_subscription(
            BsplinePath, path_topic, self.path_callback, 10
        )

        # Publisher: controller state (debug)
        self.state_pub = self.create_publisher(
            ControllerState, "controller_state", 10
        )

        # Control loop timer
        self.timer = self.create_timer(0.02, self.control_loop)

        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,psi,u,v,r
        self.spline = None
        self.projection_tracker = ProjectionTracker()
        self._path_signature = None
        self._last_time = None
        self.get_logger().info("Boat LOS controller started")

    def state_callback(self, msg: BoatState) -> None:
        self.state = [
            float(msg.x),
            float(msg.y),
            float(msg.yaw),
            float(msg.u),
            float(msg.v),
            float(msg.r),
        ]

    def path_callback(self, msg: BsplinePath) -> None:
        n = len(msg.ctrl_x)
        if n < 4 or n != len(msg.ctrl_y):
            self.get_logger().warn("Received invalid spline path")
            self.spline = None
            self.projection_tracker.reset()
            self._path_signature = None
            return

        new_points = [(msg.ctrl_x[i], msg.ctrl_y[i]) for i in range(n)]
        samples_per_meter = float(
            self.get_parameter("samples_per_meter").value
        )
        signature = (
            tuple(new_points),
            bool(msg.closed),
            float(msg.start_u),
            samples_per_meter,
        )
        if self._path_signature == signature:
            return
        self._path_signature = signature
        samples = samples_from_density(
            new_points,
            samples_per_meter,
            closed=bool(msg.closed),
        )

        self.spline = BSplinePath(
            new_points, msg.start_u, samples, closed=bool(msg.closed)
        )
        self.projection_tracker.reset()

    def wrap_to_pi(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def control_loop(self) -> None:
        x, y, psi, u, v, r = self.state
        now = self.get_clock().now()
        dt = None
        if self._last_time is not None:
            dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if not self.spline or self.spline.empty():
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.thrust_pub.publish(msg)
            return

        params = {
            p.name: p.value
            for p in self.get_parameters(
                (
                    "lookahead",
                    "base_thrust",
                    "heading_kp",
                    "heading_kd",
                    "max_thrust",
                    "max_delta",
                    "max_proj_jump",
                )
            )
        }
        lookahead = float(params["lookahead"])
        max_proj_jump = float(params["max_proj_jump"])
        proj_t = self.projection_tracker.project_t(
            self.spline,
            x,
            y,
            max_jump=max_proj_jump,
            psi=psi,
            u=u,
            v=v,
            dt=dt,
        )
        if proj_t is None:
            return
        proj_sample = self.spline.sample_at_t(proj_t)
        proj_x, proj_y = proj_sample.point
        nx, ny = proj_sample.normal
        cte = (x - proj_x) * nx + (y - proj_y) * ny
        tx, ty = proj_sample.tangent
        if abs(tx) < 1e-6 and abs(ty) < 1e-6:
            proj_yaw = 0.0
        else:
            proj_yaw = math.atan2(ty, tx)

        target_t = self.spline.advance_t(proj_t, lookahead)
        target_sample = self.spline.sample_at_t(target_t)
        target_x, target_y = target_sample.point

        desired_heading = self.wrap_to_pi(
            proj_yaw - math.atan2(cte, lookahead)
        )
        heading_error = self.wrap_to_pi(desired_heading - psi)

        base_thrust = float(params["base_thrust"])
        heading_kp = float(params["heading_kp"])
        heading_kd = float(params["heading_kd"])
        max_thrust = float(params["max_thrust"])
        max_delta = float(params["max_delta"])

        thrust = max(-max_thrust, min(max_thrust, base_thrust))
        delta = -heading_kp * heading_error - heading_kd * r
        delta = max(-max_delta, min(max_delta, delta))

        state_msg = ControllerState()
        state_msg.cte = float(cte)
        state_msg.heading_error = float(heading_error)
        state_msg.target_x = float(target_x)
        state_msg.target_y = float(target_y)
        state_msg.proj_x = float(proj_x)
        state_msg.proj_y = float(proj_y)
        state_msg.proj_yaw = float(proj_yaw)
        self.state_pub.publish(state_msg)

        msg = Float32MultiArray()
        msg.data = [float(thrust), float(delta)]
        self.thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
