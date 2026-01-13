#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

from na_utils.bspline import BSplinePath, samples_from_density
from na_utils.ros_params import load_ros_params
from na_msg.msg import BsplinePath, ControllerState


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
            "heading_kp": 10.0,
            "heading_kd": 2.0,
            "max_thrust": 40.0,
            "max_delta": 15.0,
            "spline_samples": 400,
            "samples_per_meter": 4.0,
        }
        defaults = load_ros_params(
            config_path, "controller_node", defaults, logger=self.get_logger()
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # Publisher: thrust command [T_L, T_R]
        self.thrust_pub = self.create_publisher(Float32MultiArray, "/cmd_thrust", 10)

        # Subscriber: boat state [x, y, psi, u, v, r]
        self.state_sub = self.create_subscription(
            Float32MultiArray, "/boat_state", self.state_callback, 10
        )

        # Subscriber: planner spline path
        path_topic = self.get_parameter("path_topic").value
        self.path_sub = self.create_subscription(
            BsplinePath, path_topic, self.path_callback, 10
        )

        # Publisher: controller state (debug)
        self.state_pub = self.create_publisher(ControllerState, "controller_state", 10)

        # Control loop timer
        self.timer = self.create_timer(0.02, self.control_loop)

        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,psi,u,v,r
        self.spline = None
        self.last_proj_t = None
        self.get_logger().info("Boat LOS controller started")

    def state_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.state = list(msg.data)

    def path_callback(self, msg: BsplinePath) -> None:
        n = len(msg.ctrl_x)
        if n < 4 or n != len(msg.ctrl_y):
            self.get_logger().warn("Received invalid spline path")
            self.spline = None
            self.last_proj_t = None
            return

        new_points = [(msg.ctrl_x[i], msg.ctrl_y[i]) for i in range(n)]
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        if samples_per_meter > 0.0:
            samples = samples_from_density(
                new_points,
                samples_per_meter,
                closed=bool(msg.closed),
            )
        else:
            samples = int(self.get_parameter("spline_samples").value)
            if samples < 50:
                samples = 50

        self.spline = BSplinePath(
            new_points, msg.start_u, samples, closed=bool(msg.closed)
        )
        self.last_proj_t = None

    def wrap_to_pi(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def control_loop(self) -> None:
        x, y, psi, _, _, r = self.state

        if not self.spline or self.spline.empty():
            t_l = 0.0
            t_r = 0.0
            msg = Float32MultiArray()
            msg.data = [float(t_l), float(t_r)]
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
                )
            )
        }
        lookahead = float(params["lookahead"])
        projection = self.spline.project(x, y, hint_t=self.last_proj_t)
        if projection is None:
            return
        self.last_proj_t = projection.t
        proj_sample = self.spline.sample_at_t(projection.t)
        proj_x, proj_y = proj_sample.point
        cte = projection.cte
        tx, ty = proj_sample.tangent
        if abs(tx) < 1e-6 and abs(ty) < 1e-6:
            proj_yaw = 0.0
        else:
            proj_yaw = math.atan2(ty, tx)

        target_t = self.spline.advance_t(projection.t, lookahead)
        target_sample = self.spline.sample_at_t(target_t)
        target_x, target_y = target_sample.point

        desired = math.atan2(target_y - y, target_x - x)
        err = self.wrap_to_pi(desired - psi)

        base_thrust = float(params["base_thrust"])
        heading_kp = float(params["heading_kp"])
        heading_kd = float(params["heading_kd"])
        max_thrust = float(params["max_thrust"])
        max_delta = float(params["max_delta"])

        delta = heading_kp * err - heading_kd * r
        delta = max(-max_delta, min(max_delta, delta))

        t_l = base_thrust - delta
        t_r = base_thrust + delta
        t_l = max(-max_thrust, min(max_thrust, t_l))
        t_r = max(-max_thrust, min(max_thrust, t_r))

        state_msg = ControllerState()
        state_msg.cte = float(cte)
        state_msg.heading_error = float(err)
        state_msg.target_x = float(target_x)
        state_msg.target_y = float(target_y)
        state_msg.proj_x = float(proj_x)
        state_msg.proj_y = float(proj_y)
        state_msg.proj_yaw = float(proj_yaw)
        self.state_pub.publish(state_msg)

        msg = Float32MultiArray()
        msg.data = [float(t_l), float(t_r)]
        self.thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
