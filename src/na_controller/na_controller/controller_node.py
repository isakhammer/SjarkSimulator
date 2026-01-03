#!/usr/bin/env python3

import bisect
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from na_utils.bspline import build_spline_samples
from na_msg.msg import BsplinePath, ControllerState


class ControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Publisher: thrust command [T_L, T_R]
        self.thrust_pub = self.create_publisher(Float32MultiArray, "/cmd_thrust", 10)

        # Subscriber: boat state [x, y, psi, u, v, r]
        self.state_sub = self.create_subscription(
            Float32MultiArray, "/boat_state", self.state_callback, 10
        )

        # Subscriber: planner spline path
        self.declare_parameter("path_topic", "/planner_ns/path")
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.path_sub = self.create_subscription(
            BsplinePath, path_topic, self.path_callback, 10
        )

        # Publisher: controller state (debug)
        self.state_pub = self.create_publisher(ControllerState, "controller_state", 10)

        # Control loop timer
        self.timer = self.create_timer(0.02, self.control_loop)

        # LOS controller parameters
        self.declare_parameter("lookahead", 3.0)
        self.declare_parameter("base_thrust", 20.0)
        self.declare_parameter("heading_kp", 8.0)
        self.declare_parameter("heading_kd", 2.0)
        self.declare_parameter("max_thrust", 40.0)
        self.declare_parameter("max_delta", 15.0)
        self.declare_parameter("spline_samples", 400)

        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,psi,u,v,r
        self.control_points = []
        self.start_u = 0.0
        self.samples = []
        self.sample_u = []
        self.sample_s = []
        self.spline_closed = True
        self.get_logger().info("Boat LOS controller started")

    def state_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.state = list(msg.data)

    def path_callback(self, msg: BsplinePath) -> None:
        n = len(msg.ctrl_x)
        if n < 4 or n != len(msg.ctrl_y):
            self.get_logger().warn("Received invalid spline path")
            self.control_points = []
            self.samples = []
            self.sample_u = []
            self.sample_s = []
            return

        new_points = [(msg.ctrl_x[i], msg.ctrl_y[i]) for i in range(n)]
        samples = int(self.get_parameter("spline_samples").get_parameter_value().integer_value)
        if samples < 50:
            samples = 50

        self.control_points = new_points
        self.start_u = msg.start_u
        self.spline_closed = bool(msg.closed)
        self.samples, self.sample_u, self.sample_s = build_spline_samples(
            self.control_points,
            self.start_u,
            samples,
            closed=self.spline_closed,
        )

    def wrap_to_pi(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def control_loop(self) -> None:
        x, y, psi, _, _, r = self.state

        if not self.samples:
            t_l = 0.0
            t_r = 0.0
            msg = Float32MultiArray()
            msg.data = [float(t_l), float(t_r)]
            self.thrust_pub.publish(msg)
            return

        lookahead = self.get_parameter("lookahead").get_parameter_value().double_value
        m = len(self.samples)
        best_idx = 0
        best_d2 = float("inf")
        for i in range(m):
            dx = x - self.samples[i][0]
            dy = y - self.samples[i][1]
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i

        proj_x, proj_y = self.samples[best_idx]
        if self.spline_closed:
            prev_idx = best_idx - 1 if best_idx > 0 else m - 1
            next_idx = best_idx + 1 if best_idx < m - 1 else 0
        else:
            prev_idx = max(0, best_idx - 1)
            next_idx = min(m - 1, best_idx + 1)
        tx = self.samples[next_idx][0] - self.samples[prev_idx][0]
        ty = self.samples[next_idx][1] - self.samples[prev_idx][1]
        t_norm = math.hypot(tx, ty)
        if t_norm < 1e-6:
            nx, ny = 0.0, 0.0
        else:
            tx /= t_norm
            ty /= t_norm
            nx, ny = -ty, tx

        cte = (x - proj_x) * nx + (y - proj_y) * ny

        total_length = self.sample_s[-1]
        s_target = self.sample_s[best_idx] + lookahead
        if self.spline_closed and s_target > total_length and total_length > 0.0:
            s_target = s_target - total_length
        elif not self.spline_closed:
            s_target = min(s_target, total_length)

        idx2 = bisect.bisect_left(self.sample_s, s_target)
        if idx2 <= 0:
            target_x, target_y = self.samples[0]
        elif idx2 >= m:
            target_x, target_y = self.samples[-1]
        else:
            s0 = self.sample_s[idx2 - 1]
            s1 = self.sample_s[idx2]
            if s1 - s0 < 1e-6:
                alpha = 0.0
            else:
                alpha = (s_target - s0) / (s1 - s0)
            x0, y0 = self.samples[idx2 - 1]
            x1, y1 = self.samples[idx2]
            target_x = x0 + alpha * (x1 - x0)
            target_y = y0 + alpha * (y1 - y0)

        desired = math.atan2(target_y - y, target_x - x)
        err = self.wrap_to_pi(desired - psi)

        base_thrust = self.get_parameter("base_thrust").get_parameter_value().double_value
        heading_kp = self.get_parameter("heading_kp").get_parameter_value().double_value
        heading_kd = self.get_parameter("heading_kd").get_parameter_value().double_value
        max_thrust = self.get_parameter("max_thrust").get_parameter_value().double_value
        max_delta = self.get_parameter("max_delta").get_parameter_value().double_value

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
