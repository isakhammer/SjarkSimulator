import math
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
from na_sim.fossen import Fossen6DOF
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
            "Ix": 10.0,
            "Iy": 10.0,
            "Xu": 5.0,
            "Xuu": 1.0,
            "Yv": 40.0,
            "Yr": 5.0,
            "Nv": 5.0,
            "Nr": 40.0,
            "Zw": 0.0,
            "Kp": 0.0,
            "Mq": 0.0,
            "Yvv": 0.0,
            "Zww": 0.0,
            "Kpp": 0.0,
            "Mqq": 0.0,
            "Nrr": 0.0,
            "added_mass_x": 0.0,
            "added_mass_y": 0.0,
            "added_mass_z": 0.0,
            "added_mass_k": 0.0,
            "added_mass_m": 0.0,
            "added_mass_n": 0.0,
            "r_g_x": 0.0,
            "r_g_y": 0.0,
            "r_g_z": 0.0,
            "r_b_x": 0.0,
            "r_b_y": 0.0,
            "r_b_z": 0.0,
            "weight": -1.0,
            "buoyancy": -1.0,
            "l": 0.5,
            "dt": 0.05,
            "max_thrust": 40.0,
            "max_delta": 1.57079632679,
            "max_delta_rate": 0.0,
            "sim_model": "3dof",
            "integrator": "rk4",
        }
        defaults = load_ros_params(
            config_path, "sim_node", defaults, logger=self.get_logger()
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        param_names = (
            "m",
            "Ix",
            "Iy",
            "Iz",
            "Xu",
            "Xuu",
            "Yv",
            "Yr",
            "Nv",
            "Nr",
            "Zw",
            "Kp",
            "Mq",
            "Yvv",
            "Zww",
            "Kpp",
            "Mqq",
            "Nrr",
            "added_mass_x",
            "added_mass_y",
            "added_mass_z",
            "added_mass_k",
            "added_mass_m",
            "added_mass_n",
            "r_g_x",
            "r_g_y",
            "r_g_z",
            "r_b_x",
            "r_b_y",
            "r_b_z",
            "weight",
            "buoyancy",
            "l",
        )
        params = {name: float(self.get_parameter(name).value) for name in param_names}
        self.params = params

        self.sim_model = str(self.get_parameter("sim_model").value).lower()
        self.integrator = str(self.get_parameter("integrator").value).lower()

        self.sim = None
        self.state_full = None
        if self.sim_model == "6dof":
            inertia = np.diag([params["Ix"], params["Iy"], params["Iz"]])
            d_linear = np.zeros((6, 6), dtype=float)
            d_linear[0, 0] = params["Xu"]
            d_linear[1, 1] = params["Yv"]
            d_linear[2, 2] = params["Zw"]
            d_linear[3, 3] = params["Kp"]
            d_linear[4, 4] = params["Mq"]
            d_linear[5, 5] = params["Nr"]
            d_linear[1, 5] = params["Yr"]
            d_linear[5, 1] = params["Nv"]
            d_quad = np.zeros((6, 6), dtype=float)
            d_quad[0, 0] = params["Xuu"]
            d_quad[1, 1] = params["Yvv"]
            d_quad[2, 2] = params["Zww"]
            d_quad[3, 3] = params["Kpp"]
            d_quad[4, 4] = params["Mqq"]
            d_quad[5, 5] = params["Nrr"]
            added_mass = np.diag(
                [
                    params["added_mass_x"],
                    params["added_mass_y"],
                    params["added_mass_z"],
                    params["added_mass_k"],
                    params["added_mass_m"],
                    params["added_mass_n"],
                ]
            )
            r_g = np.array([params["r_g_x"], params["r_g_y"], params["r_g_z"]], dtype=float)
            r_b = np.array([params["r_b_x"], params["r_b_y"], params["r_b_z"]], dtype=float)
            weight = None if params["weight"] < 0.0 else params["weight"]
            buoyancy = None if params["buoyancy"] < 0.0 else params["buoyancy"]

            self.sim = Fossen6DOF(
                mass_kg=params["m"],
                inertia_cg_kgm2=inertia,
                added_mass_kg=added_mass,
                linear_damping=d_linear,
                quadratic_damping=d_quad,
                r_g_m=r_g,
                r_b_m=r_b,
                weight_n=weight,
                buoyancy_n=buoyancy,
            )
            self.state_full = np.zeros(13, dtype=float)
            self.state_full[3] = 1.0
        else:
            self.sim = Boat3DOF(
                {
                    "m": params["m"],
                    "Iz": params["Iz"],
                    "Xu": params["Xu"],
                    "Xuu": params["Xuu"],
                    "Yv": params["Yv"],
                    "Yr": params["Yr"],
                    "Nv": params["Nv"],
                    "Nr": params["Nr"],
                    "l": params["l"],
                }
            )

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

        self.get_logger().info(
            f"Boat simulator started (model={self.sim_model}, integrator={self.integrator})."
        )

    @staticmethod
    def yaw_from_quaternion_wxyz(quaternion_wxyz):
        qw, qx, qy, qz = quaternion_wxyz
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

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
        if self.sim_model == "6dof":
            x_force = self.thrust * math.cos(self.delta)
            y_force = self.thrust * math.sin(self.delta)
            tau = np.array(
                [x_force, y_force, 0.0, 0.0, 0.0, -self.params["l"] * y_force],
                dtype=float,
            )
            self.state_full = self.sim.step(
                self.state_full, tau, self.dt, method=self.integrator
            )
            x, y, z = self.state_full[:3]
            qw, qx, qy, qz = self.state_full[3:7]
            u, v, w, p, q, r = self.state_full[7:]
            psi = self.yaw_from_quaternion_wxyz([qw, qx, qy, qz])
        else:
            self.sim.step(self.thrust, self.delta, self.dt)
            x, y, psi, u, v, r = self.sim.state
            z = 0.0
            qw = math.cos(psi / 2.0)
            qx = 0.0
            qy = 0.0
            qz = math.sin(psi / 2.0)
            w = 0.0
            p = 0.0
            q = 0.0

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
        odom.pose.pose.position.z = z

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = u
        odom.twist.twist.linear.y = v
        odom.twist.twist.linear.z = w
        odom.twist.twist.angular.x = p
        odom.twist.twist.angular.y = q
        odom.twist.twist.angular.z = r

        self.pub_odom.publish(odom)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
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
