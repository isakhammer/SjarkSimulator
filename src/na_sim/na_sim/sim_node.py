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
from na_sim.fossen import (
    Fossen6DOF,
    coriolis_from_mass_matrix,
    restoring_forces_body,
    rotation_matrix_from_quaternion_wxyz,
)
from na_msg.msg import BoatState
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
        self.pub_state = self.create_publisher(BoatState, "/boat_state", 10)
        self.pub_state_full = self.create_publisher(
            Float32MultiArray, "/boat_state_full", 10
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

    @staticmethod
    def rpy_from_quaternion_wxyz(quaternion_wxyz):
        qw, qx, qy, qz = quaternion_wxyz
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        )
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return roll, pitch, yaw

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

        # Publish /boat_state_full (full 6DOF-friendly state)
        # Layout: [x, y, z, qw, qx, qy, qz, u, v, w, p, q, r]
        state_full_msg = Float32MultiArray()
        state_full_msg.data = [
            x,
            y,
            z,
            qw,
            qx,
            qy,
            qz,
            u,
            v,
            w,
            p,
            q,
            r,
        ]
        self.pub_state_full.publish(state_full_msg)

        now = self.get_clock().now().to_msg()

        # Publish /boat_state (named fields)
        state_debug = BoatState()
        state_debug.header.stamp = now
        state_debug.header.frame_id = "map"
        state_debug.model = self.sim_model
        state_debug.is_6dof = self.sim_model == "6dof"
        state_debug.dt = float(self.dt)
        state_debug.thrust_cmd = float(self.thrust_cmd)
        state_debug.delta_cmd = float(self.delta_cmd)
        state_debug.thrust = float(self.thrust)
        state_debug.delta = float(self.delta)

        if self.sim_model == "6dof":
            state_debug.r_g_x = float(self.sim.r_g[0])
            state_debug.r_g_y = float(self.sim.r_g[1])
            state_debug.r_g_z = float(self.sim.r_g[2])
            state_debug.r_b_x = float(self.sim.r_b[0])
            state_debug.r_b_y = float(self.sim.r_b[1])
            state_debug.r_b_z = float(self.sim.r_b[2])
            state_debug.weight_n = float(self.sim.weight)
            state_debug.buoyancy_n = float(self.sim.buoyancy)
        else:
            state_debug.r_g_x = 0.0
            state_debug.r_g_y = 0.0
            state_debug.r_g_z = 0.0
            state_debug.r_b_x = 0.0
            state_debug.r_b_y = 0.0
            state_debug.r_b_z = 0.0
            state_debug.weight_n = 0.0
            state_debug.buoyancy_n = 0.0

        state_debug.x = float(x)
        state_debug.y = float(y)
        state_debug.z = float(z)
        state_debug.qw = float(qw)
        state_debug.qx = float(qx)
        state_debug.qy = float(qy)
        state_debug.qz = float(qz)
        roll, pitch, yaw = self.rpy_from_quaternion_wxyz([qw, qx, qy, qz])
        state_debug.roll = float(roll)
        state_debug.pitch = float(pitch)
        state_debug.yaw = float(yaw)

        state_debug.u = float(u)
        state_debug.v = float(v)
        state_debug.w = float(w)
        state_debug.p = float(p)
        state_debug.q = float(q)
        state_debug.r = float(r)

        if self.sim_model == "6dof":
            r_bw = rotation_matrix_from_quaternion_wxyz([qw, qx, qy, qz])
            v_world = r_bw @ np.array([u, v, w], dtype=float)
            state_debug.x_dot = float(v_world[0])
            state_debug.y_dot = float(v_world[1])
            state_debug.z_dot = float(v_world[2])

            tau = np.array(
                [x_force, y_force, 0.0, 0.0, 0.0, -self.params["l"] * y_force],
                dtype=float,
            )
            nu = np.array([u, v, w, p, q, r], dtype=float)
            nu_r = nu
            c_rb = coriolis_from_mass_matrix(self.sim.m_rb, nu)
            c_a = coriolis_from_mass_matrix(self.sim.m_a, nu_r)
            c_rb_nu = c_rb @ nu
            c_a_nu_r = c_a @ nu_r
            d_vec = self.sim.damping(nu_r)
            g_vec = restoring_forces_body(
                r_bw,
                self.sim.weight,
                self.sim.buoyancy,
                self.sim.r_g,
                self.sim.r_b,
            )
            rhs = tau - c_rb_nu - c_a_nu_r - d_vec - g_vec
            nu_dot = np.linalg.solve(self.sim.m, rhs)

            state_debug.u_dot = float(nu_dot[0])
            state_debug.v_dot = float(nu_dot[1])
            state_debug.w_dot = float(nu_dot[2])
            state_debug.p_dot = float(nu_dot[3])
            state_debug.q_dot = float(nu_dot[4])
            state_debug.r_dot = float(nu_dot[5])

            state_debug.tau_x = float(tau[0])
            state_debug.tau_y = float(tau[1])
            state_debug.tau_z = float(tau[2])
            state_debug.tau_k = float(tau[3])
            state_debug.tau_m = float(tau[4])
            state_debug.tau_n = float(tau[5])

            state_debug.coriolis_rb_x = float(c_rb_nu[0])
            state_debug.coriolis_rb_y = float(c_rb_nu[1])
            state_debug.coriolis_rb_z = float(c_rb_nu[2])
            state_debug.coriolis_rb_k = float(c_rb_nu[3])
            state_debug.coriolis_rb_m = float(c_rb_nu[4])
            state_debug.coriolis_rb_n = float(c_rb_nu[5])

            state_debug.coriolis_a_x = float(c_a_nu_r[0])
            state_debug.coriolis_a_y = float(c_a_nu_r[1])
            state_debug.coriolis_a_z = float(c_a_nu_r[2])
            state_debug.coriolis_a_k = float(c_a_nu_r[3])
            state_debug.coriolis_a_m = float(c_a_nu_r[4])
            state_debug.coriolis_a_n = float(c_a_nu_r[5])

            state_debug.damping_x = float(d_vec[0])
            state_debug.damping_y = float(d_vec[1])
            state_debug.damping_z = float(d_vec[2])
            state_debug.damping_k = float(d_vec[3])
            state_debug.damping_m = float(d_vec[4])
            state_debug.damping_n = float(d_vec[5])

            state_debug.restoring_x = float(g_vec[0])
            state_debug.restoring_y = float(g_vec[1])
            state_debug.restoring_z = float(g_vec[2])
            state_debug.restoring_k = float(g_vec[3])
            state_debug.restoring_m = float(g_vec[4])
            state_debug.restoring_n = float(g_vec[5])
        else:
            x_dot = u * math.cos(psi) - v * math.sin(psi)
            y_dot = u * math.sin(psi) + v * math.cos(psi)
            state_debug.x_dot = float(x_dot)
            state_debug.y_dot = float(y_dot)
            state_debug.z_dot = 0.0

            X = self.thrust * math.cos(self.delta)
            Y = self.thrust * math.sin(self.delta)
            N = -self.params["l"] * Y

            damping_x = self.params["Xu"] * u + self.params["Xuu"] * abs(u) * u
            damping_y = self.params["Yv"] * v + self.params["Yr"] * r
            damping_n = self.params["Nv"] * v + self.params["Nr"] * r

            coriolis_x = -self.params["m"] * v * r
            coriolis_y = self.params["m"] * u * r

            du = (X - damping_x + self.params["m"] * v * r) / self.params["m"]
            dv = (Y - damping_y - self.params["m"] * u * r) / self.params["m"]
            dr = (N - damping_n) / self.params["Iz"]

            state_debug.u_dot = float(du)
            state_debug.v_dot = float(dv)
            state_debug.w_dot = 0.0
            state_debug.p_dot = 0.0
            state_debug.q_dot = 0.0
            state_debug.r_dot = float(dr)

            state_debug.tau_x = float(X)
            state_debug.tau_y = float(Y)
            state_debug.tau_z = 0.0
            state_debug.tau_k = 0.0
            state_debug.tau_m = 0.0
            state_debug.tau_n = float(N)

            state_debug.coriolis_rb_x = float(coriolis_x)
            state_debug.coriolis_rb_y = float(coriolis_y)
            state_debug.coriolis_rb_z = 0.0
            state_debug.coriolis_rb_k = 0.0
            state_debug.coriolis_rb_m = 0.0
            state_debug.coriolis_rb_n = 0.0

            state_debug.coriolis_a_x = 0.0
            state_debug.coriolis_a_y = 0.0
            state_debug.coriolis_a_z = 0.0
            state_debug.coriolis_a_k = 0.0
            state_debug.coriolis_a_m = 0.0
            state_debug.coriolis_a_n = 0.0

            state_debug.damping_x = float(damping_x)
            state_debug.damping_y = float(damping_y)
            state_debug.damping_z = 0.0
            state_debug.damping_k = 0.0
            state_debug.damping_m = 0.0
            state_debug.damping_n = float(damping_n)

            state_debug.restoring_x = 0.0
            state_debug.restoring_y = 0.0
            state_debug.restoring_z = 0.0
            state_debug.restoring_k = 0.0
            state_debug.restoring_m = 0.0
            state_debug.restoring_n = 0.0

        self.pub_state.publish(state_debug)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
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
