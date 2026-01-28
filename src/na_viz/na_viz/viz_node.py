import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path

from na_utils.bspline import eval_bspline, samples_from_density
from na_msg.msg import BsplinePath, ControllerState
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

from na_msg.msg import BoatState


class BoatVisualizer(Node):
    def __init__(self):
        super().__init__("boat_visualizer")
        self.declare_parameter("samples_per_meter", 4.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("frenet_frame", "path_frenet")
        self.declare_parameter("flip_tf_yaw", False)
        self.declare_parameter("convert_legacy_ned_to_enu", False)
        self.declare_parameter("use_urdf", False)
        self.declare_parameter("urdf_path", "/root/code/src/na_launch/urdf/boat.urdf")
        self.declare_parameter("show_z_plus_marker", False)
        self.declare_parameter("z_plus_marker_offset", 0.4)
        self.declare_parameter("z_plus_marker_scale", 0.12)
        self.declare_parameter("debug_force_scale", 0.01)
        self.declare_parameter("debug_moment_scale", 0.02)
        self.declare_parameter("debug_thrust_scale", 0.01)
        self.declare_parameter("debug_velocity_scale", 1.0)
        self.declare_parameter("debug_marker_z_offset", 0.0)
        self.declare_parameter("show_rotor_marker", True)
        self.declare_parameter("rotor_marker_length", 0.6)
        self.declare_parameter("rotor_offset_x", -0.6)
        self.declare_parameter("rotor_offset_y", 0.0)
        self.declare_parameter("rotor_offset_z", 0.0)
        self.declare_parameter("debug_marker_alpha", 0.9)
        self.declare_parameter("debug_marker_shaft_diameter", 0.04)
        self.declare_parameter("debug_marker_head_diameter", 0.08)
        self.declare_parameter("debug_marker_head_length", 0.12)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.frenet_frame = str(self.get_parameter("frenet_frame").value)
        self.flip_tf_yaw = bool(self.get_parameter("flip_tf_yaw").value)
        self.convert_legacy_ned_to_enu = bool(
            self.get_parameter("convert_legacy_ned_to_enu").value
        )
        self.use_urdf = bool(self.get_parameter("use_urdf").value)
        self.urdf_path = str(self.get_parameter("urdf_path").value)
        self.show_z_plus_marker = bool(self.get_parameter("show_z_plus_marker").value)
        self._q_enu_from_legacy_ned = np.array(
            [0.0, 1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0],
            dtype=float,
        )

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 20)
        self.sub_path = self.create_subscription(
            BsplinePath, "/planner_ns/path", self.path_cb, 20)
        self.sub_controller = self.create_subscription(
            ControllerState, "/controller_ns/controller_state", self.controller_state_cb, 20)
        self.sub_state = self.create_subscription(
            BoatState, "/boat_state", self.state_cb, 20)

        # Publishers
        self.pub_path_trace = self.create_publisher(Path, "/viz/path_trace", 10)
        self.pub_boat = self.create_publisher(Marker, "/viz/boat_marker", 10)
        self.pub_planner_path = self.create_publisher(Marker, "/viz/path", 10)
        self.pub_cte = self.create_publisher(Marker, "/viz/cte", 10)
        self.pub_los_target = self.create_publisher(Marker, "/viz/los_target", 10)
        self.pub_debug_markers = self.create_publisher(
            MarkerArray, "/viz/debug_markers", 10
        )

        # Internal path storage
        self.path_trace = Path()
        self.path_trace.header.frame_id = self.map_frame
        self.last_pose = None

        self.get_logger().info("BoatVisualizer started")

    @staticmethod
    def rotation_matrix_from_quaternion_wxyz(quaternion_wxyz):
        qw, qx, qy, qz = quaternion_wxyz
        return np.array(
            [
                [
                    1.0 - 2.0 * (qy * qy + qz * qz),
                    2.0 * (qx * qy - qz * qw),
                    2.0 * (qx * qz + qy * qw),
                ],
                [
                    2.0 * (qx * qy + qz * qw),
                    1.0 - 2.0 * (qx * qx + qz * qz),
                    2.0 * (qy * qz - qx * qw),
                ],
                [
                    2.0 * (qx * qz - qy * qw),
                    2.0 * (qy * qz + qx * qw),
                    1.0 - 2.0 * (qx * qx + qy * qy),
                ],
            ],
            dtype=float,
        )

    @staticmethod
    def quaternion_multiply_wxyz(lhs_wxyz, rhs_wxyz):
        lw, lx, ly, lz = lhs_wxyz
        rw, rx, ry, rz = rhs_wxyz
        return np.array(
            [
                lw * rw - lx * rx - ly * ry - lz * rz,
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
            ],
            dtype=float,
        )

    @staticmethod
    def normalize_quaternion_wxyz(quaternion_wxyz):
        q = np.asarray(quaternion_wxyz, dtype=float).reshape(4,)
        norm = float(np.linalg.norm(q))
        if norm <= 0.0:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return q / norm

    def _legacy_ned_to_enu_vec(self, vec_xyz):
        v = np.asarray(vec_xyz, dtype=float).reshape(3,)
        return np.array([v[1], v[0], -v[2]], dtype=float)

    def _convert_quaternion_legacy_ned_to_enu(self, quat_wxyz):
        q = self.quaternion_multiply_wxyz(self._q_enu_from_legacy_ned, quat_wxyz)
        return self.normalize_quaternion_wxyz(q)

    def _convert_pose_legacy_ned_to_enu(self, pose):
        pos = self._legacy_ned_to_enu_vec(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        q_wxyz = np.array(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=float,
        )
        q_enu = self._convert_quaternion_legacy_ned_to_enu(q_wxyz)
        pose_enu = Pose()
        pose_enu.position.x = float(pos[0])
        pose_enu.position.y = float(pos[1])
        pose_enu.position.z = float(pos[2])
        pose_enu.orientation.w = float(q_enu[0])
        pose_enu.orientation.x = float(q_enu[1])
        pose_enu.orientation.y = float(q_enu[2])
        pose_enu.orientation.z = float(q_enu[3])
        return pose_enu

    def _make_sphere_marker(self, ns, marker_id, position_xyz, color_rgb, scale, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        marker.color.a = float(self.get_parameter("debug_marker_alpha").value)
        marker.pose.position.x = float(position_xyz[0])
        marker.pose.position.y = float(position_xyz[1])
        marker.pose.position.z = float(position_xyz[2])
        marker.pose.orientation.w = 1.0
        return marker

    def _make_arrow_marker(self, ns, marker_id, start_xyz, vec_xyz, color_rgb, scale, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = float(self.get_parameter("debug_marker_shaft_diameter").value)
        marker.scale.y = float(self.get_parameter("debug_marker_head_diameter").value)
        marker.scale.z = float(self.get_parameter("debug_marker_head_length").value)
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        marker.color.a = float(self.get_parameter("debug_marker_alpha").value)

        p1 = Point()
        p1.x = float(start_xyz[0])
        p1.y = float(start_xyz[1])
        p1.z = float(start_xyz[2])
        p2 = Point()
        p2.x = float(start_xyz[0] + scale * vec_xyz[0])
        p2.y = float(start_xyz[1] + scale * vec_xyz[1])
        p2.z = float(start_xyz[2] + scale * vec_xyz[2])
        marker.points.append(p1)
        marker.points.append(p2)
        return marker

    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def path_cb(self, msg):
        n = len(msg.ctrl_x)
        if n < 4 or n != len(msg.ctrl_y):
            self.get_logger().error("Invalid planner spline")
            return

        control = [(msg.ctrl_x[i], msg.ctrl_y[i]) for i in range(n)]
        samples_per_meter = float(self.get_parameter("samples_per_meter").value)
        samples = samples_from_density(
            control, samples_per_meter, closed=bool(msg.closed)
        )
        u_start = msg.start_u
        if msg.closed:
            u_end = msg.start_u + n
        else:
            u_start = 0.0
            u_end = max(1.0, n - 3.0)
        du = (u_end - u_start) / (samples - 1)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "path_viz"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0

        for i in range(samples):
            p = Point()
            u = u_start + du * i
            x, y = eval_bspline(control, u)
            if self.convert_legacy_ned_to_enu:
                x, y, _ = self._legacy_ned_to_enu_vec([x, y, 0.0])
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.pub_planner_path.publish(marker)

    # --------------------------------------------------------
    # PATH TRACE
    # --------------------------------------------------------
    def update_path_trace(self, stamp, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose = pose

        self.path_trace.header.stamp = stamp
        self.path_trace.header.frame_id = self.map_frame
        self.path_trace.poses.append(pose_msg)
        self.pub_path_trace.publish(self.path_trace)

    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def odom_cb(self, msg):
        pose = msg.pose.pose
        if self.convert_legacy_ned_to_enu:
            pose = self._convert_pose_legacy_ned_to_enu(pose)
        self.last_pose = pose
        self.update_path_trace(msg.header.stamp, pose)
        self.publish_boat_marker(msg.header.stamp, pose)

    def controller_state_cb(self, msg):
        if self.last_pose is None:
            return

        target = Marker()
        target.header.frame_id = self.map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.ns = "los_target"
        target.id = 0
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.scale.x = 0.6
        target.scale.y = 0.6
        target.scale.z = 0.6
        target.color.r = 1.0
        target.color.g = 0.8
        target.color.b = 0.0
        target.color.a = 1.0
        target_x = msg.target_x
        target_y = msg.target_y
        if self.convert_legacy_ned_to_enu:
            target_x, target_y, _ = self._legacy_ned_to_enu_vec([target_x, target_y, 0.0])
        target.pose.position.x = float(target_x)
        target.pose.position.y = float(target_y)
        target.pose.position.z = 0.0
        target.pose.orientation.w = 1.0
        self.pub_los_target.publish(target)

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cte"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        p0 = Point()
        p0.x = self.last_pose.position.x
        p0.y = self.last_pose.position.y
        p0.z = 0.0

        p1 = Point()
        proj_x = msg.proj_x
        proj_y = msg.proj_y
        if self.convert_legacy_ned_to_enu:
            proj_x, proj_y, _ = self._legacy_ned_to_enu_vec([proj_x, proj_y, 0.0])
        p1.x = float(proj_x)
        p1.y = float(proj_y)
        p1.z = 0.0

        m.points.append(p0)
        m.points.append(p1)

        self.pub_cte.publish(m)

    def state_cb(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        position = np.array([msg.x, msg.y, msg.z], dtype=float)
        quat_wxyz = np.array([msg.qw, msg.qx, msg.qy, msg.qz], dtype=float)
        r_bw = self.rotation_matrix_from_quaternion_wxyz(quat_wxyz)

        r_g = np.array([msg.r_g_x, msg.r_g_y, msg.r_g_z], dtype=float)
        r_b = np.array([msg.r_b_x, msg.r_b_y, msg.r_b_z], dtype=float)
        com_world = position + r_bw @ r_g
        cob_world = position + r_bw @ r_b
        rotor_offset_body = np.array(
            [
                float(self.get_parameter("rotor_offset_x").value),
                float(self.get_parameter("rotor_offset_y").value),
                float(self.get_parameter("rotor_offset_z").value),
            ],
            dtype=float,
        )
        rotor_pos_world = position + r_bw @ rotor_offset_body
        rotor_dir_body = np.array(
            [math.cos(msg.delta), math.sin(msg.delta), 0.0], dtype=float
        )
        rotor_dir_world = r_bw @ rotor_dir_body

        force_scale = float(self.get_parameter("debug_force_scale").value)
        thrust_scale = float(self.get_parameter("debug_thrust_scale").value)
        moment_scale = float(self.get_parameter("debug_moment_scale").value)
        velocity_scale = float(self.get_parameter("debug_velocity_scale").value)
        debug_marker_z_offset = float(self.get_parameter("debug_marker_z_offset").value)
        rotor_length = float(self.get_parameter("rotor_marker_length").value)

        gravity_world = np.array([0.0, 0.0, -msg.weight_n], dtype=float)
        buoyancy_world = np.array([0.0, 0.0, msg.buoyancy_n], dtype=float)

        tau_force_body = np.array([msg.tau_x, msg.tau_y, msg.tau_z], dtype=float)
        tau_moment_body = np.array([msg.tau_k, msg.tau_m, msg.tau_n], dtype=float)
        damping_body = np.array([msg.damping_x, msg.damping_y, msg.damping_z], dtype=float)
        damping_moment_body = np.array([msg.damping_k, msg.damping_m, msg.damping_n], dtype=float)
        coriolis_rb_body = np.array(
            [msg.coriolis_rb_x, msg.coriolis_rb_y, msg.coriolis_rb_z], dtype=float
        )
        coriolis_rb_moment_body = np.array(
            [msg.coriolis_rb_k, msg.coriolis_rb_m, msg.coriolis_rb_n], dtype=float
        )
        coriolis_a_body = np.array(
            [msg.coriolis_a_x, msg.coriolis_a_y, msg.coriolis_a_z], dtype=float
        )
        coriolis_a_moment_body = np.array(
            [msg.coriolis_a_k, msg.coriolis_a_m, msg.coriolis_a_n], dtype=float
        )
        restoring_body = np.array(
            [msg.restoring_x, msg.restoring_y, msg.restoring_z], dtype=float
        )
        restoring_moment_body = np.array(
            [msg.restoring_k, msg.restoring_m, msg.restoring_n], dtype=float
        )

        tau_force_world = r_bw @ tau_force_body
        tau_moment_world = r_bw @ tau_moment_body
        damping_world = r_bw @ damping_body
        damping_moment_world = r_bw @ damping_moment_body
        coriolis_rb_world = r_bw @ coriolis_rb_body
        coriolis_rb_moment_world = r_bw @ coriolis_rb_moment_body
        coriolis_a_world = r_bw @ coriolis_a_body
        coriolis_a_moment_world = r_bw @ coriolis_a_moment_body
        restoring_world = r_bw @ restoring_body
        restoring_moment_world = r_bw @ restoring_moment_body

        if self.convert_legacy_ned_to_enu:
            position = self._legacy_ned_to_enu_vec(position)
            com_world = self._legacy_ned_to_enu_vec(com_world)
            cob_world = self._legacy_ned_to_enu_vec(cob_world)
            rotor_pos_world = self._legacy_ned_to_enu_vec(rotor_pos_world)
            gravity_world = self._legacy_ned_to_enu_vec(gravity_world)
            buoyancy_world = self._legacy_ned_to_enu_vec(buoyancy_world)
            tau_force_world = self._legacy_ned_to_enu_vec(tau_force_world)
            tau_moment_world = self._legacy_ned_to_enu_vec(tau_moment_world)
            damping_world = self._legacy_ned_to_enu_vec(damping_world)
            damping_moment_world = self._legacy_ned_to_enu_vec(damping_moment_world)
            coriolis_rb_world = self._legacy_ned_to_enu_vec(coriolis_rb_world)
            coriolis_rb_moment_world = self._legacy_ned_to_enu_vec(coriolis_rb_moment_world)
            coriolis_a_world = self._legacy_ned_to_enu_vec(coriolis_a_world)
            coriolis_a_moment_world = self._legacy_ned_to_enu_vec(coriolis_a_moment_world)
            restoring_world = self._legacy_ned_to_enu_vec(restoring_world)
            restoring_moment_world = self._legacy_ned_to_enu_vec(restoring_moment_world)

        debug_offset = np.array([0.0, 0.0, debug_marker_z_offset], dtype=float)
        position_dbg = position + debug_offset
        com_world_dbg = com_world + debug_offset
        cob_world_dbg = cob_world + debug_offset
        rotor_pos_dbg = rotor_pos_world + debug_offset

        markers = []
        markers.append(
            self._make_sphere_marker(
                "com", 0, com_world_dbg, (0.95, 0.85, 0.2), 0.2, stamp
            )
        )
        markers.append(
            self._make_sphere_marker(
                "cob", 0, cob_world_dbg, (0.2, 0.8, 0.95), 0.2, stamp
            )
        )
        markers.append(
            self._make_arrow_marker(
                "gravity",
                0,
                com_world_dbg,
                gravity_world,
                (0.9, 0.2, 0.2),
                force_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "buoyancy",
                0,
                cob_world_dbg,
                buoyancy_world,
                (0.2, 0.9, 0.3),
                force_scale,
                stamp,
            )
        )

        markers.append(
            self._make_arrow_marker(
                "tau_force",
                0,
                position_dbg,
                tau_force_world,
                (1.0, 0.5, 0.0),
                force_scale,
                stamp,
            )
        )
        thrust_body = np.array(
            [
                msg.thrust * np.cos(msg.delta),
                msg.thrust * np.sin(msg.delta),
                0.0,
            ],
            dtype=float,
        )
        thrust_world = r_bw @ thrust_body
        if self.convert_legacy_ned_to_enu:
            thrust_world = self._legacy_ned_to_enu_vec(thrust_world)
        markers.append(
            self._make_arrow_marker(
                "thrust",
                0,
                position_dbg,
                thrust_world,
                (1.0, 0.2, 0.2),
                thrust_scale,
                stamp,
            )
        )
        if bool(self.get_parameter("show_rotor_marker").value):
            if self.convert_legacy_ned_to_enu:
                rotor_dir_world = self._legacy_ned_to_enu_vec(rotor_dir_world)
            markers.append(
                self._make_arrow_marker(
                    "rotor",
                    0,
                    rotor_pos_dbg,
                    rotor_dir_world,
                    (1.0, 0.6, 0.0),
                    rotor_length,
                    stamp,
                )
            )
        markers.append(
            self._make_arrow_marker(
                "tau_moment",
                0,
                position_dbg,
                tau_moment_world,
                (1.0, 0.3, 0.0),
                moment_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "damping_force",
                0,
                position_dbg,
                damping_world,
                (0.6, 0.6, 0.6),
                force_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "damping_moment",
                0,
                position_dbg,
                damping_moment_world,
                (0.5, 0.5, 0.5),
                moment_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "coriolis_rb_force",
                0,
                position_dbg,
                coriolis_rb_world,
                (0.7, 0.2, 0.9),
                force_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "coriolis_rb_moment",
                0,
                position_dbg,
                coriolis_rb_moment_world,
                (0.6, 0.2, 0.8),
                moment_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "coriolis_a_force",
                0,
                position_dbg,
                coriolis_a_world,
                (0.2, 0.6, 0.9),
                force_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "coriolis_a_moment",
                0,
                position_dbg,
                coriolis_a_moment_world,
                (0.2, 0.5, 0.8),
                moment_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "restoring_force",
                0,
                position_dbg,
                restoring_world,
                (0.1, 0.9, 0.4),
                force_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "restoring_moment",
                0,
                position_dbg,
                restoring_moment_world,
                (0.1, 0.7, 0.3),
                moment_scale,
                stamp,
            )
        )

        vel_body = np.array([msg.u, msg.v, msg.w], dtype=float)
        omega_body = np.array([msg.p, msg.q, msg.r], dtype=float)
        vel_world = r_bw @ vel_body
        omega_world = r_bw @ omega_body
        if self.convert_legacy_ned_to_enu:
            vel_world = self._legacy_ned_to_enu_vec(vel_world)
            omega_world = self._legacy_ned_to_enu_vec(omega_world)
        markers.append(
            self._make_arrow_marker(
                "velocity",
                0,
                position_dbg,
                vel_world,
                (0.2, 0.9, 0.9),
                velocity_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "omega",
                0,
                position_dbg,
                omega_world,
                (0.9, 0.2, 0.9),
                velocity_scale,
                stamp,
            )
        )

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.pub_debug_markers.publish(marker_array)

    # --------------------------------------------------------
    # BOAT MARKER (cube hull)
    # --------------------------------------------------------
    def publish_boat_marker(self, stamp, pose):
        if self.use_urdf:
            self.publish_z_plus_marker(stamp, pose)
            return
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = "boat"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Boat dimensions
        m.scale.x = 1.2  # length
        m.scale.y = 0.6  # width
        m.scale.z = 0.2  # height

        m.color.r = 0.1
        m.color.g = 0.2
        m.color.b = 0.9
        m.color.a = 1.0

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation = q

        self.pub_boat.publish(m)
        self.publish_z_plus_marker(stamp, pose)

    def publish_z_plus_marker(self, stamp, pose):
        if not self.show_z_plus_marker:
            return
        offset = float(self.get_parameter("z_plus_marker_offset").value)
        scale = float(self.get_parameter("z_plus_marker_scale").value)

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = "z_plus"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.6
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.pose.position.x = pose.position.x
        m.pose.position.y = pose.position.y
        m.pose.position.z = float(pose.position.z + offset)

        self.pub_boat.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BoatVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
