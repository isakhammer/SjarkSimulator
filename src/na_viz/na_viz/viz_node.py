import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path

from na_utils.bspline import eval_bspline, samples_from_density
from na_msg.msg import BsplinePath, ControllerState
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

import numpy as np

from na_msg.msg import BoatState


class BoatVisualizer(Node):
    def __init__(self):
        super().__init__("boat_visualizer")
        self.declare_parameter("samples_per_meter", 4.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("frenet_frame", "path_frenet")
        self.declare_parameter("debug_force_scale", 0.01)
        self.declare_parameter("debug_moment_scale", 0.02)
        self.declare_parameter("debug_thrust_scale", 0.01)
        self.declare_parameter("debug_velocity_scale", 1.0)
        self.declare_parameter("debug_marker_alpha", 0.9)
        self.declare_parameter("debug_marker_shaft_diameter", 0.04)
        self.declare_parameter("debug_marker_head_diameter", 0.08)
        self.declare_parameter("debug_marker_head_length", 0.12)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.frenet_frame = str(self.get_parameter("frenet_frame").value)

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

        # TF broadcaster for projection frame
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal path storage
        self.path_trace = Path()
        self.path_trace.header.frame_id = "map"
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
        marker.header.frame_id = "map"
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
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.pub_planner_path.publish(marker)

    # --------------------------------------------------------
    # PATH TRACE
    # --------------------------------------------------------
    def update_path_trace(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path_trace.header.stamp = msg.header.stamp
        self.path_trace.poses.append(pose)
        self.pub_path_trace.publish(self.path_trace)

    # --------------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------------
    def odom_cb(self, msg):
        self.last_pose = msg.pose.pose
        self.update_path_trace(msg)
        self.publish_boat_marker(msg)

    def controller_state_cb(self, msg):
        self.publish_frenet_from_state(msg)
        if self.last_pose is None:
            return

        target = Marker()
        target.header.frame_id = "map"
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
        target.pose.position.x = msg.target_x
        target.pose.position.y = msg.target_y
        target.pose.position.z = 0.0
        target.pose.orientation.w = 1.0
        self.pub_los_target.publish(target)

        m = Marker()
        m.header.frame_id = "map"
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
        p1.x = msg.proj_x
        p1.y = msg.proj_y
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

        force_scale = float(self.get_parameter("debug_force_scale").value)
        thrust_scale = float(self.get_parameter("debug_thrust_scale").value)
        moment_scale = float(self.get_parameter("debug_moment_scale").value)
        velocity_scale = float(self.get_parameter("debug_velocity_scale").value)

        markers = []
        markers.append(
            self._make_sphere_marker("com", 0, com_world, (0.95, 0.85, 0.2), 0.2, stamp)
        )
        markers.append(
            self._make_sphere_marker("cob", 0, cob_world, (0.2, 0.8, 0.95), 0.2, stamp)
        )

        gravity_world = np.array([0.0, 0.0, -msg.weight_n], dtype=float)
        buoyancy_world = np.array([0.0, 0.0, msg.buoyancy_n], dtype=float)
        markers.append(
            self._make_arrow_marker(
                "gravity", 0, com_world, gravity_world, (0.9, 0.2, 0.2), force_scale, stamp
            )
        )
        markers.append(
            self._make_arrow_marker(
                "buoyancy", 0, cob_world, buoyancy_world, (0.2, 0.9, 0.3), force_scale, stamp
            )
        )

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

        markers.append(
            self._make_arrow_marker(
                "tau_force",
                0,
                position,
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
        markers.append(
            self._make_arrow_marker(
                "thrust",
                0,
                position,
                thrust_world,
                (1.0, 0.2, 0.2),
                thrust_scale,
                stamp,
            )
        )
        markers.append(
            self._make_arrow_marker(
                "tau_moment",
                0,
                position,
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
                position,
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
                position,
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
                position,
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
                position,
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
                position,
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
                position,
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
                position,
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
                position,
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
        markers.append(
            self._make_arrow_marker(
                "velocity",
                0,
                position,
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
                position,
                omega_world,
                (0.9, 0.2, 0.9),
                velocity_scale,
                stamp,
            )
        )

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.pub_debug_markers.publish(marker_array)

    def publish_frenet_from_state(self, msg):
        yaw = float(msg.proj_yaw)
        qz = float(np.sin(yaw / 2.0))
        qw = float(np.cos(yaw / 2.0))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.frenet_frame
        t.transform.translation.x = float(msg.proj_x)
        t.transform.translation.y = float(msg.proj_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------------
    # BOAT MARKER (cube hull)
    # --------------------------------------------------------
    def publish_boat_marker(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = msg.header.stamp
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


def main(args=None):
    rclpy.init(args=args)
    node = BoatVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
