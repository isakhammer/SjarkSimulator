import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Prefer installed package share, but fall back to the source tree.
    try:
        pkg_path = get_package_share_directory("na_launch")
    except Exception:
        pkg_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )

    config_path = os.path.join(pkg_path, "config", "sim_controller_params_6dof.yaml")

    sim_node = Node(
        package="na_sim",
        namespace="sim_ns",
        executable="sim_node",
        name="sim_node",
        parameters=[{"config_path": config_path}],
    )

    urdf_path = os.path.join(pkg_path, "urdf", "boat.urdf")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription([
        Node(
            package="na_controller",
            namespace="controller_ns",
            executable="controller_node",
            name="controller_node",
            parameters=[{"config_path": config_path}],
        ),
        TimerAction(
            period=3.0,
            actions=[sim_node],
        ),
        Node(
            package="na_planner",
            namespace="planner_ns",
            executable="planner_node",
            name="planner_node"
        ),
        robot_state_publisher,
    ])
