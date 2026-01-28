from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sj_controller",
            namespace="controller_ns",
            executable="controller_node",
            name="controller_node",
        )
    ])
