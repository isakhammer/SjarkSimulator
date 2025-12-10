from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                package='na_controller',
                namespace="controller_ns",
                executable='throttle',
                name='throttle'
                )
    ])
