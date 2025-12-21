import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Absolute path to this file
pkg_path = '/root/code/src/na_launch'
print(pkg_path)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='na_controller',
            namespace='controller_ns',
            executable='throttle',
            name='throttle'
        ),
        Node(
            package='na_sim',
            namespace='sim_ns',
            executable='sim_node',
            name='sim_node'
        ),
        Node(
            package='na_planner',
            namespace='planner_ns',
            executable='planner_node',
            name='planner_node'
        ),
        Node(
            package='na_viz',
            namespace='viz_ns',
            executable='viz_node',
            name='viz_node'
        ),
         Node(
           package='rviz2',
           executable='rviz2',
           name="rviz2",
           output='screen',
           arguments=['-d', [os.path.join(pkg_path, 'rviz', 'default.rviz')]]
         )
    ])
