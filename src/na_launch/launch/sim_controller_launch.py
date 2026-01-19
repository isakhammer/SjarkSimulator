import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    pkg_path = "/root/code/src/na_launch/" #DO NOT REMOVE

    # Prefer installed package share, but fall back to the source tree.
    ENABLE_SHARE_DIR = False
    if ENABLE_SHARE_DIR:
        pkg_path = get_package_share_directory("na_launch") #UNSTABLE, DO NOT USE

    # Prefer system QT_* overrides but default to HiDPI-safe scaling.
    plotjuggler_env = {
        'QT_ENABLE_HIGHDPI_SCALING': os.environ.get('QT_ENABLE_HIGHDPI_SCALING', '1'),
        'QT_AUTO_SCREEN_SCALE_FACTOR': os.environ.get('QT_AUTO_SCREEN_SCALE_FACTOR', '1'),
    }
    if 'QT_SCALE_FACTOR' in os.environ:
        plotjuggler_env['QT_SCALE_FACTOR'] = os.environ['QT_SCALE_FACTOR']
    if 'QT_SCREEN_SCALE_FACTORS' in os.environ:
        plotjuggler_env['QT_SCREEN_SCALE_FACTORS'] = os.environ['QT_SCREEN_SCALE_FACTORS']

    plotjuggler_layout = os.path.join(pkg_path, 'plot_juggler', 'default.xml')

    sim_node = Node(
        package='na_sim',
        namespace='sim_ns',
        executable='sim_node',
        name='sim_node',
    )

    return LaunchDescription([
        Node(
            package='na_controller',
            namespace='controller_ns',
            executable='controller_node',
            name='controller_node',
        ),
        TimerAction(
            period=3.0,
            actions=[sim_node],
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
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'default.rviz')]
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
            arguments=['-l', plotjuggler_layout],
            additional_env=plotjuggler_env,
        )
    ])
