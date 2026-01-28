import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = "/root/code/src/na_launch/"  # DO NOT REMOVE

    # Prefer installed package share, but fall back to the source tree.
    ENABLE_SHARE_DIR = False
    if ENABLE_SHARE_DIR:
        pkg_path = get_package_share_directory("na_launch")  # UNSTABLE, DO NOT USE

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
    config_path = os.path.join(pkg_path, 'config', 'sim_controller_params.yaml')

    sim_node = Node(
        package='na_sim',
        namespace='sim_ns',
        executable='sim_node',
        name='sim_node',
        parameters=[config_path],
    )

    urdf_path = os.path.join(pkg_path, 'urdf', 'boat.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "path_type",
            default_value="SQUARE_SINUS",
            description="Planner path type (CIRCLE, SQUARE_SINUS, STRAIGHT, SQUIRCLE, COMPLEX).",
        ),
        Node(
            package='na_controller',
            namespace='controller_ns',
            executable='controller_node',
            name='controller_node',
            parameters=[config_path],
        ),
        TimerAction(
            period=3.0,
            actions=[sim_node],
        ),
        Node(
            package='na_planner',
            namespace='planner_ns',
            executable='planner_node',
            name='planner_node',
            parameters=[config_path, {"path_type": LaunchConfiguration("path_type")}],
        ),
        Node(
            package='na_viz',
            namespace='viz_ns',
            executable='viz_node',
            name='viz_node',
            parameters=[config_path],
        ),
        Node(
            package='na_viz',
            namespace='viz_ns',
            executable='tf_node',
            name='tf_node',
            parameters=[config_path],
        ),
        robot_state_publisher,
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
