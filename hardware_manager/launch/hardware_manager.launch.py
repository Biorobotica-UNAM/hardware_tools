from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hardware_manager'), 'config', 'hardware_manager.yaml'
            ]),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='hardware_manager',
            executable='hardware_manager_node',
            output='screen',
            parameters=[
                params,
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                )},
            ],
        ),
    ])
