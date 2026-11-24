import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vs_disparity_extender')
    default_config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Full path to YAML file with disparity extender parameters'
    )

    disparity_extender_node = Node(
        package='vs_disparity_extender',
        executable='disparity_extender',
        name='disparity_extender_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    return LaunchDescription([
        params_arg,
        disparity_extender_node
    ])
