import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('loam_mapper')

    loam_mapper_file_param = os.path.join(path_package, 'config/loam_mapper_params.yaml')
    loam_mapper_node = Node(
        package='loam_mapper',
        executable='loam_mapper',
        parameters=[loam_mapper_file_param]
    )


    return launch.LaunchDescription(
        [loam_mapper_node])
