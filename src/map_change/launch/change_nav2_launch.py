import os
from ament_index_python import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    declare_map_yaml_cmd = DeclareLaunchArgument(
            'map',
            default_value = '/home/nwrlab/map/13hall_test2.yaml',
            description = 'Full path to map yaml file to load'
    

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/bringup_launch.py'
            )
        ),
        launch_arguments={'map': '/home/nwrlab/map/13hall_test2.yaml'}.items(),
    )


    initial_pose_publisher = Node(
        package='map_change',
        node_executable='initial_pose_pub',
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        nav2_launch,
        initial_pose_publisher
    ])

