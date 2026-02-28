import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([

        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', node_name='bl_base', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']),
    
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', node_name='bl_camera_color', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_color_optical_frame']),
        
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', node_name='bl_camera_depth', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_depth_optical_frame']),

        # Rviz
        launch_ros.actions.Node(
            package='rviz2', node_executable='rviz2', output='screen',
            arguments=['--display-config', './default.rviz']
            ),
    ])
