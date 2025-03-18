#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set TurtleBot3 model environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Get the share directory of this package
    pkg_share = get_package_share_directory('vff_avoidance')
    
    # Path to your YAML config file (adjust the path if needed)
    config_file = os.path.join(pkg_share, 'config', 'AvoidanceNodeConfig.yaml')

    # Include Gazebo launch file from turtlebot3_gazebo package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    # Launch your avoidance node using the YAML config file, overriding max_linear_speed
    avoidance_node = Node(
        package='vff_avoidance',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[
            config_file,
            { 'max_linear_speed': 0.113}  # override to slow down the robot
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        avoidance_node
    ])
