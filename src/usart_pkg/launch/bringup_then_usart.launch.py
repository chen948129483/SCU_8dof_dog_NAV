#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )


    leg_bringup_share = get_package_share_directory('leg_bringup')
    bringup_in_real_launch = os.path.join(
        leg_bringup_share,
        'launch',
        'bringup_in_real.launch.py'
    )

    start_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_in_real_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    start_slalom = Node(
        package='nav_pose',
        executable='slalom_through_poses',
        name='slalom_through_poses',
        output='screen',
    )

    start_usart_node = Node(
        package='usart_pkg',
        executable='usart_node',
        name='usart_node',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        start_slalom,
        start_usart_node,
        start_bringup,
    ])
