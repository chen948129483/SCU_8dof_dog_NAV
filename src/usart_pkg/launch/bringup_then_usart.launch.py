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
    pole_points = LaunchConfiguration('pole_points')
    offset = LaunchConfiguration('offset')
    blend_distance = LaunchConfiguration('blend_distance')
    start_from_right = LaunchConfiguration('start_from_right')
    use_current_pose_as_start_anchor = LaunchConfiguration('use_current_pose_as_start_anchor')
    auto_order_poles = LaunchConfiguration('auto_order_poles')
    auto_compute_end_anchor = LaunchConfiguration('auto_compute_end_anchor')
    end_extension = LaunchConfiguration('end_extension')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    dynamic_replan = LaunchConfiguration('dynamic_replan')
    replan_period_sec = LaunchConfiguration('replan_period_sec')
    replan_min_start_shift = LaunchConfiguration('replan_min_start_shift')
    update_navigation_on_replan = LaunchConfiguration('update_navigation_on_replan')
    nav_goal_update_period_sec = LaunchConfiguration('nav_goal_update_period_sec')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
    declare_pole_points = DeclareLaunchArgument(
        'pole_points',
        default_value='0.0,0.4;-0.3,1.7;1.0,1.4;2.0,1.4',
        description='Semicolon-separated slalom pole coordinates, e.g. "0.0,0.4;-0.3,1.7"'
    )
    declare_offset = DeclareLaunchArgument(
        'offset',
        default_value='0.40',
        description='Lateral distance from each pole to the slalom apex point'
    )
    declare_blend_distance = DeclareLaunchArgument(
        'blend_distance',
        default_value='0.18',
        description='Entry/exit distance around each pole apex'
    )
    declare_start_from_right = DeclareLaunchArgument(
        'start_from_right',
        default_value='true',
        description='Whether to pass the first pole on the robot right side'
    )
    declare_use_current_pose_as_start_anchor = DeclareLaunchArgument(
        'use_current_pose_as_start_anchor',
        default_value='true',
        description='Use current TF pose as slalom start anchor'
    )
    declare_auto_order_poles = DeclareLaunchArgument(
        'auto_order_poles',
        default_value='true',
        description='Order poles from the current start using nearest-neighbor order'
    )
    declare_auto_compute_end_anchor = DeclareLaunchArgument(
        'auto_compute_end_anchor',
        default_value='true',
        description='Automatically place the end anchor beyond the last pole'
    )
    declare_end_extension = DeclareLaunchArgument(
        'end_extension',
        default_value='0.45',
        description='Distance beyond the last pole for the auto end anchor'
    )
    declare_robot_base_frame = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base',
        description='Robot base frame used for current pose lookup'
    )
    declare_dynamic_replan = DeclareLaunchArgument(
        'dynamic_replan',
        default_value='true',
        description='Continuously recompute slalom through-poses from the current robot pose'
    )
    declare_replan_period_sec = DeclareLaunchArgument(
        'replan_period_sec',
        default_value='1.0',
        description='Dynamic slalom replanning period in seconds'
    )
    declare_replan_min_start_shift = DeclareLaunchArgument(
        'replan_min_start_shift',
        default_value='0.05',
        description='Minimum robot movement before sending a refreshed slalom plan'
    )
    declare_update_navigation_on_replan = DeclareLaunchArgument(
        'update_navigation_on_replan',
        default_value='false',
        description='Also send refreshed through-poses goals to Nav2 during dynamic replanning'
    )
    declare_nav_goal_update_period_sec = DeclareLaunchArgument(
        'nav_goal_update_period_sec',
        default_value='5.0',
        description='Minimum seconds between refreshed Nav2 through-poses goals'
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
        parameters=[{
            'pole_points': pole_points,
            'offset': offset,
            'blend_distance': blend_distance,
            'start_from_right': start_from_right,
            'use_current_pose_as_start_anchor': use_current_pose_as_start_anchor,
            'auto_order_poles': auto_order_poles,
            'auto_compute_end_anchor': auto_compute_end_anchor,
            'end_extension': end_extension,
            'robot_base_frame': robot_base_frame,
            'dynamic_replan': dynamic_replan,
            'replan_period_sec': replan_period_sec,
            'replan_min_start_shift': replan_min_start_shift,
            'update_navigation_on_replan': update_navigation_on_replan,
            'nav_goal_update_period_sec': nav_goal_update_period_sec,
        }],
    )

    start_usart_node = Node(
        package='usart_pkg',
        executable='usart_node',
        name='usart_node',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_pole_points,
        declare_offset,
        declare_blend_distance,
        declare_start_from_right,
        declare_use_current_pose_as_start_anchor,
        declare_auto_order_poles,
        declare_auto_compute_end_anchor,
        declare_end_extension,
        declare_robot_base_frame,
        declare_dynamic_replan,
        declare_replan_period_sec,
        declare_replan_min_start_shift,
        declare_update_navigation_on_replan,
        declare_nav_goal_update_period_sec,
        start_slalom,
        start_usart_node,
        start_bringup,
    ])
