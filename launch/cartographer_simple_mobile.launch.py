#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_pkg_prefix = get_package_share_directory('prototyping-project2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')
    
    resolution = LaunchConfiguration('resolution', default='0.005')
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value=resolution,
        description='Resolution of a grid cell in the published occuapncy grid')
    
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value=publish_period_sec,
        description='OccupancyGrid publishing period')
    
    cartographer_config_dir = LaunchConfiguration('carrographer_config.dir',
                                                  default=os.path.join(my_pkg_prefix, 'config'))
    cartographer_config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load')
    
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='simple_mobile_lds_2d.lua')
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')
    
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link1_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-6.4','-2.1','0','0','0','0','1', 'map', 'odom'],
        output='screen'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        output='screen',
        remappings=[('echoes', 'scan')]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution,
                   '-publish_period_sec', publish_period_sec],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        publish_period_sec_arg,
        cartographer_config_dir_arg,
        configuration_basename_arg,
        tf2_node,
        cartographer_node,
        occupancy_grid_node
])