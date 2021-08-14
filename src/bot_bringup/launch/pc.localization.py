#!/usr/bin/env python3

import os
import sys


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Load the URDF into a parameer
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default = os.path.join(
            '~',
            'map',
            'test_map_dev.yaml'
        )
    )
    autostart_nav = LaunchConfiguration('autostart', default = 'false')

    nav_param = LaunchConfiguration(
        'params_file',
        default= os.path.join(
            get_package_share_directory('bot_navigation2'),
            'param',
            'nav_param.yaml'
        )
    )

    declare_slam_params_file_cmd = LaunchConfiguration(
        'slam_params_file',
        default= os.path.join(get_package_share_directory("bot_navigation2"),
                                'config', 'mapper_params_localization.yaml'),
    )


    rviz_launch = os.path.join(
        get_package_share_directory('rviz_launch'),
        'launch',
        'display.launch.py'
    )

    mapping_launch = os.path.join(
        get_package_share_directory('bot_navigation2'),
        'launch',
        'online_async_launch.py'
    ) 

    nav2_bringup = PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py')
    ]
        #package = 'nav2_bringup',
        #launch = 'navigation_launch.py',
        #arguments=[]

    )

    #ld.add_action(node_tf2_fp2odom)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value= nav_param,
            description= 'Full path to nav2 parameter file'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value= 'false',
            description= 'nav2 autostart'
        ),
        
        DeclareLaunchArgument(
            'slam_params_file',
            default_value= declare_slam_params_file_cmd,
            description= 'test slam toolbox launch'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [rviz_launch]
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mapping_launch]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': declare_slam_params_file_cmd
            }.items(),
        
        ),
        IncludeLaunchDescription(
            nav2_bringup,
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'autostart': autostart_nav,
            'params_file': nav_param
        }.items(),
        ),
        ## tf2 - map to odom
        #Node(
        #    name='tf2_ros_fp_odom',
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    output='screen',
        #    arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'odom'],
        #),
        
    ])
    

