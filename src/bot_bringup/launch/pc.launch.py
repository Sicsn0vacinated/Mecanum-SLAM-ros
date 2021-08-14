#!/usr/bin/env python3

import os
import sys
#from typing import DefaultDict

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

VALID_MODEL_ = ['A1', 'PROTOTYPE']

def generate_launch_description():
    try:
        # Use A1 params if unspecified
        BOT_MODEL = os.environ['BOT_MODEL'].upper()
    except:
        BOT_MODEL = 'A1'

    if BOT_MODEL not in ['A1', 'PROTOTYPE']:
        print("BOT_MODEL not set correctly")
        BOT_MODEL = 'A1'
     

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
            BOT_MODEL + '_nav2.yaml'
        )
    )

    declare_slam_params_file_cmd = LaunchConfiguration(
        'slam_params_file',
        default= os.path.join(get_package_share_directory("bot_navigation2"),
                                'config', 'mapper_params_online_async.yaml'),
    )
    
    #RVIZ launch

    default_rviz_config_path = os.path.join(
        #get_package_share_directory('nav2_bringup'),
        #rviz', 'nav2_default_view.rviz')
        get_package_share_directory('bot_bringup'),
        'rviz','urdf_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    mapping_launch = PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory('bot_navigation2'),
        'launch',
        'online_async_launch.py')
        ]
    ) 

    nav2_launch = PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py')
        ]
    )

    # Robot localization
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(
           get_package_share_directory('bot_navigation2'), 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)
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
            default_value= 'true',
            description= 'nav2 autostart'
        ),
        
        DeclareLaunchArgument(
            'slam_params_file',
            default_value= declare_slam_params_file_cmd,
            description= 'test slam toolbox launch'
        ),

        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),

        IncludeLaunchDescription(
            mapping_launch,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': declare_slam_params_file_cmd
            }.items(),
        
        ),

        IncludeLaunchDescription(
            nav2_launch,
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'autostart': autostart_nav,
                'param_file': nav_param
            }.items()
        ),

        rviz_node,



        #IncludeLaunchDescription(
        #    nav2_bringup,
        #launch_arguments={         
        #    'map': map_dir,  
        #    'use_sim_time': use_sim_time,
        #    'autostart': autostart_nav,
        #    'params_file': nav_param
        #}.items(),
        #),

    ])
    

