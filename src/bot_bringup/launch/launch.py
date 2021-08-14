#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Lidar paramaters
    usb_port = LaunchConfiguration('port', default= '/dev/ttyUSB0')
    laser_frame_id = LaunchConfiguration('laser_frame_id', default= 'base_laser')
    
    bot_desc_launch = os.path.join(
        get_package_share_directory('bot_node'),
        'launch',
        'bot_state_publisher.launch.py'
    )

    lidar_launch = os.path.join(
        get_package_share_directory('delta_2a_ros'),
        'launch',
        'delta_2a.launch.py'
    )
    mecanum_control_launch = os.path.join(
        get_package_share_directory('bot_node'),
        'launch',
        'bot_control_sub.launch.py'
    ) 


    return LaunchDescription([

        DeclareLaunchArgument(
            'port',
            default_value= usb_port,
            description = 'Connected port with Delta lidar'
        ),
        DeclareLaunchArgument(
            'laser_frame_id',
            default_value= laser_frame_id,
            description= 'Robot lidar link'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bot_desc_launch]
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [mecanum_control_launch]
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_launch]),
            launch_arguments={'port': usb_port, 'laser_frame_id': laser_frame_id}.items(),
        ),
        
    ])
    

