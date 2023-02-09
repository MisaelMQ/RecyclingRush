#!/usr/bin/env python3

import launch_ros.actions
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # Importing existing Camera Launch
    camera_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([
                                FindPackageShare('main_pkg'),
                                'oakd_cam_launch.py'
                            ])
                        ])
                    )

    # Importing Code for Showing Camera
    camera_show = launch_ros.actions.Node(
            package='main_pkg', 
            executable='camera_sub',
            name='camera_sub')
    
    # Packages to be Launched
    ld = LaunchDescription()
    ld.add_action(camera_launch)
    ld.add_action(camera_show)

    return ld
