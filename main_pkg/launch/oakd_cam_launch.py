#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    # Camera Information
    camera_model     = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix        = LaunchConfiguration('tf_prefix',   default = 'oak')
    camera_param_uri = LaunchConfiguration('camera_param_uri', default = 'package://depthai_examples/params/camera')

    # Arguments for Camera Information
    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')
   
    declare_camera_param_uri_cmd = DeclareLaunchArgument(
        'camera_param_uri',
        default_value=camera_param_uri,
        description='Sending camera yaml path')
   
    # Node to launch RGB Camera
    camera_node = launch_ros.actions.Node(
            package='depthai_examples', executable='rgb_node',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'camera_param_uri': camera_param_uri}])

    # Parameters to be launched
    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_camera_param_uri_cmd)
    
    ld.add_action(camera_node)

    return ld

