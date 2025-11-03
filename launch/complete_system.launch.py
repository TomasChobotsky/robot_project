#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'robot_project'
    
    # Declare launch arguments for ball tracking controller
    target_mode_arg = DeclareLaunchArgument(
        'target_mode',
        default_value='green',
        description='Target mode: green, orange, closest, or alternate'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='manual',
        description='Control mode: manual or auto'
    )
    
    camera_x_arg = DeclareLaunchArgument(
        'camera_x_offset',
        default_value='0.2',
        description='Camera X offset from robot base (meters)'
    )
    
    camera_y_arg = DeclareLaunchArgument(
        'camera_y_offset',
        default_value='0.0',
        description='Camera Y offset from robot base (meters)'
    )
    
    camera_z_arg = DeclareLaunchArgument(
        'camera_z_offset',
        default_value='0.3',
        description='Camera Z offset from robot base (meters)'
    )
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30'
        }.items()
    )
    
    # Ball detector node
    node_ball_detector = Node(
        package=package_name,
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Ball tracking controller node
    node_ball_tracking = Node(
        package=package_name,
        executable='ball_tracking_controller',
        name='ball_tracking_controller',
        output='screen',
        parameters=[{
            'target_mode': LaunchConfiguration('target_mode'),
            'control_mode': LaunchConfiguration('control_mode'),
            'camera_x_offset': LaunchConfiguration('camera_x_offset'),
            'camera_y_offset': LaunchConfiguration('camera_y_offset'),
            'camera_z_offset': LaunchConfiguration('camera_z_offset'),
            'approach_height_offset': 0.05
        }]
    )
    
    # Original robot nodes
    node_robot_visualizer = Node(
        package=package_name,
        executable='robot_visualizer',
        name='robot_visualizer',
        output='screen'
    )
    
    node_visualizer_to_robot = Node(
        package=package_name,
        executable='visualizer_to_robot',
        name='visualizer_to_robot',
        output='screen'
    )
    
    node_send_single_joint_command = Node(
        package=package_name,
        executable='send_single_joint_command',
        name='send_single_joint_command',
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        target_mode_arg,
        control_mode_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        
        # Launch RealSense camera
        realsense_launch,
        
        # Launch ball detection and tracking
        node_ball_detector,
        node_ball_tracking,
        
        # Launch original robot control nodes
        # Comment these out if you want to run ball tracking standalone
        node_robot_visualizer,
        node_visualizer_to_robot,
        node_send_single_joint_command
    ])