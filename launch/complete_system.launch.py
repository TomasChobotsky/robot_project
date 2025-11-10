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
    
    # Ball detector optimization arguments
    enable_viz_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable OpenCV visualization window (disable for better performance)'
    )
    
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate',
        default_value='5',
        description='Ball detection processing rate in Hz (lower = better performance)'
    )
    
    # Ball tracking controller arguments
    target_mode_arg = DeclareLaunchArgument(
        'target_mode',
        default_value='closest',
        description='Target mode: green, orange, closest, or alternate'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='auto',
        description='Control mode: manual or auto'
    )
    
    tracking_rate_arg = DeclareLaunchArgument(
        'tracking_rate',
        default_value='1.0',
        description='Auto-tracking rate in Hz (only applies in auto mode)'
    )
    
    # Camera calibration arguments
    camera_x_arg = DeclareLaunchArgument(
    'camera_x_offset',
    default_value='1.0',  # 200-300mm front of robot (in robot's forward direction)
    description='Camera X offset from robot base (meters)'
    )

    camera_y_arg = DeclareLaunchArgument(
        'camera_y_offset',
        default_value='0.0',  # 450mm to the side (in robot's left direction)
        description='Camera Y offset from robot base (meters)'
    )

    camera_z_arg = DeclareLaunchArgument(
        'camera_z_offset',
        default_value='0.10',  # 100-200mm up from robot base
        description='Camera Z offset from robot base (meters)'
    )

    camera_rot_arg = DeclareLaunchArgument(
        'camera_rotation',
        default_value='180',  # 90° rotation (camera looking from side toward robot)
        description='Camera rotation angle around vertical axis (degrees)'
    )
    
    approach_height_arg = DeclareLaunchArgument(
        'approach_height_offset',
        default_value='0.02',
        description='Height offset above ball for approach (meters)'
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
            'depth_module.profile': '1280x720x30',
            'rgb_camera.profile': '1280x720x30'
        }.items()
    )
    
    # Ball detector node with optimization parameters
    node_ball_detector = Node(
        package=package_name,
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'processing_rate': LaunchConfiguration('processing_rate')
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
            'camera_rotation': LaunchConfiguration('camera_rotation'),
            'approach_height_offset': LaunchConfiguration('approach_height_offset'),
            'tracking_rate': LaunchConfiguration('tracking_rate')
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
        enable_viz_arg,
        processing_rate_arg,
        
        target_mode_arg,
        control_mode_arg,
        tracking_rate_arg,
        
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_rot_arg,
        approach_height_arg,
        
        realsense_launch,
        
        node_ball_detector,
        node_ball_tracking,
        
        node_robot_visualizer,
        node_send_single_joint_command
    ])