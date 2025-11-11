#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_project'

    # Original robot nodes
    node_robot_visualizer = Node(
        package=package_name,
        executable='robot_visualizer',
        name='robot_visualizer',
        output='screen'
    )
    
    node_send_single_joint_command = Node(
        package=package_name,
        executable='send_single_joint_command',
        name='send_single_joint_command',
        output='screen'
    )

    node_precision_test = Node(
        package=package_name,
        executable='precision_test',
        name='precision_test',
        output='screen'
    )
    
    return LaunchDescription([        
        node_robot_visualizer,
        node_send_single_joint_command,
        node_precision_test
    ])