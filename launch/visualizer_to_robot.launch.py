from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'robot_project'
    package_path = get_package_share_directory(package_name)

    node_robot_visualizer = Node(
        package=package_name,
        executable='robot_visualizer',
        name='robot_visualizer'
    )

    node_visualizer_to_robot = Node(
        package=package_name,
        executable='visualizer_to_robot',
        name='visualizer_to_robot'
    )

    node_send_single_joint_command = Node(
        package=package_name,
        executable='send_single_joint_command',
        name='send_single_joint_command'
    )

    return LaunchDescription([
        node_robot_visualizer,
        node_visualizer_to_robot,
        node_send_single_joint_command
    ])