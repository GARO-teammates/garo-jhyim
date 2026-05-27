#!/usr/bin/env python3
"""
RX-1 Robot Visualization Launch File (with Carter)
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Base directory
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # URDF file path (Carter included)
    urdf_file = os.path.join(base_dir, 'urdf/RRR_Cater/RX1/combined/rx1_with_lift_cater_v4.urdf')

    # RViz config file path
    rviz_config = os.path.join(base_dir, 'rx1_config.rviz')

    # Package path for meshes
    package_path = os.path.join(base_dir, 'urdf/RRR_Cater/RX1/combined')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Replace package:// URIs with file:// URIs
    robot_description = robot_description.replace(
        'package://RRR_Cater',
        f'file://{base_dir}/urdf/RRR_Cater'
    )
    robot_description = robot_description.replace(
        'package://rx1_description',
        f'file://{base_dir}/urdf/RRR_Cater/RX1'
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher process
    joint_state_publisher_process = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'joint_state_publisher.py')],
        output='screen'
    )

    # RViz node with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_process,
        rviz_node
    ])
