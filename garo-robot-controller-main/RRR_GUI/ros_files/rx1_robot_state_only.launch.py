#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # 독립형 URDF (상대경로 사용)
    urdf_dir = os.path.join(base_dir, 'urdf/RRR_Cater/RX1/rx1_standalone')
    urdf_file = os.path.join(urdf_dir, 'rx1_with_lidar.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 상대 경로를 절대 file:// 경로로 변환
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="file://{urdf_dir}/meshes/'
    )

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

    return LaunchDescription([
        robot_state_publisher_node,
    ])
