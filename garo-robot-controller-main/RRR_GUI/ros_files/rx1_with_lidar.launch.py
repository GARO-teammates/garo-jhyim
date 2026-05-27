#!/usr/bin/env python3
"""
RX-1 Robot + RPLIDAR A2M12 통합 Launch File
- 로봇 시각화 (URDF + robot_state_publisher)
- 라이다 노드 (sllidar_ros2)
- 하나의 RViz에서 모두 표시
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Base directory
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    print("=" * 60)
    print("[DEBUG] Launch file 시작")
    print(f"[DEBUG] Base dir: {base_dir}")

    # URDF file path (독립형 - 상대경로 사용)
    urdf_dir = os.path.join(base_dir, 'urdf/RRR_Cater/RX1/rx1_standalone')
    urdf_file = os.path.join(urdf_dir, 'rx1_with_lidar.urdf')
    print(f"[DEBUG] URDF file: {urdf_file}")
    print(f"[DEBUG] URDF exists: {os.path.exists(urdf_file)}")

    # RViz config file path
    rviz_config = os.path.join(base_dir, 'rx1_lidar_config.rviz')
    print(f"[DEBUG] RViz config: {rviz_config}")
    print(f"[DEBUG] RViz config exists: {os.path.exists(rviz_config)}")

    # Read URDF file
    try:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
        print(f"[DEBUG] URDF 로드 성공! 크기: {len(robot_description)} bytes")
        print(f"[DEBUG] URDF 첫 100자: {robot_description[:100]}")
    except Exception as e:
        print(f"[ERROR] URDF 로드 실패: {e}")
        robot_description = ""

    # Replace relative mesh paths with absolute file:// URIs (독립형 URDF용)
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="file://{urdf_dir}/meshes/'
    )
    # 기존 package:// 경로도 처리 (혹시 있을 경우)
    robot_description = robot_description.replace(
        'package://RRR_Cater',
        f'file://{base_dir}/urdf/RRR_Cater'
    )
    robot_description = robot_description.replace(
        'package://rx1_description',
        f'file://{base_dir}/urdf/RRR_Cater/RX1'
    )

    # 변환 후 확인
    pkg_count = robot_description.count('package://')
    file_count = robot_description.count('file://')
    link_count = robot_description.count('<link')
    joint_count = robot_description.count('<joint')
    print(f"[DEBUG] 변환 후 - package:// 남음: {pkg_count}, file://: {file_count}")
    print(f"[DEBUG] URDF 내용 - links: {link_count}, joints: {joint_count}")
    print("=" * 60)

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
        cmd=['/usr/bin/python3', os.path.join(base_dir, 'ros_files/joint_state_publisher.py')],
        output='screen'
    )

    # RPLIDAR A2M12 노드 - 자동으로 RPLIDAR 포트 찾기
    import glob
    import subprocess

    def find_rplidar_port():
        """RPLIDAR USB 포트 자동 탐지"""
        # 먼저 symlink 확인
        if os.path.exists('/dev/rplidar'):
            print("[RPLIDAR] Found /dev/rplidar symlink")
            return '/dev/rplidar'

        # ttyUSB 포트들 검색
        usb_ports = sorted(glob.glob('/dev/ttyUSB*'))
        available_ports = []  # 서보 컨트롤러가 아닌 포트들

        for port in usb_ports:
            try:
                # udevadm으로 vendor/product ID 확인
                result = subprocess.run(
                    ['udevadm', 'info', '-q', 'property', '-n', port],
                    capture_output=True, text=True, timeout=2
                )
                output = result.stdout

                # RPLIDAR CP2102 (10c4:ea60) 확인
                if 'ID_VENDOR_ID=10c4' in output:
                    print(f"[RPLIDAR] Found CP2102 at {port}")
                    return port
                # CH340은 서보 컨트롤러용 - RPLIDAR 아님, 스킵!
                if 'ID_VENDOR_ID=1a86' in output:
                    print(f"[RPLIDAR] Skipping CH340 at {port} (servo controller)")
                    continue
                # CH340이 아닌 다른 포트는 후보로 추가
                available_ports.append(port)
            except Exception as e:
                print(f"[RPLIDAR] Error checking {port}: {e}")
                available_ports.append(port)  # 에러 시에도 후보로

        # RPLIDAR가 없으면 None 반환 (lidar 노드 실행 안 함)
        if available_ports:
            print(f"[RPLIDAR] Using available port: {available_ports[-1]}")
            return available_ports[-1]

        print("[RPLIDAR] No RPLIDAR port found (only servo controllers detected)")
        return None  # RPLIDAR 없음

    serial_port = find_rplidar_port()
    print(f"[RPLIDAR] Final port selection: {serial_port}")

    # RViz node with config (GUI에서 별도 실행하므로 사용 안 함)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    # 기본 노드들
    nodes = [robot_state_publisher_node, joint_state_publisher_process]

    # RPLIDAR가 연결된 경우에만 sllidar_node 추가
    if serial_port:
        sllidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen'
        )
        nodes.append(sllidar_node)
        print("[RPLIDAR] RPLIDAR node added to launch")
    else:
        print("[RPLIDAR] No RPLIDAR connected - skipping sllidar_node (servo port protected)")

    return LaunchDescription(nodes)
