# ros_files - ROS2 관련 파일

## Launch 파일
- `rx1_with_lidar.launch.py`: 라이다 포함 전체 로봇
- `rx1_robot_only.launch.py`: 로봇만
- `rx1_visualize.launch.py`: RViz 시각화
- `rx1_robot_state_only.launch.py`: 상태 퍼블리셔만

## 기타 파일
- `joint_state_publisher.py`: 관절 상태 퍼블리셔
- `rx1_baseline.json`: ROS용 기본 자세
- `rx1_lidar_config.rviz`: RViz 설정

## 사용법
```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_files/rx1_with_lidar.launch.py
```
