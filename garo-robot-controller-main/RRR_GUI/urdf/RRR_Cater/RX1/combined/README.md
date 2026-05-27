# RX1 URDF Files

## 사용 중인 URDF

| 파일 | 용도 | 사용 위치 |
|------|------|----------|
| `rx1_with_lidar.urdf` | GUI v5 ROS/RViz 시각화 | `Real_RRR_GUI_v5.py:3632` |
| `rx1_with_lift_cater_v4.urdf` | ROS2 launch 시각화 | `ros_files/rx1_visualize.launch.py:15` |

## 사용법

### 1. GUI v5에서 RViz 실행
```bash
./start_gui_v7.sh
# GUI에서 "RViz 시작" 버튼 클릭
```
- 자동으로 `rx1_with_lidar.urdf` 로드
- joint_state_publisher와 연동

### 2. Launch 파일로 직접 실행
```bash
source /opt/ros/humble/setup.bash
cd /home/rl02/Desktop/RRR/RRR_GUI
ros2 launch ros_files/rx1_visualize.launch.py
```
- `rx1_with_lift_cater_v4.urdf` 사용 (리프트 포함)

## 파일 구조

```
combined/
├── rx1_with_lidar.urdf          # GUI v5 메인 URDF
├── rx1_with_lift_cater_v4.urdf  # Launch용 (리프트 포함)
├── *.obj                         # 메쉬 파일들
└── README.md
```

## 메쉬 파일

| 파일 | 설명 |
|------|------|
| `Cater.obj` | 케이터 본체 |
| `Cater_frame.obj` | 케이터 프레임 |
| `lidar.obj` | 라이다 |
| `lift1.obj` | 리프트 |
| `carter_wheel.obj` | 구동 바퀴 |
| `caster_wheel.obj` | 캐스터 바퀴 |
| `NEMA42.obj` | 모터 |
| `XP1000.obj` | 배터리 |
| `pivot.obj` | 피벗 |
