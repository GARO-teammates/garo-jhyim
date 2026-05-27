#!/bin/bash
# RRR Robot GUI v7 Start Script
# 사용법: ./start_gui_v7.sh

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

echo "=========================================="
echo "  RRR Robot GUI v7 시작"
echo "=========================================="
echo "경로: $SCRIPT_DIR"
echo ""

# Conda RRR 환경 활성화
echo "Conda RRR 환경 활성화 중..."
source "$HOME/miniconda3/bin/activate" RRR
echo "  [OK] Conda RRR"
echo ""

# ROS2 환경 설정
echo "ROS2 환경 설정 중..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  [OK] ROS2 Humble"
fi
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
    echo "  [OK] ros2_ws (sllidar)"
fi
echo ""

# Check dependencies
echo "의존성 확인 중..."
python3 -c "import numpy, tkinter, serial, cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "! 일부 Python 패키지가 없습니다."
    echo "  다음 명령으로 설치하세요:"
    echo "  pip3 install numpy pyserial opencv-python"
    echo ""
fi

# 업데이트 파일 복사 (2024-01-24)
UPDATE_DIR="$SCRIPT_DIR/updates_2024_01_24"
if [ -d "$UPDATE_DIR" ]; then
    echo "업데이트 파일 적용 중..."
    cp "$UPDATE_DIR/Real_RRR_GUI_v5.py" "$SCRIPT_DIR/1_Robot_GUI/"
    cp "$UPDATE_DIR/rx1_ik_custom.py" "$SCRIPT_DIR/1_Robot_GUI/"
    cp "$UPDATE_DIR/joint_state_publisher.py" "$SCRIPT_DIR/ros_files/"
    echo "  [OK] 업데이트 적용 완료"
    echo ""
fi

# Python GUI 실행
cd "$SCRIPT_DIR/1_Robot_GUI"
python3 Real_RRR_GUI_v5.py
