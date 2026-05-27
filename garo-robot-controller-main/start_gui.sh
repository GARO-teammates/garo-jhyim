#!/bin/bash
# ============================================
#  RRR Robot GUI - Standalone Start Script
#  사용법: ./start_gui.sh
# ============================================

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
GUI_DIR="$SCRIPT_DIR/RRR_GUI"

echo "=========================================="
echo "  RRR Robot GUI v7 시작 (Standalone)"
echo "=========================================="
echo "프로젝트 경로: $SCRIPT_DIR"
echo ""

# ========== Conda 환경 활성화 ==========
echo "Conda RRR 환경 활성화 중..."
if [ -f "$HOME/miniconda3/bin/activate" ]; then
    source "$HOME/miniconda3/bin/activate" RRR
    echo "  [OK] Conda RRR (miniconda3)"
elif [ -f "$HOME/anaconda3/bin/activate" ]; then
    source "$HOME/anaconda3/bin/activate" RRR
    echo "  [OK] Conda RRR (anaconda3)"
else
    echo "  [WARN] Conda를 찾을 수 없습니다. 시스템 Python을 사용합니다."
fi
echo ""

# ========== ROS2 환경 설정 ==========
echo "ROS2 환경 설정 중..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  [OK] ROS2 Humble"
fi
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
    echo "  [OK] ros2_ws (sllidar)"
fi
echo ""

# ========== 의존성 확인 ==========
echo "의존성 확인 중..."
python3 -c "import numpy, tkinter, serial, cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "  [!] 일부 Python 패키지가 없습니다."
    echo "      다음 명령으로 설치하세요:"
    echo "      pip3 install numpy pyserial opencv-python pillow"
    echo ""
fi

# ========== 업데이트 파일 적용 ==========
UPDATE_DIR="$GUI_DIR/updates_2026_01_24"
if [ -d "$UPDATE_DIR" ]; then
    echo "업데이트 파일 적용 중..."
    cp "$UPDATE_DIR/Real_RRR_GUI_v5.py" "$GUI_DIR/1_Robot_GUI/" 2>/dev/null
    cp "$UPDATE_DIR/rx1_ik_custom.py" "$GUI_DIR/1_Robot_GUI/" 2>/dev/null
    cp "$UPDATE_DIR/joint_state_publisher.py" "$GUI_DIR/ros_files/" 2>/dev/null
    echo "  [OK] 업데이트 적용 완료"
    echo ""
fi

# ========== GUI 실행 ==========
echo "GUI 실행 중..."
echo ""
cd "$GUI_DIR/1_Robot_GUI"
python3 Real_RRR_GUI_v5.py
