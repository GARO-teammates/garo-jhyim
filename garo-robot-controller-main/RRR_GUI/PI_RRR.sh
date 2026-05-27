#!/bin/bash
# Pi0.5 Inference GUI Start Script
# 학습된 Pi0.5 모델로 로봇 제어

echo "=========================================="
echo "  Pi0.5 Inference GUI"
echo "=========================================="

# Determine script location dynamically
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SCRIPT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

# 사용 가능한 모델 확인
echo ""
echo "[모델 확인]"
MODEL_DIR="$SCRIPT_DIR/pi0.5_trained"
if [ -d "$MODEL_DIR" ]; then
    # 기존 구조 (v10, v11)
    OLD_MODELS=$(find "$MODEL_DIR" -maxdepth 2 -name "model.safetensors" -not -path "*/pretrained_model/*" 2>/dev/null | wc -l)
    # 신규 구조 (v20/010000/pretrained_model)
    NEW_MODELS=$(find "$MODEL_DIR" -name "model.safetensors" -path "*/pretrained_model/*" 2>/dev/null | wc -l)
    TOTAL=$((OLD_MODELS + NEW_MODELS))
    echo "  발견된 모델: $TOTAL 개"

    if [ $TOTAL -eq 0 ]; then
        echo "  [경고] 모델이 없습니다!"
        echo "  모델을 먼저 다운로드하세요."
    fi
else
    echo "  [경고] 모델 폴더가 없습니다: $MODEL_DIR"
fi

# RRR conda 환경 활성화
echo ""
echo "[환경 설정]"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate RRR
echo "  [OK] Conda RRR 환경"

# ROS2 환경 (필요시)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  [OK] ROS2 Humble"
fi

# CUDA 확인
echo ""
echo "[GPU 확인]"
python3 -c "import torch; print(f'  CUDA: {torch.cuda.is_available()}, GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')" 2>/dev/null || echo "  [경고] PyTorch CUDA 확인 실패"

# 업데이트 파일 복사
UPDATE_DIR="$SCRIPT_DIR/RRR_GUI/updates_2026_01_24"
if [ -d "$UPDATE_DIR" ] && [ -f "$UPDATE_DIR/Pi05_Inference_GUI.py" ]; then
    echo ""
    echo "[업데이트 적용]"
    cp "$UPDATE_DIR/Pi05_Inference_GUI.py" "$SCRIPT_DIR/RRR_GUI/1_Robot_GUI/"
    echo "  [OK] Pi05_Inference_GUI.py 업데이트 적용"
fi

# GUI 실행
echo ""
echo "=========================================="
echo "  GUI 시작..."
echo "=========================================="
cd "$SCRIPT_DIR/RRR_GUI"
python3 1_Robot_GUI/Pi05_Inference_GUI.py
