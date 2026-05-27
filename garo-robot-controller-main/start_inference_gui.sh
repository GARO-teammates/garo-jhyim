#!/bin/bash
# ============================================
#  Pi0.5 Inference GUI - Standalone Start Script
#  사용법: ./start_inference_gui.sh
# ============================================

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
GUI_DIR="$SCRIPT_DIR/RRR_GUI"

echo "=========================================="
echo "  Pi0.5 Inference GUI 시작 (Standalone)"
echo "=========================================="
echo "프로젝트 경로: $SCRIPT_DIR"
echo ""

# Conda 환경 활성화
if [ -f "$HOME/miniconda3/bin/activate" ]; then
    source "$HOME/miniconda3/bin/activate" RRR
    echo "[OK] Conda RRR"
elif [ -f "$HOME/anaconda3/bin/activate" ]; then
    source "$HOME/anaconda3/bin/activate" RRR
    echo "[OK] Conda RRR"
fi
echo ""

# 모델 폴더 확인
if [ ! -d "$SCRIPT_DIR/pi0.5_trained" ] || [ -z "$(ls -A "$SCRIPT_DIR/pi0.5_trained" 2>/dev/null | grep -v README)" ]; then
    echo "[!] 경고: pi0.5_trained/ 폴더에 모델이 없습니다."
    echo "    모델 체크포인트를 pi0.5_trained/ 폴더에 넣어주세요."
    echo "    자세한 내용: pi0.5_trained/README.md"
    echo ""
fi

# GUI 실행
cd "$GUI_DIR/1_Robot_GUI"
python3 Pi05_Inference_GUI.py
