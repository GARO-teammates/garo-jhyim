#!/bin/bash
# RX-1 IK Package 설치 스크립트
# 이 스크립트는 IK 모듈을 1_Robot_GUI 폴더에 복사합니다.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$SCRIPT_DIR/../1_Robot_GUI"

echo "=================================================="
echo "RX-1 IK Package 설치"
echo "=================================================="

# Check if target directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo "✗ 오류: 1_Robot_GUI 폴더를 찾을 수 없습니다."
    echo "  예상 경로: $TARGET_DIR"
    exit 1
fi

# Check if rx1_ik_custom.py already exists
if [ -f "$TARGET_DIR/rx1_ik_custom.py" ]; then
    echo "! 주의: rx1_ik_custom.py가 이미 존재합니다."
    read -p "덮어쓸까요? (y/N): " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        echo "설치 취소됨."
        exit 0
    fi
    # Backup existing file
    cp "$TARGET_DIR/rx1_ik_custom.py" "$TARGET_DIR/rx1_ik_custom.py.backup"
    echo "  기존 파일 백업됨: rx1_ik_custom.py.backup"
fi

# Copy the IK module
cp "$SCRIPT_DIR/rx1_ik_custom.py" "$TARGET_DIR/"
echo "✓ rx1_ik_custom.py 복사 완료"

echo ""
echo "=================================================="
echo "설치 완료!"
echo "=================================================="
echo ""
echo "다음 단계:"
echo "1. Real_RRR_GUI_v5.py의 init_ik_solver() 함수 수정"
echo "   (README.md 참고)"
echo ""
echo "2. GUI 실행 후 IK 활성화 테스트"
echo ""
