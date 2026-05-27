#!/bin/bash
# RX-1 IK 마커 ROS2 노드 실행 스크립트
# RViz에서 IK 엔드포인트 시각화를 위해 실행

source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

echo "=== RX-1 IK 마커 노드 시작 ==="
echo "RViz에서 /ik_markers 토픽을 MarkerArray로 추가하세요"
echo ""
echo "마커 설명:"
echo "  - 파란 구: 현재 FK 위치 (End Effector)"
echo "  - 녹색 구: IK 목표 위치"
echo "  - 노란 선: FK → IK 연결선"
echo ""
echo "종료: Ctrl+C"
echo ""

/usr/bin/python3 rx1_ik_marker.py --ros
