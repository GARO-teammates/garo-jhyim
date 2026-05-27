# RRR GUI Complete Backup - 설치 가이드

## 포함된 기능
- **텔레오퍼레이션**: 텔레옵 장비 연동 (연동 Hz 조절 가능)
- **로봇 제어**: 서보 모터 제어 GUI
- **카메라**: Gemini2, C270 자동 감지 및 녹화
- **ROS2 연동**: URDF, Launch 파일, RViz 설정
- **Isaac Sim**: USD 파일, 시뮬레이션 연동
- **IK 솔버**: 역기구학 계산
- **데이터 수집**: Pi0.5 LeRobot v2 포맷 녹화

## 필요 의존성 (Ubuntu/Jetson)
```bash
# Python 패키지
pip3 install pyserial pillow opencv-python numpy

# ROS2 Humble (선택)
# 별도 설치 필요

# Orbbec SDK (Gemini2 카메라용, 선택)
# https://github.com/orbbec/OrbbecSDK_ROS2
```

## 실행 방법
```bash
cd RRR_GUI_Complete_Backup
./start_gui_v7.sh
# 또는
cd 1_Robot_GUI
python3 Real_RRR_GUI_v5.py
```

## 폴더 구조
```
RRR_GUI_Complete_Backup/
├── 1_Robot_GUI/          # 메인 GUI (Real_RRR_GUI_v5.py)
├── IK_Package/           # IK 솔버 모듈
├── datasets/             # 녹화 데이터 저장 (빈 폴더)
├── isaac_sim_integration/ # Isaac Sim 연동
├── ros_files/            # ROS2 launch 파일
├── urdf/                 # URDF, USD, 메쉬 파일
│   └── RRR_Cater/RX1/
│       ├── rx1_description/  # ROS 패키지 형식
│       ├── rx1_standalone/   # 독립 URDF
│       └── combined/         # 통합 URDF
└── start_gui_v7.sh       # 시작 스크립트
```

## 주의사항
- datasets 폴더는 빈 상태입니다 (녹화 시 자동 생성됨)
- USB 포트 경로는 시스템마다 다를 수 있음
- 카메라 config는 처음 실행 시 자동 감지됨
