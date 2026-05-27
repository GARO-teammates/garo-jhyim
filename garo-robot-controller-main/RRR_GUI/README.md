# RRR GUI Complete Package

RX1 듀얼암 로봇 제어를 위한 완전한 GUI 패키지

## 빠른 시작
```bash
./start_gui_v7.sh
# 또는
cd 1_Robot_GUI && python3 Real_RRR_GUI_v5.py
```

## 주요 기능

### 1. 로봇 제어
- 서보 모터 연결 및 실시간 제어
- 슬라이더/키보드 제어
- 기본 자세 저장/로드

### 2. 텔레오퍼레이션
- 텔레옵 장비 연동 (60Hz)
- 기어비 자동 적용 (1:3)
- 실시간 동기화

### 3. 카메라 & 녹화
- Gemini2 RGB 자동 감지
- C270 웹캠 지원
- Pi0.5 LeRobot v2 포맷 녹화
- 카메라 멈춤 자동 감지/경고

### 4. ROS2 연동
- URDF/Xacro 지원
- RViz 시각화
- Launch 파일 제공

### 5. Isaac Sim 연동
- USD 파일 제공
- 시뮬레이션 명령 연동

## 폴더 구조
```
├── 1_Robot_GUI/          # 메인 GUI
├── IK_Package/           # IK 솔버
├── datasets/             # 녹화 데이터
├── isaac_sim_integration/ # Isaac Sim
├── ros_files/            # ROS2 파일
├── urdf/                 # URDF/USD/메쉬
├── INSTALL_GUIDE.md      # 설치 가이드
└── start_gui_v7.sh       # 시작 스크립트
```

## 의존성
```bash
pip3 install pyserial pillow opencv-python numpy
```

## 상세 문서
- 각 폴더의 README.md 참조
- INSTALL_GUIDE.md: 전체 설치 가이드
