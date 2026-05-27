# GARO Robot GUI - Standalone Package

GARO 듀얼암 로봇 제어를 위한 독립 실행형 패키지입니다.
이 폴더만으로 모든 GUI 기능을 실행할 수 있습니다.

## 빠른 시작

```bash
# 1. 메인 로봇 제어 GUI 실행
./start_gui.sh

# 2. Pi0.5 추론 GUI 실행 (모델 필요)
./start_inference_gui.sh
```

## 사전 요구사항

### Conda 환경
```bash
conda create -n RRR python=3.10
conda activate RRR
pip install numpy pyserial opencv-python pillow
```

### 추가 패키지 (선택)
```bash
# Pi0.5 추론용
pip install torch lerobot h5py

# VR 텔레옵용
pip install openvr

# 음성 인식용
pip install SpeechRecognition googletrans==4.0.0-rc.1

# ROS2 (별도 설치 필요)
# ROS2 Humble: https://docs.ros.org/en/humble/Installation.html
```

## 폴더 구조

```
RRR_Standalone/
├── start_gui.sh              # 메인 GUI 시작 스크립트
├── start_inference_gui.sh    # 추론 GUI 시작 스크립트
├── README.md                 # 이 파일
│
├── RRR_GUI/                  # GUI 코어
│   ├── 1_Robot_GUI/          # 메인 Python 코드
│   │   ├── Real_RRR_GUI_v5.py       # 메인 GUI (서보, 카메라, IK, 텔레옵)
│   │   ├── Pi05_Inference_GUI.py     # Pi0.5 추론 GUI
│   │   ├── rx1_ik_custom.py          # FK/IK 솔버
│   │   ├── rx1_baseline_v5.json      # 서보 기본 위치
│   │   ├── teleop_baseline_v5.json   # 텔레옵 기본 위치
│   │   └── camera_config.json        # 카메라 설정
│   │
│   ├── ros_files/            # ROS2 launch 파일, joint state publisher
│   ├── isaac_sim_integration/ # Isaac Sim 연동 JSON
│   ├── urdf/                 # 로봇 URDF + 메시 파일 (RViz 시각화용)
│   ├── datasets/             # 녹화 데이터 저장 (빈 폴더, 자동 생성)
│   ├── updates_2026_01_24/   # 업데이트 파일
│   └── pi05_inference_files/ # 추론 설정/통계 파일
│
├── pi0.5_trained/            # 학습된 모델 (빈 폴더 - README 참고)
│   └── README.md             # 모델 폴더 구조 설명
│
└── ros2_ws/                  # (선택) ROS2 워크스페이스 - 별도 빌드 필요
```

## 주요 기능

- **서보 제어**: 17개 서보 실시간 제어 (양팔 7DOF + 그리퍼)
- **카메라**: 3대 카메라 실시간 피드 (탑뷰, 좌/우 손목)
- **텔레옵**: 텔레옵 장치를 통한 원격 조종
- **VR 텔레옵**: SteamVR 컨트롤러 지원
- **IK 제어**: XYZ 좌표 기반 팔 제어
- **Isaac Sim**: Isaac Sim 시뮬레이터 연동
- **ROS2**: RViz 시각화, LIDAR 통합
- **데이터 수집**: LeRobot v2 포맷 녹화/재생
- **Pi0.5 추론**: 학습된 VLA 모델로 자율 동작

## 다른 컴퓨터에 설치

1. 이 폴더 전체를 복사합니다
2. Conda 환경을 설치합니다 (위 사전 요구사항 참고)
3. `./start_gui.sh`로 실행합니다
4. Pi0.5 추론을 사용하려면 `pi0.5_trained/` 폴더에 모델을 넣습니다
