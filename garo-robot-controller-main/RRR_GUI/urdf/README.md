# urdf - 로봇 모델 파일

## 구조
```
RRR_Cater/RX1/
├── rx1_description/     # ROS 패키지 형식
│   ├── urdf/           # URDF, USD 파일
│   ├── meshes/         # STL, OBJ 메쉬
│   ├── launch/         # ROS1 launch
│   └── rviz/           # RViz 설정
├── rx1_standalone/      # 독립 실행용
│   ├── rx1_with_lidar.urdf
│   ├── rx1_with_lidar.usda  # Isaac Sim용
│   └── meshes/
└── combined/            # 통합 URDF
    └── rx1_with_lidar.urdf
```

## USD 파일 (Isaac Sim)
- `rx1_description/urdf/rx1_calibrated_v8/rx1_calibrated_v8.usd`: 최신 보정 버전
- `rx1_standalone/rx1_with_lidar.usda`: 독립 실행용
