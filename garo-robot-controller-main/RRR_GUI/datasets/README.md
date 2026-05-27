# 데이터셋 폴더

GUI에서 데이터를 녹화하면 이 폴더에 저장됩니다.

## 폴더 구조

```
datasets/
└── rx1_teleop_v1/
    └── YYYY_MM_DD/
        └── pick_and_place/
            ├── episode_0000/
            │   ├── metadata.json
            │   ├── episode_data.json
            │   ├── observation_images_top/
            │   ├── observation_images_wrist_left/
            │   └── observation_images_wrist_right/
            ├── episode_0001/
            └── ...
```

## 설명

- LeRobot v2 포맷으로 저장됩니다
- 각 에피소드에는 관절 상태, 카메라 이미지(3대), 메타데이터가 포함됩니다
- 카메라: top (탑뷰), wrist_right (우손목), wrist_left (좌손목)
- 기존 데이터가 있으면 이 폴더에 복사해 넣으면 재생/분석이 가능합니다
