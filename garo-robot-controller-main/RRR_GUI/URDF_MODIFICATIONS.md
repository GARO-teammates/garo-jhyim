# URDF 수정 내역

## 수정 파일
`urdf/RRR_Cater/RX1/combined/rx1_with_lidar.urdf`

## 변경 내역

### 2026-01-06: left_gripper_tip1 위치 조정

**조인트**: `left_gripper_joint[31]`
**링크**: `left_gripper_tip1_link`

| 항목 | 원본 값 | 수정 값 | 변경량 |
|------|---------|---------|--------|
| Y 위치 | -0.0605 m | -0.0425 m | +1.8 cm |

**수정 전:**
```xml
<joint name="left_gripper_joint[31]" type="prismatic">
  <origin rpy="0.0 0.0 0.0" xyz="0.0 -0.0605 0.05" />
```

**수정 후:**
```xml
<joint name="left_gripper_joint[31]" type="prismatic">
  <origin rpy="0.0 0.0 0.0" xyz="0.0 -0.0425 0.05" />
```

## 참고사항

- 이 URDF 파일은 RRR_GUI_Package_v7과 함께 배포됩니다
- 다른 컴퓨터에서 사용 시 RRR_GUI_Package_v7 폴더 전체를 복사하면 수정된 URDF가 포함됩니다
- Isaac Sim USD 파일과 동기화가 필요한 경우 별도로 USD 파일도 수정해야 합니다
