# RRR GUI 업데이트 (2026-01-24 ~ 2026-01-27)

## 변경된 파일

| 파일 | 복사 위치 |
|------|----------|
| `Real_RRR_GUI_v5.py` | `RRR_GUI/1_Robot_GUI/` |
| `rx1_ik_custom.py` | `RRR_GUI/1_Robot_GUI/` |
| `joint_state_publisher.py` | `RRR_GUI/ros_files/` |
| `Pi05_Inference_GUI.py` | `RRR_GUI/1_Robot_GUI/` |
| `PI_RRR.sh` | `RRR/` (루트) |

## 적용 방법

```bash
cd /home/rl02/Desktop/RRR/RRR_GUI/updates_2026_01_24

# Real_RRR_GUI 관련
cp Real_RRR_GUI_v5.py ../1_Robot_GUI/
cp rx1_ik_custom.py ../1_Robot_GUI/
cp joint_state_publisher.py ../ros_files/

# Pi0.5 Inference GUI 관련
cp Pi05_Inference_GUI.py ../1_Robot_GUI/
cp PI_RRR.sh /home/rl02/Desktop/RRR/
```

---

## 변경 내용 (Real_RRR_GUI)

### 1. VR 그리퍼 범위 확대
- 트리거 100% → 그리퍼 4095 (이전: 2000)
- 슬라이더와 동일한 범위로 움직임

### 2. VR IK 리밋 개선
- 리밋 걸려도 전체 IK 멈추지 않음 (개별 관절만 클램핑)
- 펜딩 콜백 취소로 지연 현상 해결
- 리밋 복귀 시 즉시 반영

### 3. VR 팔 선택 버튼 (All / L / R)
- VR 텔레옵 옆에 버튼 추가
- All: 양쪽 팔 모두 움직임
- L: 왼손만 (오른손 고정)
- R: 오른손만 (왼손 고정)

### 4. 6-DOF IK 적용
- 위치(x,y,z) + 방향(roll,pitch,yaw) 동시 제어
- 13, 23번 서보(arm rotation)도 움직임
- 현재 서보값에서만 IK 시작 (튀는 현상 방지)
- 프레임당 최대 변화량 제한 (MAX_DELTA=150)

### 5. 리프트 제어 (Q/A 키)
- ROS joint_state_publisher에 리프트 연동 추가
- `lift_commands_v5.json` 읽어서 `base_to_lift_joint`에 반영
- 키 디바운스 0.05초 (가속 현상 방지)

### 6. joint_state_publisher 경로 수정
- sim_commands: `ros_files/isaac_sim_integration/`
- wheel_commands: `RRR_GUI/isaac_sim_integration/` (v5)
- lift_commands: `RRR_GUI/isaac_sim_integration/` (v5)

### 11. ★★★ 데이터 녹화 버그 수정 (2026-01-27) ★★★

**심각한 버그 발견 및 수정:**

기존 문제:
- 녹화 시 `명령값(commanded position)` 저장
- `robot_baseline + teleop_delta` 계산값 사용
- baseline 파일 변경 시 모든 데이터 불일치 발생

수정 내용:
- 녹화 시 `실제 서보 피드백(actual position)` 저장
- `read_real_servo_position()` 함수 사용하여 직접 읽기
- 텔레옵과 녹화 동시 동작 가능

```python
# 수정 전 (잘못됨)
raw_pos = self.real_active_joints[joint_id]['current']

# 수정 후 (올바름)
actual_pos = self.read_real_servo_position(joint_id)
raw_pos = actual_pos if actual_pos is not None else fallback
```

**기존 데이터 영향:**
- 2026-01-27 이전 수집 데이터는 명령값으로 저장됨
- baseline 파일 변경 시 학습/추론 불일치 발생 가능
- 새로 수집하는 데이터는 정상

---

## 변경 내용 (Pi0.5 Inference GUI) - 2026-01-27

### 7. 신규 모델 폴더 구조 지원
기존 구조와 신규 구조 모두 지원:
```
# 기존 (v10, v11)
pi0.5_trained/v10/model.safetensors

# 신규 (v20, v21, v22)
pi0.5_trained/v20/010000/pretrained_model/model.safetensors
pi0.5_trained/v20/020000/pretrained_model/model.safetensors
...
```

### 8. preprocessor safetensors 지원
- 기존: `stats.json`에서 정규화 통계 로드
- 신규: `policy_preprocessor_step_2_normalizer_processor.safetensors`에서 로드
- 자동 감지하여 둘 중 하나 사용

### 9. PI_RRR.sh 개선
- 시작 시 모델 개수 표시
- GPU/CUDA 상태 확인
- ROS2 환경 자동 설정

### 10. GPU 메모리 관리 개선
- 모델 전환 시 이전 모델 자동 해제
- `torch.cuda.empty_cache()` + `gc.collect()` 호출
- CUDA Out of Memory 오류 방지

---

## IK 파라미터

```python
# 6-DOF IK
max_iterations=50    # 빠른 응답
pos_weight=1.0       # 위치 우선
rot_weight=0.3       # 방향은 보조

# 튀는 현상 방지
MAX_DELTA=150        # 프레임당 최대 서보 변화량
```

---

## 모델 다운로드 (v20, v21, v22)

```bash
# SSH 키 설정 필요
ssh-copy-id reality

# 다운로드 스크립트
~/Desktop/RRR/pi0.5_trained/download_all.sh
```

각 버전당 4개 체크포인트 (010000, 020000, 040000, 080000)
총 12개 모델, 약 78GB
