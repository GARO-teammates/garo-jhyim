# 1_Robot_GUI - 메인 로봇 제어 GUI

## 메인 파일
- **Real_RRR_GUI_v5.py**: 메인 GUI 프로그램 (데이터 수집용)
- **Pi05_Inference_GUI.py**: Pi0.5 모델 추론 GUI

## 실행
```bash
# 데이터 수집
python3 Real_RRR_GUI_v5.py

# Pi0.5 추론
python3 Pi05_Inference_GUI.py
```

## 주요 기능
- 로봇 서보 연결/제어
- 텔레오퍼레이션 장비 연동
- 카메라 녹화 (Pi0.5 LeRobot v2 포맷)
- IK 솔버 연동
- **Pi0.5 VLM 기반 추론**

## 설정 파일
- `rx1_baseline_v5.json`: 로봇 기본 자세
- `teleop_baseline_v5.json`: 텔레옵 기본 자세
- `camera_config.json`: 카메라 USB 포트 매핑

## IK 관련 파일
- `rx1_ik_custom.py`: 커스텀 IK 솔버
- `rx1_ik_solver.py`: 기본 IK 솔버
- `rx1_ik_accurate.py`: 정밀 IK

---

# Pi0.5 추론 시스템 (Pi05_Inference_GUI.py)

## 개요

Pi0.5는 Physical Intelligence에서 개발한 Vision-Language-Action (VLA) 모델로,
카메라 이미지와 언어 명령을 입력받아 로봇 관절 액션을 출력합니다.

## 모델 버전별 설정

| 버전 | FPS | 특징 |
|------|-----|------|
| v31 | 50fps | 기존 학습 |
| v42, v43 | **11fps** | 새 학습 (권장) |

**중요**: 추론 FPS는 반드시 학습 FPS와 일치해야 합니다!

## 추론 아키텍처

### 1. 입력 (Observation)

```
┌─────────────────────────────────────────────────────────┐
│                    Observation                          │
├─────────────────────────────────────────────────────────┤
│  이미지 (3개 카메라)                                      │
│  ├─ top: 상단 카메라 (224x224, ImageNet 정규화)           │
│  ├─ wrist_right: 오른손목 카메라                          │
│  └─ wrist_left: 왼손목 카메라                             │
│                                                         │
│  관절 상태 (16개)                                         │
│  ├─ Index 0-7: 오른팔 (J11-J17, J41)                     │
│  └─ Index 8-15: 왼팔 (J21-J27, J31)                      │
│                                                         │
│  언어 명령                                                │
│  └─ 예: "Put the snack in the box"                       │
└─────────────────────────────────────────────────────────┘
```

### 2. 정규화 방식

#### 이미지 정규화 (ImageNet)
```python
mean = [0.485, 0.456, 0.406]
std = [0.229, 0.224, 0.225]
normalized = (image / 255.0 - mean) / std
```

#### 관절 상태 정규화 (MEAN_STD)
```python
# 학습 데이터에서 계산된 mean, std 사용
normalized = (value - mean) / std

# 역정규화 (추론 후)
value = normalized * std + mean
```

### 3. 출력 (Action Chunk)

모델은 한 번에 **50개 액션**을 예측합니다 (Action Chunking).

```
┌─────────────────────────────────────────────────────────┐
│              Action Chunk (50 actions)                  │
├─────────────────────────────────────────────────────────┤
│  action[0]:  [J11, J12, ..., J31] (16개 관절)            │
│  action[1]:  [J11, J12, ..., J31]                        │
│  ...                                                    │
│  action[49]: [J11, J12, ..., J31]                        │
└─────────────────────────────────────────────────────────┘

11fps 기준: 50개 = 4.5초 분량
50fps 기준: 50개 = 1.0초 분량
```

---

## RTC (Real-Time Chunking) 추론 방식

### 기존 방식의 문제점

```
[청크1 생성] ──→ [50개 전부 실행] ──→ [청크2 생성] ──→ ...
                              ↑
                         여기서 끊김/진동 발생
```

- 청크 전환 시 불연속점 발생
- 다음 청크가 완전히 다른 값으로 시작할 수 있음
- 로봇이 진동하거나 급격한 움직임 발생

### RTC 방식 (적용됨)

```
실행: ████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
                 ↑
          trigger_threshold(15) 도달
                 │
추론:            ├──[다음 청크 추론 시작]──→
                 │
                 ▼
      ░░░░░░░░░░░████████████████████████████████████████
                 └─ 블렌딩으로 부드럽게 연결
```

### RTC 핵심 개념

#### 1. 미리 추론 시작 (Trigger Threshold)
```python
trigger_threshold = 15  # 큐에 15개 이하 남으면 다음 추론 시작

if len(action_queue) <= trigger_threshold and not is_predicting:
    # 다음 청크 추론 시작 (비동기)
    start_async_prediction()
```

#### 2. Frozen Actions (하드 제약)
추론이 진행되는 동안 로봇은 계속 이전 청크를 실행합니다.
이 시간 동안 실행된 액션 수 = `inference_delay`

```python
inference_delay = int(inference_time * target_fps) + 1
# 예: 450ms 추론, 11fps → delay = 5 스텝

# 새 청크의 처음 5개는 이전 청크 값으로 고정
for i in range(inference_delay):
    blended[i] = prev_leftover[i]  # Frozen!
```

#### 3. Soft Blending (소프트 가이던스)
Frozen zone 이후는 지수 감소 가중치로 부드럽게 블렌딩:

```python
blend_decay = 0.7
execution_horizon = 10  # 블렌딩 영역

for i in range(inference_delay, execution_horizon):
    weight = blend_decay ** (execution_horizon - i - 1)
    # weight: 앞쪽은 이전 청크, 뒤로 갈수록 새 청크
    blended[i] = (1 - weight) * prev[i] + weight * new[i]
```

### RTC 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|-------|------|
| `execution_horizon` | 10 | 블렌딩 적용할 스텝 수 |
| `trigger_threshold` | 15 | 새 추론 시작 트리거 |
| `blend_decay` | 0.7 | 지수 감소율 (0.5~0.9) |

### RTC 타임라인 예시 (11fps)

```
시간    | 액션 큐 | 이벤트
--------|---------|----------------------------------
0.0s    | 50      | 첫 청크 실행 시작
...     |         |
3.2s    | 15      | trigger! 다음 청크 추론 시작
3.2s    | 15      | (추론 중, 실행 계속)
3.6s    | 11      | (추론 중, 실행 계속)
4.0s    | 7       | 추론 완료 (delay=5)
4.0s    | 50      | 블렌딩 적용, 새 큐로 교체
...     |         |
```

---

## 디버그 로그

### 로그 파일 위치
```
debug_logs/debug_YYYYMMDD_HHMMSS.txt
```

### 주요 로그 태그
- `[RTC]`: RTC 시스템 상태
- `[DEBUG]`: 모델 입출력 값 (정규화된 값)
- `[추론]`: 추론 관련 일반 정보

### 로그 예시
```
[RTC] 11 FPS 모드 시작 (chunk=50, horizon=10, trigger=15)
[RTC] 추론 요청 (남은 큐: 12, trigger: 15)
[RTC] 모델 #1: 450ms, delay=5스텝
[DEBUG] 첫액션(정규화): L arm=['-0.091', '0.130', ...]
[DEBUG] 끝액션(정규화): L arm=['0.064', '0.306', ...]
[RTC] 블렌딩 완료: prev=7, delay=5, horizon=10
```

---

## 관절 인덱스 매핑

```
인덱스 0-7:  오른팔
  0: J11 (어깨1)
  1: J12 (어깨2)
  2: J13 (어깨3)
  3: J14 (팔꿈치)
  4: J15 (손목1)
  5: J16 (손목2)
  6: J17 (손목3)
  7: J41 (그리퍼)

인덱스 8-15: 왼팔
  8:  J21 (어깨1)
  9:  J22 (어깨2)
  10: J23 (어깨3)
  11: J24 (팔꿈치)
  12: J25 (손목1)
  13: J26 (손목2)
  14: J27 (손목3)
  15: J31 (그리퍼)
```

---

## 문제 해결

### 진동 발생 시
1. RTC 파라미터 조정
   - `blend_decay` 낮추기 (0.5~0.6) → 더 부드러운 전환
   - `execution_horizon` 늘리기 (12~15) → 더 긴 블렌딩

2. FPS 확인
   - 모델 학습 FPS와 추론 FPS 일치 여부

### 반응 느림
1. `trigger_threshold` 높이기 (20~25) → 더 일찍 추론 시작
2. GPU 상태 확인

### 베이스라인 불일치
- 추론 시작 위치가 학습 데이터 분포와 2σ 이상 차이나면 성능 저하
- 로봇 홈 포지션을 학습 평균 근처로 조정 권장
