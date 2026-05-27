# pi05_aloha_snack_gen5 (gen5_expert_only)

## 개요
이 체크포인트는 `pi05_aloha_snack_gen5` 설정을 기반으로 합니다. 기본 pi05 모델을 `garo` 데이터셋에 맞게 파인튜닝(fine-tuning)한 버전입니다. 가장 최근에 실행된 학습 세션입니다.

## 학습 세부 정보
- **기본 모델 (Base Model)**: `pi05_base` (`gemma_2b` PaliGemma + `gemma_300m` action expert)
- **데이터셋**: `garo/pi0_dataset_v43_new` (LeRobot 데이터 형식)
- **사용된 카메라**: `cam_high` (탑 뷰), `cam_left_wrist` (좌측 손목), `cam_right_wrist` (우측 손목)
- **총 학습 스텝**: 20,000 (현재 스텝 2000까지 저장됨)
- **배치 사이즈 (Batch Size)**: 16
- **학습률 (Learning Rate)**: Cosine Decay (`warmup_steps`=8000, `decay_steps`=80000, `peak_lr`=2.5e-5)
- **학습된 파라미터 (Unfrozen)**:
  - `llm.*_1.*`
  - `action_.*proj.*`
  - `time_mlp.*`
  *(기존에 학습된 지식을 보존하기 위해 Vision 인코더와 핵심 LLM 가중치를 포함한 나머지 모든 요소는 동결(frozen) 상태로 유지되었습니다.)*
- **FSDP 디바이스 수**: 2

## 실행 명령어
이 학습을 시작하기 위해 아래 명령어가 사용되었습니다:
```bash
export HF_DATASETS_CACHE=/data/jhyim0823/hf_cache
CUDA_VISIBLE_DEVICES=2,3 XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_aloha_snack_gen5 --exp-name=gen5_expert_only --overwrite
```

### 📈 Training Loss Trend
👉 [WandB Interactive Report에서 자세한 수치 확인하기](https://api.wandb.ai/links/sonicwarp_hello-soongsil-cyber-university/hrx5wrcf)

## 🚀 이전 최고 모델(v02) 대비 발전된 점 (FULL_MODEL_PERFORMANCE_ANALYSIS 기준)

기존 분석 보고서(`FULL_MODEL_PERFORMANCE_ANALYSIS.md`)에서 도출된 **가장 최적의 하이퍼파라미터**는 유지하되, **치명적인 문제점(소규모 데이터셋에서의 과적합)**을 해결하기 위한 강력한 전략이 추가되었습니다.

### 1. 그대로 계승한 성공 요인 (Copied from v02)
- **최적의 Learning Rate 유지 (`2.5e-5`)**: 더 빠르고 깊은 학습을 제공했던 높은 LR을 그대로 채택했습니다.
- **정규화 방식 (`MEAN_STD`)**: 실제 로봇 환경에서 실패했던 QUANTILES 방식을 배제하고 `use_quantile_norm=False`로 검증된 방식을 유지했습니다.
- **느린 Decay (`80,000 steps`) & Batch Size (`16`)**: 안정적인 궤적을 위해 유지되었습니다.

### 2. 새롭게 디벨롭된 핵심 포인트 (Developed over v02)
- **핵심 개발: Expert-Only 파라미터 동결 (Freeze Filter)**
  - 기존 모델들은 전체 파라미터(Full Model)를 학습시켜 "적은 데이터(278개)로 인한 심각한 과적합" 문제가 발생했습니다.
  - 본 모델은 거대 LLM 본체와 Vision 인코더 가중치를 동결하고, 오직 로봇 행동을 결정하는 **Action Expert 레이어(`llm.*_1.*`), Action 프로젝션, Time MLP만 선별적으로 학습**합니다. 이를 통해 파라미터 수를 대폭 줄여 **과적합을 원천 차단**했습니다.
- **Warmup Step 최적화 (`10,000` ➡️ `8,000`)**
  - 워밍업을 단축하여 Peak LR에 더 일찍 도달하게 만들었으며, 20,000 스텝 안에서 최적의 학습 구간(Golden Zone)을 앞당겨 확보하려는 의도가 반영되었습니다.
- **데이터셋 정제 (`pi0_dataset_v43` ➡️ `pi0_dataset_v43_new`)**
  - 기존 보고서에서 지적된 텔레오퍼레이션 시작 자세(J21, J31 모터 틱 값)와 추론 베이스라인의 불일치 문제를 해결한 정제된 데이터를 사용하여 모델 성능을 향상시켰습니다.
