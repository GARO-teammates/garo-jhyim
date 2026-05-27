================================================================================
Pi0.5 추론 관련 파일 모음
================================================================================

폴더 경로: /home/rl02/Desktop/RRR/RRR_GUI/pi05_inference_files/

파일 목록:
--------------------------------------------------------------------------------

1. Pi05_Inference_GUI.py
   - 메인 추론 GUI 스크립트
   - 카메라 입력, 모델 추론, 서보 제어 담당

2. rx1_baseline_v5.json
   - 로봇 기본자세 설정
   - 각 서보의 baseline 위치값

3. camera_config.json
   - 카메라 설정 (USB 포트 매핑)

4. dataset_stats.json
   - 학습 데이터 정규화 통계
   - min, max, mean, std 값
   - QUANTILES 정규화에 사용

5. model_config.json
   - Pi0.5 모델 설정
   - input_features, output_features, normalization_mapping 등

6. sample_episode_metadata.json
   - 샘플 에피소드 메타데이터
   - fps, joint_names, camera_keys 등

7. sample_episode_data.json
   - 샘플 에피소드 실제 데이터
   - 프레임별 state, action, 이미지 경로

8. pi05_inference_analysis.txt
   - 추론 문제 분석 보고서
   - 수정 내역, 의심 문제점, 권장 조치

9. latest_debug_log.txt
   - 최신 디버그 로그
   - 추론 시 State, Action, Camera 상태

--------------------------------------------------------------------------------

참고 경로 (복사되지 않음):
- 모델 가중치: /home/rl02/Desktop/RRR/pi0.5_trained/red_snack_pp/model.safetensors
- 전체 데이터셋: /home/rl02/Desktop/RRR/RRR_GUI/datasets/rx1_teleop_v1/

================================================================================
