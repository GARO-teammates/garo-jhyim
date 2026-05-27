#!/usr/bin/env python3
"""
초기 프레임 수정 스크립트
녹화 시작 시 캐시가 비어있어 잘못 저장된 프레임 0, 1, 2를
기본자세 값으로 교체합니다.
"""

import os
import json
from pathlib import Path

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_ROOT = os.path.dirname(_THIS_DIR)

# 수정할 에피소드 경로
BASE_PATH = Path(os.path.join(_GUI_ROOT, "datasets", "rx1_teleop_v1", "2026_01_28", "pick_and_place"))
BASELINE_PATH = Path(os.path.join(_THIS_DIR, "rx1_baseline_v5.json"))

# 학습용 관절 ID 순서
LEARNING_JOINT_IDS = [11, 12, 13, 14, 15, 16, 17, 41, 21, 22, 23, 24, 25, 26, 27, 31]

def load_baseline():
    """기본자세 값 로드"""
    with open(BASELINE_PATH, 'r') as f:
        data = json.load(f)

    baseline_raw = []
    for joint_id in LEARNING_JOINT_IDS:
        pos = data['servos'].get(str(joint_id), {}).get('position', 2048)
        baseline_raw.append(pos)

    return baseline_raw

def fix_episode(episode_dir, baseline_raw):
    """에피소드의 초기 프레임 수정"""
    episode_file = episode_dir / "episode_data.json"

    if not episode_file.exists():
        return False, "episode_data.json 없음"

    with open(episode_file, 'r') as f:
        frames = json.load(f)

    if len(frames) < 3:
        return False, f"프레임 수 부족 ({len(frames)})"

    # 프레임 0, 1, 2를 기본자세 값으로 교체
    for i in range(3):
        frames[i]['observation.state_raw'] = baseline_raw.copy()
        # 정규화 값도 업데이트
        normalized = [(pos - 2048) / 2048 for pos in baseline_raw]
        frames[i]['observation.state'] = normalized

    # 저장
    with open(episode_file, 'w') as f:
        json.dump(frames, f)

    return True, f"수정 완료 (프레임 0-2 → 기본자세)"

def main():
    if not BASE_PATH.exists():
        print(f"경로 없음: {BASE_PATH}")
        return

    # 기본자세 값 로드
    baseline_raw = load_baseline()
    print(f"기본자세 로드 완료: {baseline_raw[:4]}... (총 {len(baseline_raw)}개)")

    # 모든 에피소드 폴더 찾기
    episode_dirs = sorted(BASE_PATH.glob("episode_*"))

    print(f"총 {len(episode_dirs)}개 에피소드 발견")
    print("=" * 60)

    success_count = 0
    fail_count = 0

    for episode_dir in episode_dirs:
        episode_name = episode_dir.name
        success, message = fix_episode(episode_dir, baseline_raw)

        if success:
            print(f"✓ {episode_name}: {message}")
            success_count += 1
        else:
            print(f"✗ {episode_name}: {message}")
            fail_count += 1

    print("=" * 60)
    print(f"완료: {success_count}개 성공, {fail_count}개 실패")

if __name__ == "__main__":
    main()
