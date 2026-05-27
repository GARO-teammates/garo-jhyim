#!/usr/bin/env python3
"""
에피소드 재생 스크립트
저장된 에피소드의 관절값을 로봇에 그대로 재생합니다.
"""

import os
import json
import serial
import time
import sys

# 설정
ROBOT_PORT = "/dev/ttyUSB0"  # 로봇 시리얼 포트
BAUDRATE = 1000000
PLAYBACK_FPS = 11  # 재생 속도 (녹화 FPS와 동일)
SERVO_ACC = 6  # ACC 값 (GUI 기본값과 동일)

# 관절 ID 순서 (학습용 16 DOF)
JOINT_IDS = [11, 12, 13, 14, 15, 16, 17, 41, 21, 22, 23, 24, 25, 26, 27, 31]

def set_servo_acc(ser, servo_id, acc_value):
    """서보 ACC(가속도) 설정 (레지스터 0x29)"""
    acc = max(0, min(254, acc_value))
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x03, 0x29, acc]
    checksum = (~sum(packet[2:]) % 256) & 0xFF
    ser.write(bytes(packet + [checksum]))
    time.sleep(0.003)

def set_all_servo_acc(ser, servo_ids, acc_value):
    """모든 서보에 ACC 설정"""
    print(f"ACC={acc_value} 설정 중...")
    for servo_id in servo_ids:
        set_servo_acc(ser, servo_id, acc_value)
    print("ACC 설정 완료")

def send_servo_position(ser, servo_id, position):
    """STS3215 서보에 위치 명령 전송"""
    position = int(max(0, min(4095, position)))
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF

    # WRITE 명령: FF FF ID LEN CMD ADDR DATA_L DATA_H CHECKSUM
    packet = [0xFF, 0xFF, servo_id, 0x05, 0x03, 0x2A, pos_l, pos_h]
    checksum = (~sum(packet[2:]) % 256) & 0xFF

    ser.write(bytes(packet + [checksum]))

def sync_write_positions(ser, positions):
    """여러 서보에 동시에 위치 명령 전송 (Sync Write)"""
    # Sync Write: FF FF FE LEN CMD ADDR DATA_LEN [ID1 DATA1_L DATA1_H] [ID2 ...] CHECKSUM
    servo_data = []
    for servo_id, position in positions.items():
        position = int(max(0, min(4095, position)))
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        servo_data.extend([servo_id, pos_l, pos_h])

    data_len = 2  # 위치 데이터 길이 (2바이트)
    length = 4 + len(servo_data)  # LEN = 4 + (서보 수 * 3)

    packet = [0xFF, 0xFF, 0xFE, length, 0x83, 0x2A, data_len] + servo_data
    checksum = (~sum(packet[2:]) % 256) & 0xFF

    ser.write(bytes(packet + [checksum]))

def play_episode(episode_path, start_frame=0, end_frame=None):
    """에피소드 재생"""
    print(f"에피소드 로드: {episode_path}")

    with open(episode_path, 'r') as f:
        frames = json.load(f)

    total_frames = len(frames)
    if end_frame is None:
        end_frame = total_frames

    print(f"총 프레임: {total_frames}")
    print(f"재생 범위: {start_frame} ~ {end_frame}")
    print()

    # 로봇 연결
    print(f"로봇 연결 중... ({ROBOT_PORT})")
    try:
        ser = serial.Serial(ROBOT_PORT, BAUDRATE, timeout=0.1)
        time.sleep(0.5)
        print("로봇 연결 완료!")
    except Exception as e:
        print(f"로봇 연결 실패: {e}")
        return

    # ACC 설정 (GUI와 동일하게 6)
    set_all_servo_acc(ser, JOINT_IDS, SERVO_ACC)

    print()
    print("=" * 50)
    print("3초 후 재생 시작... (Ctrl+C로 중지)")
    print("=" * 50)
    time.sleep(3)

    interval = 1.0 / PLAYBACK_FPS

    try:
        for i in range(start_frame, min(end_frame, total_frames)):
            frame = frames[i]
            state_raw = frame['observation.state_raw']

            # 관절값을 딕셔너리로 변환
            positions = {}
            for j, joint_id in enumerate(JOINT_IDS):
                if j < len(state_raw):
                    positions[joint_id] = state_raw[j]

            # Sync Write로 전송
            sync_write_positions(ser, positions)

            # 진행 상황 출력 (10프레임마다)
            if i % 10 == 0:
                print(f"프레임 {i}/{end_frame} - L그리퍼(J31): {positions.get(31, 'N/A')}")

            time.sleep(interval)

        print()
        print("=" * 50)
        print("재생 완료!")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n재생 중지됨")
    finally:
        ser.close()

def main():
    if len(sys.argv) < 2:
        print("사용법: python3 play_episode.py <에피소드_번호> [시작_프레임] [끝_프레임]")
        print()
        print("예시:")
        print("  python3 play_episode.py 50        # episode_0050 전체 재생")
        print("  python3 play_episode.py 50 0 30   # 처음 30프레임만 재생")
        print()

        # 기본값으로 episode_0050 재생
        episode_num = 50
    else:
        episode_num = int(sys.argv[1])

    start_frame = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    end_frame = int(sys.argv[3]) if len(sys.argv) > 3 else None

    # 에피소드 경로
    _gui_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    base_path = os.path.join(_gui_root, "datasets", "rx1_teleop_v1", "2026_01_28", "pick_and_place")
    episode_path = f"{base_path}/episode_{episode_num:04d}/episode_data.json"

    play_episode(episode_path, start_frame, end_frame)

if __name__ == "__main__":
    main()
