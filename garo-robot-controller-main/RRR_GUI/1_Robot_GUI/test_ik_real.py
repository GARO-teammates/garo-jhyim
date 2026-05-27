#!/usr/bin/env python3
"""
실제 로봇 IK 테스트
==================
수정된 IK 코드가 실제 로봇에서 작동하는지 확인

사용법:
    python3 test_ik_real.py

출력된 서보값을 GUI에 입력하거나 로봇에 직접 전송해서 확인하세요.
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rx1_ik_accurate import RX1AccurateIK, RIGHT_ARM_SERVO_IDS, ik_debug_clear
import numpy as np

def print_separator(title=""):
    print("\n" + "=" * 60)
    if title:
        print(f" {title}")
        print("=" * 60)

def test_basic_ik():
    """기본 IK 테스트"""
    print_separator("RX-1 실제 로봇 IK 테스트")

    ik_debug_clear()

    # IK 초기화
    ik = RX1AccurateIK('right')
    print(f"활성 관절: {len(ik.active_indices)}개")
    print(f"인덱스: {ik.active_indices}")

    # 제로 포지션
    servo_zero = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}
    pos_zero = ik.get_end_effector_position(servo_zero)

    print_separator("1. 제로 서보 (2048) 기준점")
    print(f"EE 위치: X={pos_zero[0]*100:.2f}cm, Y={pos_zero[1]*100:.2f}cm, Z={pos_zero[2]*100:.2f}cm")
    print(f"         X={pos_zero[0]*1000:.1f}mm, Y={pos_zero[1]*1000:.1f}mm, Z={pos_zero[2]*1000:.1f}mm")

    # 테스트 케이스들
    tests = [
        ("X +5cm", [pos_zero[0] + 0.05, pos_zero[1], pos_zero[2]]),
        ("X +10cm", [pos_zero[0] + 0.10, pos_zero[1], pos_zero[2]]),
        ("Y +5cm", [pos_zero[0], pos_zero[1] + 0.05, pos_zero[2]]),
        ("Z +5cm", [pos_zero[0], pos_zero[1], pos_zero[2] + 0.05]),
    ]

    current_servos = servo_zero.copy()

    for name, target in tests:
        print_separator(f"2. {name} 이동")
        print(f"목표: X={target[0]*100:.2f}cm, Y={target[1]*100:.2f}cm, Z={target[2]*100:.2f}cm")

        ik.reset()
        new_servos, success = ik.solve_for_position(target, servo_zero)

        if success:
            result = ik.get_end_effector_position(new_servos)

            # 오차 계산
            err_x = abs(result[0] - target[0]) * 1000
            err_y = abs(result[1] - target[1]) * 1000
            err_z = abs(result[2] - target[2]) * 1000

            print(f"결과: X={result[0]*100:.2f}cm, Y={result[1]*100:.2f}cm, Z={result[2]*100:.2f}cm")
            print(f"오차: dX={err_x:.2f}mm, dY={err_y:.2f}mm, dZ={err_z:.2f}mm")

            print(f"\n>>> 서보 명령 <<<")
            for sid in RIGHT_ARM_SERVO_IDS:
                delta = new_servos[sid] - 2048
                marker = "*" if abs(delta) > 50 else ""
                print(f"    Servo {sid}: {new_servos[sid]:4d}  (Δ={delta:+4d}) {marker}")
        else:
            print("IK 실패!")

    print_separator("테스트 완료")
    print("""
다음 단계:
1. 위 서보값을 GUI에서 직접 입력하거나 로봇에 전송
2. 실제 로봇 EE 위치가 목표와 일치하는지 확인
3. 불일치하면 URDF 모델 또는 서보 변환에 문제가 있음
""")

def test_incremental():
    """점진적 이동 테스트"""
    print_separator("점진적 X 이동 테스트 (1cm 단위)")

    ik = RX1AccurateIK('right')
    servo_current = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}
    pos_zero = ik.get_end_effector_position(servo_current)

    print(f"시작점: X={pos_zero[0]*100:.2f}cm")
    print(f"\nX 이동 (Y,Z 고정):")

    for x_cm in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
        target = [pos_zero[0] + x_cm/100, pos_zero[1], pos_zero[2]]
        new_servos, success = ik.solve_for_position(target, servo_current)

        if success:
            result = ik.get_end_effector_position(new_servos)
            dy_mm = (result[1] - pos_zero[1]) * 1000
            dz_mm = (result[2] - pos_zero[2]) * 1000

            # 주요 서보 변화만 표시
            s11 = new_servos[11] - 2048
            s14 = new_servos[14] - 2048

            print(f"  X+{x_cm:2d}cm: S11={new_servos[11]:4d}(Δ{s11:+4d}), S14={new_servos[14]:4d}(Δ{s14:+4d}) | dY={dy_mm:+.1f}mm, dZ={dz_mm:+.1f}mm")
            servo_current = new_servos
        else:
            print(f"  X+{x_cm:2d}cm: FAILED")
            break

if __name__ == "__main__":
    test_basic_ik()
    test_incremental()
