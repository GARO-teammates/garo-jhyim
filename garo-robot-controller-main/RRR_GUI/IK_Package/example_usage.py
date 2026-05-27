#!/usr/bin/env python3
"""
RX-1 IK 사용 예제

이 스크립트는 rx1_ik_custom.py의 사용법을 보여줍니다.
실행 전에 rx1_ik_custom.py가 같은 폴더에 있어야 합니다.
"""

import sys
import json

# IK 모듈 임포트
from rx1_ik_custom import RX1ArmIK, RIGHT_ARM_SERVO_IDS, ik_debug_clear

def main():
    # 디버그 로그 초기화
    ik_debug_clear()

    # ========================================
    # 1. 로봇 베이스라인 로드
    # ========================================
    # rx1_baseline_v5.json에서 로봇의 기본 자세 서보값 로드
    baseline_path = '../1_Robot_GUI/rx1_baseline_v5.json'

    try:
        with open(baseline_path) as f:
            data = json.load(f)
        robot_baseline = {sid: data['servos'][str(sid)]['position']
                          for sid in RIGHT_ARM_SERVO_IDS}
        print("✓ 베이스라인 로드 완료")
        print(f"  서보값: {robot_baseline}")
    except FileNotFoundError:
        print(f"✗ 베이스라인 파일을 찾을 수 없습니다: {baseline_path}")
        print("  rx1_baseline_v5.json 경로를 확인하세요.")
        return

    # ========================================
    # 2. IK 솔버 생성
    # ========================================
    ik = RX1ArmIK('right', robot_baseline=robot_baseline)
    print("✓ IK 솔버 생성 완료")

    # ========================================
    # 3. Forward Kinematics (FK) 테스트
    # ========================================
    print("\n" + "="*50)
    print("Forward Kinematics 테스트")
    print("="*50)

    # 베이스라인 위치에서 FK 계산
    pos = ik.get_end_effector_position(robot_baseline)
    print(f"베이스라인 엔드이펙터 위치:")
    print(f"  X = {pos[0]*100:.2f} cm")
    print(f"  Y = {pos[1]*100:.2f} cm")
    print(f"  Z = {pos[2]*100:.2f} cm")

    # ========================================
    # 4. Inverse Kinematics (IK) 테스트
    # ========================================
    print("\n" + "="*50)
    print("Inverse Kinematics 테스트")
    print("="*50)

    # X축 +5cm 이동
    print("\n[테스트 1] X축 +5cm 이동")
    target_x = [pos[0] + 0.05, pos[1], pos[2]]
    new_servos_x, success_x = ik.solve_for_position(target_x, robot_baseline)

    if success_x:
        result_x = ik.get_end_effector_position(new_servos_x)
        dx = (result_x[0] - pos[0]) * 100
        dy = (result_x[1] - pos[1]) * 100
        dz = (result_x[2] - pos[2]) * 100
        print(f"  결과: dX={dx:+.2f}cm, dY={dy:+.2f}cm, dZ={dz:+.2f}cm")
        print(f"  서보 변화: {dict((k, new_servos_x[k] - robot_baseline[k]) for k in robot_baseline)}")
    else:
        print("  ✗ IK 실패")

    # Y축 +5cm 이동
    print("\n[테스트 2] Y축 +5cm 이동")
    target_y = [pos[0], pos[1] + 0.05, pos[2]]
    new_servos_y, success_y = ik.solve_for_position(target_y, robot_baseline)

    if success_y:
        result_y = ik.get_end_effector_position(new_servos_y)
        dx = (result_y[0] - pos[0]) * 100
        dy = (result_y[1] - pos[1]) * 100
        dz = (result_y[2] - pos[2]) * 100
        print(f"  결과: dX={dx:+.2f}cm, dY={dy:+.2f}cm, dZ={dz:+.2f}cm")
    else:
        print("  ✗ IK 실패")

    # Z축 +5cm 이동
    print("\n[테스트 3] Z축 +5cm 이동")
    target_z = [pos[0], pos[1], pos[2] + 0.05]
    new_servos_z, success_z = ik.solve_for_position(target_z, robot_baseline)

    if success_z:
        result_z = ik.get_end_effector_position(new_servos_z)
        dx = (result_z[0] - pos[0]) * 100
        dy = (result_z[1] - pos[1]) * 100
        dz = (result_z[2] - pos[2]) * 100
        print(f"  결과: dX={dx:+.2f}cm, dY={dy:+.2f}cm, dZ={dz:+.2f}cm")
    else:
        print("  ✗ IK 실패")

    # ========================================
    # 5. 요약
    # ========================================
    print("\n" + "="*50)
    print("요약")
    print("="*50)
    all_passed = success_x and success_y and success_z
    if all_passed:
        print("✓ 모든 IK 테스트 통과!")
    else:
        print("✗ 일부 테스트 실패")

    print("\n디버그 로그 위치: ik_debug.txt")

if __name__ == "__main__":
    main()
