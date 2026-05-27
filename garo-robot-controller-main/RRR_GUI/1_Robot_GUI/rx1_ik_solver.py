"""
RX-1 Robot Inverse Kinematics Solver
=====================================
Isaac Sim에서 완벽하게 검증된 empirical Jacobian 데이터 사용
원본: /home/rl02/Downloads/ik/rx1_ik_controller.py

주요 특징:
- Damped Least Squares (Levenberg-Marquardt) IK 솔버
- Isaac Sim 측정 기반 정확한 Jacobian
- ROS2 연동 가능 (joint_states, TF)
- 실시간 FK/IK 제공

GitHub rx1_ik 베이스 + Isaac Sim 검증 데이터 적용
"""

import os
import numpy as np
import math
from typing import Tuple, Optional, Dict, List
from datetime import datetime

# ============================================================================
# Debug Logging
# ============================================================================

DEBUG_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ik_debug.txt")
DEBUG_ENABLED = True


def ik_debug_log(msg):
    """IK 디버그 로그"""
    if not DEBUG_ENABLED:
        return
    try:
        with open(DEBUG_LOG_FILE, 'a') as f:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            f.write(f"[{timestamp}] {msg}\n")
    except:
        pass


def ik_debug_clear():
    """디버그 로그 초기화"""
    try:
        with open(DEBUG_LOG_FILE, 'w') as f:
            f.write(f"=== RX-1 IK Debug Log ===\n")
            f.write(f"시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"솔버: Empirical Jacobian + Damped Least Squares\n")
            f.write(f"데이터: Isaac Sim 검증 완료\n")
            f.write(f"=" * 50 + "\n\n")
    except:
        pass


# ============================================================================
# EMPIRICAL JACOBIAN DATA (Isaac Sim에서 측정 - 검증 완료)
# ============================================================================
# 원본: /home/rl02/Downloads/ik/rx1_ik_controller.py
# 각 관절을 ±0.3 rad 움직였을 때 end effector 위치 변화 측정
#
# Joint mapping (7-DOF per arm):
#   0: shoulder yaw (shoul_base2shoul_joint)
#   1: shoulder pitch (shoul2shoul_rot_joint)
#   2: upper arm rotation (arm2armrot_joint)
#   3: elbow (armrot2elbow_joint) - NOTE: 음수 방향!
#   4: forearm rotation (forearm2forearmrot_joint)
#   5: wrist pitch (forearmrot2forearm_pitch_joint)
#   6: wrist roll (forearm_pitch2forearm_roll_joint)
# ============================================================================

# Left arm: Joint effects (Isaac Sim 측정 - hand_link 기준)
LEFT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([-0.183, -0.014, +0.024]), "test_angle": +0.3},  # shoulder yaw
    1: {"delta": np.array([+0.000, +0.211, +0.032]), "test_angle": +0.3},  # shoulder pitch
    2: {"delta": np.array([-0.000, +0.000, -0.000]), "test_angle": +0.3},  # upper arm rotation
    3: {"delta": np.array([+0.109, -0.000, +0.017]), "test_angle": -0.3},  # elbow (negative!)
    4: {"delta": np.array([+0.000, +0.000, -0.000]), "test_angle": +0.3},  # forearm rotation
    5: {"delta": np.array([-0.030, -0.000, +0.004]), "test_angle": +0.3},  # wrist pitch
    6: {"delta": np.array([+0.000, +0.030, +0.004]), "test_angle": +0.3},  # wrist roll
}

# Right arm: Joint effects (Left arm과 Y축 대칭)
RIGHT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([+0.183, +0.014, +0.024]), "test_angle": +0.3},  # shoulder yaw (Y 반전)
    1: {"delta": np.array([-0.000, +0.211, +0.032]), "test_angle": +0.3},  # shoulder pitch
    2: {"delta": np.array([-0.000, +0.000, +0.000]), "test_angle": +0.3},  # upper arm rotation
    3: {"delta": np.array([+0.109, +0.000, +0.017]), "test_angle": -0.3},  # elbow (negative!)
    4: {"delta": np.array([+0.000, +0.000, +0.000]), "test_angle": +0.3},  # forearm rotation
    5: {"delta": np.array([-0.030, +0.000, +0.004]), "test_angle": +0.3},  # wrist pitch
    6: {"delta": np.array([-0.000, +0.030, +0.004]), "test_angle": +0.3},  # wrist roll (Y 반전)
}

# Joint limits (URDF 기준)
JOINT_LIMITS = {
    'left': [
        (-3.14, 3.14),   # 0: shoulder yaw
        (-3.14, 3.14),   # 1: shoulder pitch
        (-3.14, 3.14),   # 2: upper arm rotation
        (-2.2, 0.0),     # 3: elbow (upper limit = 0!)
        (-3.14, 3.14),   # 4: forearm rotation
        (-3.14, 3.14),   # 5: wrist pitch
        (-3.14, 3.14),   # 6: wrist roll
    ],
    'right': [
        (-3.14, 3.14),
        (-3.14, 3.14),
        (-3.14, 3.14),
        (-2.2, 0.0),     # elbow
        (-3.14, 3.14),
        (-3.14, 3.14),
        (-3.14, 3.14),
    ]
}

# Gripper center position at zero joint angles (from URDF FK)
# End effector: gripper_center_link (xyz="0.0 0.0 -0.03" from gripper_base)
INITIAL_EE_POSITIONS = {
    'left': np.array([0.0, 0.5153, -0.2425]),
    'right': np.array([0.0, -0.5153, -0.2425])
}


# ============================================================================
# Servo Mapping (rx1_motor.hpp 원본)
# ============================================================================

# Right Arm Servos
RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
RIGHT_ARM_SERVO_DIRS = [-1, -1, 1, 1, 1, 1, -1]
RIGHT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

# Left Arm Servos
LEFT_ARM_SERVO_IDS = [21, 22, 23, 24, 25, 26, 27]
LEFT_ARM_SERVO_DIRS = [-1, -1, 1, -1, 1, -1, -1]
LEFT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

# ROS Joint Names (GitHub rx1_ik 원본)
LEFT_ARM_JOINTS = [
    "left_shoul_base2shoul_joint",
    "left_shoul2shoul_rot_joint",
    "left_arm2armrot_joint",
    "left_armrot2elbow_joint",
    "left_forearm2forearmrot_joint",
    "left_forearmrot2forearm_pitch_joint",
    "left_forearm_pitch2forearm_roll_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoul_base2shoul_joint",
    "right_shoul2shoul_rot_joint",
    "right_arm2armrot_joint",
    "right_armrot2elbow_joint",
    "right_forearm2forearmrot_joint",
    "right_forearmrot2forearm_pitch_joint",
    "right_forearm_pitch2forearm_roll_joint",
]

SERVO_CENTER = 2048


def servo_to_joint_angle(servo_pos: int, direction: int, gear: int) -> float:
    """서보 위치 → 관절 각도 (라디안)"""
    relative = servo_pos - SERVO_CENTER
    angle = relative / (SERVO_CENTER * direction * gear) * math.pi
    return angle


def joint_angle_to_servo(angle: float, direction: int, gear: int) -> int:
    """관절 각도 (라디안) → 서보 위치"""
    servo_pos = angle / math.pi * SERVO_CENTER * direction * gear + SERVO_CENTER
    servo_pos = max(0, min(4095, int(servo_pos)))
    return servo_pos


# ============================================================================
# Differential IK Solver (Damped Least Squares)
# ============================================================================

class DifferentialIKSolver:
    """
    Damped Least Squares (Levenberg-Marquardt) IK Solver

    공식: dq = J^T (J J^T + λ²I)^(-1) * e

    특징:
    - 특이점 근처에서도 안정적
    - damping 파라미터로 수렴 속도/안정성 조절
    """

    def __init__(
        self,
        num_joints: int = 7,
        damping: float = 0.05,
        position_gain: float = 1.0,
    ):
        self.num_joints = num_joints
        self.damping = damping
        self.position_gain = position_gain

    def compute_joint_delta(
        self,
        jacobian: np.ndarray,
        position_error: np.ndarray,
    ) -> np.ndarray:
        """
        위치 오차에서 관절 변화량 계산

        Args:
            jacobian: 3xN Jacobian 행렬
            position_error: 3D 위치 오차 [dx, dy, dz]

        Returns:
            joint_delta: N차원 관절 변화량
        """
        JJT = jacobian @ jacobian.T
        damping_term = self.damping ** 2 * np.eye(3)

        try:
            inv_term = np.linalg.inv(JJT + damping_term)
            joint_delta = jacobian.T @ inv_term @ (position_error * self.position_gain)
        except np.linalg.LinAlgError:
            # 역행렬 실패 시 pseudo-inverse
            joint_delta = np.linalg.pinv(jacobian) @ (position_error * self.position_gain)

        return joint_delta


# ============================================================================
# RX1 Arm IK (단일 팔)
# ============================================================================

class RX1ArmIK:
    """
    RX-1 단일 팔 IK 솔버 (Isaac Sim 검증 버전)

    인터페이스:
    - forward(joint_angles) → XYZ 위치
    - inverse(target_xyz, current_angles) → joint_angles, success
    - get_end_effector_position(servo_values) → XYZ
    - solve_for_position(target_xyz, servo_values) → servo_values, success
    """

    def __init__(self, arm: str = 'right'):
        """
        Args:
            arm: 'right' 또는 'left'
        """
        self.arm = arm
        self.num_joints = 7

        # Servo mapping
        if arm == 'right':
            self.servo_ids = RIGHT_ARM_SERVO_IDS
            self.servo_dirs = RIGHT_ARM_SERVO_DIRS
            self.servo_gears = RIGHT_ARM_SERVO_GEARS
            self.joint_names = RIGHT_ARM_JOINTS
        else:
            self.servo_ids = LEFT_ARM_SERVO_IDS
            self.servo_dirs = LEFT_ARM_SERVO_DIRS
            self.servo_gears = LEFT_ARM_SERVO_GEARS
            self.joint_names = LEFT_ARM_JOINTS

        # Jacobian 빌드
        self._build_jacobian()

        # 관절 한계
        self.joint_limits = JOINT_LIMITS[arm]

        # 초기 EE 위치
        self._initial_ee_pos = INITIAL_EE_POSITIONS[arm].copy()

        # 현재 관절 상태 (반복 IK용)
        self._current_joints = np.zeros(self.num_joints)

        # DLS 솔버
        self._dls_solver = DifferentialIKSolver(
            num_joints=self.num_joints,
            damping=0.05,
            position_gain=3.0
        )

        ik_debug_log(f"[{arm}] RX1ArmIK 초기화 완료 (Isaac Sim 검증 데이터)")

    def _build_jacobian(self):
        """Empirical 데이터에서 Jacobian 행렬 생성"""
        data = LEFT_ARM_JACOBIAN_DATA if self.arm == 'left' else RIGHT_ARM_JACOBIAN_DATA

        self.jacobian = np.zeros((3, self.num_joints))

        for i in range(self.num_joints):
            delta = data[i]["delta"]
            test_angle = data[i]["test_angle"]
            # 라디안당 EE 변화로 정규화
            self.jacobian[:, i] = delta / test_angle

    def servo_to_angles(self, servo_values) -> np.ndarray:
        """서보 값 → 7개 관절 각도 (라디안)"""
        angles = []
        for i in range(7):
            if isinstance(servo_values, dict):
                sid = self.servo_ids[i]
                servo_pos = servo_values.get(sid, SERVO_CENTER)
            else:
                servo_pos = servo_values[i] if i < len(servo_values) else SERVO_CENTER

            angle = servo_to_joint_angle(
                servo_pos,
                self.servo_dirs[i],
                self.servo_gears[i]
            )
            angles.append(angle)

        return np.array(angles)

    def angles_to_servo(self, angles: np.ndarray) -> Dict[int, int]:
        """7개 관절 각도 (라디안) → 서보 값 dict"""
        servo_values = {}
        for i in range(7):
            angle = angles[i] if i < len(angles) else 0.0
            servo_pos = joint_angle_to_servo(
                angle,
                self.servo_dirs[i],
                self.servo_gears[i]
            )
            servo_values[self.servo_ids[i]] = servo_pos

        return servo_values

    def forward(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        순기구학: 관절 각도 → EE 위치

        선형 근사: EE_pos = initial_pos + J * joints
        """
        joint_angles = np.array(joint_angles).flatten()[:self.num_joints]

        ee_delta = self.jacobian @ joint_angles
        ee_pos = self._initial_ee_pos + ee_delta

        return ee_pos

    def inverse(
        self,
        target_xyz: np.ndarray,
        current_angles: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 0.005,  # 5mm
    ) -> Tuple[np.ndarray, bool]:
        """
        역기구학: 목표 XYZ → 관절 각도

        Args:
            target_xyz: [x, y, z] 목표 위치 (meters)
            current_angles: 초기 관절 각도
            max_iterations: 최대 반복
            tolerance: 허용 오차 (meters)

        Returns:
            (joint_angles, success)
        """
        target = np.array(target_xyz).flatten()[:3]

        # 초기값
        if current_angles is not None:
            joints = np.array(current_angles).flatten()[:self.num_joints]
        else:
            joints = self._current_joints.copy()

        best_joints = joints.copy()
        best_error = float('inf')

        # 반복 IK
        for iteration in range(max_iterations):
            current_ee = self.forward(joints)
            error = target - current_ee
            error_norm = np.linalg.norm(error)

            # 최적 저장
            if error_norm < best_error:
                best_error = error_norm
                best_joints = joints.copy()

            # 수렴 확인
            if error_norm < tolerance:
                self._current_joints = joints.copy()
                return joints, True

            # DLS로 관절 변화량 계산
            delta_joints = self._dls_solver.compute_joint_delta(self.jacobian, error)

            # 스텝 크기 (가까워지면 느리게)
            step_size = min(1.0, error_norm / 0.1)

            # 관절 업데이트
            joints = joints + delta_joints * step_size

            # 관절 한계 적용
            joints = self._enforce_limits(joints)

        # 최대 반복 도달
        self._current_joints = best_joints.copy()
        success = best_error < tolerance * 2

        return best_joints, success

    def _enforce_limits(self, joints: np.ndarray) -> np.ndarray:
        """관절 한계 적용"""
        for i in range(self.num_joints):
            lower, upper = self.joint_limits[i]
            joints[i] = np.clip(joints[i], lower, upper)
        return joints

    def reset(self):
        """상태 초기화"""
        self._current_joints = np.zeros(self.num_joints)

    def get_end_effector_position(self, servo_values) -> np.ndarray:
        """현재 서보 값에서 EE 위치 계산 (FK)"""
        angles = self.servo_to_angles(servo_values)
        return self.forward(angles)

    def solve_for_position(
        self,
        target_xyz,
        servo_values,
        max_iterations: int = 100
    ) -> Tuple[Dict[int, int], bool]:
        """
        목표 위치로 가기 위한 서보 값 계산

        Args:
            target_xyz: [x, y, z] 목표 위치
            servo_values: 현재 서보 값
            max_iterations: 최대 IK 반복

        Returns:
            (new_servo_values, success)
        """
        current_pos = self.get_end_effector_position(servo_values)
        target = np.array(target_xyz)

        ik_debug_log(f"========== IK solve_for_position ({self.arm}) ==========")
        ik_debug_log(f"현재 FK: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
        ik_debug_log(f"목표:    X={target[0]:.4f}, Y={target[1]:.4f}, Z={target[2]:.4f}")
        ik_debug_log(f"이동량:  dX={target[0]-current_pos[0]:.4f}, dY={target[1]-current_pos[1]:.4f}, dZ={target[2]-current_pos[2]:.4f}")

        current_angles = self.servo_to_angles(servo_values)
        distance = np.linalg.norm(target - current_pos)

        # 큰 이동은 스텝으로 분할 (부드러운 궤적)
        if distance > 0.08:  # 8cm 이상
            num_steps = min(10, max(3, int(distance / 0.03)))
            ik_debug_log(f"큰 이동 ({distance*100:.1f}cm) → {num_steps} 스텝 분할")

            angles = current_angles.copy()
            for step in range(num_steps):
                t = (step + 1) / num_steps
                intermediate = current_pos + t * (target - current_pos)
                angles, _ = self.inverse(
                    intermediate, angles,
                    max_iterations=max(50, max_iterations // num_steps)
                )
            new_angles = angles
            final_error = np.linalg.norm(self.forward(angles) - target)
            success = final_error < 0.05
        else:
            new_angles, success = self.inverse(target, current_angles, max_iterations)

        if success:
            new_servos = self.angles_to_servo(new_angles)
            result_pos = self.get_end_effector_position(new_servos)
            error_mm = np.linalg.norm(result_pos - target) * 1000

            ik_debug_log(f"IK 성공! 오차: {error_mm:.2f}mm")
            ik_debug_log(f"결과 FK: X={result_pos[0]:.4f}, Y={result_pos[1]:.4f}, Z={result_pos[2]:.4f}")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log(f"IK 실패!")
            ik_debug_log("")
            return {}, False

    def get_workspace_bounds(self) -> Dict:
        """작업 공간 범위 반환"""
        if self.arm == 'right':
            return {'x': (-0.2, 0.5), 'y': (-0.7, 0.0), 'z': (-0.4, 0.8)}
        else:
            return {'x': (-0.2, 0.5), 'y': (0.0, 0.7), 'z': (-0.4, 0.8)}

    def get_joint_names(self) -> List[str]:
        """관절 이름 반환 (ROS 연동용)"""
        return self.joint_names.copy()


# ============================================================================
# RX1 Dual Arm IK
# ============================================================================

class RX1DualArmIK:
    """
    RX-1 양팔 IK 솔버
    """

    def __init__(self):
        self.left = RX1ArmIK('left')
        self.right = RX1ArmIK('right')

    def forward(self, arm: str, joint_angles: np.ndarray) -> np.ndarray:
        """순기구학"""
        solver = self.right if arm == 'right' else self.left
        return solver.forward(joint_angles)

    def inverse(
        self,
        arm: str,
        target_xyz: np.ndarray,
        **kwargs
    ) -> Tuple[np.ndarray, bool]:
        """역기구학"""
        solver = self.right if arm == 'right' else self.left
        return solver.inverse(target_xyz, **kwargs)

    def reset(self):
        """양팔 초기화"""
        self.left.reset()
        self.right.reset()


# Alias for backward compatibility
RX1IKSolver = RX1DualArmIK


# ============================================================================
# Test Code
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("RX-1 IK Solver Test (Isaac Sim 검증 버전)")
    print("=" * 60)

    ik_debug_clear()

    # Right arm test
    print("\n--- Right Arm Test ---")
    right_ik = RX1ArmIK('right')

    # FK test (zero joints)
    zero_angles = np.zeros(7)
    pos = right_ik.forward(zero_angles)
    print(f"FK (joints=0): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

    # FK test (elbow bent)
    bent_angles = np.zeros(7)
    bent_angles[3] = -0.5  # Bend elbow
    pos_bent = right_ik.forward(bent_angles)
    print(f"FK (elbow=-0.5): [{pos_bent[0]:.4f}, {pos_bent[1]:.4f}, {pos_bent[2]:.4f}]")

    # IK test
    print("\n--- IK Test ---")
    right_ik.reset()
    target = [0.15, -0.30, 0.50]
    print(f"Target: {target}")

    angles, success = right_ik.inverse(target)
    print(f"IK success: {success}")
    print(f"Joint angles (deg): {np.degrees(angles).round(1)}")

    # Verify FK
    verify_pos = right_ik.forward(angles)
    error = np.linalg.norm(np.array(target) - verify_pos) * 1000
    print(f"FK verify: [{verify_pos[0]:.4f}, {verify_pos[1]:.4f}, {verify_pos[2]:.4f}]")
    print(f"Error: {error:.1f} mm")

    # Left arm test
    print("\n--- Left Arm Test ---")
    left_ik = RX1ArmIK('left')

    pos_l = left_ik.forward(np.zeros(7))
    print(f"FK (joints=0): [{pos_l[0]:.4f}, {pos_l[1]:.4f}, {pos_l[2]:.4f}]")

    target_l = [0.15, 0.30, 0.50]
    angles_l, success_l = left_ik.inverse(target_l)
    print(f"IK target: {target_l}")
    print(f"IK success: {success_l}")

    verify_l = left_ik.forward(angles_l)
    error_l = np.linalg.norm(np.array(target_l) - verify_l) * 1000
    print(f"Error: {error_l:.1f} mm")

    # X-only movement test (중요!)
    print("\n--- X-only Movement Test ---")
    servo_center = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}

    target1 = [0.1, -0.25, 0.53]
    target2 = [0.2, -0.25, 0.53]

    servo1, _ = right_ik.solve_for_position(target1, servo_center)
    pos1 = right_ik.get_end_effector_position(servo1)

    servo2, _ = right_ik.solve_for_position(target2, servo1)
    pos2 = right_ik.get_end_effector_position(servo2)

    print(f"Move X: 0.1 → 0.2")
    print(f"  dX = {(pos2[0]-pos1[0])*1000:.1f} mm (target: 100)")
    print(f"  dY = {(pos2[1]-pos1[1])*1000:.1f} mm (target: 0)")
    print(f"  dZ = {(pos2[2]-pos1[2])*1000:.1f} mm (target: 0)")

    print("\n" + "=" * 60)
    print("Test Complete!")
    print(f"Debug log: {DEBUG_LOG_FILE}")
    print("=" * 60)
