#!/usr/bin/env python3
"""
RX-1 Custom FK/IK Implementation
================================
Pure Python FK/IK without ikpy dependency

- Direct transform computation from URDF parameters
- Numerical Jacobian calculation
- Damped Least Squares IK solver
"""

import numpy as np
import math
import os
from typing import Dict, List, Tuple, Optional
from datetime import datetime

# Debug log
_IK_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEBUG_LOG_FILE = os.path.join(_IK_THIS_DIR, "ik_debug.txt")

def ik_debug_log(msg):
    try:
        with open(DEBUG_LOG_FILE, 'a') as f:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            f.write(f"[{ts}] {msg}\n")
    except:
        pass

def ik_debug_clear():
    try:
        with open(DEBUG_LOG_FILE, 'w') as f:
            f.write(f"=== RX-1 Custom IK Debug Log ===\n")
            f.write(f"Start: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
    except:
        pass

# ============================================================
# 서보 설정 (시각화와 동일하게 맞춤)
# ============================================================

RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
LEFT_ARM_SERVO_IDS = [21, 22, 23, 24, 25, 26, 27]

# 시각화에서 사용하는 베이스라인 (ROS_BASELINE_OVERRIDE 적용)
# 서보 14는 4095, 나머지는 robot_baseline 사용
VIZ_BASELINE_OVERRIDE = {
    11: 2850, 12: 186, 13: 1984, 14: 4095, 15: 2991, 16: 2008,
    21: 1131, 22: 4003, 23: 1856, 24: 150, 25: 2061, 26: 2139
}

# 시각화에서 사용하는 반전 (RVIZ_AUTO_INVERTED + ROS_SIM_INVERTED 상쇄됨)
# 서보 14: (-1) * (-1) = 1, 즉 반전 없음
# 결론: 오른팔은 반전 없음
VIZ_RIGHT_ARM_INVERSIONS = [1, 1, 1, 1, 1, 1, 1]  # 모두 1 (반전 없음)

# 왼팔 반전 설정 - 하드웨어 반전 적용
# 서보 21: INVERTED → -1
# 서보 22: INVERTED → -1
# 서보 23: normal → 1
# 서보 24: REASSEMBLED → -1
# 서보 25-27: normal → 1
VIZ_LEFT_ARM_INVERSIONS = [-1, -1, 1, -1, 1, 1, 1]

# GUI's Real_RRR_GUI_v5.py
NO_GEARBOX_SERVOS = [15, 25]  # 360 deg
SERVO_CENTER = 2048

# 로봇 베이스라인 (JSON에서 로드, 초기화 시 설정됨)
ROBOT_BASELINE = {}

# ============================================================
# 4+3 Split IK 상수
# ============================================================

# 체인에서 손목 그룹이 시작되는 인덱스 (Joint 15/25, forearm roll)
WRIST_CHAIN_SPLIT_INDEX = 12

# 위치 그룹 (joints 0-3)의 null-space 선호 각도
# joint 3 (elbow) = -1.6 rad ≈ -92° → 팔꿈치가 자연스럽게 아래로
POSITION_PREFERRED_ANGLES = [0, 0, 0, -1.6]

# ============================================================
# 회전 행렬 유틸리티
# ============================================================

def rot_x(angle: float) -> np.ndarray:
    """X축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

def rot_y(angle: float) -> np.ndarray:
    """Y축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(angle: float) -> np.ndarray:
    """Z축 회전 행렬"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (Roll-Pitch-Yaw) → 회전 행렬 (URDF 규약: Z-Y-X)"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def make_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """4x4 변환 행렬 생성"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T

def axis_angle_to_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """축-각도 → 회전 행렬 (Rodrigues' formula)"""
    axis = axis / np.linalg.norm(axis)  # 정규화
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def rotation_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    회전 행렬 → 축-각도 (역변환)

    Returns:
        axis: 회전축 단위 벡터
        angle: 회전 각도 (라디안)
    """
    # trace(R) = 1 + 2*cos(angle)
    trace = np.trace(R)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))

    if abs(angle) < 1e-6:
        # 거의 회전 없음
        return np.array([0, 0, 1]), 0.0

    if abs(angle - np.pi) < 1e-6:
        # 180도 회전 - 특수 처리
        # R = I + 2*K^2 에서 K 추출
        # 대각 원소에서 축 방향 추출
        diag = np.diag(R)
        idx = np.argmax(diag)
        axis = np.zeros(3)
        axis[idx] = 1
        # 부호 결정
        if idx == 0:
            axis[1] = R[0, 1] / (2 * axis[0]) if axis[0] != 0 else 0
            axis[2] = R[0, 2] / (2 * axis[0]) if axis[0] != 0 else 0
        return axis / np.linalg.norm(axis), angle

    # 일반 케이스: axis = (R - R^T) / (2*sin(angle))
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))

    return axis / np.linalg.norm(axis), angle


def orientation_error(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """
    두 회전 행렬 사이의 방향 오차를 축-각도 벡터로 반환

    R_error = R_target @ R_current^T
    오차 벡터 = axis * angle (3D 벡터)

    Args:
        R_current: 현재 3x3 회전 행렬
        R_target: 목표 3x3 회전 행렬

    Returns:
        error: 3D 오차 벡터 (axis * angle)
    """
    R_error = R_target @ R_current.T
    axis, angle = rotation_to_axis_angle(R_error)
    return axis * angle


def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """
    회전 행렬 → RPY (Roll-Pitch-Yaw) 각도
    URDF 규약: Z-Y-X (yaw-pitch-roll)

    Returns:
        roll, pitch, yaw (라디안)
    """
    # ZYX Euler: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw


# ============================================================
# URDF 기반 킨레마틱 체인 정의 (world 기준, ROS TF와 동일)
# ============================================================

# 오른팔 체인 (world 기준)
# world → base_link → lift → torso → head_base → right arm → gripper_center
#
# URDF joints 추출:
#   world_joint: xyz=[0, 0, 0.621251]
#   base_to_lift_joint: xyz=[-0.041, 0, 0.99] (prismatic, assume 0)
#   torso_yaw2pitch_joint: xyz=[0.03, 0, -0.1775]
#   torso_pitch2roll_joint: xyz=[0, 0, 0]
#   torso_roll2torso_joint: xyz=[0, 0, 0.08]
#   torso2head_base_joint: xyz=[0, 0, 0.21]
#   head_base2right_shoul_base_joint: xyz=[0, -0.12, -0.05], rpy=[1.04706195, 0, 0]
#   right_shoul_base2shoul_joint[11]: xyz=[0, 0, 0.075], axis=[0, 0, -1]
#   right_shoul2shoul_rot_joint[12]: xyz=[0, 0, 0.08], rpy=[-1.04706195, 0, 0], axis=[-1, 0, 0]
#   right_shoul_rot2upper_arm_joint (fixed): xyz=[0, 0, -0.0625]
#   right_arm2armrot_joint[13]: xyz=[0, 0, -0.0625], axis=[0, 0, 1]
#   right_armrot2elbow_joint[14]: xyz=[0, 0, -0.22], axis=[0, 1, 0]
#   right_elbow2forearm_joint (fixed): xyz=[0, 0, -0.045]
#   right_forearm2forearmrot_joint[15]: xyz=[0, 0, -0.045], axis=[0, 0, 1]
#   right_forearmrot2forearm_pitch_joint[16]: xyz=[0, 0, -0.18], axis=[0, 1, 0]
#   right_forearm_pitch2forearm_roll_joint[17]: xyz=[0, 0, 0], axis=[1, 0, 0]
#   right_forearm_roll2wrist_joint (fixed): xyz=[0, 0, -0.015]
#   right_wrist2gripper_base_joint (fixed): xyz=[0, 0, -0.05], rpy=[0, 0, 3.14159]
#   right_gripper_center_joint (fixed): xyz=[0, 0, -0.03]

RIGHT_ARM_CHAIN = [
    # (type, xyz, rpy, axis)

    # 0: world → base_link
    ('fixed', [0, 0, 0.621251], [0, 0, 0], None),

    # 1: base_link → lift_platform (prismatic at position 0)
    ('fixed', [-0.041, 0, 0.99], [0, 0, 0], None),

    # 2: lift → torso_pitch
    ('fixed', [0.03, 0, -0.1775], [0, 0, 0], None),

    # 3: torso_roll → torso (pitch→roll is zero offset)
    ('fixed', [0, 0, 0.08], [0, 0, 0], None),

    # 4: torso → head_base
    ('fixed', [0, 0, 0.21], [0, 0, 0], None),

    # 5: head_base → right_shoul_base (60 deg roll)
    ('fixed', [0, -0.12, -0.05], [1.04706195, 0, 0], None),

    # 6: Joint [11]: origin + revolute (shoul_base → shoul)
    ('revolute', [0, 0, 0.075], [0, 0, 0], [0, 0, -1]),

    # 7: Joint [12]: origin (-60 deg roll) + revolute (shoul → shoul_rot)
    ('revolute', [0, 0, 0.08], [-1.04706195, 0, 0], [-1, 0, 0]),

    # 8: shoul_rot → upper_arm (fixed)
    ('fixed', [0, 0, -0.0625], [0, 0, 0], None),

    # 9: Joint [13]: origin + revolute (upper_arm → armrot)
    ('revolute', [0, 0, -0.0625], [0, 0, 0], [0, 0, 1]),

    # 10: Joint [14]: origin + revolute (armrot → elbow)
    ('revolute', [0, 0, -0.22], [0, 0, 0], [0, 1, 0]),

    # 11: elbow → forearm (fixed)
    ('fixed', [0, 0, -0.045], [0, 0, 0], None),

    # 12: Joint [15]: origin + revolute (forearm → forearmrot)
    ('revolute', [0, 0, -0.045], [0, 0, 0], [0, 0, 1]),

    # 13: Joint [16]: origin + revolute (forearmrot → forearm_pitch)
    ('revolute', [0, 0, -0.18], [0, 0, 0], [0, 1, 0]),

    # 14: Joint [17]: origin + revolute (forearm_pitch → forearm_roll)
    ('revolute', [0, 0, 0], [0, 0, 0], [1, 0, 0]),

    # 15: forearm_roll → wrist (fixed)
    ('fixed', [0, 0, -0.015], [0, 0, 0], None),

    # 16: wrist → gripper_base (fixed, 180 deg yaw)
    ('fixed', [0, 0, -0.05], [0, 0, 3.14159], None),

    # 17: gripper_base → gripper_center (fixed, IK endpoint)
    ('fixed', [0, 0, -0.03], [0, 0, 0], None),
]

# 관절 인덱스 (체인에서 revolute 타입의 인덱스)
RIGHT_ARM_JOINT_INDICES = [6, 7, 9, 10, 12, 13, 14]  # 7개 관절


# ============================================================
# 왼팔 킨레마틱 체인 (오른팔의 Y축 미러링)
# ============================================================

LEFT_ARM_CHAIN = [
    # (type, xyz, rpy, axis) - 오른팔과 동일, Y와 roll만 반전

    # 0: world → base_link
    ('fixed', [0, 0, 0.621251], [0, 0, 0], None),

    # 1: base_link → lift_platform (prismatic at position 0)
    ('fixed', [-0.041, 0, 0.99], [0, 0, 0], None),

    # 2: lift → torso_pitch
    ('fixed', [0.03, 0, -0.1775], [0, 0, 0], None),

    # 3: torso_roll → torso (pitch→roll is zero offset)
    ('fixed', [0, 0, 0.08], [0, 0, 0], None),

    # 4: torso → head_base
    ('fixed', [0, 0, 0.21], [0, 0, 0], None),

    # 5: head_base → left_shoul_base (-60 deg roll, Y반전: -0.12 → +0.12)
    ('fixed', [0, 0.12, -0.05], [-1.04706195, 0, 0], None),

    # 6: Joint [21]: URDF axis Z=+1 (오른팔은 Z=-1이므로 다름!)
    ('revolute', [0, 0, 0.075], [0, 0, 0], [0, 0, 1]),

    # 7: Joint [22]: roll 반전 (-60 → +60), URDF axis X=+1 (오른팔은 X=-1이므로 다름!)
    ('revolute', [0, 0, 0.08], [1.04706195, 0, 0], [1, 0, 0]),

    # 8: shoul_rot → upper_arm (fixed)
    ('fixed', [0, 0, -0.0625], [0, 0, 0], None),

    # 9: Joint [23]: 오른팔과 동일
    ('revolute', [0, 0, -0.0625], [0, 0, 0], [0, 0, 1]),

    # 10: Joint [24]: 오른팔과 동일
    ('revolute', [0, 0, -0.22], [0, 0, 0], [0, 1, 0]),

    # 11: elbow → forearm (fixed)
    ('fixed', [0, 0, -0.045], [0, 0, 0], None),

    # 12: Joint [25]: 오른팔과 동일
    ('revolute', [0, 0, -0.045], [0, 0, 0], [0, 0, 1]),

    # 13: Joint [26]: 오른팔과 동일
    ('revolute', [0, 0, -0.18], [0, 0, 0], [0, 1, 0]),

    # 14: Joint [27]: 오른팔과 동일
    ('revolute', [0, 0, 0], [0, 0, 0], [1, 0, 0]),

    # 15: forearm_roll → wrist (fixed)
    ('fixed', [0, 0, -0.015], [0, 0, 0], None),

    # 16: wrist → gripper_base (fixed, 180 deg yaw)
    ('fixed', [0, 0, -0.05], [0, 0, 3.14159], None),

    # 17: gripper_base → gripper_center (fixed, IK endpoint)
    ('fixed', [0, 0, -0.03], [0, 0, 0], None),
]

LEFT_ARM_JOINT_INDICES = [6, 7, 9, 10, 12, 13, 14]  # 7개 관절 (오른팔과 동일한 구조)


# ============================================================
# RX-1 자체 FK/IK 클래스
# ============================================================

class RX1CustomIK:
    """
    RX-1 로봇 자체 FK/IK 구현

    - ikpy 의존성 없음
    - URDF에서 직접 추출한 킨레마틱 파라미터 사용
    - 수치 미분 기반 Jacobian
    - Damped Least Squares IK

    Uses two conventions:
    1. Internal FK uses original convention (SERVO_CENTER + servo_dirs) - matches ROS TF
    2. Output servo values converted for visualization compatibility
    """

    def __init__(self, arm: str = 'right', robot_baseline: dict = None):
        """
        Initialize IK solver with VIZ_BASELINE_OVERRIDE.

        Args:
            arm: 'right' or 'left'
            robot_baseline: {servo_id: position} from rx1_baseline_v5.json
        """
        self.arm = arm

        if arm == 'right':
            self.chain = RIGHT_ARM_CHAIN
            self.joint_indices = RIGHT_ARM_JOINT_INDICES
            self.servo_ids = RIGHT_ARM_SERVO_IDS
            self.viz_inversions = VIZ_RIGHT_ARM_INVERSIONS
        else:
            self.chain = LEFT_ARM_CHAIN
            self.joint_indices = LEFT_ARM_JOINT_INDICES
            self.servo_ids = LEFT_ARM_SERVO_IDS
            self.viz_inversions = VIZ_LEFT_ARM_INVERSIONS

        self.num_joints = 7

        # 로봇 베이스라인 저장
        self.robot_baseline = robot_baseline or {}

        # VIZ_BASELINE_OVERRIDE 사용 (URDF 체인과 매칭)
        self.viz_baseline = {}
        for sid in self.servo_ids:
            if sid in VIZ_BASELINE_OVERRIDE:
                self.viz_baseline[sid] = VIZ_BASELINE_OVERRIDE[sid]
            else:
                self.viz_baseline[sid] = SERVO_CENTER

        # IK 파라미터
        self.damping = 0.05
        self.max_iterations = 100
        self.tolerance = 0.002  # 2mm

    def servo_to_angle(self, servo_val: int, joint_idx: int) -> float:
        """
        Servo value -> joint angle (radians)

        시각화와 동일한 공식:
        angle = inversion * (servo - viz_baseline) / 4095 * range
        """
        sid = self.servo_ids[joint_idx]
        inversion = self.viz_inversions[joint_idx]
        baseline = self.viz_baseline.get(sid, SERVO_CENTER)

        relative = servo_val - baseline

        if sid in NO_GEARBOX_SERVOS:
            deg = (relative / 4095) * 360
        else:
            deg = (relative / 4095) * 120

        return inversion * math.radians(deg)

    def angle_to_servo(self, angle: float, joint_idx: int) -> int:
        """
        Joint angle (radians) -> servo value

        시각화 공식의 역변환:
        servo = viz_baseline + (angle / inversion) / range * 4095
        """
        sid = self.servo_ids[joint_idx]
        inversion = self.viz_inversions[joint_idx]
        baseline = self.viz_baseline.get(sid, SERVO_CENTER)

        deg = math.degrees(angle / inversion) if inversion != 0 else 0

        if sid in NO_GEARBOX_SERVOS:
            relative = (deg / 360) * 4095
        else:
            relative = (deg / 120) * 4095

        servo_val = int(baseline + relative)
        return max(0, min(4095, servo_val))

    def forward_kinematics_full(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Full FK: joint angles -> 4x4 transform matrix

        Args:
            joint_angles: 7 joint angles (radians)

        Returns:
            T: 4x4 homogeneous transform matrix
        """
        T = np.eye(4)
        joint_counter = 0

        for item in self.chain:
            link_type = item[0]
            xyz = np.array(item[1])
            rpy = np.array(item[2])
            axis = item[3]

            T_link = make_transform(xyz, rpy)
            T = T @ T_link

            if link_type == 'revolute' and joint_counter < len(joint_angles):
                angle = joint_angles[joint_counter]
                axis_vec = np.array(axis)
                R_joint = axis_angle_to_rotation(axis_vec, angle)

                T_rot = np.eye(4)
                T_rot[:3, :3] = R_joint
                T = T @ T_rot

                joint_counter += 1

        return T

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        FK: joint angles -> EE position (XYZ)

        Args:
            joint_angles: 7 joint angles (radians)

        Returns:
            position: [x, y, z] EE position (meters)
        """
        T = self.forward_kinematics_full(joint_angles)
        return T[:3, 3]

    # ============================================================
    # 4+3 Split IK 메서드
    # ============================================================

    def forward_kinematics_wrist(self, position_angles: np.ndarray) -> np.ndarray:
        """
        위치 그룹(joints 0-3)만으로 wrist point의 4x4 변환 행렬 계산.

        체인 0~WRIST_CHAIN_SPLIT_INDEX까지의 link transform을 적용하되,
        WRIST_CHAIN_SPLIT_INDEX의 joint rotation은 적용하지 않음.

        Args:
            position_angles: 4개 관절 각도 (radians) [shoulder yaw, shoulder pitch, upper arm roll, elbow]

        Returns:
            T: 4x4 wrist point 변환 행렬
        """
        T = np.eye(4)
        joint_counter = 0

        for item in self.chain:
            link_type = item[0]
            xyz = np.array(item[1])
            rpy = np.array(item[2])
            axis = item[3]

            # Link transform 적용
            T_link = make_transform(xyz, rpy)
            T = T @ T_link

            if link_type == 'revolute':
                if joint_counter < len(position_angles):
                    # 위치 그룹 관절: rotation 적용
                    angle = position_angles[joint_counter]
                    axis_vec = np.array(axis)
                    R_joint = axis_angle_to_rotation(axis_vec, angle)
                    T_rot = np.eye(4)
                    T_rot[:3, :3] = R_joint
                    T = T @ T_rot
                joint_counter += 1
                # 손목 그룹 첫 관절의 link transform은 적용했지만 rotation은 안 함 → 여기서 종료
                if joint_counter > len(position_angles):
                    break

        return T

    def forward_kinematics_wrist_pos(self, position_angles: np.ndarray) -> np.ndarray:
        """
        위치 그룹(joints 0-3)만으로 wrist point 위치(XYZ) 계산.

        Args:
            position_angles: 4개 관절 각도 (radians)

        Returns:
            position: [x, y, z] wrist point 위치 (meters)
        """
        T = self.forward_kinematics_wrist(position_angles)
        return T[:3, 3]

    def compute_jacobian_wrist(self, position_angles: np.ndarray, delta: float = 0.001) -> np.ndarray:
        """
        wrist point에 대한 3x4 수치 야코비안 (중앙 차분).

        Args:
            position_angles: 4개 관절 각도 (radians)
            delta: 미분용 각도 변화량

        Returns:
            J: 3x4 Jacobian 행렬
        """
        n = len(position_angles)
        J = np.zeros((3, n))

        for i in range(n):
            angles_plus = np.array(position_angles, dtype=float)
            angles_plus[i] += delta

            angles_minus = np.array(position_angles, dtype=float)
            angles_minus[i] -= delta

            pos_plus = self.forward_kinematics_wrist_pos(angles_plus)
            pos_minus = self.forward_kinematics_wrist_pos(angles_minus)

            J[:, i] = (pos_plus - pos_minus) / (2 * delta)

        return J

    def inverse_kinematics_wrist(self, target_xyz: np.ndarray,
                                  initial_angles: np.ndarray,
                                  nullspace_gain: float = 0.5,
                                  max_iterations: int = 50,
                                  tolerance: float = 0.002) -> Tuple[np.ndarray, bool]:
        """
        4관절 위치 IK + null-space projection.

        dq = J†·e + (I - J†J)·gain·(q_pref - q)
             ^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             위치추적   팔꿈치 아래 유도 (null-space)

        Args:
            target_xyz: [x, y, z] wrist point 목표 위치
            initial_angles: 4개 초기 관절 각도
            nullspace_gain: null-space 보정 강도
            max_iterations: 최대 반복
            tolerance: 수렴 허용 오차 (m)

        Returns:
            angles: 4개 관절 각도
            success: 성공 여부
        """
        target = np.array(target_xyz)
        angles = np.array(initial_angles[:4], dtype=float)
        q_pref = np.array(POSITION_PREFERRED_ANGLES)

        best_angles = angles.copy()
        best_error = float('inf')
        stuck_count = 0

        for iteration in range(max_iterations):
            current_pos = self.forward_kinematics_wrist_pos(angles)
            error = target - current_pos
            error_norm = np.linalg.norm(error)

            if error_norm < best_error:
                best_error = error_norm
                best_angles = angles.copy()
                stuck_count = 0
            else:
                stuck_count += 1

            if error_norm < tolerance:
                return angles, True

            # 3x4 Jacobian
            J = self.compute_jacobian_wrist(angles)

            # Stuck 탈출
            if stuck_count > 15:
                sign = 1 if (iteration % 2 == 0) else -1
                scale = 0.05 + 0.02 * (stuck_count // 10)
                perturbation = np.array([sign * scale, sign * scale * 0.5, 0, sign * scale])
                angles = angles + perturbation
                stuck_count = 0
                continue

            # DLS pseudo-inverse: J_pinv = J^T (JJ^T + λ²I)^-1
            JJT = J @ J.T
            damping_term = self.damping ** 2 * np.eye(3)

            try:
                inv_term = np.linalg.inv(JJT + damping_term)
                J_pinv = J.T @ inv_term
            except np.linalg.LinAlgError:
                J_pinv = np.linalg.pinv(J)

            # Primary task: 위치 추적
            dq_primary = J_pinv @ error

            # Null-space projection: N = I₄ - J†J
            N = np.eye(4) - J_pinv @ J

            # Secondary task: 팔꿈치 선호 각도
            dq_null = N @ (nullspace_gain * (q_pref - angles))

            dq = dq_primary + dq_null

            # Step size 제한
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.3:
                dq = dq / dq_norm * 0.3

            angles = angles + dq
            angles = np.clip(angles, -np.pi, np.pi)

        return best_angles, best_error < 0.05

    def compute_wrist_to_gripper_offset(self, orientation_angles: np.ndarray) -> np.ndarray:
        """
        손목 관절(joints 4-6)로 wrist point → gripper center 변환 계산.

        forward_kinematics_wrist()와 합치면 전체 FK가 됨:
            T_full = T_wrist @ T_wrist_to_gripper

        Args:
            orientation_angles: 3개 손목 관절 각도 (radians) [forearm roll, wrist pitch, wrist roll]

        Returns:
            T: 4x4 wrist→gripper 변환 행렬
        """
        T = np.eye(4)
        joint_counter = 0

        for idx, item in enumerate(self.chain):
            if idx < WRIST_CHAIN_SPLIT_INDEX:
                continue

            link_type = item[0]
            xyz = np.array(item[1])
            rpy = np.array(item[2])
            axis = item[3]

            if idx == WRIST_CHAIN_SPLIT_INDEX:
                # Split point: link transform은 wrist FK에 포함됨, rotation만 적용
                if link_type == 'revolute' and joint_counter < len(orientation_angles):
                    angle = orientation_angles[joint_counter]
                    axis_vec = np.array(axis)
                    R_joint = axis_angle_to_rotation(axis_vec, angle)
                    T_rot = np.eye(4)
                    T_rot[:3, :3] = R_joint
                    T = T @ T_rot
                    joint_counter += 1
                continue

            # 이후 링크: transform + rotation 전부 적용
            T_link = make_transform(xyz, rpy)
            T = T @ T_link

            if link_type == 'revolute' and joint_counter < len(orientation_angles):
                angle = orientation_angles[joint_counter]
                axis_vec = np.array(axis)
                R_joint = axis_angle_to_rotation(axis_vec, angle)
                T_rot = np.eye(4)
                T_rot[:3, :3] = R_joint
                T = T @ T_rot
                joint_counter += 1

        return T

    def orientation_to_servo_direct(self, d_roll: float, d_pitch: float, d_yaw: float,
                                     rotation_offset: List[int]) -> Dict[int, int]:
        """
        VR 회전 delta → 손목 서보값 직접 변환 (IK 안 거침).

        서보 15/25 (forearm roll): 360° 범위 → 4095/(2π) ticks/rad
        서보 16/26 (wrist pitch):  120° 범위 → 4095/(120°·π/180) ticks/rad
        서보 17/27 (wrist roll):   120° 범위 → 4095/(120°·π/180) ticks/rad

        Args:
            d_roll:  회전 delta (rad) → forearm roll (서보 15/25)
            d_pitch: 회전 delta (rad) → wrist pitch (서보 16/26)
            d_yaw:   회전 delta (rad) → wrist roll (서보 17/27)
            rotation_offset: [servo15_cal, servo16_cal, servo17_cal] 캘리브레이션 시점 서보값

        Returns:
            {servo_id: value} 손목 서보 3개
        """
        wrist_servo_ids = self.servo_ids[4:7]      # [15,16,17] or [25,26,27]
        wrist_inversions = self.viz_inversions[4:7]

        # Ticks per radian
        ticks_360 = 4095.0 / (2.0 * math.pi)       # ≈ 651.7 ticks/rad (360° servos)
        ticks_120 = 4095.0 / (120.0 * math.pi / 180.0)  # ≈ 1955.2 ticks/rad (120° servos)

        result = {}

        # Servo 15/25 (forearm roll) - 360° range
        result[wrist_servo_ids[0]] = int(rotation_offset[0] + wrist_inversions[0] * d_roll * ticks_360)

        # Servo 16/26 (wrist pitch) - 120° range
        result[wrist_servo_ids[1]] = int(rotation_offset[1] + wrist_inversions[1] * d_pitch * ticks_120)

        # Servo 17/27 (wrist roll) - 120° range
        result[wrist_servo_ids[2]] = int(rotation_offset[2] + wrist_inversions[2] * d_yaw * ticks_120)

        # Clamp to [0, 4095]
        for sid in result:
            result[sid] = max(0, min(4095, result[sid]))

        return result

    def solve_for_pose_split(self, target_xyz: List[float], delta_rpy: List[float],
                              servo_values: Dict[int, int],
                              rotation_offset: List[int],
                              max_iterations: int = 50,
                              nullspace_gain: float = 0.5) -> Tuple[Dict[int, int], bool]:
        """
        4+3 Split IK: 위치(joints 0-3) IK + 손목(joints 4-6) 직접 매핑.

        1. 손목 서보값 직접 계산 (orientation_to_servo_direct)
        2. 손목 orientation angles → wrist-to-gripper offset 계산
        3. wrist_target = gripper_target - R_wrist·offset
        4. 4관절 position IK with null-space (inverse_kinematics_wrist)
        5. 위치 서보(0-3) + 손목 서보(4-6) 합쳐서 반환

        Args:
            target_xyz: [x, y, z] gripper 목표 위치 (m, world frame)
            delta_rpy: [d_roll, d_pitch, d_yaw] 회전 delta (rad, 캘리브레이션 기준)
            servo_values: 현재 서보값 (IK warm-start용)
            rotation_offset: [servo15_cal, servo16_cal, servo17_cal] 캘리브레이션 시점 서보값
            max_iterations: 위치 IK 최대 반복
            nullspace_gain: null-space 보정 강도

        Returns:
            new_servo_values: 전체 7관절 서보값
            success: 성공 여부
        """
        target_pos = np.array(target_xyz)

        ik_debug_log(f"=== Split IK ({self.arm}) ===")
        ik_debug_log(f"Gripper target: X={target_pos[0]*100:.2f}, Y={target_pos[1]*100:.2f}, Z={target_pos[2]*100:.2f} cm")
        ik_debug_log(f"Delta RPY: R={np.degrees(delta_rpy[0]):.1f}, P={np.degrees(delta_rpy[1]):.1f}, Y={np.degrees(delta_rpy[2]):.1f} deg")

        # 현재 각도 (warm-start)
        current_angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            current_angles[i] = self.servo_to_angle(servo_val, i)

        # Step 1: 손목 서보값 직접 계산
        wrist_servos = self.orientation_to_servo_direct(
            delta_rpy[0], delta_rpy[1], delta_rpy[2], rotation_offset
        )

        # Step 2: 손목 서보값 → orientation angles → wrist-to-gripper offset
        orientation_angles = np.zeros(3)
        for i in range(3):
            wrist_sid = self.servo_ids[4 + i]
            orientation_angles[i] = self.servo_to_angle(wrist_servos[wrist_sid], 4 + i)

        T_offset = self.compute_wrist_to_gripper_offset(orientation_angles)

        # Step 3: wrist_target = gripper_target - R_wrist · offset
        T_wrist = self.forward_kinematics_wrist(current_angles[:4])
        R_wrist = T_wrist[:3, :3]
        d_world = R_wrist @ T_offset[:3, 3]
        wrist_target = target_pos - d_world

        ik_debug_log(f"Wrist offset (local): [{T_offset[0,3]*100:.2f}, {T_offset[1,3]*100:.2f}, {T_offset[2,3]*100:.2f}] cm")
        ik_debug_log(f"Wrist target: X={wrist_target[0]*100:.2f}, Y={wrist_target[1]*100:.2f}, Z={wrist_target[2]*100:.2f} cm")

        # Step 4: 4관절 position IK with null-space
        position_angles, pos_success = self.inverse_kinematics_wrist(
            wrist_target, current_angles[:4], nullspace_gain, max_iterations
        )

        if pos_success:
            # Wrist target 보정: IK 결과의 wrist rotation으로 재계산
            T_wrist_new = self.forward_kinematics_wrist(position_angles)
            R_wrist_new = T_wrist_new[:3, :3]
            d_world_new = R_wrist_new @ T_offset[:3, 3]
            wrist_target_refined = target_pos - d_world_new

            # 보정된 target으로 재풀이 (짧은 반복)
            position_angles, _ = self.inverse_kinematics_wrist(
                wrist_target_refined, position_angles, nullspace_gain, max_iterations // 2
            )
        else:
            # 위치 IK 실패 시 현재 위치 유지
            ik_debug_log(f"Position IK failed, keeping current position angles")
            position_angles = current_angles[:4].copy()

        # Step 5: 위치 서보 + 손목 서보 합침 (손목은 항상 적용)
        result_servos = {}
        for i in range(4):
            sid = self.servo_ids[i]
            result_servos[sid] = self.angle_to_servo(position_angles[i], i)
        result_servos.update(wrist_servos)

        # 결과 로깅
        result_wrist_pos = self.forward_kinematics_wrist_pos(position_angles)
        ik_debug_log(f"Position IK: {'OK' if pos_success else 'FAIL (current kept)'}")
        ik_debug_log(f"Wrist servos: {wrist_servos}")
        ik_debug_log("")

        return result_servos, True

    def get_gripper_transform(self, servo_values: Dict[int, int]) -> np.ndarray:
        """
        Get gripper full transform (position + orientation)

        Args:
            servo_values: {servo_id: position} dict

        Returns:
            T: 4x4 transform matrix (rotation + translation)
        """
        angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            angles[i] = self.servo_to_angle(servo_val, i)

        return self.forward_kinematics_full(angles)

    def get_gripper_orientation(self, servo_values: Dict[int, int]) -> np.ndarray:
        """
        Get gripper Z axis direction (pointing direction) in world frame

        Returns:
            direction: [x, y, z] unit vector
        """
        T = self.get_gripper_transform(servo_values)
        return T[:3, 2]  # Z column = gripper forward direction

    def get_end_effector_position(self, servo_values: Dict[int, int]) -> np.ndarray:
        """
        서보값 → EE 위치 (internal servo values)

        Args:
            servo_values: {servo_id: position} 딕셔너리

        Returns:
            position: [x, y, z] EE 위치 (미터)
        """
        angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            angles[i] = self.servo_to_angle(servo_val, i)

        return self.forward_kinematics(angles)

    def get_ee_position_from_viz_servos(self, servo_values: Dict[int, int]) -> np.ndarray:
        """
        서보값 → EE 위치 (direct, no conversion)

        Args:
            servo_values: 서보값 딕셔너리

        Returns:
            position: [x, y, z] EE 위치 (미터)
        """
        # Use servo values directly (no conversion needed)
        return self.get_end_effector_position(servo_values)

    def compute_jacobian(self, joint_angles: np.ndarray, delta: float = 0.001) -> np.ndarray:
        """
        수치 미분으로 Jacobian 계산 (중앙 차분)

        Args:
            joint_angles: 현재 관절 각도
            delta: 미분용 각도 변화량

        Returns:
            J: 3x7 Jacobian 행렬
        """
        J = np.zeros((3, self.num_joints))

        for i in range(self.num_joints):
            angles_plus = joint_angles.copy()
            angles_plus[i] += delta

            angles_minus = joint_angles.copy()
            angles_minus[i] -= delta

            pos_plus = self.forward_kinematics(angles_plus)
            pos_minus = self.forward_kinematics(angles_minus)

            J[:, i] = (pos_plus - pos_minus) / (2 * delta)

        return J

    def compute_jacobian_6dof(self, joint_angles: np.ndarray, delta: float = 0.001) -> np.ndarray:
        """
        6-DOF Jacobian 계산: 위치 (3) + 방향 (3)

        Args:
            joint_angles: 현재 관절 각도
            delta: 미분용 각도 변화량

        Returns:
            J: 6x7 Jacobian 행렬 [position (3), orientation (3)]
        """
        J = np.zeros((6, self.num_joints))

        # 현재 변환 행렬
        T_current = self.forward_kinematics_full(joint_angles)
        R_current = T_current[:3, :3]

        for i in range(self.num_joints):
            angles_plus = joint_angles.copy()
            angles_plus[i] += delta

            angles_minus = joint_angles.copy()
            angles_minus[i] -= delta

            # Forward kinematics
            T_plus = self.forward_kinematics_full(angles_plus)
            T_minus = self.forward_kinematics_full(angles_minus)

            # Position Jacobian (dp/dq)
            pos_plus = T_plus[:3, 3]
            pos_minus = T_minus[:3, 3]
            J[:3, i] = (pos_plus - pos_minus) / (2 * delta)

            # Orientation Jacobian (dR/dq as axis-angle)
            R_plus = T_plus[:3, :3]
            R_minus = T_minus[:3, :3]

            # dR = R_plus @ R_minus^T → axis-angle 오차
            dR = R_plus @ R_minus.T
            axis, angle = rotation_to_axis_angle(dR)
            omega = axis * angle / (2 * delta)  # 각속도

            J[3:6, i] = omega

        return J

    def inverse_kinematics(self, target_xyz: np.ndarray,
                          initial_angles: Optional[np.ndarray] = None,
                          max_iterations: Optional[int] = None,
                          tolerance: Optional[float] = None) -> Tuple[np.ndarray, bool]:
        """
        역기구학: 목표 위치 → 관절 각도 (Damped Least Squares)

        Includes singularity escape: when Jacobian row is near-zero but error exists
        in that direction, add small perturbation to escape the singularity.

        Args:
            target_xyz: [x, y, z] 목표 위치
            initial_angles: 초기 관절 각도 (없으면 현재값)
            max_iterations: 최대 반복
            tolerance: 수렴 허용 오차

        Returns:
            angles: 7개 관절 각도
            success: 성공 여부
        """
        if max_iterations is None:
            max_iterations = self.max_iterations
        if tolerance is None:
            tolerance = self.tolerance

        target = np.array(target_xyz)

        # 초기 각도
        if initial_angles is not None:
            angles = np.array(initial_angles, dtype=float)
        else:
            angles = np.zeros(self.num_joints)

        # Check for singularity at start - if Z-row is near zero, apply initial escape
        J_initial = self.compute_jacobian(angles)
        z_row_norm = np.linalg.norm(J_initial[2])
        initial_pos = self.forward_kinematics(angles)
        z_error = abs(target[2] - initial_pos[2])

        if z_row_norm < 0.01 and z_error > 0.005:
            # At Z-singularity with Z error - apply deterministic escape
            # Move joints 0,1,3 slightly to break singularity
            escape_amount = 0.15 if (target[2] > initial_pos[2]) else -0.15
            angles[0] += escape_amount  # shoulder yaw
            angles[1] += escape_amount * 0.5  # shoulder pitch
            angles[3] += escape_amount  # elbow
            ik_debug_log(f"Initial Z-singularity escape: z_row_norm={z_row_norm:.4f}, z_error={z_error*100:.2f}cm")

        best_angles = angles.copy()
        best_error = float('inf')
        stuck_count = 0

        for iteration in range(max_iterations):
            # 현재 위치
            current_pos = self.forward_kinematics(angles)

            # 오차
            error = target - current_pos
            error_norm = np.linalg.norm(error)

            # 최적 결과 저장
            if error_norm < best_error:
                best_error = error_norm
                best_angles = angles.copy()
                stuck_count = 0
            else:
                stuck_count += 1

            # 수렴 체크
            if error_norm < tolerance:
                return angles, True

            # Jacobian 계산
            J = self.compute_jacobian(angles)

            # If stuck for many iterations, try perturbation
            if stuck_count > 15 and error_norm > tolerance:
                # Deterministic escape based on iteration
                sign = 1 if (iteration % 2 == 0) else -1
                scale = 0.05 + 0.02 * (stuck_count // 10)
                perturbation = np.zeros(self.num_joints)
                perturbation[:4] = sign * scale
                angles = angles + perturbation
                stuck_count = 0

            # Damped Least Squares: dq = J^T (J J^T + λ²I)^(-1) e
            JJT = J @ J.T
            damping_term = self.damping ** 2 * np.eye(3)

            try:
                inv_term = np.linalg.inv(JJT + damping_term)
                dq = J.T @ inv_term @ error
            except np.linalg.LinAlgError:
                # 역행렬 실패 시 pseudo-inverse
                dq = np.linalg.lstsq(J.T, error, rcond=None)[0]

            # 스텝 크기 제한
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.3:
                dq = dq / dq_norm * 0.3

            # 관절 각도 업데이트
            angles = angles + dq

            # 관절 한계 적용 (±π)
            angles = np.clip(angles, -np.pi, np.pi)

        # 최대 반복 도달
        return best_angles, best_error < 0.05  # 5cm 이내면 성공

    def inverse_kinematics_6dof(self, target_xyz: np.ndarray, target_rot: np.ndarray,
                                initial_angles: Optional[np.ndarray] = None,
                                max_iterations: Optional[int] = None,
                                pos_tolerance: float = 0.003,  # 3mm
                                rot_tolerance: float = 0.05,   # ~3 degrees
                                pos_weight: float = 1.0,
                                rot_weight: float = 0.3,
                                joint_weights: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        6-DOF 역기구학: 목표 위치 + 방향 → 관절 각도

        Args:
            target_xyz: [x, y, z] 목표 위치
            target_rot: 3x3 목표 회전 행렬
            initial_angles: 초기 관절 각도
            max_iterations: 최대 반복
            pos_tolerance: 위치 허용 오차 (m)
            rot_tolerance: 방향 허용 오차 (rad)
            pos_weight: 위치 오차 가중치
            rot_weight: 방향 오차 가중치

        Returns:
            angles: 7개 관절 각도
            success: 성공 여부
        """
        if max_iterations is None:
            max_iterations = self.max_iterations

        target_pos = np.array(target_xyz)
        target_R = np.array(target_rot)

        # 초기 각도
        if initial_angles is not None:
            angles = np.array(initial_angles, dtype=float)
        else:
            angles = np.zeros(self.num_joints)

        best_angles = angles.copy()
        best_error = float('inf')
        stuck_count = 0

        for iteration in range(max_iterations):
            # 현재 변환
            T_current = self.forward_kinematics_full(angles)
            current_pos = T_current[:3, 3]
            current_R = T_current[:3, :3]

            # 위치 오차
            pos_error = target_pos - current_pos
            pos_error_norm = np.linalg.norm(pos_error)

            # 방향 오차 (axis-angle 벡터)
            rot_error = orientation_error(current_R, target_R)
            rot_error_norm = np.linalg.norm(rot_error)

            # 가중치 적용한 총 오차
            total_error = pos_weight * pos_error_norm + rot_weight * rot_error_norm

            # 최적 결과 저장
            if total_error < best_error:
                best_error = total_error
                best_angles = angles.copy()
                stuck_count = 0
            else:
                stuck_count += 1

            # 수렴 체크
            if pos_error_norm < pos_tolerance and rot_error_norm < rot_tolerance:
                return angles, True

            # 6-DOF Jacobian
            J = self.compute_jacobian_6dof(angles)

            # 가중치 적용된 오차 벡터
            error_6d = np.concatenate([pos_error * pos_weight, rot_error * rot_weight])

            # Stuck 탈출
            if stuck_count > 15:
                sign = 1 if (iteration % 2 == 0) else -1
                scale = 0.03 + 0.01 * (stuck_count // 10)
                perturbation = np.zeros(self.num_joints)
                perturbation[:4] = sign * scale
                angles = angles + perturbation
                stuck_count = 0

            # Damped Least Squares for 6-DOF
            damping = self.damping * 1.5  # 6-DOF는 더 높은 damping
            damping_term = damping ** 2 * np.eye(6)

            try:
                if joint_weights is not None:
                    # Weighted DLS: 관절별 가중치 (높을수록 안 움직임)
                    # dq = W^-2 J^T (J W^-2 J^T + λ²I)^-1 e
                    W_inv_sq = np.diag(1.0 / np.array(joint_weights) ** 2)
                    JWJ = J @ W_inv_sq @ J.T
                    inv_term = np.linalg.inv(JWJ + damping_term)
                    dq = W_inv_sq @ J.T @ inv_term @ error_6d
                else:
                    JJT = J @ J.T
                    inv_term = np.linalg.inv(JJT + damping_term)
                    dq = J.T @ inv_term @ error_6d
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(J.T, error_6d, rcond=None)[0]

            # 스텝 크기 제한
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.2:  # 6-DOF는 더 작은 스텝
                dq = dq / dq_norm * 0.2

            # 관절 각도 업데이트
            angles = angles + dq
            angles = np.clip(angles, -np.pi, np.pi)

        # 최대 반복 도달 - 위치만이라도 성공했으면 OK
        final_T = self.forward_kinematics_full(best_angles)
        final_pos_error = np.linalg.norm(final_T[:3, 3] - target_pos)
        return best_angles, final_pos_error < 0.05

    def solve_for_pose(self, target_xyz: List[float], target_rpy: List[float],
                       servo_values: Dict[int, int],
                       max_iterations: int = 100,
                       pos_weight: float = 1.0,
                       rot_weight: float = 0.5,
                       joint_weights: Optional[np.ndarray] = None) -> Tuple[Dict[int, int], bool]:
        """
        6-DOF IK: 목표 위치 + 방향(RPY)

        여러 초기값에서 시도하여 robust하게 동작.

        Args:
            target_xyz: [x, y, z] 목표 위치 (m)
            target_rpy: [roll, pitch, yaw] 목표 방향 (rad)
            servo_values: 현재 서보값
            max_iterations: 최대 반복
            pos_weight: 위치 가중치
            rot_weight: 방향 가중치

        Returns:
            new_servo_values: 새 서보값
            success: 성공 여부
        """
        target_pos = np.array(target_xyz)
        target_R = rpy_to_rotation(target_rpy[0], target_rpy[1], target_rpy[2])

        ik_debug_log(f"=== IK solve_for_pose 6-DOF ({self.arm}) ===")
        ik_debug_log(f"Target pos: X={target_pos[0]*100:.2f}, Y={target_pos[1]*100:.2f}, Z={target_pos[2]*100:.2f} cm")
        ik_debug_log(f"Target RPY: R={np.degrees(target_rpy[0]):.1f}, P={np.degrees(target_rpy[1]):.1f}, Y={np.degrees(target_rpy[2]):.1f} deg")

        # 현재 각도
        current_angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            current_angles[i] = self.servo_to_angle(servo_val, i)

        # 현재 서보값만 사용 (연속성 유지, 튀는 현상 방지)
        initial_guesses = [
            current_angles.copy(),
        ]

        best_result = None
        best_total_error = float('inf')

        for i, init_angles in enumerate(initial_guesses):
            new_angles, success = self.inverse_kinematics_6dof(
                target_pos, target_R, init_angles, max_iterations // 2,
                pos_weight=pos_weight, rot_weight=rot_weight,
                joint_weights=joint_weights
            )

            if success:
                new_servos = {}
                for j, sid in enumerate(self.servo_ids):
                    new_servos[sid] = self.angle_to_servo(new_angles[j], j)

                result_T = self.forward_kinematics_full(new_angles)
                result_pos = result_T[:3, 3]
                result_R = result_T[:3, :3]

                pos_error = np.linalg.norm(result_pos - target_pos)
                rot_error = np.linalg.norm(orientation_error(result_R, target_R))
                total_error = pos_weight * pos_error + rot_weight * rot_error

                if total_error < best_total_error:
                    best_total_error = total_error
                    best_result = (new_servos, new_angles, result_pos, result_R)

                # 충분히 좋으면 즉시 반환
                if pos_error < 0.003 and rot_error < 0.05:
                    ik_debug_log(f"6-DOF IK success (attempt {i+1})")
                    break

        if best_result and best_total_error < 0.1:
            new_servos, new_angles, result_pos, result_R = best_result
            result_rpy = rotation_matrix_to_rpy(result_R)
            pos_error = np.linalg.norm(result_pos - target_pos) * 1000
            rot_error = np.linalg.norm(orientation_error(result_R, target_R))

            ik_debug_log(f"Result pos: X={result_pos[0]*100:.2f}, Y={result_pos[1]*100:.2f}, Z={result_pos[2]*100:.2f} cm")
            ik_debug_log(f"Result RPY: R={np.degrees(result_rpy[0]):.1f}, P={np.degrees(result_rpy[1]):.1f}, Y={np.degrees(result_rpy[2]):.1f} deg")
            ik_debug_log(f"Pos error: {pos_error:.2f} mm, Rot error: {np.degrees(rot_error):.1f} deg")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log(f"6-DOF IK FAILED! Best error: {best_total_error:.4f}")
            ik_debug_log("")
            return {}, False

    def solve_for_position(self, target_xyz: List[float],
                          servo_values: Dict[int, int],
                          max_iterations: int = 100) -> Tuple[Dict[int, int], bool]:
        """
        Solve IK for target position (WORLD coordinates)

        Uses multiple initial guesses for robustness.

        Args:
            target_xyz: [x, y, z] target position in WORLD frame
            servo_values: current servo values dict (used as one initial guess)
            max_iterations: max IK iterations per attempt

        Returns:
            new_servo_values: new servo values dict
            success: success flag
        """
        target = np.array(target_xyz)

        # 현재 서보값에서 각도 계산
        current_angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            current_angles[i] = self.servo_to_angle(servo_val, i)

        current_pos = self.forward_kinematics(current_angles)

        ik_debug_log(f"=== IK solve_for_position WORLD ({self.arm}) ===")
        ik_debug_log(f"Target:  X={target[0]*100:.2f}, Y={target[1]*100:.2f}, Z={target[2]*100:.2f} cm")
        ik_debug_log(f"Current: X={current_pos[0]*100:.2f}, Y={current_pos[1]*100:.2f}, Z={current_pos[2]*100:.2f} cm")

        # 여러 초기값에서 IK 시도
        initial_guesses = [
            np.zeros(self.num_joints),                    # Zero position
            current_angles.copy(),                         # Current position
            np.array([0.1, -0.2, 0, -0.3, 0, 0, 0]),      # Slight bend
            np.array([-0.1, 0.2, 0, 0.3, 0, 0, 0]),       # Opposite bend
        ]

        best_result = None
        best_error = float('inf')

        for i, init_angles in enumerate(initial_guesses):
            new_angles, success = self.inverse_kinematics(
                target_xyz, init_angles, max_iterations // 2  # 각 시도에 절반 반복
            )

            if success:
                # 결과 검증
                new_servos = {}
                for j, sid in enumerate(self.servo_ids):
                    new_servos[sid] = self.angle_to_servo(new_angles[j], j)

                result_pos = self.get_end_effector_position(new_servos)
                error = np.linalg.norm(result_pos - target)

                if error < best_error:
                    best_error = error
                    best_result = (new_servos, new_angles, result_pos)

                # 충분히 좋으면 즉시 반환
                if error < self.tolerance:
                    ik_debug_log(f"IK success (attempt {i+1}): error={error*1000:.2f}mm")
                    break

        if best_result and best_error < 0.05:  # 5cm 이내
            new_servos, new_angles, result_pos = best_result
            error_vec = result_pos - target
            error_total = best_error * 1000

            ik_debug_log(f"Result:  X={result_pos[0]*100:.2f}, Y={result_pos[1]*100:.2f}, Z={result_pos[2]*100:.2f} cm")
            ik_debug_log(f"Error Total: {error_total:.2f} mm")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log(f"IK FAILED! Best error: {best_error*1000:.2f}mm")
            ik_debug_log("")
            return {}, False

    def solve_for_position_local(self, local_delta: List[float],
                                 servo_values: Dict[int, int],
                                 max_iterations: int = 100) -> Tuple[Dict[int, int], bool]:
        """
        Solve IK for movement in LOCAL (gripper) coordinates

        Local frame definition:
            X: gripper right (+) / left (-)
            Y: gripper up (+) / down (-)
            Z: gripper forward (+) / backward (-)  (pointing direction)

        Args:
            local_delta: [dx, dy, dz] movement in gripper's LOCAL frame (meters)
            servo_values: current servo values dict
            max_iterations: max IK iterations

        Returns:
            new_servo_values: new servo values dict
            success: success flag
        """
        # Get current transform
        T = self.get_gripper_transform(servo_values)
        current_pos = T[:3, 3]
        R = T[:3, :3]  # Rotation matrix (columns = local axes in world)

        # Local axes in world frame
        gripper_x = R[:, 0]  # local X (right)
        gripper_y = R[:, 1]  # local Y (up)
        gripper_z = R[:, 2]  # local Z (forward)

        # Convert local delta to world delta
        local_delta = np.array(local_delta)
        world_delta = (local_delta[0] * gripper_x +
                       local_delta[1] * gripper_y +
                       local_delta[2] * gripper_z)

        # Target in world coordinates
        target_world = current_pos + world_delta

        ik_debug_log(f"=== IK solve_for_position LOCAL ({self.arm}) ===")
        ik_debug_log(f"Local delta requested: dX={local_delta[0]*100:.2f}, dY={local_delta[1]*100:.2f}, dZ={local_delta[2]*100:.2f} cm")
        ik_debug_log(f"Gripper axes in world:")
        ik_debug_log(f"  X (right):   [{gripper_x[0]:.3f}, {gripper_x[1]:.3f}, {gripper_x[2]:.3f}]")
        ik_debug_log(f"  Y (up):      [{gripper_y[0]:.3f}, {gripper_y[1]:.3f}, {gripper_y[2]:.3f}]")
        ik_debug_log(f"  Z (forward): [{gripper_z[0]:.3f}, {gripper_z[1]:.3f}, {gripper_z[2]:.3f}]")
        ik_debug_log(f"World delta: dX={world_delta[0]*100:.2f}, dY={world_delta[1]*100:.2f}, dZ={world_delta[2]*100:.2f} cm")
        ik_debug_log(f"Target world: X={target_world[0]*100:.2f}, Y={target_world[1]*100:.2f}, Z={target_world[2]*100:.2f} cm")

        # Solve in world coordinates
        new_servos, success = self.solve_for_position(target_world.tolist(), servo_values, max_iterations)

        # Calculate local coordinate error
        if success:
            result_pos = self.get_end_effector_position(new_servos)
            actual_world_delta = result_pos - current_pos

            # Project world delta to local axes
            actual_local_x = np.dot(actual_world_delta, gripper_x)
            actual_local_y = np.dot(actual_world_delta, gripper_y)
            actual_local_z = np.dot(actual_world_delta, gripper_z)

            local_error_x = (actual_local_x - local_delta[0]) * 1000
            local_error_y = (actual_local_y - local_delta[1]) * 1000
            local_error_z = (actual_local_z - local_delta[2]) * 1000
            local_error_total = np.sqrt(local_error_x**2 + local_error_y**2 + local_error_z**2)

            ik_debug_log(f"--- Local Coordinate Error ---")
            ik_debug_log(f"Actual local move: dX={actual_local_x*100:.2f}, dY={actual_local_y*100:.2f}, dZ={actual_local_z*100:.2f} cm")
            ik_debug_log(f"Local Error X (right):   {local_error_x:+.2f} mm")
            ik_debug_log(f"Local Error Y (up):      {local_error_y:+.2f} mm")
            ik_debug_log(f"Local Error Z (forward): {local_error_z:+.2f} mm")
            ik_debug_log(f"Local Error Total: {local_error_total:.2f} mm")
            ik_debug_log("")

        return new_servos, success


# ============================================================
# 테스트
# ============================================================

if __name__ == "__main__":
    import json

    print("=== RX-1 Custom FK/IK Test ===\n")

    ik = RX1CustomIK('right')

    # 1. 제로 포지션 테스트
    print("1. Zero position (all servos = 2048)")
    servo_zero = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}
    pos_zero = ik.get_end_effector_position(servo_zero)
    print(f"   EE Position: X={pos_zero[0]*100:.2f}cm, Y={pos_zero[1]*100:.2f}cm, Z={pos_zero[2]*100:.2f}cm")

    # 2. Baseline 테스트
    print("\n2. Baseline position (from rx1_baseline_v5.json)")
    try:
        with open(os.path.join(_IK_THIS_DIR, 'rx1_baseline_v5.json')) as f:
            baseline_data = json.load(f)

        servo_baseline = {}
        for sid in RIGHT_ARM_SERVO_IDS:
            servo_baseline[sid] = baseline_data['servos'][str(sid)]['position']

        pos_baseline = ik.get_end_effector_position(servo_baseline)
        print(f"   Servos: {[servo_baseline[sid] for sid in RIGHT_ARM_SERVO_IDS]}")
        print(f"   EE Position: X={pos_baseline[0]*100:.2f}cm, Y={pos_baseline[1]*100:.2f}cm, Z={pos_baseline[2]*100:.2f}cm")
    except Exception as e:
        print(f"   Error loading baseline: {e}")
        pos_baseline = pos_zero
        servo_baseline = servo_zero

    # 3. IK 테스트 - X만 이동
    print("\n3. IK Test: X +5cm only")
    target = [pos_baseline[0] + 0.05, pos_baseline[1], pos_baseline[2]]
    print(f"   Target: X={target[0]*100:.2f}cm, Y={target[1]*100:.2f}cm, Z={target[2]*100:.2f}cm")

    new_servos, success = ik.solve_for_position(target, servo_baseline)

    if success:
        result_pos = ik.get_end_effector_position(new_servos)
        print(f"   Result: X={result_pos[0]*100:.2f}cm, Y={result_pos[1]*100:.2f}cm, Z={result_pos[2]*100:.2f}cm")
        print(f"   Delta:  dX={((result_pos[0]-pos_baseline[0])*100):+.2f}cm, dY={((result_pos[1]-pos_baseline[1])*100):+.2f}cm, dZ={((result_pos[2]-pos_baseline[2])*100):+.2f}cm")

        error = np.linalg.norm(result_pos - np.array(target)) * 1000
        print(f"   Error: {error:.2f}mm")

        print(f"   New servos: {[new_servos[sid] for sid in RIGHT_ARM_SERVO_IDS]}")
    else:
        print("   IK FAILED!")

    # 4. 연속 이동 테스트
    print("\n4. Sequential movement test (X direction)")
    current_servos = servo_baseline.copy()
    current_pos = pos_baseline.copy()

    for dx_cm in [2, 4, 6, 8, 10]:
        target = [pos_baseline[0] + dx_cm/100, pos_baseline[1], pos_baseline[2]]
        new_servos, success = ik.solve_for_position(target, current_servos)

        if success:
            result = ik.get_end_effector_position(new_servos)
            dy_mm = (result[1] - pos_baseline[1]) * 1000
            dz_mm = (result[2] - pos_baseline[2]) * 1000
            print(f"   X+{dx_cm}cm: dY={dy_mm:+.1f}mm, dZ={dz_mm:+.1f}mm")
            current_servos = new_servos
        else:
            print(f"   X+{dx_cm}cm: FAILED")
            break

    print("\n=== Test Complete ===")


# Alias for GUI compatibility
RX1ArmIK = RX1CustomIK
