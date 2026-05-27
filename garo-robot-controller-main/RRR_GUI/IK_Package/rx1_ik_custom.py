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
from typing import Dict, List, Tuple, Optional
from datetime import datetime

# Debug log
DEBUG_LOG_FILE = "/home/rrr/Desktop/RRR/RRR_GUI_Package_v7/1_Robot_GUI/ik_debug.txt"

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
VIZ_BASELINE_OVERRIDE = {14: 4095, 24: 150}

# 시각화에서 사용하는 반전 (RVIZ_AUTO_INVERTED + ROS_SIM_INVERTED 상쇄됨)
# 서보 14: (-1) * (-1) = 1, 즉 반전 없음
# 결론: 오른팔은 반전 없음
VIZ_RIGHT_ARM_INVERSIONS = [1, 1, 1, 1, 1, 1, 1]  # 모두 1 (반전 없음)

# GUI's Real_RRR_GUI_v5.py
NO_GEARBOX_SERVOS = [15, 25]  # 360 deg
SERVO_CENTER = 2048

# 로봇 베이스라인 (JSON에서 로드, 초기화 시 설정됨)
ROBOT_BASELINE = {}

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
        Initialize IK solver with visualization-matching conventions.

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
            raise NotImplementedError("Left arm not implemented yet")

        self.num_joints = 7

        # 로봇 베이스라인 저장 (시각화 베이스라인 계산용)
        self.robot_baseline = robot_baseline or {}

        # 시각화 베이스라인 계산 (VIZ_BASELINE_OVERRIDE 적용)
        self.viz_baseline = {}
        for sid in self.servo_ids:
            if sid in VIZ_BASELINE_OVERRIDE:
                self.viz_baseline[sid] = VIZ_BASELINE_OVERRIDE[sid]
            else:
                self.viz_baseline[sid] = self.robot_baseline.get(sid, SERVO_CENTER)

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

    def solve_for_position(self, target_xyz: List[float],
                          servo_values: Dict[int, int],
                          max_iterations: int = 100) -> Tuple[Dict[int, int], bool]:
        """
        Solve IK for target position (WORLD coordinates)

        Uses servo values DIRECTLY (no conversion needed).
        The FK maps servo values -> joint angles -> EE position.

        Args:
            target_xyz: [x, y, z] target position in WORLD frame
            servo_values: current servo values dict
            max_iterations: max IK iterations

        Returns:
            new_servo_values: new servo values dict
            success: success flag
        """
        # Use servo values directly for FK (no conversion)
        current_pos = self.get_end_effector_position(servo_values)
        target = np.array(target_xyz)

        ik_debug_log(f"=== IK solve_for_position WORLD ({self.arm}) ===")
        ik_debug_log(f"Input servos: {[servo_values.get(sid, 0) for sid in self.servo_ids]}")
        ik_debug_log(f"Current: X={current_pos[0]*100:.2f}, Y={current_pos[1]*100:.2f}, Z={current_pos[2]*100:.2f} cm")
        ik_debug_log(f"Target:  X={target[0]*100:.2f}, Y={target[1]*100:.2f}, Z={target[2]*100:.2f} cm")
        ik_debug_log(f"Delta:   dX={((target[0]-current_pos[0])*100):+.2f}, dY={((target[1]-current_pos[1])*100):+.2f}, dZ={((target[2]-current_pos[2])*100):+.2f} cm")

        # Current angles directly from servo values
        current_angles = np.zeros(self.num_joints)
        for i, sid in enumerate(self.servo_ids):
            servo_val = servo_values.get(sid, SERVO_CENTER)
            current_angles[i] = self.servo_to_angle(servo_val, i)

        # IK solve
        new_angles, success = self.inverse_kinematics(
            target_xyz, current_angles, max_iterations
        )

        if success:
            # Convert angles directly to servo values (no conversion)
            new_servos = {}
            for i, sid in enumerate(self.servo_ids):
                new_servos[sid] = self.angle_to_servo(new_angles[i], i)

            # Verify result
            result_pos = self.get_end_effector_position(new_servos)
            error_vec = result_pos - target
            error_total = np.linalg.norm(error_vec) * 1000

            ik_debug_log(f"Result:  X={result_pos[0]*100:.2f}, Y={result_pos[1]*100:.2f}, Z={result_pos[2]*100:.2f} cm")
            ik_debug_log(f"Actual move: dX={((result_pos[0]-current_pos[0])*100):+.2f}, dY={((result_pos[1]-current_pos[1])*100):+.2f}, dZ={((result_pos[2]-current_pos[2])*100):+.2f} cm")
            ik_debug_log(f"--- World Coordinate Error ---")
            ik_debug_log(f"Error X: {error_vec[0]*1000:+.2f} mm")
            ik_debug_log(f"Error Y: {error_vec[1]*1000:+.2f} mm")
            ik_debug_log(f"Error Z: {error_vec[2]*1000:+.2f} mm")
            ik_debug_log(f"Error Total: {error_total:.2f} mm")
            ik_debug_log(f"Output servos: {[new_servos[sid] for sid in self.servo_ids]}")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log("IK FAILED!")
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
        with open('/home/rrr/Desktop/RRR/RRR_GUI_Package_v7/1_Robot_GUI/rx1_baseline_v5.json') as f:
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
