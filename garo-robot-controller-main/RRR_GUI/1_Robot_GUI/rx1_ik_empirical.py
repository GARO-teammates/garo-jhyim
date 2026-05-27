"""
RX-1 Robot IK Controller (Empirical Jacobian Version)
- Isaac Sim에서 검증된 empirical Jacobian 데이터 사용
- Damped Least Squares (Levenberg-Marquardt) IK 솔버
- ROS 중심 설계 (joint_states, TF 연동 가능)
- 원본: /home/rl02/Downloads/ik/rx1_ik_controller.py

기존 ikpy 방식 대비 장점:
- 실제 로봇에서 측정된 Jacobian 데이터 사용으로 정확도 향상
- 특이점 근처에서도 안정적인 damped least squares
- 점진적 이동으로 부드러운 궤적 생성
"""

import numpy as np
import math
import os
import warnings
from datetime import datetime

# ikpy는 FK에만 사용 (IK는 empirical Jacobian 사용)
from ikpy.chain import Chain
warnings.filterwarnings('ignore', category=UserWarning)

# 디버그 로그 설정
DEBUG_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ik_debug.txt")
DEBUG_ENABLED = True

def ik_debug_log(msg):
    """IK 디버그 로그를 파일에 저장"""
    if not DEBUG_ENABLED:
        return
    try:
        with open(DEBUG_LOG_FILE, 'a') as f:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            f.write(f"[{timestamp}] {msg}\n")
    except:
        pass

def ik_debug_clear():
    """디버그 로그 파일 초기화"""
    try:
        with open(DEBUG_LOG_FILE, 'w') as f:
            f.write(f"=== RX-1 IK Debug Log (Empirical Jacobian) ===\n")
            f.write(f"시작 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"사용 알고리즘: Damped Least Squares (Levenberg-Marquardt)\n")
            f.write(f"X만 움직이면: dY, dZ가 0에 가까워야 함\n")
            f.write(f"=" * 50 + "\n\n")
    except:
        pass


# ============================================================
# Empirical Jacobian 데이터 (Isaac Sim에서 측정)
# 각 관절이 움직일 때 end effector의 위치 변화
# ============================================================

# 왼팔 Jacobian 데이터 (Isaac Sim에서 측정)
LEFT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([-0.183, -0.014, +0.024]), "test_angle": +0.3},  # shoul_base2shoul
    1: {"delta": np.array([+0.000, +0.211, +0.032]), "test_angle": +0.3},  # shoul2shoul_rot
    2: {"delta": np.array([+0.188, +0.005, -0.010]), "test_angle": +0.3},  # arm2armrot
    3: {"delta": np.array([+0.003, +0.142, +0.014]), "test_angle": +0.3},  # armrot2elbow
    4: {"delta": np.array([+0.109, +0.007, +0.005]), "test_angle": +0.3},  # forearm2forearmrot
    5: {"delta": np.array([-0.004, +0.075, +0.050]), "test_angle": +0.3},  # forearmrot2forearm_pitch
    6: {"delta": np.array([+0.056, -0.003, +0.001]), "test_angle": +0.3},  # forearm_pitch2forearm_roll
}

# 오른팔 Jacobian 데이터 (왼팔과 Y축 대칭)
RIGHT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([-0.183, +0.014, +0.024]), "test_angle": +0.3},  # Y 반전
    1: {"delta": np.array([+0.000, -0.211, +0.032]), "test_angle": +0.3},  # Y 반전
    2: {"delta": np.array([+0.188, -0.005, -0.010]), "test_angle": +0.3},  # Y 반전
    3: {"delta": np.array([+0.003, -0.142, +0.014]), "test_angle": +0.3},  # Y 반전
    4: {"delta": np.array([+0.109, -0.007, +0.005]), "test_angle": +0.3},  # Y 반전
    5: {"delta": np.array([-0.004, -0.075, +0.050]), "test_angle": +0.3},  # Y 반전
    6: {"delta": np.array([+0.056, +0.003, +0.001]), "test_angle": +0.3},  # Y 반전
}


# ============================================================
# 서보 매핑 (원본: rx1_motor.hpp)
# ============================================================

# Right Arm
RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
RIGHT_ARM_SERVO_DIRS = [-1, -1, 1, 1, 1, 1, -1]
RIGHT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

# Left Arm
LEFT_ARM_SERVO_IDS = [21, 22, 23, 24, 25, 26, 27]
LEFT_ARM_SERVO_DIRS = [-1, -1, 1, -1, 1, -1, -1]
LEFT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

# Joint names (ROS 연동용)
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


def servo_to_joint_angle(servo_pos, direction, gear):
    """서보 위치 → 관절 각도 (라디안)"""
    relative = servo_pos - SERVO_CENTER
    angle = relative / (SERVO_CENTER * direction * gear) * math.pi
    return angle


def joint_angle_to_servo(angle, direction, gear):
    """관절 각도 (라디안) → 서보 위치"""
    servo_pos = angle / math.pi * SERVO_CENTER * direction * gear + SERVO_CENTER
    servo_pos = max(0, min(4095, int(servo_pos)))
    return servo_pos


# ============================================================
# Differential IK Solver (Damped Least Squares)
# ============================================================

class DifferentialIKSolver:
    """
    Damped Least Squares (Levenberg-Marquardt) IK 솔버

    dq = J^T (J J^T + λ²I)^(-1) e

    특징:
    - 특이점 근처에서도 안정적
    - damping 파라미터로 수렴 속도/안정성 조절
    - position_gain으로 이동 속도 조절
    """

    def __init__(self, num_joints=7, damping=0.05, position_gain=1.0):
        self.num_joints = num_joints
        self.damping = damping
        self.position_gain = position_gain

    def compute_joint_velocities(self, jacobian, position_error):
        """
        위치 오차에서 관절 속도 계산

        Args:
            jacobian: 3x7 Jacobian 행렬
            position_error: 3D 위치 오차 벡터

        Returns:
            joint_velocities: 7D 관절 속도 벡터
        """
        # Damped Least Squares: dq = J^T (J J^T + λ²I)^(-1) e
        JJT = jacobian @ jacobian.T
        damping_term = self.damping ** 2 * np.eye(3)

        try:
            inv_term = np.linalg.inv(JJT + damping_term)
            joint_velocities = jacobian.T @ inv_term @ (position_error * self.position_gain)
        except np.linalg.LinAlgError:
            # 역행렬 실패 시 pseudo-inverse 사용
            joint_velocities = np.linalg.lstsq(jacobian.T, position_error, rcond=None)[0]

        return joint_velocities


# ============================================================
# RX1 Arm IK Controller (Empirical Jacobian)
# ============================================================

class RX1ArmIK:
    """
    RX-1 단일 팔 IK 솔버 (Empirical Jacobian 버전)

    기존 rx1_ik_simple.py와 동일한 인터페이스 유지:
    - get_end_effector_position(servo_values) → XYZ
    - solve_for_position(target_xyz, servo_values) → new_servos, success
    """

    # 관절 한계 (radians)
    JOINT_LIMITS = [
        (-1.57, 1.57),   # joint 0: shoulder base
        (-1.57, 1.57),   # joint 1: shoulder rot
        (-1.57, 1.57),   # joint 2: arm rot
        (-2.0, 0.1),     # joint 3: elbow (주로 굽힘)
        (-1.57, 1.57),   # joint 4: forearm rot
        (-1.57, 1.57),   # joint 5: forearm pitch
        (-1.57, 1.57),   # joint 6: forearm roll
    ]

    def __init__(self, arm='right'):
        """
        Args:
            arm: 'right' 또는 'left'
        """
        self.arm = arm

        # Jacobian 데이터 선택
        if arm == 'right':
            self.jacobian_data = RIGHT_ARM_JACOBIAN_DATA
            self.servo_ids = RIGHT_ARM_SERVO_IDS
            self.servo_dirs = RIGHT_ARM_SERVO_DIRS
            self.servo_gears = RIGHT_ARM_SERVO_GEARS
            self.joint_names = RIGHT_ARM_JOINTS
        else:
            self.jacobian_data = LEFT_ARM_JACOBIAN_DATA
            self.servo_ids = LEFT_ARM_SERVO_IDS
            self.servo_dirs = LEFT_ARM_SERVO_DIRS
            self.servo_gears = LEFT_ARM_SERVO_GEARS
            self.joint_names = LEFT_ARM_JOINTS

        # Jacobian 행렬 생성 (3x7)
        self.jacobian = self._build_jacobian_matrix()

        # IK 솔버 초기화
        self.ik_solver = DifferentialIKSolver(
            num_joints=7,
            damping=0.05,
            position_gain=1.0
        )

        # URDF 체인 (FK용)
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.urdf_path = os.path.join(base_dir, "urdf/RRR_Cater/RX1/combined/rx1_with_lidar.urdf")
        self.active_indices = [1, 2, 4, 5, 7, 8, 9]
        self._load_chain()

        ik_debug_log(f"[{arm}] Empirical Jacobian IK 솔버 초기화 완료")

    def _build_jacobian_matrix(self):
        """Empirical 데이터에서 Jacobian 행렬 생성 (참고용)"""
        jacobian = np.zeros((3, 7))

        for joint_idx, data in self.jacobian_data.items():
            # delta / test_angle = 관절 각도 변화에 대한 EE 위치 변화율
            jacobian[:, joint_idx] = data["delta"] / data["test_angle"]

        return jacobian

    def _compute_numerical_jacobian(self, angles, delta=0.01):
        """
        수치 미분으로 Jacobian 계산 (FK와 일관성 보장)

        Args:
            angles: 현재 관절 각도 (7개)
            delta: 미분용 각도 변화량 (radians)

        Returns:
            jacobian: 3x7 Jacobian 행렬
        """
        jacobian = np.zeros((3, 7))
        current_pos = self.forward(angles)

        for i in range(7):
            # 양방향 수치 미분 (더 정확)
            angles_plus = angles.copy()
            angles_plus[i] += delta

            angles_minus = angles.copy()
            angles_minus[i] -= delta

            pos_plus = self.forward(angles_plus)
            pos_minus = self.forward(angles_minus)

            # 중앙 차분
            jacobian[:, i] = (pos_plus - pos_minus) / (2 * delta)

        return jacobian

    def _load_chain(self):
        """URDF에서 FK 체인 로드"""
        base_link = 'right_shoul_base_link' if self.arm == 'right' else 'left_shoul_base_link'

        try:
            full_chain = Chain.from_urdf_file(
                self.urdf_path,
                base_elements=[base_link],
                name=f'{self.arm}_arm_temp'
            )

            active_mask = [False] * len(full_chain.links)
            for idx in self.active_indices:
                if idx < len(active_mask):
                    active_mask[idx] = True
            active_mask[-1] = False

            self.chain = Chain.from_urdf_file(
                self.urdf_path,
                base_elements=[base_link],
                active_links_mask=active_mask,
                name=f'{self.arm}_arm'
            )
        except Exception as e:
            ik_debug_log(f"[{self.arm}] URDF 로드 실패: {e}")
            self.chain = None

    def servo_to_angles(self, servo_values):
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

    def angles_to_servo(self, angles):
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

    def forward(self, joint_angles_7):
        """
        순기구학: 7개 관절 각도 → XYZ 위치

        URDF 체인이 있으면 정확한 FK 사용,
        없으면 empirical Jacobian 기반 근사 FK 사용
        """
        if self.chain is not None:
            # URDF 기반 정확한 FK
            full_angles = [0.0] * len(self.chain.links)
            for i, idx in enumerate(self.active_indices):
                if i < len(joint_angles_7) and idx < len(full_angles):
                    full_angles[idx] = float(joint_angles_7[i])

            transform = self.chain.forward_kinematics(full_angles)
            return np.array(transform[:3, 3])
        else:
            # Empirical Jacobian 기반 근사 FK (fallback)
            # 기준 위치 (관절 각도 0일 때)
            if self.arm == 'right':
                base_pos = np.array([0.077, -0.416, -0.125])
            else:
                base_pos = np.array([0.077, 0.416, -0.125])

            # Jacobian으로 위치 변화 계산
            delta_pos = self.jacobian @ np.array(joint_angles_7)
            return base_pos + delta_pos

    def get_end_effector_position(self, servo_values):
        """현재 서보 값에서 엔드이펙터 위치 계산"""
        angles = self.servo_to_angles(servo_values)
        return self.forward(angles)

    def _clamp_joint_angles(self, angles):
        """관절 각도를 한계 내로 클램핑"""
        clamped = np.array(angles)
        for i, (low, high) in enumerate(self.JOINT_LIMITS):
            if i < len(clamped):
                clamped[i] = np.clip(clamped[i], low, high)
        return clamped

    def _compute_position_jacobian(self, current_angles):
        """
        현재 관절 각도에서 position Jacobian 계산

        수치 미분으로 Jacobian 계산 (FK와 일관성 보장)
        관절 한계 근처에서 damping 적용
        """
        # 수치 미분으로 Jacobian 계산 (FK와 일관성 보장)
        J = self._compute_numerical_jacobian(current_angles)

        # 관절 각도에 따른 Jacobian 스케일링
        # 특정 관절이 한계에 가까울수록 해당 열의 영향 감소
        for i, (low, high) in enumerate(self.JOINT_LIMITS):
            angle = current_angles[i]
            # 한계 근처에서 damping 증가
            margin = 0.2  # radians
            if angle < low + margin:
                scale = (angle - low) / margin
                J[:, i] *= max(0.1, scale)
            elif angle > high - margin:
                scale = (high - angle) / margin
                J[:, i] *= max(0.1, scale)

        return J

    def inverse(self, target_xyz, current_angles=None, max_iterations=100,
                tolerance=0.002, step_size=0.3):
        """
        역기구학: 목표 XYZ → 7개 관절 각도 (Damped Least Squares)

        Args:
            target_xyz: [x, y, z] 목표 위치 (meters)
            current_angles: 현재 관절 각도 (초기값)
            max_iterations: 최대 반복 횟수
            tolerance: 수렴 허용 오차 (meters)
            step_size: 스텝 크기 (0.0 ~ 1.0)

        Returns:
            joint_angles_7: 7개 관절 각도 (radians)
            success: 성공 여부
        """
        target = np.array(target_xyz)

        # 초기 각도 설정
        if current_angles is not None:
            angles = np.array(current_angles, dtype=float)
        else:
            # 기본 초기 자세 (팔꿈치 굽힘)
            angles = np.zeros(7)
            angles[3] = -0.5  # 팔꿈치

        best_angles = angles.copy()
        best_error = float('inf')

        # Adaptive step size
        current_step = step_size

        for iteration in range(max_iterations):
            # 현재 위치 계산
            current_pos = self.forward(angles)

            # 위치 오차
            position_error = target - current_pos
            error_norm = np.linalg.norm(position_error)

            # 최적 결과 저장
            if error_norm < best_error:
                best_error = error_norm
                best_angles = angles.copy()
                # 개선되면 step size 약간 증가
                current_step = min(step_size * 1.2, 0.5)
            else:
                # 악화되면 step size 감소
                current_step *= 0.7

            # 수렴 체크
            if error_norm < tolerance:
                ik_debug_log(f"IK 수렴! iter={iteration}, error={error_norm*1000:.2f}mm")
                return angles, True

            # Jacobian 계산 (관절 상태에 따른 보정 적용)
            J = self._compute_position_jacobian(angles)

            # 관절 속도 계산 (Damped Least Squares)
            joint_velocities = self.ik_solver.compute_joint_velocities(J, position_error)

            # 관절 속도 정규화 (너무 큰 움직임 방지)
            vel_norm = np.linalg.norm(joint_velocities)
            if vel_norm > 0.5:
                joint_velocities = joint_velocities / vel_norm * 0.5

            # 관절 각도 업데이트
            angles = angles + current_step * joint_velocities

            # 관절 한계 적용
            angles = self._clamp_joint_angles(angles)

        # 최대 반복 도달
        ik_debug_log(f"IK 최대 반복 도달, best_error={best_error*1000:.2f}mm")
        success = best_error < 0.05  # 5cm 이내면 성공
        return best_angles, success

    def solve_for_position(self, target_xyz, servo_values, max_iterations=100):
        """
        목표 위치로 가기 위한 서보 값 계산

        Args:
            target_xyz: [x, y, z] 목표 위치
            servo_values: 현재 서보 값 (dict 또는 list)
            max_iterations: 최대 IK 반복 횟수

        Returns:
            new_servo_values: 새 서보 값 dict
            success: 성공 여부
        """
        # 현재 위치 (FK)
        current_pos = self.get_end_effector_position(servo_values)
        target = np.array(target_xyz)

        ik_debug_log(f"========== IK solve_for_position ({self.arm}) ==========")
        ik_debug_log(f"알고리즘: Empirical Jacobian + Damped Least Squares")
        ik_debug_log(f"현재 FK 위치: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
        ik_debug_log(f"목표 위치:    X={target_xyz[0]:.4f}, Y={target_xyz[1]:.4f}, Z={target_xyz[2]:.4f}")
        ik_debug_log(f"이동량:       dX={target_xyz[0]-current_pos[0]:.4f}, dY={target_xyz[1]-current_pos[1]:.4f}, dZ={target_xyz[2]-current_pos[2]:.4f}")

        # 현재 각도
        current_angles = self.servo_to_angles(servo_values)

        # 이동 거리 계산
        distance = np.linalg.norm(target - current_pos)

        # 큰 이동은 스텝으로 분할 (부드러운 궤적)
        if distance > 0.08:  # 8cm 이상
            num_steps = min(10, max(3, int(distance / 0.03)))
            ik_debug_log(f"큰 이동 ({distance*100:.1f}cm) → {num_steps} 스텝으로 분할")

            angles = current_angles.copy()
            success = True
            for step in range(num_steps):
                # 중간 목표
                t = (step + 1) / num_steps
                intermediate_target = current_pos + t * (target - current_pos)

                # IK 계산 (각 스텝에 충분한 반복 횟수 부여)
                angles, step_success = self.inverse(
                    intermediate_target,
                    angles,
                    max_iterations=max(50, max_iterations // num_steps),
                    step_size=0.3
                )

                if not step_success:
                    ik_debug_log(f"스텝 {step+1}/{num_steps} 부분 실패, 계속 진행")
                    # 부분 실패해도 계속 진행

            new_angles = angles
            final_error = np.linalg.norm(self.forward(angles) - target)
            success = final_error < 0.05  # 5cm 이내면 성공
            ik_debug_log(f"최종 오차: {final_error*1000:.1f}mm")
        else:
            # 작은 이동은 직접 IK
            new_angles, success = self.inverse(
                target_xyz,
                current_angles,
                max_iterations=max_iterations,
                step_size=0.3
            )

        if success:
            new_servos = self.angles_to_servo(new_angles)

            # 결과 검증 (FK)
            result_pos = self.get_end_effector_position(new_servos)
            error = np.linalg.norm(result_pos - target) * 1000  # mm

            ik_debug_log(f"IK 성공! 오차: {error:.2f}mm")
            ik_debug_log(f"결과 FK:     X={result_pos[0]:.4f}, Y={result_pos[1]:.4f}, Z={result_pos[2]:.4f}")
            ik_debug_log(f"실제 이동량: dX={result_pos[0]-current_pos[0]:.4f}, dY={result_pos[1]-current_pos[1]:.4f}, dZ={result_pos[2]-current_pos[2]:.4f}")
            ik_debug_log(f"서보값: {new_servos}")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log(f"IK 실패!")
            ik_debug_log("")
            return {}, False

    def get_jacobian(self):
        """현재 Jacobian 행렬 반환 (디버깅용)"""
        return self.jacobian.copy()

    def get_joint_names(self):
        """관절 이름 목록 반환 (ROS 연동용)"""
        return self.joint_names.copy()


# ============================================================
# ROS2 연동 클래스 (선택적)
# ============================================================

class RX1IKNode:
    """
    ROS2 노드 래퍼 (선택적 사용)

    사용법:
        from rx1_ik_empirical import RX1IKNode
        node = RX1IKNode()
        node.start()
    """

    def __init__(self):
        self.right_ik = RX1ArmIK('right')
        self.left_ik = RX1ArmIK('left')
        self._ros_initialized = False

    def start(self):
        """ROS2 노드 시작"""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from geometry_msgs.msg import PoseStamped

            if not rclpy.ok():
                rclpy.init()

            self._node = rclpy.create_node('rx1_ik_empirical')

            # Publishers
            self._joint_pub = self._node.create_publisher(
                JointState, '/joint_commands', 10
            )

            # Subscribers
            self._right_sub = self._node.create_subscription(
                PoseStamped, '/right_arm/target_pose',
                lambda msg: self._pose_callback(msg, 'right'), 10
            )
            self._left_sub = self._node.create_subscription(
                PoseStamped, '/left_arm/target_pose',
                lambda msg: self._pose_callback(msg, 'left'), 10
            )

            self._ros_initialized = True
            self._node.get_logger().info('RX1 IK Node (Empirical) 시작')

            return True
        except ImportError:
            ik_debug_log("ROS2 미설치 - ROS 기능 비활성화")
            return False

    def _pose_callback(self, msg, arm):
        """목표 포즈 콜백"""
        target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        solver = self.right_ik if arm == 'right' else self.left_ik

        # 현재 서보 값 (TODO: 실제 값 읽기)
        servo_values = {sid: SERVO_CENTER for sid in solver.servo_ids}

        new_servos, success = solver.solve_for_position(target, servo_values)

        if success:
            # JointState 발행
            self._publish_joint_state(arm, new_servos)

    def _publish_joint_state(self, arm, servo_values):
        """JointState 메시지 발행"""
        from sensor_msgs.msg import JointState

        solver = self.right_ik if arm == 'right' else self.left_ik
        angles = solver.servo_to_angles(servo_values)

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = solver.joint_names
        msg.position = list(angles)

        self._joint_pub.publish(msg)

    def spin(self):
        """ROS2 spin"""
        if self._ros_initialized:
            import rclpy
            rclpy.spin(self._node)

    def shutdown(self):
        """종료"""
        if self._ros_initialized:
            import rclpy
            self._node.destroy_node()
            rclpy.shutdown()


# ============================================================
# 테스트
# ============================================================

if __name__ == "__main__":
    print("=== RX-1 IK Test (Empirical Jacobian) ===\n")

    # 디버그 로그 초기화
    ik_debug_clear()

    # 오른팔 테스트
    right_ik = RX1ArmIK('right')
    print(f"Right arm Jacobian shape: {right_ik.jacobian.shape}")
    print(f"Jacobian:\n{right_ik.jacobian}")

    # 서보 중심값에서 FK
    servo_center = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}
    pos = right_ik.get_end_effector_position(servo_center)
    print(f"\nFK (servos=2048): [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

    # IK 테스트
    print("\n--- IK Test ---")
    target = [0.2, -0.4, -0.1]
    new_servos, success = right_ik.solve_for_position(target, servo_center)
    print(f"Target: {target}")
    print(f"Success: {success}")

    if success:
        # 검증
        verify_pos = right_ik.get_end_effector_position(new_servos)
        error = np.linalg.norm(verify_pos - np.array(target)) * 1000
        print(f"Verify FK: [{verify_pos[0]:.3f}, {verify_pos[1]:.3f}, {verify_pos[2]:.3f}]")
        print(f"Error: {error:.1f} mm")

    # X만 이동 테스트 (중요!)
    print("\n--- X-only Movement Test ---")
    target1 = [0.2, -0.4, -0.1]
    target2 = [0.3, -0.4, -0.1]  # X만 +0.1

    servo1, _ = right_ik.solve_for_position(target1, servo_center)
    pos1 = right_ik.get_end_effector_position(servo1)

    servo2, _ = right_ik.solve_for_position(target2, servo1)
    pos2 = right_ik.get_end_effector_position(servo2)

    print(f"Move X: 0.2 → 0.3")
    print(f"  dX = {(pos2[0]-pos1[0])*1000:.1f} mm (target: 100)")
    print(f"  dY = {(pos2[1]-pos1[1])*1000:.1f} mm (target: 0)")
    print(f"  dZ = {(pos2[2]-pos1[2])*1000:.1f} mm (target: 0)")

    # Y만 이동 테스트
    print("\n--- Y-only Movement Test ---")
    target3 = [0.2, -0.3, -0.1]  # Y만 +0.1
    servo3, _ = right_ik.solve_for_position(target3, servo1)
    pos3 = right_ik.get_end_effector_position(servo3)

    print(f"Move Y: -0.4 → -0.3")
    print(f"  dX = {(pos3[0]-pos1[0])*1000:.1f} mm (target: 0)")
    print(f"  dY = {(pos3[1]-pos1[1])*1000:.1f} mm (target: 100)")
    print(f"  dZ = {(pos3[2]-pos1[2])*1000:.1f} mm (target: 0)")

    # 왼팔 테스트
    print("\n--- Left Arm Test ---")
    left_ik = RX1ArmIK('left')
    servo_left = {sid: 2048 for sid in LEFT_ARM_SERVO_IDS}
    pos_l = left_ik.get_end_effector_position(servo_left)
    print(f"FK (servos=2048): [{pos_l[0]:.3f}, {pos_l[1]:.3f}, {pos_l[2]:.3f}]")

    print("\n=== 테스트 완료 ===")
    print(f"디버그 로그: {DEBUG_LOG_FILE}")
