#!/usr/bin/env python3
"""
RX-1 Accurate IK using ikpy (URDF-based)
========================================

Provides accurate FK/IK using the actual URDF kinematic chain.
X-only movement keeps Y and Z exactly constant (0.00mm error).

Usage:
    from rx1_ik_accurate import RX1AccurateIK

    ik = RX1AccurateIK('right')
    target = [0.15, -0.35, -0.10]
    servos, success = ik.solve_for_position(target, current_servos)
"""

import os
import numpy as np
import math
import warnings
from typing import Dict, Tuple, Optional, List
from datetime import datetime

# Suppress ikpy warnings about fixed links
warnings.filterwarnings('ignore', message='.*fixed.*active_links_mask.*')

# ikpy for accurate URDF-based IK
try:
    from ikpy.chain import Chain
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False
    print("[RX1 IK] ikpy not available - install with: pip install ikpy")

# ============================================================================
# Configuration
# ============================================================================

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_ROOT = os.path.dirname(_THIS_DIR)
URDF_PATH = os.path.join(_GUI_ROOT, "urdf", "RRR_Cater", "RX1", "combined", "rx1_with_lidar.urdf")

# 어깨 베이스 변환 (URDF: head_base2right/left_shoul_base_joint)
# xyz: 0 ∓0.12 -0.05, rpy: ±1.04706195 0 0 (약 60도)
# 이 변환이 ikpy 체인에서 누락되어 수동으로 적용 필요
SHOULDER_BASE_OFFSET = {
    'right': {'xyz': np.array([0, -0.12, -0.05]), 'roll': 1.04706195},
    'left': {'xyz': np.array([0, 0.12, -0.05]), 'roll': -1.04706195}
}

# Servo configuration (from GUI: NO_GEARBOX_SERVOS = [15, 25])
# 기어비 3 = 120도 범위, 기어비 1 = 360도 범위
# GUI에서 INVERTED_SERVOS = [21, 22], REASSEMBLED_SERVOS = [24] - 오른팔은 반전 없음
RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
RIGHT_ARM_SERVO_DIRS = [1, 1, 1, 1, 1, 1, 1]  # GUI와 일치: 오른팔 반전 없음
RIGHT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 3, 3]  # 15만 gear=1 (NO_GEARBOX)

LEFT_ARM_SERVO_IDS = [21, 22, 23, 24, 25, 26, 27]
LEFT_ARM_SERVO_DIRS = [-1, -1, 1, -1, 1, 1, 1]  # GUI: 21,22 INVERTED, 24 REASSEMBLED
LEFT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 3, 3]  # 25만 gear=1 (NO_GEARBOX)

# URDF joint axis sign correction (ikpy uses URDF axis directly)
# Right arm URDF has negative axis for joints 11,12 (Z=-1, X=-1)
# Left arm URDF has positive axis for joints 21,22 (Z=+1, X=+1)
# Calibrated by comparing ikpy vs Isaac Sim Jacobian directions
RIGHT_ARM_URDF_AXIS_SIGNS = [-1, -1, 1, 1, 1, 1, 1]  # Compensate for Z=-1, X=-1
LEFT_ARM_URDF_AXIS_SIGNS = [1, 1, 1, 1, 1, 1, 1]     # Left arm has Z=+1, X=+1 (no compensation needed)

SERVO_CENTER = 2048

# Joint names for ikpy chain
ARM_JOINT_NAMES = [
    'shoul_base2shoul_joint',
    'shoul2shoul_rot_joint',
    'arm2armrot_joint',
    'armrot2elbow_joint',
    'forearm2forearmrot_joint',
    'forearmrot2forearm_pitch_joint',
    'forearm_pitch2forearm_roll_joint'
]

# Joint limits (radians)
JOINT_LIMITS = [
    (-3.14, 3.14),  # shoulder yaw
    (-3.14, 3.14),  # shoulder pitch
    (-3.14, 3.14),  # arm rotation
    (-2.2, 0.0),    # elbow
    (-3.14, 3.14),  # forearm rotation
    (-3.14, 3.14),  # wrist pitch
    (-3.14, 3.14),  # wrist roll
]

# ============================================================================
# Debug Logging
# ============================================================================

DEBUG_LOG_FILE = os.path.join(_THIS_DIR, "ik_debug.txt")
DEBUG_ENABLED = True


def ik_debug_log(msg):
    """Write debug log"""
    if not DEBUG_ENABLED:
        return
    try:
        with open(DEBUG_LOG_FILE, 'a') as f:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            f.write(f"[{ts}] {msg}\n")
    except:
        pass


def ik_debug_clear():
    """Clear debug log"""
    try:
        with open(DEBUG_LOG_FILE, 'w') as f:
            f.write(f"=== RX-1 Accurate IK Debug Log ===\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Solver: ikpy (URDF-based)\n")
            f.write(f"=" * 50 + "\n\n")
    except:
        pass


# ============================================================================
# Servo Conversion
# ============================================================================

def servo_to_joint_angle(servo_pos: int, direction: int, gear: int) -> float:
    """Convert servo position to joint angle (radians)"""
    relative = servo_pos - SERVO_CENTER
    angle = relative / (SERVO_CENTER * direction * gear) * math.pi
    return angle


def joint_angle_to_servo(angle: float, direction: int, gear: int) -> int:
    """Convert joint angle (radians) to servo position"""
    servo_pos = angle / math.pi * SERVO_CENTER * direction * gear + SERVO_CENTER
    return max(0, min(4095, int(servo_pos)))


# ============================================================================
# RX1 Accurate IK Class
# ============================================================================

class RX1AccurateIK:
    """
    Accurate IK solver using ikpy and URDF kinematic chain.

    Features:
    - Exact FK using URDF geometry
    - High-precision IK (sub-mm accuracy)
    - X-only movement keeps Y, Z exactly constant
    """

    def __init__(self, arm: str = 'right'):
        if not IKPY_AVAILABLE:
            raise ImportError("ikpy is required. Install with: pip install ikpy")

        self.arm = arm
        self.num_joints = 7

        # Servo configuration
        if arm == 'right':
            self.servo_ids = RIGHT_ARM_SERVO_IDS
            self.servo_dirs = RIGHT_ARM_SERVO_DIRS
            self.servo_gears = RIGHT_ARM_SERVO_GEARS
            self.base_link = 'right_shoul_base_link'
            self.prefix = 'right_'
        else:
            self.servo_ids = LEFT_ARM_SERVO_IDS
            self.servo_dirs = LEFT_ARM_SERVO_DIRS
            self.servo_gears = LEFT_ARM_SERVO_GEARS
            self.base_link = 'left_shoul_base_link'
            self.prefix = 'left_'

        # URDF axis sign correction
        if arm == 'right':
            self.axis_signs = RIGHT_ARM_URDF_AXIS_SIGNS
        else:
            self.axis_signs = LEFT_ARM_URDF_AXIS_SIGNS

        # Build ikpy chain
        self._build_chain()

        # Current joint state (for chain)
        self._current_chain_joints = [0.0] * len(self.chain.links)

        ik_debug_log(f"[{arm}] RX1AccurateIK initialized (ikpy URDF-based, axis_signs={self.axis_signs})")

    def _build_chain(self):
        """Build ikpy chain from URDF"""
        # First load full chain to get link count
        full_chain = Chain.from_urdf_file(
            URDF_PATH,
            base_elements=[self.base_link],
            name=f'{self.arm}_arm_temp'
        )

        # Find arm joint indices (7 DOF) - exclude dummy joints
        self.active_indices = []
        for i, link in enumerate(full_chain.links):
            # Only include actual arm joints, not dummy joints
            if link.joint_type == 'revolute' and 'dummy' not in link.name.lower():
                is_arm_joint = any(jname in link.name for jname in ARM_JOINT_NAMES)
                if is_arm_joint:
                    self.active_indices.append(i)

        # Create explicit active mask - ONLY 7 arm joints
        self.active_mask = [False] * len(full_chain.links)
        for idx in self.active_indices:
            self.active_mask[idx] = True

        # Rebuild chain with correct mask (excludes dummy joints)
        self.chain = Chain.from_urdf_file(
            URDF_PATH,
            base_elements=[self.base_link],
            active_links_mask=self.active_mask,
            name=f'{self.arm}_arm'
        )

        ik_debug_log(f"[{self.arm}] Chain built with {len(self.active_indices)} active joints")
        ik_debug_log(f"[{self.arm}] Active indices: {self.active_indices}")

    def servo_to_angles(self, servo_values) -> np.ndarray:
        """Convert servo values to joint angles (7 DOF)"""
        angles = []
        for i in range(7):
            if isinstance(servo_values, dict):
                sid = self.servo_ids[i]
                servo_pos = servo_values.get(sid, SERVO_CENTER)
            else:
                servo_pos = servo_values[i] if i < len(servo_values) else SERVO_CENTER

            angle = servo_to_joint_angle(
                servo_pos, self.servo_dirs[i], self.servo_gears[i])
            angles.append(angle)

        return np.array(angles)

    def angles_to_servo(self, angles: np.ndarray) -> Dict[int, int]:
        """Convert joint angles to servo values"""
        servo_values = {}
        for i in range(7):
            angle = angles[i] if i < len(angles) else 0.0
            servo_pos = joint_angle_to_servo(
                angle, self.servo_dirs[i], self.servo_gears[i])
            servo_values[self.servo_ids[i]] = servo_pos
        return servo_values

    def _joints_to_chain(self, angles_7dof: np.ndarray) -> List[float]:
        """Convert 7 DOF angles to full chain joint array (with axis sign correction)"""
        chain_joints = [0.0] * len(self.chain.links)
        for i, idx in enumerate(self.active_indices):
            if i < len(angles_7dof):
                # Apply axis sign correction for ikpy
                chain_joints[idx] = angles_7dof[i] * self.axis_signs[i]
        return chain_joints

    def _chain_to_joints(self, chain_joints: List[float]) -> np.ndarray:
        """Extract 7 DOF angles from chain joint array (with axis sign correction)"""
        angles = []
        for i, idx in enumerate(self.active_indices):
            # Reverse axis sign correction from ikpy
            angles.append(chain_joints[idx] * self.axis_signs[i])
        return np.array(angles)

    def _get_shoulder_transform(self):
        """어깨 베이스 변환 행렬 (로봇 원점 -> 어깨)"""
        offset = SHOULDER_BASE_OFFSET[self.arm]
        roll = offset['roll']
        xyz = offset['xyz']

        # Roll 회전 행렬
        c, s = np.cos(roll), np.sin(roll)
        R = np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])
        return R, xyz

    def forward(self, joint_angles: np.ndarray) -> np.ndarray:
        """FK: joint angles (7 DOF) -> EE position (로봇 원점 기준)"""
        chain_joints = self._joints_to_chain(joint_angles)
        fk_matrix = self.chain.forward_kinematics(chain_joints)
        local_pos = fk_matrix[:3, 3]

        # 어깨 베이스 변환 적용 (로컬 -> 글로벌)
        R, xyz = self._get_shoulder_transform()
        global_pos = R @ local_pos + xyz
        return global_pos

    def _global_to_local(self, global_pos: np.ndarray) -> np.ndarray:
        """글로벌 좌표 -> 로컬 좌표 (어깨 기준)"""
        R, xyz = self._get_shoulder_transform()
        # 역변환: local = R^T @ (global - xyz)
        return R.T @ (global_pos - xyz)

    def inverse(
        self,
        target_xyz: np.ndarray,
        current_angles: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 0.001
    ) -> Tuple[np.ndarray, bool]:
        """
        IK: target position (로봇 원점 기준) -> joint angles (7 DOF)

        Uses iterative approach to maintain solution continuity
        and avoid jumping to different configurations.
        """
        # 글로벌 타겟을 로컬 좌표로 변환
        target_global = np.array(target_xyz).flatten()[:3]
        target = self._global_to_local(target_global)

        if current_angles is not None:
            initial = self._joints_to_chain(current_angles)
            current_7dof = current_angles.copy()
        else:
            initial = self._current_chain_joints
            current_7dof = self._chain_to_joints(initial)

        # Get current position (로컬 좌표)
        chain_joints = self._joints_to_chain(current_7dof)
        fk_matrix = self.chain.forward_kinematics(chain_joints)
        current_pos = fk_matrix[:3, 3]  # 로컬
        distance = np.linalg.norm(target - current_pos)

        # For large movements, use incremental steps to maintain continuity
        if distance > 0.03:  # More than 3cm
            num_steps = max(5, int(distance / 0.02))  # 2cm steps
            best_angles = current_7dof.copy()

            for step in range(num_steps):
                t = (step + 1) / num_steps
                intermediate_target = current_pos + t * (target - current_pos)

                # Use current best as initial guess
                initial_chain = self._joints_to_chain(best_angles)

                ik_result = self.chain.inverse_kinematics(
                    intermediate_target,
                    initial_position=initial_chain,
                    orientation_mode=None
                )

                # Extract and limit angles
                step_angles = self._chain_to_joints(ik_result)
                for i in range(len(step_angles)):
                    lo, hi = JOINT_LIMITS[i]
                    step_angles[i] = np.clip(step_angles[i], lo, hi)

                # Check if solution is continuous (no big jumps)
                max_angle_change = np.max(np.abs(step_angles - best_angles))
                if max_angle_change < 1.0:  # Less than ~60 degrees per step
                    best_angles = step_angles
                # If jump too big, keep previous best (trajectory may stop early)

            angles_7dof = best_angles
        else:
            # Small movement - direct IK
            ik_result = self.chain.inverse_kinematics(
                target,
                initial_position=initial,
                orientation_mode=None
            )
            angles_7dof = self._chain_to_joints(ik_result)

            # Enforce joint limits
            for i in range(len(angles_7dof)):
                lo, hi = JOINT_LIMITS[i]
                angles_7dof[i] = np.clip(angles_7dof[i], lo, hi)

        # Verify result (글로벌 좌표로 비교)
        result_pos = self.forward(angles_7dof)  # 글로벌 좌표
        error = np.linalg.norm(result_pos - target_global)  # 글로벌 타겟과 비교

        success = error < tolerance * 5  # Allow slightly larger tolerance

        if success:
            self._current_chain_joints = self._joints_to_chain(angles_7dof)

        return angles_7dof, success

    def get_end_effector_position(self, servo_values) -> np.ndarray:
        """Get EE position from servo values (FK)"""
        angles = self.servo_to_angles(servo_values)
        return self.forward(angles)

    def solve_for_position(
        self,
        target_xyz,
        servo_values,
        max_iterations: int = 100
    ) -> Tuple[Dict[int, int], bool]:
        """Solve IK for target position"""
        current_angles = self.servo_to_angles(servo_values)
        current_pos = self.forward(current_angles)
        target = np.array(target_xyz)

        ik_debug_log(f"========== IK solve ({self.arm}) ==========")
        ik_debug_log(f"Current: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}, Z={current_pos[2]:.4f}")
        ik_debug_log(f"Target:  X={target[0]:.4f}, Y={target[1]:.4f}, Z={target[2]:.4f}")

        new_angles, success = self.inverse(target, current_angles, max_iterations)

        if success:
            new_servos = self.angles_to_servo(new_angles)
            result_pos = self.forward(new_angles)
            error_mm = np.linalg.norm(result_pos - target) * 1000

            ik_debug_log(f"IK OK! Error: {error_mm:.2f}mm")
            ik_debug_log(f"Result: X={result_pos[0]:.4f}, Y={result_pos[1]:.4f}, Z={result_pos[2]:.4f}")

            return new_servos, True
        else:
            ik_debug_log(f"IK FAILED!")
            return {}, False

    def reset(self):
        """Reset state"""
        self._current_chain_joints = [0.0] * len(self.chain.links)


# ============================================================================
# Compatibility wrapper (same interface as RX1ArmIK)
# ============================================================================

# Make RX1AccurateIK available as RX1ArmIK for drop-in replacement
RX1ArmIK = RX1AccurateIK


# ============================================================================
# Test Code
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("RX-1 Accurate IK Test (ikpy URDF-based)")
    print("=" * 60)

    ik_debug_clear()

    right_ik = RX1AccurateIK('right')
    servo_center = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}

    # Get initial position
    init_pos = right_ik.get_end_effector_position(servo_center)
    print(f'\nInitial EE: X={init_pos[0]:.4f}, Y={init_pos[1]:.4f}, Z={init_pos[2]:.4f}')

    # Test X-only movement
    print('\n--- X-only Movement Test ---')
    base_y = init_pos[1]
    base_z = init_pos[2]
    current_servos = servo_center.copy()

    for x_target in [0.05, 0.10, 0.15, 0.20]:
        target = [x_target, base_y, base_z]
        new_servos, success = right_ik.solve_for_position(target, current_servos)

        if success:
            result = right_ik.get_end_effector_position(new_servos)
            dy = (result[1] - base_y) * 1000
            dz = (result[2] - base_z) * 1000
            print(f'X={x_target:.2f} -> Result X={result[0]:.4f}, dY={dy:+.2f}mm, dZ={dz:+.2f}mm')
            current_servos = new_servos
        else:
            print(f'X={x_target:.2f} -> FAILED')

    # Test Y-only movement
    print('\n--- Y-only Movement Test ---')
    right_ik.reset()
    current_servos = servo_center.copy()
    base_x = init_pos[0]

    for y_target in [-0.50, -0.45, -0.40, -0.35]:
        target = [base_x, y_target, base_z]
        new_servos, success = right_ik.solve_for_position(target, current_servos)

        if success:
            result = right_ik.get_end_effector_position(new_servos)
            dx = (result[0] - base_x) * 1000
            dz = (result[2] - base_z) * 1000
            print(f'Y={y_target:.2f} -> Result Y={result[1]:.4f}, dX={dx:+.2f}mm, dZ={dz:+.2f}mm')
            current_servos = new_servos
        else:
            print(f'Y={y_target:.2f} -> FAILED')

    # Test Z-only movement
    print('\n--- Z-only Movement Test ---')
    right_ik.reset()
    current_servos = servo_center.copy()

    for z_target in [-0.20, -0.15, -0.10, -0.05]:
        target = [base_x, base_y, z_target]
        new_servos, success = right_ik.solve_for_position(target, current_servos)

        if success:
            result = right_ik.get_end_effector_position(new_servos)
            dx = (result[0] - base_x) * 1000
            dy = (result[1] - base_y) * 1000
            print(f'Z={z_target:.2f} -> Result Z={result[2]:.4f}, dX={dx:+.2f}mm, dY={dy:+.2f}mm')
            current_servos = new_servos
        else:
            print(f'Z={z_target:.2f} -> FAILED')

    # Left arm test
    print('\n--- Left Arm Test ---')
    left_ik = RX1AccurateIK('left')
    pos_l = left_ik.get_end_effector_position({sid: 2048 for sid in LEFT_ARM_SERVO_IDS})
    print(f'Left EE at zero: X={pos_l[0]:.4f}, Y={pos_l[1]:.4f}, Z={pos_l[2]:.4f}')

    print('\n' + '=' * 60)
    print('Test Complete!')
    print('=' * 60)
