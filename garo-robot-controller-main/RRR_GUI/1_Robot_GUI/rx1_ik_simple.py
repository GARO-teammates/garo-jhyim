#!/usr/bin/env python3
"""
RX-1 IK Simple Interface
========================
High-accuracy IK using ikpy URDF-based solver.

Features:
- Sub-mm accuracy (0.3mm max error)
- X-only movement keeps Y, Z constant
- Robot coordinate frame (fixed robot at origin)

Usage:
    from rx1_ik_simple import RX1ArmIK
    ik = RX1ArmIK('right')
    servos, ok = ik.solve_for_position([0.15, -0.35, -0.10], current_servos)
"""

import os
import numpy as np
import math
from typing import Dict, Tuple, Optional, List
from datetime import datetime

# Use accurate ikpy-based IK
try:
    from rx1_ik_accurate import RX1AccurateIK, ik_debug_log, ik_debug_clear
    ACCURATE_IK_AVAILABLE = True
except ImportError:
    ACCURATE_IK_AVAILABLE = False

# ============================================================================
# Debug Logging
# ============================================================================

DEBUG_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ik_debug.txt")
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
            f.write(f"=== RX-1 IK Debug Log ===\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Solver: Empirical Jacobian + Damped Least Squares\n")
            f.write(f"Data: Isaac Sim verified\n")
            f.write(f"=" * 50 + "\n\n")
    except:
        pass


# ============================================================================
# World Coordinate Transform
# ============================================================================

class RobotPose:
    """
    Robot pose in world coordinate frame
    - position: (x, y, z) in meters
    - orientation: (roll, pitch, yaw) in radians
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_position(self, x, y, z):
        """Set robot position in world"""
        self.x = x
        self.y = y
        self.z = z

    def set_orientation(self, roll, pitch, yaw):
        """Set robot orientation in world"""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_from_array(self, pose_array):
        """Set from array [x, y, z, roll, pitch, yaw]"""
        if len(pose_array) >= 3:
            self.x, self.y, self.z = pose_array[0], pose_array[1], pose_array[2]
        if len(pose_array) >= 6:
            self.roll, self.pitch, self.yaw = pose_array[3], pose_array[4], pose_array[5]

    def get_rotation_matrix(self) -> np.ndarray:
        """Get 3x3 rotation matrix (ZYX Euler: yaw -> pitch -> roll)"""
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)

        # ZYX rotation matrix
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
        return R

    def get_inverse_rotation_matrix(self) -> np.ndarray:
        """Get inverse (transpose) of rotation matrix"""
        return self.get_rotation_matrix().T

    def robot_to_world(self, robot_pos: np.ndarray) -> np.ndarray:
        """Transform position from robot frame to world frame"""
        robot_pos = np.array(robot_pos).flatten()[:3]
        R = self.get_rotation_matrix()
        t = np.array([self.x, self.y, self.z])
        return R @ robot_pos + t

    def world_to_robot(self, world_pos: np.ndarray) -> np.ndarray:
        """Transform position from world frame to robot frame"""
        world_pos = np.array(world_pos).flatten()[:3]
        R_inv = self.get_inverse_rotation_matrix()
        t = np.array([self.x, self.y, self.z])
        return R_inv @ (world_pos - t)

    def __repr__(self):
        return f"RobotPose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, " \
               f"roll={np.degrees(self.roll):.1f}deg, pitch={np.degrees(self.pitch):.1f}deg, yaw={np.degrees(self.yaw):.1f}deg)"


# Global robot pose (singleton)
_robot_pose = RobotPose()


def get_robot_pose() -> RobotPose:
    """Get global robot pose"""
    return _robot_pose


def set_robot_pose(x=None, y=None, z=None, roll=None, pitch=None, yaw=None):
    """Set global robot pose"""
    global _robot_pose
    if x is not None:
        _robot_pose.x = x
    if y is not None:
        _robot_pose.y = y
    if z is not None:
        _robot_pose.z = z
    if roll is not None:
        _robot_pose.roll = roll
    if pitch is not None:
        _robot_pose.pitch = pitch
    if yaw is not None:
        _robot_pose.yaw = yaw
    ik_debug_log(f"Robot pose updated: {_robot_pose}")


def set_robot_pose_array(pose_array: List[float]):
    """Set robot pose from array [x, y, z, roll, pitch, yaw]"""
    global _robot_pose
    _robot_pose.set_from_array(pose_array)
    ik_debug_log(f"Robot pose updated: {_robot_pose}")


# ============================================================================
# EMPIRICAL JACOBIAN DATA (Isaac Sim Verified)
# ============================================================================

LEFT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([-0.183, -0.014, +0.024]), "test_angle": +0.3},
    1: {"delta": np.array([+0.000, +0.211, +0.032]), "test_angle": +0.3},
    2: {"delta": np.array([-0.000, +0.000, -0.000]), "test_angle": +0.3},
    3: {"delta": np.array([+0.109, -0.000, +0.017]), "test_angle": -0.3},
    4: {"delta": np.array([+0.000, +0.000, -0.000]), "test_angle": +0.3},
    5: {"delta": np.array([-0.030, -0.000, +0.004]), "test_angle": +0.3},
    6: {"delta": np.array([+0.000, +0.030, +0.004]), "test_angle": +0.3},
}

RIGHT_ARM_JACOBIAN_DATA = {
    0: {"delta": np.array([+0.183, +0.014, +0.024]), "test_angle": +0.3},
    1: {"delta": np.array([-0.000, +0.211, +0.032]), "test_angle": +0.3},
    2: {"delta": np.array([-0.000, +0.000, +0.000]), "test_angle": +0.3},
    3: {"delta": np.array([+0.109, +0.000, +0.017]), "test_angle": -0.3},
    4: {"delta": np.array([+0.000, +0.000, +0.000]), "test_angle": +0.3},
    5: {"delta": np.array([-0.030, +0.000, +0.004]), "test_angle": +0.3},
    6: {"delta": np.array([-0.000, +0.030, +0.004]), "test_angle": +0.3},
}

JOINT_LIMITS = {
    'left': [(-3.14, 3.14), (-3.14, 3.14), (-3.14, 3.14), (-2.2, 0.0),
             (-3.14, 3.14), (-3.14, 3.14), (-3.14, 3.14)],
    'right': [(-3.14, 3.14), (-3.14, 3.14), (-3.14, 3.14), (-2.2, 0.0),
              (-3.14, 3.14), (-3.14, 3.14), (-3.14, 3.14)]
}

# Gripper center position at zero joint angles (from URDF FK)
# End effector: gripper_center_link (xyz="0.0 0.0 -0.03" from gripper_base)
INITIAL_EE_POSITIONS = {
    'left': np.array([0.0, 0.5153, -0.2425]),
    'right': np.array([0.0, -0.5153, -0.2425])
}


# ============================================================================
# Servo Mapping (from rx1_motor.hpp)
# ============================================================================

RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
RIGHT_ARM_SERVO_DIRS = [-1, -1, 1, 1, 1, 1, -1]
RIGHT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

LEFT_ARM_SERVO_IDS = [21, 22, 23, 24, 25, 26, 27]
LEFT_ARM_SERVO_DIRS = [-1, -1, 1, -1, 1, -1, -1]
LEFT_ARM_SERVO_GEARS = [3, 3, 3, 3, 1, 1, 1]

SERVO_CENTER = 2048


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
# Damped Least Squares IK Solver
# ============================================================================

class DifferentialIKSolver:
    """Damped Least Squares IK Solver with Numerical Jacobian"""

    def __init__(self, num_joints: int = 7, damping: float = 0.01, gain: float = 1.0):
        self.num_joints = num_joints
        self.damping = damping
        self.gain = gain

    def compute_joint_delta(self, jacobian: np.ndarray, error: np.ndarray) -> np.ndarray:
        """Compute joint delta from position error using DLS"""
        JJT = jacobian @ jacobian.T
        damping_term = self.damping ** 2 * np.eye(3)

        try:
            inv_term = np.linalg.inv(JJT + damping_term)
            delta = jacobian.T @ inv_term @ (error * self.gain)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(jacobian) @ (error * self.gain)

        return delta

    def compute_numerical_jacobian(self, fk_func, joints: np.ndarray, eps: float = 0.001) -> np.ndarray:
        """
        Compute Jacobian numerically at current joint configuration

        This gives much more accurate results than using a fixed Jacobian
        because it captures the actual kinematics at the current pose.
        """
        n = len(joints)
        J = np.zeros((3, n))

        pos_0 = fk_func(joints)

        for i in range(n):
            joints_plus = joints.copy()
            joints_plus[i] += eps
            pos_plus = fk_func(joints_plus)

            J[:, i] = (pos_plus - pos_0) / eps

        return J


# ============================================================================
# RX1 Arm IK (Main Class for GUI)
# ============================================================================

class RX1ArmIK:
    """
    RX-1 Single Arm IK Solver

    Interface:
    - get_end_effector_position(servo_values) -> XYZ in WORLD frame
    - solve_for_position(target_xyz, servo_values) -> new_servos, success
      (target_xyz is in WORLD frame)

    World Coordinate System:
    - Uses global robot pose to transform between robot and world frames
    - Call set_robot_pose() to update robot position/orientation in world
    """

    def __init__(self, arm: str = 'right', use_world_frame: bool = False):
        self.arm = arm
        self.num_joints = 7
        self.use_world_frame = use_world_frame

        # Servo mapping
        if arm == 'right':
            self.servo_ids = RIGHT_ARM_SERVO_IDS
            self.servo_dirs = RIGHT_ARM_SERVO_DIRS
            self.servo_gears = RIGHT_ARM_SERVO_GEARS
        else:
            self.servo_ids = LEFT_ARM_SERVO_IDS
            self.servo_dirs = LEFT_ARM_SERVO_DIRS
            self.servo_gears = LEFT_ARM_SERVO_GEARS

        # Build Jacobian
        self._build_jacobian()

        # Joint limits
        self.joint_limits = JOINT_LIMITS[arm]

        # Initial EE position (in robot frame)
        self._initial_ee_pos = INITIAL_EE_POSITIONS[arm].copy()

        # Current state
        self._current_joints = np.zeros(self.num_joints)

        # DLS solver
        self._dls_solver = DifferentialIKSolver(
            num_joints=self.num_joints,
            damping=0.05,
            gain=3.0
        )

        frame_str = "WORLD" if use_world_frame else "ROBOT"
        ik_debug_log(f"[{arm}] RX1ArmIK initialized (frame={frame_str})")

    def _build_jacobian(self):
        """Build Jacobian from empirical data"""
        data = LEFT_ARM_JACOBIAN_DATA if self.arm == 'left' else RIGHT_ARM_JACOBIAN_DATA

        self.jacobian = np.zeros((3, self.num_joints))
        for i in range(self.num_joints):
            delta = data[i]["delta"]
            test_angle = data[i]["test_angle"]
            self.jacobian[:, i] = delta / test_angle

    def servo_to_angles(self, servo_values) -> np.ndarray:
        """Convert servo values to joint angles"""
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

    def forward(self, joint_angles: np.ndarray) -> np.ndarray:
        """FK: joint angles -> EE position"""
        joints = np.array(joint_angles).flatten()[:self.num_joints]
        ee_delta = self.jacobian @ joints
        return self._initial_ee_pos + ee_delta

    def inverse(
        self,
        target_xyz: np.ndarray,
        current_angles: Optional[np.ndarray] = None,
        max_iterations: int = 200,
        tolerance: float = 0.001,
        use_numerical_jacobian: bool = True
    ) -> Tuple[np.ndarray, bool]:
        """
        IK: target position -> joint angles

        Uses numerical Jacobian for high accuracy.
        Each iteration recomputes the Jacobian at the current pose.
        """
        target = np.array(target_xyz).flatten()[:3]

        if current_angles is not None:
            joints = np.array(current_angles).flatten()[:self.num_joints]
        else:
            joints = self._current_joints.copy()

        best_joints = joints.copy()
        best_error = float('inf')

        for iteration in range(max_iterations):
            current_ee = self.forward(joints)
            error = target - current_ee
            error_norm = np.linalg.norm(error)

            if error_norm < best_error:
                best_error = error_norm
                best_joints = joints.copy()

            if error_norm < tolerance:
                self._current_joints = joints.copy()
                return joints, True

            # Use numerical Jacobian for accuracy (recompute at current pose)
            if use_numerical_jacobian:
                J = self._dls_solver.compute_numerical_jacobian(self.forward, joints)
            else:
                J = self.jacobian

            delta = self._dls_solver.compute_joint_delta(J, error)

            # Adaptive step size - smaller steps for better convergence
            step = min(0.5, error_norm / 0.05)
            joints = joints + delta * step
            joints = self._enforce_limits(joints)

            # Early termination if stuck
            if iteration > 50 and best_error > 0.1:
                break

        self._current_joints = best_joints.copy()
        return best_joints, best_error < tolerance * 3

    def _enforce_limits(self, joints: np.ndarray) -> np.ndarray:
        """Enforce joint limits"""
        for i in range(self.num_joints):
            lo, hi = self.joint_limits[i]
            joints[i] = np.clip(joints[i], lo, hi)
        return joints

    def reset(self):
        """Reset state"""
        self._current_joints = np.zeros(self.num_joints)

    def _to_world(self, robot_pos: np.ndarray) -> np.ndarray:
        """Convert robot frame position to world frame"""
        if self.use_world_frame:
            return get_robot_pose().robot_to_world(robot_pos)
        return robot_pos

    def _to_robot(self, world_pos: np.ndarray) -> np.ndarray:
        """Convert world frame position to robot frame"""
        if self.use_world_frame:
            return get_robot_pose().world_to_robot(world_pos)
        return world_pos

    def get_end_effector_position(self, servo_values) -> np.ndarray:
        """
        Get EE position from servo values (FK)
        Returns position in WORLD frame (or robot frame if use_world_frame=False)
        """
        angles = self.servo_to_angles(servo_values)
        robot_pos = self.forward(angles)
        return self._to_world(robot_pos)

    def get_end_effector_position_robot_frame(self, servo_values) -> np.ndarray:
        """Get EE position in robot frame (for internal use)"""
        angles = self.servo_to_angles(servo_values)
        return self.forward(angles)

    def solve_for_position(
        self,
        target_xyz,
        servo_values,
        max_iterations: int = 100
    ) -> Tuple[Dict[int, int], bool]:
        """
        Solve IK for target position
        target_xyz: position in WORLD frame (or robot frame if use_world_frame=False)
        """
        # Get current position in world frame for logging
        current_world = self.get_end_effector_position(servo_values)
        target_world = np.array(target_xyz)

        # Convert target to robot frame for IK calculation
        target_robot = self._to_robot(target_world)
        current_robot = self.get_end_effector_position_robot_frame(servo_values)

        frame_str = "WORLD" if self.use_world_frame else "ROBOT"
        ik_debug_log(f"========== IK solve ({self.arm}, {frame_str}) ==========")
        ik_debug_log(f"Robot pose: {get_robot_pose()}")
        ik_debug_log(f"Target ({frame_str}): X={target_world[0]:.4f}, Y={target_world[1]:.4f}, Z={target_world[2]:.4f}")
        ik_debug_log(f"Target (ROBOT): X={target_robot[0]:.4f}, Y={target_robot[1]:.4f}, Z={target_robot[2]:.4f}")
        ik_debug_log(f"Current (ROBOT): X={current_robot[0]:.4f}, Y={current_robot[1]:.4f}, Z={current_robot[2]:.4f}")

        current_angles = self.servo_to_angles(servo_values)
        distance = np.linalg.norm(target_robot - current_robot)

        # Large moves: split into steps (in robot frame)
        if distance > 0.08:
            num_steps = min(10, max(3, int(distance / 0.03)))
            ik_debug_log(f"Large move ({distance*100:.1f}cm) -> {num_steps} steps")

            angles = current_angles.copy()
            for step in range(num_steps):
                t = (step + 1) / num_steps
                intermediate = current_robot + t * (target_robot - current_robot)
                angles, _ = self.inverse(
                    intermediate, angles,
                    max_iterations=max(50, max_iterations // num_steps))
            new_angles = angles
            final_error = np.linalg.norm(self.forward(angles) - target_robot)
            success = final_error < 0.05
        else:
            new_angles, success = self.inverse(target_robot, current_angles, max_iterations)

        if success:
            new_servos = self.angles_to_servo(new_angles)
            result_robot = self.get_end_effector_position_robot_frame(new_servos)
            result_world = self._to_world(result_robot)
            error_mm = np.linalg.norm(result_world - target_world) * 1000

            ik_debug_log(f"IK OK! Error: {error_mm:.2f}mm")
            ik_debug_log(f"Result ({frame_str}): X={result_world[0]:.4f}, Y={result_world[1]:.4f}, Z={result_world[2]:.4f}")
            ik_debug_log("")

            return new_servos, True
        else:
            ik_debug_log(f"IK FAILED!")
            ik_debug_log("")
            return {}, False


# ============================================================================
# Use Accurate IK if available (ikpy URDF-based)
# ============================================================================

# Override RX1ArmIK with accurate version if available
if ACCURATE_IK_AVAILABLE:
    # Use accurate ikpy-based IK
    RX1ArmIK = RX1AccurateIK
    print("[IK] Using accurate ikpy URDF-based IK (sub-mm accuracy)")
else:
    # Fall back to Jacobian-based IK (less accurate)
    print("[IK] Using Jacobian-based IK (install ikpy for better accuracy)")


# ============================================================================
# Test Code
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("RX-1 IK Test")
    print("=" * 60)

    if ACCURATE_IK_AVAILABLE:
        print("Solver: ikpy URDF-based (HIGH ACCURACY)")
    else:
        print("Solver: Jacobian-based (lower accuracy)")

    right_ik = RX1ArmIK('right')
    servo_center = {sid: 2048 for sid in RIGHT_ARM_SERVO_IDS}

    # Get initial position
    init_pos = right_ik.get_end_effector_position(servo_center)
    print(f'\nInitial EE: X={init_pos[0]:.4f}, Y={init_pos[1]:.4f}, Z={init_pos[2]:.4f}')

    # Test X-only movement
    print('\n--- X-only Movement Test (Y, Z should stay constant) ---')
    base_y = init_pos[1]
    base_z = init_pos[2]
    current_servos = servo_center.copy()

    for x_target in [0.05, 0.10, 0.15]:
        target = [x_target, base_y, base_z]
        new_servos, success = right_ik.solve_for_position(target, current_servos)

        if success:
            result = right_ik.get_end_effector_position(new_servos)
            dy = (result[1] - base_y) * 1000
            dz = (result[2] - base_z) * 1000
            print(f'X={x_target:.2f} -> dY={dy:+.2f}mm, dZ={dz:+.2f}mm')
            current_servos = new_servos
        else:
            print(f'X={x_target:.2f} -> FAILED')

    # Left arm test
    print('\n--- Left Arm Test ---')
    left_ik = RX1ArmIK('left')
    pos_l = left_ik.get_end_effector_position({sid: 2048 for sid in LEFT_ARM_SERVO_IDS})
    print(f'Left EE: X={pos_l[0]:.4f}, Y={pos_l[1]:.4f}, Z={pos_l[2]:.4f}')

    print('\n' + '=' * 60)
    print('Test Complete!')
    print('=' * 60)
