#!/usr/bin/env python3
"""
RX-1 IK ROS2 Node
=================
Based on GitHub rx1_ik + Isaac Sim verified empirical Jacobian data

Features:
- ROS2 topic interface for gripper pose control
- Joint state publishing
- Interactive marker support
- TF2 integration for FK visualization
- WORLD COORDINATE SYSTEM support

World Coordinate System:
- All target positions are in WORLD frame
- Robot pose (x, y, z, roll, pitch, yaw) defines robot in world
- Call set_robot_pose() to update robot position

Original C++ rx1_ik: https://github.com/Red-Rabbit-Robotics/rx1/tree/master/rx1_ik
Isaac Sim data: /home/rl02/Downloads/ik/rx1_ik_controller.py
"""

import numpy as np
import math
from typing import Dict, List, Optional, Tuple

# Try ROS2 import
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
    from interactive_markers import InteractiveMarkerServer
    from tf2_ros import TransformBroadcaster, Buffer, TransformListener
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[RX1 IK] ROS2 not available - running in standalone mode")


# ============================================================================
# EMPIRICAL JACOBIAN DATA (Verified in Isaac Sim)
# ============================================================================
# Measured by moving each joint +/- 0.3 rad and observing EE position change
# Source: /home/rl02/Downloads/ik/rx1_ik_controller.py

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

# Joint limits from URDF
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

# Joint names (from GitHub rx1_ik)
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


# ============================================================================
# Damped Least Squares IK Solver
# ============================================================================

class DLSIKSolver:
    """
    Damped Least Squares (Levenberg-Marquardt) IK Solver

    Formula: dq = J^T (J J^T + lambda^2 * I)^(-1) * e
    """

    def __init__(self, damping: float = 0.05, gain: float = 3.0):
        self.damping = damping
        self.gain = gain

    def solve(self, jacobian: np.ndarray, error: np.ndarray) -> np.ndarray:
        """Compute joint delta from position error"""
        JJT = jacobian @ jacobian.T
        damping_term = self.damping ** 2 * np.eye(3)

        try:
            inv_term = np.linalg.inv(JJT + damping_term)
            delta = jacobian.T @ inv_term @ (error * self.gain)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(jacobian) @ (error * self.gain)

        return delta


# ============================================================================
# RX1 Arm IK Controller
# ============================================================================

class RX1ArmIKController:
    """
    Single arm IK controller using empirical Jacobian from Isaac Sim
    """

    def __init__(self, arm: str = 'right'):
        self.arm = arm
        self.num_joints = 7

        # Select arm-specific data
        if arm == 'right':
            self.jacobian_data = RIGHT_ARM_JACOBIAN_DATA
            self.joint_names = RIGHT_ARM_JOINTS
        else:
            self.jacobian_data = LEFT_ARM_JACOBIAN_DATA
            self.joint_names = LEFT_ARM_JOINTS

        # Build Jacobian matrix
        self.jacobian = self._build_jacobian()

        # Joint limits and initial position
        self.joint_limits = JOINT_LIMITS[arm]
        self.initial_ee_pos = INITIAL_EE_POSITIONS[arm].copy()

        # Current joint state
        self.current_joints = np.zeros(self.num_joints)
        self.current_joints[3] = -1.57  # Initial elbow bend (from rx1_ik.cpp)

        # Previous joint state for smoothing
        self.prev_joints = self.current_joints.copy()

        # IK solver
        self.ik_solver = DLSIKSolver(damping=0.05, gain=3.0)

        # Parameters from rx1_ik.cpp
        self.max_angle_change = 0.6  # rad
        self.smoothing_factor = 0.1  # 0.9/0.1 smoothing

    def _build_jacobian(self) -> np.ndarray:
        """Build Jacobian matrix from empirical data"""
        J = np.zeros((3, self.num_joints))
        for i in range(self.num_joints):
            delta = self.jacobian_data[i]["delta"]
            test_angle = self.jacobian_data[i]["test_angle"]
            J[:, i] = delta / test_angle
        return J

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """FK: joint angles -> EE position (linear approximation)"""
        joints = np.array(joint_angles).flatten()[:self.num_joints]
        ee_delta = self.jacobian @ joints
        return self.initial_ee_pos + ee_delta

    def inverse_kinematics(
        self,
        target_pos: np.ndarray,
        current_angles: Optional[np.ndarray] = None,
        max_iter: int = 100,
        tolerance: float = 0.005
    ) -> Tuple[np.ndarray, bool]:
        """IK: target position -> joint angles"""
        target = np.array(target_pos).flatten()[:3]

        if current_angles is not None:
            joints = np.array(current_angles).flatten()[:self.num_joints]
        else:
            joints = self.current_joints.copy()

        best_joints = joints.copy()
        best_error = float('inf')

        for _ in range(max_iter):
            current_ee = self.forward_kinematics(joints)
            error = target - current_ee
            error_norm = np.linalg.norm(error)

            if error_norm < best_error:
                best_error = error_norm
                best_joints = joints.copy()

            if error_norm < tolerance:
                return joints, True

            # Compute joint delta
            delta = self.ik_solver.solve(self.jacobian, error)

            # Adaptive step size
            step = min(1.0, error_norm / 0.1)
            joints = joints + delta * step

            # Enforce limits
            joints = self._enforce_limits(joints)

        return best_joints, best_error < tolerance * 2

    def _enforce_limits(self, joints: np.ndarray) -> np.ndarray:
        """Enforce joint limits"""
        for i in range(self.num_joints):
            lo, hi = self.joint_limits[i]
            joints[i] = np.clip(joints[i], lo, hi)
        return joints

    def solve_with_smoothing(
        self,
        target_pos: np.ndarray,
        check_angle_change: bool = True
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve IK with smoothing (from rx1_ik.cpp)

        Applies:
        1. Max angle change check
        2. 0.9/0.1 smoothing filter
        """
        new_joints, success = self.inverse_kinematics(target_pos, self.current_joints)

        if not success:
            return self.current_joints, False

        # Check max angle change
        if check_angle_change:
            for i in range(self.num_joints):
                if abs(self.prev_joints[i] - new_joints[i]) > self.max_angle_change:
                    # Angle change too large - skip this solution
                    return self.current_joints, False

        # Apply smoothing: cur = cur * 0.9 + new * 0.1
        smoothed = self.current_joints * (1 - self.smoothing_factor) + new_joints * self.smoothing_factor

        self.prev_joints = self.current_joints.copy()
        self.current_joints = smoothed

        return smoothed, True

    def set_joint_positions(self, positions: np.ndarray):
        """Set current joint positions (from external source)"""
        self.current_joints = np.array(positions).flatten()[:self.num_joints]
        self.prev_joints = self.current_joints.copy()

    def get_ee_position(self) -> np.ndarray:
        """Get current EE position"""
        return self.forward_kinematics(self.current_joints)


# ============================================================================
# ROS2 IK Node
# ============================================================================

if ROS2_AVAILABLE:
    class RX1IKNode(Node):
        """
        ROS2 IK Node for RX-1 robot

        Based on rx1_ik.cpp from GitHub

        Topics:
        - /right_gripper_pose (Pose) - target pose for right arm
        - /left_gripper_pose (Pose) - target pose for left arm
        - /right_arm_joint_states (JointState) - output joint states
        - /left_arm_joint_states (JointState) - output joint states
        """

        def __init__(self):
            super().__init__('rx1_ik_node')

            # Parameters
            self.declare_parameter('chain_start', 'head_base_link')
            self.declare_parameter('chain_r_end', 'right_forearm_roll_link')
            self.declare_parameter('chain_l_end', 'left_forearm_roll_link')
            self.declare_parameter('max_angle_change', 0.6)
            self.declare_parameter('tracking_timeout', 1.0)
            self.declare_parameter('smoothing_factor', 0.1)

            # Get parameters
            self.chain_start = self.get_parameter('chain_start').value
            self.max_angle_change = self.get_parameter('max_angle_change').value
            self.tracking_timeout = self.get_parameter('tracking_timeout').value
            self.smoothing_factor = self.get_parameter('smoothing_factor').value

            # IK controllers
            self.right_ik = RX1ArmIKController('right')
            self.left_ik = RX1ArmIKController('left')

            # Apply parameters
            self.right_ik.max_angle_change = self.max_angle_change
            self.right_ik.smoothing_factor = self.smoothing_factor
            self.left_ik.max_angle_change = self.max_angle_change
            self.left_ik.smoothing_factor = self.smoothing_factor

            # Last IK time for tracking timeout
            self.right_last_ik_time = 0.0
            self.left_last_ik_time = 0.0

            # Publishers
            self.right_joint_pub = self.create_publisher(
                JointState, 'right_arm_joint_states', 10)
            self.left_joint_pub = self.create_publisher(
                JointState, 'left_arm_joint_states', 10)

            # Subscribers
            self.right_pose_sub = self.create_subscription(
                Pose, 'right_gripper_pose',
                self.right_pose_callback, 10)
            self.left_pose_sub = self.create_subscription(
                Pose, 'left_gripper_pose',
                self.left_pose_callback, 10)

            # TF broadcaster
            self.tf_broadcaster = TransformBroadcaster(self)

            # Update timer (20Hz like rx1_ik.cpp)
            self.timer = self.create_timer(0.05, self.update_callback)

            self.get_logger().info('RX1 IK Node started (Isaac Sim verified)')
            self.get_logger().info(f'  Chain start: {self.chain_start}')
            self.get_logger().info(f'  Max angle change: {self.max_angle_change}')

        def right_pose_callback(self, msg: Pose):
            """Handle right gripper target pose"""
            target = np.array([msg.position.x, msg.position.y, msg.position.z])

            current_time = self.get_clock().now().nanoseconds / 1e9
            within_timeout = (current_time - self.right_last_ik_time) < self.tracking_timeout

            new_joints, success = self.right_ik.solve_with_smoothing(
                target, check_angle_change=within_timeout)

            if success:
                self.right_last_ik_time = current_time
                self.get_logger().debug('Right IK success')
            else:
                self.get_logger().debug('Right IK failed or angle change too large')

        def left_pose_callback(self, msg: Pose):
            """Handle left gripper target pose"""
            target = np.array([msg.position.x, msg.position.y, msg.position.z])

            current_time = self.get_clock().now().nanoseconds / 1e9
            within_timeout = (current_time - self.left_last_ik_time) < self.tracking_timeout

            new_joints, success = self.left_ik.solve_with_smoothing(
                target, check_angle_change=within_timeout)

            if success:
                self.left_last_ik_time = current_time
                self.get_logger().debug('Left IK success')
            else:
                self.get_logger().debug('Left IK failed or angle change too large')

        def update_callback(self):
            """Periodic update - publish joint states"""
            now = self.get_clock().now().to_msg()

            # Right arm joint state
            right_msg = JointState()
            right_msg.header.stamp = now
            right_msg.name = self.right_ik.joint_names
            right_msg.position = list(self.right_ik.current_joints)
            self.right_joint_pub.publish(right_msg)

            # Left arm joint state
            left_msg = JointState()
            left_msg.header.stamp = now
            left_msg.name = self.left_ik.joint_names
            left_msg.position = list(self.left_ik.current_joints)
            self.left_joint_pub.publish(left_msg)

            # Publish base TF
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = 'map'
            tf.child_frame_id = 'base_link'
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)


# ============================================================================
# Standalone Interface (for GUI without ROS2)
# ============================================================================

# Import the working IK solver from rx1_ik_simple (with world coordinate support)
try:
    from rx1_ik_simple import (
        RX1ArmIK as SimpleArmIK,
        get_robot_pose, set_robot_pose, set_robot_pose_array, RobotPose
    )
    SIMPLE_IK_AVAILABLE = True
except ImportError:
    SIMPLE_IK_AVAILABLE = False


class RX1IKStandalone:
    """
    Standalone IK interface for GUI (no ROS2 required)

    Uses rx1_ik_simple.RX1ArmIK which is verified to work correctly.

    World Coordinate System:
    - All positions are in WORLD frame by default
    - Call set_robot_pose() to update robot position/orientation in world
    """

    def __init__(self, use_world_frame: bool = False):
        """
        Initialize IK interface

        Args:
            use_world_frame: If True, all positions are in world frame (default)
                           If False, positions are in robot frame
        """
        self.use_world_frame = use_world_frame

        if SIMPLE_IK_AVAILABLE:
            self.right_ik = SimpleArmIK('right', use_world_frame=use_world_frame)
            self.left_ik = SimpleArmIK('left', use_world_frame=use_world_frame)
        else:
            self.right_ik = RX1ArmIKController('right')
            self.left_ik = RX1ArmIKController('left')
            self.right_ik.current_joints = np.zeros(7)
            self.left_ik.current_joints = np.zeros(7)

        self._right_servos = {sid: 2048 for sid in [11, 12, 13, 14, 15, 16, 17]}
        self._left_servos = {sid: 2048 for sid in [21, 22, 23, 24, 25, 26, 27]}

    def set_robot_pose(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None):
        """
        Set robot position/orientation in world frame

        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
        """
        if SIMPLE_IK_AVAILABLE:
            set_robot_pose(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

    def set_robot_pose_array(self, pose: List[float]):
        """
        Set robot pose from array [x, y, z, roll, pitch, yaw]
        """
        if SIMPLE_IK_AVAILABLE:
            set_robot_pose_array(pose)

    def get_robot_pose(self):
        """Get current robot pose"""
        if SIMPLE_IK_AVAILABLE:
            return get_robot_pose()
        return None

    def solve_right(self, target_xyz: List[float]) -> Tuple[List[float], bool]:
        """
        Solve IK for right arm

        Args:
            target_xyz: Target position in WORLD frame (or robot frame if use_world_frame=False)

        Returns:
            (joint_angles, success)
        """
        if SIMPLE_IK_AVAILABLE:
            servos, success = self.right_ik.solve_for_position(target_xyz, self._right_servos)
            if success:
                self._right_servos = servos
                angles = self.right_ik.servo_to_angles(servos)
                return list(angles), True
            return list(np.zeros(7)), False
        else:
            joints, success = self.right_ik.inverse_kinematics(np.array(target_xyz))
            return list(joints), success

    def solve_left(self, target_xyz: List[float]) -> Tuple[List[float], bool]:
        """
        Solve IK for left arm

        Args:
            target_xyz: Target position in WORLD frame (or robot frame if use_world_frame=False)

        Returns:
            (joint_angles, success)
        """
        if SIMPLE_IK_AVAILABLE:
            servos, success = self.left_ik.solve_for_position(target_xyz, self._left_servos)
            if success:
                self._left_servos = servos
                angles = self.left_ik.servo_to_angles(servos)
                return list(angles), True
            return list(np.zeros(7)), False
        else:
            joints, success = self.left_ik.inverse_kinematics(np.array(target_xyz))
            return list(joints), success

    def get_right_ee(self) -> List[float]:
        """Get right arm EE position in WORLD frame"""
        if SIMPLE_IK_AVAILABLE:
            return list(self.right_ik.get_end_effector_position(self._right_servos))
        else:
            return list(self.right_ik.forward_kinematics(self.right_ik.current_joints))

    def get_left_ee(self) -> List[float]:
        """Get left arm EE position in WORLD frame"""
        if SIMPLE_IK_AVAILABLE:
            return list(self.left_ik.get_end_effector_position(self._left_servos))
        else:
            return list(self.left_ik.forward_kinematics(self.left_ik.current_joints))

    def get_right_ee_robot_frame(self) -> List[float]:
        """Get right arm EE position in robot frame"""
        if SIMPLE_IK_AVAILABLE:
            return list(self.right_ik.get_end_effector_position_robot_frame(self._right_servos))
        else:
            return list(self.right_ik.forward_kinematics(self.right_ik.current_joints))

    def get_left_ee_robot_frame(self) -> List[float]:
        """Get left arm EE position in robot frame"""
        if SIMPLE_IK_AVAILABLE:
            return list(self.left_ik.get_end_effector_position_robot_frame(self._left_servos))
        else:
            return list(self.left_ik.forward_kinematics(self.left_ik.current_joints))

    def set_right_joints(self, joints: List[float]):
        """Set right arm joint positions"""
        if SIMPLE_IK_AVAILABLE:
            angles = np.array(joints)[:7]
            self._right_servos = self.right_ik.angles_to_servo(angles)
        else:
            self.right_ik.current_joints = np.array(joints)[:7]

    def set_left_joints(self, joints: List[float]):
        """Set left arm joint positions"""
        if SIMPLE_IK_AVAILABLE:
            angles = np.array(joints)[:7]
            self._left_servos = self.left_ik.angles_to_servo(angles)
        else:
            self.left_ik.current_joints = np.array(joints)[:7]

    def reset(self):
        """Reset both arms to zero position"""
        if SIMPLE_IK_AVAILABLE:
            self._right_servos = {sid: 2048 for sid in [11, 12, 13, 14, 15, 16, 17]}
            self._left_servos = {sid: 2048 for sid in [21, 22, 23, 24, 25, 26, 27]}
            self.right_ik.reset()
            self.left_ik.reset()
        else:
            self.right_ik.current_joints = np.zeros(7)
            self.left_ik.current_joints = np.zeros(7)


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    """Main entry point for ROS2 node"""
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Use RX1IKStandalone for GUI mode.")
        return

    rclpy.init()
    node = RX1IKNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
