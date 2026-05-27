"""
RX-1 Robot IK Chain Wrapper (Empirical Jacobian Version)

This file provides backward compatibility for any code that imports
from rx1_ik_chain. It wraps the empirical Jacobian IK solver.

For new code, use rx1_ik_solver.py or rx1_ik_simple.py directly.
"""

import numpy as np
from typing import Tuple, Optional
import os

# Import the empirical Jacobian solver
from rx1_ik_solver import (
    RX1ArmIK,
    RX1DualArmIK,
    LEFT_ARM_JACOBIAN_DATA,
    RIGHT_ARM_JACOBIAN_DATA,
    JOINT_LIMITS,
    INITIAL_EE_POSITIONS
)


class RX1IKSolver:
    """
    RX-1 Dual Arm IK Solver (Wrapper Class)

    This provides the same interface as the old ikpy-based solver
    but uses the empirical Jacobian approach internally.
    """

    def __init__(self, urdf_path=None):
        """
        Initialize IK solver.

        Args:
            urdf_path: Ignored (kept for backward compatibility)
        """
        # Create dual arm solver
        self._dual_ik = RX1DualArmIK()

        # Active joint indices (for compatibility)
        # These correspond to the 7 arm joints
        self.active_indices = [1, 2, 4, 5, 7, 8, 9]

        # Store URDF path for compatibility (not used)
        if urdf_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            urdf_path = os.path.join(base_dir, "urdf/RRR_Cater/RX1/combined/rx1_with_lidar.urdf")
        self.urdf_path = urdf_path

    def solve_ik(self, arm: str, target_position, target_orientation_rpy=None) -> Tuple[np.ndarray, bool]:
        """
        Inverse Kinematics.

        Args:
            arm: 'right' or 'left'
            target_position: [x, y, z] target position (meters)
            target_orientation_rpy: Ignored (position-only IK)

        Returns:
            joint_angles: 7 joint angles (radians)
            success: Whether IK converged
        """
        solver = self._dual_ik.right if arm == 'right' else self._dual_ik.left

        try:
            angles, success = solver.inverse(
                target_position,
                max_iterations=100,
                tolerance=0.005
            )
            return angles, success
        except Exception as e:
            print(f"IK solve error: {e}")
            return np.zeros(7), False

    def solve_fk(self, arm: str, joint_angles) -> Tuple[np.ndarray, np.ndarray]:
        """
        Forward Kinematics.

        Args:
            arm: 'right' or 'left'
            joint_angles: 7 joint angles (radians)

        Returns:
            position: [x, y, z] end effector position
            orientation_rpy: [0, 0, 0] (orientation not computed in empirical FK)
        """
        solver = self._dual_ik.right if arm == 'right' else self._dual_ik.left

        position = solver.forward(joint_angles)
        orientation_rpy = np.zeros(3)  # Not computed in empirical Jacobian approach

        return position, orientation_rpy

    def get_workspace_bounds(self, arm: str) -> dict:
        """Get approximate workspace bounds."""
        solver = self._dual_ik.right if arm == 'right' else self._dual_ik.left
        return solver.get_workspace_bounds()


# Test code
if __name__ == "__main__":
    print("=" * 60)
    print("RX-1 IK Chain Test (Empirical Jacobian Wrapper)")
    print("=" * 60)

    solver = RX1IKSolver()

    print(f"\nActive joint indices: {solver.active_indices}")

    # FK test (all joints = 0)
    print("\n--- FK Test (all joints = 0) ---")
    zero_angles = np.zeros(7)

    pos_r, ori_r = solver.solve_fk('right', zero_angles)
    print(f"Right arm position: [{pos_r[0]:.4f}, {pos_r[1]:.4f}, {pos_r[2]:.4f}]")

    pos_l, ori_l = solver.solve_fk('left', zero_angles)
    print(f"Left arm position: [{pos_l[0]:.4f}, {pos_l[1]:.4f}, {pos_l[2]:.4f}]")

    # IK test
    print("\n--- IK Test ---")
    target = [0.15, -0.45, 0.15]
    print(f"Target position: {target}")

    angles, success = solver.solve_ik('right', target)
    print(f"IK success: {success}")
    print(f"Joint angles (deg): {np.degrees(angles).round(1)}")

    # FK verification
    verify_pos, _ = solver.solve_fk('right', angles)
    error = np.linalg.norm(verify_pos - np.array(target)) * 1000
    print(f"FK verification: [{verify_pos[0]:.4f}, {verify_pos[1]:.4f}, {verify_pos[2]:.4f}]")
    print(f"Position error: {error:.2f} mm")

    print("\n" + "=" * 60)
    print("Test Complete!")
    print("=" * 60)
