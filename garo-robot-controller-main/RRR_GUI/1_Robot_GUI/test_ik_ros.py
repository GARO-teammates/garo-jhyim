#!/usr/bin/env python3
"""ROS2 IK Test - publish joint_states and verify movement"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import numpy as np
import json
import time
import math

from rx1_ik_custom import RX1CustomIK, RIGHT_ARM_SERVO_IDS

class IKTestNode(Node):
    def __init__(self):
        super().__init__('ik_test_node')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik = RX1CustomIK('right')

        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rx1_baseline_v5.json')) as f:
            baseline_data = json.load(f)

        self.baseline_servos = {sid: baseline_data['servos'][str(sid)]['position']
                                for sid in RIGHT_ARM_SERVO_IDS}

        self.joint_names = [
            'right_shoul_base2shoul_joint[11]',
            'right_shoul2shoul_rot_joint[12]',
            'right_arm2armrot_joint[13]',
            'right_armrot2elbow_joint[14]',
            'right_forearm2forearmrot_joint[15]',
            'right_forearmrot2forearm_pitch_joint[16]',
            'right_forearm_pitch2forearm_roll_joint[17]',
            'left_shoul_base2shoul_joint[21]',
            'left_shoul2shoul_rot_joint[22]',
            'left_arm2armrot_joint[23]',
            'left_armrot2elbow_joint[24]',
            'left_forearm2forearmrot_joint[25]',
            'left_forearmrot2forearm_pitch_joint[26]',
            'left_forearm_pitch2forearm_roll_joint[27]',
        ]

        self.get_logger().info("IK Test Node started")

    def servo_to_angle(self, servo_val, idx):
        """서보값 -> 관절 각도 (GUI 방식)"""
        NO_GEARBOX = [15, 25]
        sid = RIGHT_ARM_SERVO_IDS[idx]
        relative = servo_val - 2048

        if sid in NO_GEARBOX:
            deg = (relative / 4095) * 360
        else:
            deg = (relative / 4095) * 120

        return math.radians(deg)

    def publish_joint_states(self, right_servos, description=""):
        """Joint states 발행"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # 오른팔 관절 각도
        right_angles = []
        for i, sid in enumerate(RIGHT_ARM_SERVO_IDS):
            angle = self.servo_to_angle(right_servos[sid], i)
            right_angles.append(angle)

        # 왼팔은 baseline 유지 (0)
        left_angles = [0.0] * 7

        msg.position = right_angles + left_angles
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

        if description:
            self.get_logger().info(f"{description}")
            self.get_logger().info(f"  서보: {[right_servos[sid] for sid in RIGHT_ARM_SERVO_IDS]}")

    def test_ik_movement(self):
        """IK 이동 테스트"""
        # 1. Baseline 위치 확인
        baseline_pos = self.ik.get_end_effector_position(self.baseline_servos)
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Baseline EE: X={baseline_pos[0]*100:.2f}cm, Y={baseline_pos[1]*100:.2f}cm, Z={baseline_pos[2]*100:.2f}cm")

        # Baseline 발행
        self.publish_joint_states(self.baseline_servos, "1. Baseline 위치")
        time.sleep(2)

        # 2. X +5cm 이동
        target_x5 = [baseline_pos[0] + 0.05, baseline_pos[1], baseline_pos[2]]
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"목표: X +5cm → X={target_x5[0]*100:.2f}cm")

        servos_x5, success = self.ik.solve_for_position(target_x5, self.baseline_servos)

        if success:
            result_pos = self.ik.get_end_effector_position(servos_x5)
            self.get_logger().info(f"IK 결과: X={result_pos[0]*100:.2f}cm, Y={result_pos[1]*100:.2f}cm, Z={result_pos[2]*100:.2f}cm")

            # 서보 변화 출력
            self.get_logger().info("서보 변화:")
            for sid in RIGHT_ARM_SERVO_IDS:
                delta = servos_x5[sid] - self.baseline_servos[sid]
                self.get_logger().info(f"  Servo {sid}: {self.baseline_servos[sid]} → {servos_x5[sid]} (Δ={delta:+d})")

            self.publish_joint_states(servos_x5, "2. X +5cm 이동")
            time.sleep(3)
        else:
            self.get_logger().error("X +5cm IK 실패!")

        # 3. X +10cm 이동
        self.ik.reset()
        target_x10 = [baseline_pos[0] + 0.10, baseline_pos[1], baseline_pos[2]]
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"목표: X +10cm → X={target_x10[0]*100:.2f}cm")

        servos_x10, success = self.ik.solve_for_position(target_x10, self.baseline_servos)

        if success:
            result_pos = self.ik.get_end_effector_position(servos_x10)
            self.get_logger().info(f"IK 결과: X={result_pos[0]*100:.2f}cm, Y={result_pos[1]*100:.2f}cm, Z={result_pos[2]*100:.2f}cm")

            self.publish_joint_states(servos_x10, "3. X +10cm 이동")
            time.sleep(3)
        else:
            self.get_logger().error("X +10cm IK 실패!")

        # 4. Y +5cm 이동
        self.ik.reset()
        target_y5 = [baseline_pos[0], baseline_pos[1] + 0.05, baseline_pos[2]]
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"목표: Y +5cm → Y={target_y5[1]*100:.2f}cm")

        servos_y5, success = self.ik.solve_for_position(target_y5, self.baseline_servos)

        if success:
            result_pos = self.ik.get_end_effector_position(servos_y5)
            self.get_logger().info(f"IK 결과: X={result_pos[0]*100:.2f}cm, Y={result_pos[1]*100:.2f}cm, Z={result_pos[2]*100:.2f}cm")

            self.publish_joint_states(servos_y5, "4. Y +5cm 이동")
            time.sleep(3)
        else:
            self.get_logger().error("Y +5cm IK 실패!")

        # 5. 다시 Baseline
        self.get_logger().info("=" * 60)
        self.publish_joint_states(self.baseline_servos, "5. Baseline 복귀")

        self.get_logger().info("=" * 60)
        self.get_logger().info("테스트 완료! RViz에서 움직임 확인하세요.")


def main():
    rclpy.init()
    node = IKTestNode()

    try:
        node.test_ik_movement()

        # 계속 publish하면서 대기
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
