#!/usr/bin/env python3
"""Direct TF2 lookup via Python"""
import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState
import math
import json
import time

class TFLookup(Node):
    def __init__(self):
        super().__init__('tf_lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish_joints(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
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
        msg.position = list(angles) + [0.0] * 7
        self.joint_pub.publish(msg)

    def lookup(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return t.transform.translation
        except Exception as e:
            return None

def main():
    rclpy.init()
    node = TFLookup()

    # Load baseline
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rx1_baseline_v5.json')) as f:
        baseline = json.load(f)

    NO_GEARBOX = [15, 25]
    angles = []
    for sid in [11, 12, 13, 14, 15, 16, 17]:
        pos = baseline['servos'][str(sid)]['position']
        rel = pos - 2048
        if sid in NO_GEARBOX:
            deg = (rel / 4095) * 360
        else:
            deg = (rel / 4095) * 120
        angles.append(math.radians(deg))

    print(f"Joint angles (rad): {[f'{a:.4f}' for a in angles]}")

    # Publish for a while
    for i in range(30):
        node.publish_joints(angles)
        rclpy.spin_once(node, timeout_sec=0.1)

    # Lookup TF
    print("\n=== TF Lookup ===")
    for child in ['right_shoul_base_link', 'right_wrist_link', 'right_gripper_center_link']:
        tf = node.lookup('base_link', child)
        if tf:
            print(f"base_link -> {child}:")
            print(f"  X={tf.x*100:.2f}cm, Y={tf.y*100:.2f}cm, Z={tf.z*100:.2f}cm")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
