#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import time
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rx1_ik_custom import RX1CustomIK, RIGHT_ARM_SERVO_IDS
import json
import traceback

rclpy.init()

# All non-fixed joints in URDF (must publish all for TF tree to be connected)
ALL_JOINTS = [
    # CRITICAL: base_to_lift_joint connects base_link to lift_platform
    'base_to_lift_joint',
    # Right arm
    'right_shoul_base2shoul_joint[11]','right_shoul2shoul_rot_joint[12]',
    'right_arm2armrot_joint[13]','right_armrot2elbow_joint[14]',
    'right_forearm2forearmrot_joint[15]','right_forearmrot2forearm_pitch_joint[16]',
    'right_forearm_pitch2forearm_roll_joint[17]',
    # Dummy joints
    'dummy_joint[18]','dummy_joint[19]','dummy_joint[20]','dummy_joint[21]',
    'dummy_joint[22]','dummy_joint[23]','dummy_joint[24]','dummy_joint[25]',
    'dummy_joint[26]','dummy_joint[27]','dummy_joint[28]','dummy_joint[29]',
    'dummy_joint[30]','dummy_joint_30a','dummy_joint_30b','dummy_joint_30c',
    'dummy_joint_30d','dummy_joint_30e','dummy_joint_30f','dummy_joint_30g',
    'dummy_joint_30h','dummy_joint_30i','dummy_joint_30j',
    # Right gripper
    'right_gripper_joint[41]', 'right_gripper_tip2_joint[32]',
    # Left arm
    'left_shoul_base2shoul_joint[21]','left_shoul2shoul_rot_joint[22]',
    'left_arm2armrot_joint[23]','left_armrot2elbow_joint[24]',
    'left_forearm2forearmrot_joint[25]','left_forearmrot2forearm_pitch_joint[26]',
    'left_forearm_pitch2forearm_roll_joint[27]',
    # Left gripper
    'dummy_joint_40', 'left_gripper_joint[31]', 'left_gripper_tip2_joint[28]',
    # Other
    'base_to_nema42_joint','base_to_xp1000_joint'
]

class T(Node):
    def __init__(self):
        super().__init__('ik_ros_test')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.ik = RX1CustomIK('right')
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rx1_baseline_v5.json')) as f:
            d = json.load(f)
        self.bs = {sid: d['servos'][str(sid)]['position'] for sid in RIGHT_ARM_SERVO_IDS}

    def pub_sv(self, sv):
        ang = [float(self.ik.servo_to_angle(sv[sid], i)) for i, sid in enumerate(RIGHT_ARM_SERVO_IDS)]
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ALL_JOINTS
        # base_to_lift_joint = 0.0 (lift at home position), then 7 arm angles, then zeros for rest
        pos = [0.0] + ang + [0.0]*(len(ALL_JOINTS)-8)
        m.position = pos
        self.pub.publish(m)

    def tf(self):
        try:
            t = self.buf.lookup_transform('world','right_gripper_center_link',rclpy.time.Time())
            return [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        except Exception as e:
            print(f'   TF error: {e}')
            return None

n = T()

# Baseline
print('=== ROS IK Test ===')
print('1. Baseline')
fk1 = n.ik.get_end_effector_position(n.bs)
print(f'   FK: X={fk1[0]*100:.2f}, Y={fk1[1]*100:.2f}, Z={fk1[2]*100:.2f} cm')

for _ in range(50):
    n.pub_sv(n.bs)
    rclpy.spin_once(n, timeout_sec=0.1)
time.sleep(1)

tf1 = n.tf()
if tf1:
    print(f'   TF: X={tf1[0]*100:.2f}, Y={tf1[1]*100:.2f}, Z={tf1[2]*100:.2f} cm')

# IK X+5cm
print('2. IK X+5cm')
target = [fk1[0]+0.05, fk1[1], fk1[2]]
sv2, ok = n.ik.solve_for_position(target, n.bs)
if ok:
    fk2 = n.ik.get_end_effector_position(sv2)
    print(f'   FK: X={fk2[0]*100:.2f}, Y={fk2[1]*100:.2f}, Z={fk2[2]*100:.2f} cm')
    print(f'   FK dX={((fk2[0]-fk1[0])*100):+.2f}, dY={((fk2[1]-fk1[1])*100):+.2f}, dZ={((fk2[2]-fk1[2])*100):+.2f} cm')

    for _ in range(50):
        n.pub_sv(sv2)
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1)

    tf2 = n.tf()
    if tf2 and tf1:
        print(f'   TF: X={tf2[0]*100:.2f}, Y={tf2[1]*100:.2f}, Z={tf2[2]*100:.2f} cm')
        print(f'   TF dX={((tf2[0]-tf1[0])*100):+.2f}, dY={((tf2[1]-tf1[1])*100):+.2f}, dZ={((tf2[2]-tf1[2])*100):+.2f} cm')

n.destroy_node()
rclpy.shutdown()
print('Done')
