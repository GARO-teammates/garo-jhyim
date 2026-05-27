#!/usr/bin/env python3
"""Verify FK against ROS TF for gripper_center_link"""
import os
import subprocess
import time
import math
import json
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_ROOT = os.path.dirname(_THIS_DIR)

RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]

with open(os.path.join(_THIS_DIR, 'rx1_baseline_v5.json')) as f:
    baseline = json.load(f)

servo_values = {}
for sid in RIGHT_ARM_SERVO_IDS:
    servo_values[sid] = baseline['servos'][str(sid)]['position']

print("=== Custom FK (gripper_center) ===")
from rx1_ik_custom import RX1CustomIK
ik = RX1CustomIK('right')

# Use class method which applies direction
pos = ik.get_end_effector_position(servo_values)
print(f"  X={pos[0]*100:.2f}cm, Y={pos[1]*100:.2f}cm, Z={pos[2]*100:.2f}cm")

# Get joint angles for ROS
angles = []
for i, sid in enumerate(RIGHT_ARM_SERVO_IDS):
    angles.append(ik.servo_to_angle(servo_values[sid], i))
print(f"  Joint angles: {[f'{math.degrees(a):.1f}' for a in angles]} deg")

print("\n=== ROS TF ===")
subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
time.sleep(1)

subprocess.Popen(
    "source /opt/ros/humble/setup.bash && "
    "ros2 launch " + os.path.join(_GUI_ROOT, "ros_files", "rx1_robot_only.launch.py"),
    shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
time.sleep(4)

subprocess.run("pkill -9 -f simple_joint_state 2>/dev/null", shell=True)
time.sleep(1)

angles_str = ', '.join([f'{a:.4f}' for a in angles])
joint_names = [
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
    'left_forearm_pitch2forearm_roll_joint[27]'
]

cmd = f"""source /opt/ros/humble/setup.bash && ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{{
  name: {joint_names},
  position: [{angles_str}, 0, 0, 0, 0, 0, 0, 0]
}}" """

subprocess.Popen(cmd, shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2)

result = subprocess.run(
    "source /opt/ros/humble/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo base_link right_gripper_center_link 2>&1 | grep Translation | head -1",
    shell=True, executable='/bin/bash', capture_output=True, text=True
)
print(result.stdout.strip() if result.stdout else "  (No output)")

subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
