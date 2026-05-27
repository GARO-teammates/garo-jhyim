#!/usr/bin/env python3
"""Compare FK at zero position (no joint rotation)"""
import os
import subprocess
import time
import numpy as np

print("=== Zero Position FK Comparison ===\n")

from rx1_ik_custom import RX1CustomIK
ik = RX1CustomIK('right')

# Zero angles
zero_angles = np.zeros(7)
pos = ik.forward_kinematics(zero_angles)
print(f"Custom FK (angles=0):")
print(f"  X={pos[0]*100:.2f}cm, Y={pos[1]*100:.2f}cm, Z={pos[2]*100:.2f}cm")

# ROS TF
print("\nStarting ROS...")
subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
time.sleep(1)

subprocess.Popen(
    "source /opt/ros/humble/setup.bash && "
    "ros2 launch " + os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros_files", "rx1_robot_only.launch.py"),
    shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
time.sleep(4)

subprocess.run("pkill -9 -f simple_joint_state 2>/dev/null", shell=True)
time.sleep(1)

# Publish zero angles
cmd = """source /opt/ros/humble/setup.bash && ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{
  name: [
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
  ],
  position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}" """

subprocess.Popen(cmd, shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

result = subprocess.run(
    "source /opt/ros/humble/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo base_link right_gripper_center_link 2>&1 | grep Translation | head -1",
    shell=True, executable='/bin/bash', capture_output=True, text=True
)
print(f"\nROS TF (angles=0):")
print(f"  {result.stdout.strip()}" if result.stdout else "  (No output)")

subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
