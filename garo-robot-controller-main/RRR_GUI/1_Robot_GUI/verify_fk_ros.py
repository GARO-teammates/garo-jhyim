#!/usr/bin/env python3
"""
Verify FK against ROS TF
"""
import os
import subprocess
import time
import math
import json
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_ROOT = os.path.dirname(_THIS_DIR)

# Baseline servo values
RIGHT_ARM_SERVO_IDS = [11, 12, 13, 14, 15, 16, 17]
NO_GEARBOX_SERVOS = [15, 25]
SERVO_CENTER = 2048

def servo_to_angle(servo_val, servo_id):
    relative = servo_val - SERVO_CENTER
    if servo_id in NO_GEARBOX_SERVOS:
        deg = (relative / 4095) * 360
    else:
        deg = (relative / 4095) * 120
    return math.radians(deg)

# Load baseline
with open(os.path.join(_THIS_DIR, 'rx1_baseline_v5.json')) as f:
    baseline = json.load(f)

print("=== Baseline Servo Values ===")
angles = []
for sid in RIGHT_ARM_SERVO_IDS:
    pos = baseline['servos'][str(sid)]['position']
    angle = servo_to_angle(pos, sid)
    print(f"  Servo {sid}: {pos} -> {math.degrees(angle):.2f} deg ({angle:.4f} rad)")
    angles.append(angle)

print("\n=== Custom FK Result ===")
from rx1_ik_custom import RX1CustomIK
ik = RX1CustomIK('right')
pos = ik.forward_kinematics(np.array(angles))
print(f"  Position: X={pos[0]*100:.2f}cm, Y={pos[1]*100:.2f}cm, Z={pos[2]*100:.2f}cm")

print("\n=== Starting ROS for TF comparison ===")

# Kill existing
subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
time.sleep(1)

# Start ROS
proc = subprocess.Popen(
    "source /opt/ros/humble/setup.bash && "
    "ros2 launch " + os.path.join(_GUI_ROOT, "ros_files", "rx1_robot_only.launch.py"),
    shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
time.sleep(4)

# Kill simple_joint_state_publisher
subprocess.run("pkill -9 -f simple_joint_state 2>/dev/null", shell=True)
time.sleep(1)

# Publish joint states
joint_names = [
    'right_shoul_base2shoul_joint[11]',
    'right_shoul2shoul_rot_joint[12]',
    'right_arm2armrot_joint[13]',
    'right_armrot2elbow_joint[14]',
    'right_forearm2forearmrot_joint[15]',
    'right_forearmrot2forearm_pitch_joint[16]',
    'right_forearm_pitch2forearm_roll_joint[17]',
]

angles_str = ', '.join([f'{a:.4f}' for a in angles])
cmd = f"""source /opt/ros/humble/setup.bash && ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{{
  name: {joint_names + ['left_shoul_base2shoul_joint[21]', 'left_shoul2shoul_rot_joint[22]', 'left_arm2armrot_joint[23]', 'left_armrot2elbow_joint[24]', 'left_forearm2forearmrot_joint[25]', 'left_forearmrot2forearm_pitch_joint[26]', 'left_forearm_pitch2forearm_roll_joint[27]']},
  position: [{angles_str}, 0, 0, 0, 0, 0, 0, 0]
}}" """

pub_proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2)

# Get TF
print("\n=== ROS TF Result ===")
result = subprocess.run(
    "source /opt/ros/humble/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo base_link right_wrist_link 2>&1 | grep -A1 Translation | head -2",
    shell=True, executable='/bin/bash', capture_output=True, text=True
)
print(result.stdout if result.stdout else "  (No TF output)")

# Cleanup
subprocess.run("pkill -9 -f 'ros2' 2>/dev/null", shell=True)
print("\n=== Done ===")
