#!/bin/bash
source /opt/ros/humble/setup.bash

# simple_joint_state_publisher 종료
pkill -9 -f simple_joint_state 2>/dev/null
sleep 1

echo "=== Baseline TF (서보 ID 포함된 joint 이름 사용) ==="
# Baseline joint angles: [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232]
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{
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
  position: [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232, 0, 0, 0, 0, 0, 0, 0]
}" > /dev/null 2>&1 &
PUB_PID=$!

sleep 2
echo "Baseline joint angles: [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232]"
timeout 3 ros2 run tf2_ros tf2_echo base_link right_gripper_base_link 2>&1 | grep -E "Translation|At time" | head -2

kill $PUB_PID 2>/dev/null
sleep 1

echo ""
echo "=== IK X+5cm TF ==="
# IK 결과: [0.3872, -0.9262, -0.0527, -0.7032, 1.4653, 0.0511, -0.537]
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{
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
  position: [0.3872, -0.9262, -0.0527, -0.7032, 1.4653, 0.0511, -0.537, 0, 0, 0, 0, 0, 0, 0]
}" > /dev/null 2>&1 &
PUB_PID=$!

sleep 2
echo "IK joint angles: [0.3872, -0.9262, -0.0527, -0.7032, 1.4653, 0.0511, -0.537]"
timeout 3 ros2 run tf2_ros tf2_echo base_link right_gripper_base_link 2>&1 | grep -E "Translation|At time" | head -2

kill $PUB_PID 2>/dev/null
