#!/bin/bash
source /opt/ros/humble/setup.bash

echo "=== Baseline TF ==="
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{name: [right_shoul_base2shoul_joint, right_shoul2shoul_rot_joint, right_arm2armrot_joint, right_armrot2elbow_joint, right_forearm2forearmrot_joint, right_forearmrot2forearm_pitch_joint, right_forearm_pitch2forearm_roll_joint, left_shoul_base2shoul_joint, left_shoul2shoul_rot_joint, left_arm2armrot_joint, left_armrot2elbow_joint, left_forearm2forearmrot_joint, left_forearmrot2forearm_pitch_joint, left_forearm_pitch2forearm_roll_joint], position: [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232, 0, 0, 0, 0, 0, 0, 0]}" > /dev/null 2>&1 &
PUB_PID=$!

sleep 2
timeout 3 ros2 run tf2_ros tf2_echo base_link right_gripper_base_link 2>&1 | head -10

kill $PUB_PID 2>/dev/null
sleep 1

echo ""
echo "=== IK X+5cm TF ==="
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{name: [right_shoul_base2shoul_joint, right_shoul2shoul_rot_joint, right_arm2armrot_joint, right_armrot2elbow_joint, right_forearm2forearmrot_joint, right_forearmrot2forearm_pitch_joint, right_forearm_pitch2forearm_roll_joint, left_shoul_base2shoul_joint, left_shoul2shoul_rot_joint, left_arm2armrot_joint, left_armrot2elbow_joint, left_forearm2forearmrot_joint, left_forearmrot2forearm_pitch_joint, left_forearm_pitch2forearm_roll_joint], position: [0.3872, -0.9262, -0.0527, -0.7032, 1.4653, 0.0511, -0.537, 0, 0, 0, 0, 0, 0, 0]}" > /dev/null 2>&1 &
PUB_PID=$!

sleep 2
timeout 3 ros2 run tf2_ros tf2_echo base_link right_gripper_base_link 2>&1 | head -10

kill $PUB_PID 2>/dev/null
