#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
GUI_ROOT="$(dirname "$SCRIPT_DIR")"

export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash

pkill -9 -f ros2 2>/dev/null
pkill -9 -f robot_state 2>/dev/null
pkill -9 -f joint_state 2>/dev/null
sleep 2

ros2 launch "$GUI_ROOT/ros_files/rx1_robot_state_only.launch.py" &
LAUNCH_PID=$!
sleep 4

cd "$SCRIPT_DIR"

/usr/bin/python3 ros_ik_test.py

kill $LAUNCH_PID 2>/dev/null
