#!/usr/bin/env python3
"""Debug FK chain step by step"""
import numpy as np
import math

def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy_to_rot(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)

def make_tf(xyz, rpy):
    T = np.eye(4)
    T[:3,:3] = rpy_to_rot(rpy[0], rpy[1], rpy[2])
    T[:3,3] = xyz
    return T

def axis_rot(axis, angle):
    axis = np.array(axis) / np.linalg.norm(axis)
    K = np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)

# Joint angles (baseline)
# [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232]
q = [0.445, -0.9523, -0.0327, -0.5703, 1.4469, -0.0598, 0.3232]

print("=== Step by step FK ===\n")

T = np.eye(4)
print(f"0. Start: {T[:3,3]}")

# base -> lift
T = T @ make_tf([-0.041, 0, 0.99], [0,0,0])
print(f"1. After lift: Z={T[2,3]*100:.1f}cm")

# lift -> torso
T = T @ make_tf([0.03, 0, -0.0975], [0,0,0])
print(f"2. After torso: Z={T[2,3]*100:.1f}cm")

# torso -> head_base
T = T @ make_tf([0, 0, 0.21], [0,0,0])
print(f"3. After head_base: Z={T[2,3]*100:.1f}cm")

# head_base -> shoul_base (60 deg roll)
T = T @ make_tf([0, -0.12, -0.05], [1.04706195, 0, 0])
print(f"4. After shoul_base (60 roll): X={T[0,3]*100:.1f}, Y={T[1,3]*100:.1f}, Z={T[2,3]*100:.1f}cm")

# shoul_base -> shoul_link
T = T @ make_tf([0, 0, 0.075], [0,0,0])
print(f"5. After shoul_link offset: pos={T[:3,3]*100}")

# Joint 11 (Z-, angle=0.445)
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([0,0,-1], q[0])
T = T @ T_rot
print(f"6. After J11 (Z- rot {math.degrees(q[0]):.1f}deg): pos={T[:3,3]*100}")

# shoul -> shoul_rot (-60 deg roll)
T = T @ make_tf([0, 0, 0.08], [-1.04706195, 0, 0])
print(f"7. After shoul_rot offset (-60 roll): pos={T[:3,3]*100}")

# Joint 12 (X-, angle=-0.9523)
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([-1,0,0], q[1])
T = T @ T_rot
print(f"8. After J12 (X- rot {math.degrees(q[1]):.1f}deg): pos={T[:3,3]*100}")

# upper_arm offset
T = T @ make_tf([0, 0, -0.0625], [0,0,0])
print(f"9. After upper_arm: pos={T[:3,3]*100}")

# Joint 13 (Z+, angle=-0.0327)
T = T @ make_tf([0, 0, -0.0625], [0,0,0])
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([0,0,1], q[2])
T = T @ T_rot
print(f"10. After J13: pos={T[:3,3]*100}")

# Joint 14 (Y+, angle=-0.5703)
T = T @ make_tf([0, 0, -0.22], [0,0,0])
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([0,1,0], q[3])
T = T @ T_rot
print(f"11. After J14 (elbow): pos={T[:3,3]*100}")

# forearm offset
T = T @ make_tf([0, 0, -0.045], [0,0,0])
print(f"12. After forearm: pos={T[:3,3]*100}")

# Joint 15 (Z+, angle=1.4469)
T = T @ make_tf([0, 0, -0.045], [0,0,0])
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([0,0,1], q[4])
T = T @ T_rot
print(f"13. After J15: pos={T[:3,3]*100}")

# Joint 16 (Y+, angle=-0.0598)
T = T @ make_tf([0, 0, -0.18], [0,0,0])
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([0,1,0], q[5])
T = T @ T_rot
print(f"14. After J16: pos={T[:3,3]*100}")

# Joint 17 (X+, angle=0.3232)
T_rot = np.eye(4)
T_rot[:3,:3] = axis_rot([1,0,0], q[6])
T = T @ T_rot
print(f"15. After J17: pos={T[:3,3]*100}")

# wrist
T = T @ make_tf([0, 0, -0.015], [0,0,0])
print(f"16. After wrist: pos={T[:3,3]*100}")

# gripper_base (yaw 180)
T = T @ make_tf([0, 0, -0.05], [0, 0, 3.14159])
print(f"17. After gripper_base: pos={T[:3,3]*100}")

# gripper_center
T = T @ make_tf([0, 0, -0.03], [0,0,0])
print(f"\n=== Final (gripper_center) ===")
print(f"X={T[0,3]*100:.2f}cm, Y={T[1,3]*100:.2f}cm, Z={T[2,3]*100:.2f}cm")

print("\n=== ROS TF says ===")
print("X=33.7cm, Y=-21.4cm, Z=74.1cm")
