#!/usr/bin/env python3
"""
Test WORLD IK - verify X, Y, Z move in independent directions
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rx1_ik_custom import RX1CustomIK, RIGHT_ARM_SERVO_IDS, ik_debug_clear
import json

ik_debug_clear()

# Load baseline
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rx1_baseline_v5.json')) as f:
    data = json.load(f)

baseline = {sid: data['servos'][str(sid)]['position'] for sid in RIGHT_ARM_SERVO_IDS}

# Create IK solver
ik = RX1CustomIK('right')

# Get baseline position
base_pos = ik.get_end_effector_position(baseline)
print("=== WORLD IK Test ===")
print(f"Baseline position: X={base_pos[0]*100:.2f}, Y={base_pos[1]*100:.2f}, Z={base_pos[2]*100:.2f} cm")
print()

# Test each axis independently
delta = 0.05  # 5cm

print("--- Test 1: X +5cm ---")
target_x = [base_pos[0] + delta, base_pos[1], base_pos[2]]
result_x, ok_x = ik.solve_for_position(target_x, baseline)
if ok_x:
    pos_x = ik.get_end_effector_position(result_x)
    print(f"Result: X={pos_x[0]*100:.2f}, Y={pos_x[1]*100:.2f}, Z={pos_x[2]*100:.2f} cm")
    print(f"Delta:  dX={((pos_x[0]-base_pos[0])*100):+.2f}, dY={((pos_x[1]-base_pos[1])*100):+.2f}, dZ={((pos_x[2]-base_pos[2])*100):+.2f} cm")
else:
    print("FAILED")
print()

print("--- Test 2: Y +5cm ---")
target_y = [base_pos[0], base_pos[1] + delta, base_pos[2]]
result_y, ok_y = ik.solve_for_position(target_y, baseline)
if ok_y:
    pos_y = ik.get_end_effector_position(result_y)
    print(f"Result: X={pos_y[0]*100:.2f}, Y={pos_y[1]*100:.2f}, Z={pos_y[2]*100:.2f} cm")
    print(f"Delta:  dX={((pos_y[0]-base_pos[0])*100):+.2f}, dY={((pos_y[1]-base_pos[1])*100):+.2f}, dZ={((pos_y[2]-base_pos[2])*100):+.2f} cm")
else:
    print("FAILED")
print()

print("--- Test 3: Z +5cm ---")
target_z = [base_pos[0], base_pos[1], base_pos[2] + delta]
result_z, ok_z = ik.solve_for_position(target_z, baseline)
if ok_z:
    pos_z = ik.get_end_effector_position(result_z)
    print(f"Result: X={pos_z[0]*100:.2f}, Y={pos_z[1]*100:.2f}, Z={pos_z[2]*100:.2f} cm")
    print(f"Delta:  dX={((pos_z[0]-base_pos[0])*100):+.2f}, dY={((pos_z[1]-base_pos[1])*100):+.2f}, dZ={((pos_z[2]-base_pos[2])*100):+.2f} cm")
else:
    print("FAILED")
print()

print("=== Summary ===")
print("If WORLD IK works correctly:")
print("  X +5cm should mainly move dX ~+5cm")
print("  Y +5cm should mainly move dY ~+5cm")
print("  Z +5cm should mainly move dZ ~+5cm")
