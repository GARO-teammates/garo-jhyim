#!/usr/bin/env python3
"""
Joint state publisher for RX1 robot
Reads from sim_commands_v4.json and publishes to /joint_states
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os

class SimpleJointStatePublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_state_publisher')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Command file paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.join(script_dir, '..')  # RRR_GUI/
        # sim_commands: ros_files/isaac_sim_integration/ (GUI writes here)
        self.command_file = os.path.join(script_dir, 'isaac_sim_integration/sim_commands_v4.json')
        # wheel/lift: RRR_GUI/isaac_sim_integration/ (GUI writes here)
        self.wheel_command_file = os.path.join(base_dir, 'isaac_sim_integration/wheel_commands_v5.json')
        self.lift_command_file = os.path.join(base_dir, 'isaac_sim_integration/lift_commands_v5.json')

        # All non-fixed joints from URDF
        self.joint_names = [
            'right_shoul_base2shoul_joint[11]',
            'right_shoul2shoul_rot_joint[12]',
            'right_arm2armrot_joint[13]',
            'right_armrot2elbow_joint[14]',
            'right_forearm2forearmrot_joint[15]',
            'right_forearmrot2forearm_pitch_joint[16]',
            'right_forearm_pitch2forearm_roll_joint[17]',
            'dummy_joint[18]',
            'dummy_joint[19]',
            'dummy_joint[20]',
            'dummy_joint[21]',
            'dummy_joint[22]',
            'dummy_joint[23]',
            'dummy_joint[24]',
            'dummy_joint[25]',
            'dummy_joint[26]',
            'dummy_joint[27]',
            'dummy_joint[28]',
            'dummy_joint[29]',
            'dummy_joint[30]',
            'dummy_joint_30a',
            'dummy_joint_30b',
            'dummy_joint_30c',
            'dummy_joint_30d',
            'dummy_joint_30e',
            'dummy_joint_30f',
            'dummy_joint_30g',
            'dummy_joint_30h',
            'dummy_joint_30i',
            'dummy_joint_30j',
            'right_gripper_joint[41]',
            'right_gripper_tip2_joint[32]',
            'left_shoul_base2shoul_joint[21]',
            'left_shoul2shoul_rot_joint[22]',
            'left_arm2armrot_joint[23]',
            'left_armrot2elbow_joint[24]',
            'left_forearm2forearmrot_joint[25]',
            'left_forearmrot2forearm_pitch_joint[26]',
            'left_forearm_pitch2forearm_roll_joint[27]',
            'dummy_joint_40',
            'left_gripper_joint[31]',
            'left_gripper_tip2_joint[28]',
            'base_to_lift_joint',
            'base_to_nema42_joint',
            'base_to_xp1000_joint',
            'caster_front_left_pivot_joint',
            'caster_front_left_axle_joint',
            'caster_front_right_pivot_joint',
            'caster_front_right_axle_joint',
            'caster_rear_left_pivot_joint',
            'caster_rear_left_axle_joint',
            'caster_rear_right_pivot_joint',
            'caster_rear_right_axle_joint',
            'cater_left_wheel_joint',
            'cater_right_wheel_joint'
        ]

        # Mapping from GUI joint names to URDF joint names
        # GUI uses: "right_shoul_base2shoul_joint_11_"
        # URDF uses: "right_shoul_base2shoul_joint[11]"
        self.name_map = {
            'right_shoul_base2shoul_joint_11_': 'right_shoul_base2shoul_joint[11]',
            'right_shoul2shoul_rot_joint_12_': 'right_shoul2shoul_rot_joint[12]',
            'right_arm2armrot_joint_13_': 'right_arm2armrot_joint[13]',
            'right_armrot2elbow_joint_14_': 'right_armrot2elbow_joint[14]',
            'right_forearm2forearmrot_joint_15_': 'right_forearm2forearmrot_joint[15]',
            'right_forearmrot2forearm_pitch_joint_16_': 'right_forearmrot2forearm_pitch_joint[16]',
            'right_forearm_pitch2forearm_roll_joint_17_': 'right_forearm_pitch2forearm_roll_joint[17]',
            'right_gripper_joint_41_': 'right_gripper_joint[41]',
            'right_gripper_tip2_joint_32_': 'right_gripper_tip2_joint[32]',
            'left_shoul_base2shoul_joint_21_': 'left_shoul_base2shoul_joint[21]',
            'left_shoul2shoul_rot_joint_22_': 'left_shoul2shoul_rot_joint[22]',
            'left_arm2armrot_joint_23_': 'left_arm2armrot_joint[23]',
            'left_armrot2elbow_joint_24_': 'left_armrot2elbow_joint[24]',
            'left_forearm2forearmrot_joint_25_': 'left_forearm2forearmrot_joint[25]',
            'left_forearmrot2forearm_pitch_joint_26_': 'left_forearmrot2forearm_pitch_joint[26]',
            'left_forearm_pitch2forearm_roll_joint_27_': 'left_forearm_pitch2forearm_roll_joint[27]',
            'left_gripper_joint_31_': 'left_gripper_joint[31]',
            'left_gripper_tip2_joint_28_': 'left_gripper_tip2_joint[28]',
        }

        # Default positions (all zeros)
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # Wheel velocities and positions
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_update_time = self.get_clock().now()

        # Lift position
        self.lift_position = 0.0

        # Publish at 50 Hz (faster updates for better responsiveness)
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info(f'Publishing joint states for {len(self.joint_names)} joints')
        self.get_logger().info(f'Reading commands from: {self.command_file}')
        self.get_logger().info(f'Reading wheel commands from: {self.wheel_command_file}')
        self.get_logger().info(f'Reading lift commands from: {self.lift_command_file}')

    def read_command_file(self):
        """Read joint positions from sim_commands_v4.json"""
        try:
            if not os.path.exists(self.command_file):
                return

            with open(self.command_file, 'r') as f:
                data = json.load(f)

            # Update positions from file
            if 'joints' in data and isinstance(data['joints'], list):
                for joint_data in data['joints']:
                    gui_name = joint_data.get('name', '')
                    position = joint_data.get('position_rad', 0.0)

                    # Map GUI name to URDF name
                    urdf_name = self.name_map.get(gui_name)
                    if urdf_name and urdf_name in self.joint_positions:
                        self.joint_positions[urdf_name] = position

        except Exception as e:
            # Silently ignore read errors (file might be being written)
            pass

    def read_wheel_command_file(self):
        """Read wheel velocities from wheel_commands_v4.json"""
        try:
            if not os.path.exists(self.wheel_command_file):
                return

            with open(self.wheel_command_file, 'r') as f:
                data = json.load(f)

            # Update wheel velocities
            self.left_wheel_velocity = data.get('left_wheel_velocity', 0.0)
            self.right_wheel_velocity = data.get('right_wheel_velocity', 0.0)

        except Exception as e:
            # Silently ignore read errors
            pass

    def read_lift_command_file(self):
        """Read lift position from lift_commands_v5.json"""
        try:
            if not os.path.exists(self.lift_command_file):
                return

            with open(self.lift_command_file, 'r') as f:
                data = json.load(f)

            # Update lift position (meters)
            self.lift_position = data.get('lift_position', 0.0)

        except Exception as e:
            # Silently ignore read errors
            pass

    def publish_joint_states(self):
        # Read latest commands
        self.read_command_file()
        self.read_wheel_command_file()
        self.read_lift_command_file()

        # Update wheel positions based on velocity (integration)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        # Integrate velocity to position only if velocity is not zero
        if abs(self.left_wheel_velocity) > 0.001:
            self.left_wheel_position += self.left_wheel_velocity * dt
        if abs(self.right_wheel_velocity) > 0.001:
            self.right_wheel_position += self.right_wheel_velocity * dt

        # Update wheel positions in joint_positions dict
        if 'cater_left_wheel_joint' in self.joint_positions:
            self.joint_positions['cater_left_wheel_joint'] = self.left_wheel_position
        if 'cater_right_wheel_joint' in self.joint_positions:
            self.joint_positions['cater_right_wheel_joint'] = self.right_wheel_position

        # Update lift position
        if 'base_to_lift_joint' in self.joint_positions:
            self.joint_positions['base_to_lift_joint'] = self.lift_position

        # Publish joint states with wheel velocities
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[name] for name in self.joint_names]

        # Add velocities for wheel joints
        msg.velocity = []
        for name in self.joint_names:
            if name == 'cater_left_wheel_joint':
                msg.velocity.append(self.left_wheel_velocity)
            elif name == 'cater_right_wheel_joint':
                msg.velocity.append(self.right_wheel_velocity)
            else:
                msg.velocity.append(0.0)

        msg.effort = []

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
