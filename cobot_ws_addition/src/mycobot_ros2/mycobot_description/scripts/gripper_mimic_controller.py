#!/usr/bin/env python3
"""
Gripper Mimic Controller for Gazebo

This script creates a simple controller that manually synchronizes gripper joints
when the physics engine doesn't support mimic constraints.

:author: GitHub Copilot
:date: November 18, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class GripperMimicController(Node):
    def __init__(self):
        super().__init__('gripper_mimic_controller')
        
        # Joint names
        self.master_joint = 'gripper_controller'
        self.mimic_joints = {
            'gripper_base_to_gripper_left2': 1.0,
            'gripper_left3_to_gripper_left1': -1.0,
            'gripper_base_to_gripper_right3': -1.0,
            'gripper_base_to_gripper_right2': -1.0,
            'gripper_right3_to_gripper_right1': 1.0
        }
        
        # Subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers for each mimic joint
        self.joint_publishers = {}
        for joint_name in self.mimic_joints.keys():
            topic = f'/position_controller_{joint_name}/commands'
            self.joint_publishers[joint_name] = self.create_publisher(
                Float64MultiArray,
                topic,
                10
            )
        
        self.get_logger().info('Gripper mimic controller started')
    
    def joint_state_callback(self, msg):
        """Callback to handle joint state updates and control mimic joints"""
        try:
            # Find master joint position
            if self.master_joint in msg.name:
                master_idx = msg.name.index(self.master_joint)
                master_position = msg.position[master_idx]
                
                # Update mimic joints
                for joint_name, multiplier in self.mimic_joints.items():
                    target_position = master_position * multiplier
                    
                    # Publish command
                    cmd_msg = Float64MultiArray()
                    cmd_msg.data = [target_position]
                    
                    if joint_name in self.joint_publishers:
                        self.joint_publishers[joint_name].publish(cmd_msg)
                        
        except Exception as e:
            self.get_logger().error(f'Error in mimic control: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = GripperMimicController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()