#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from custom_dxl.CustomDXL import CustomDXL
from std_msgs.msg import Int32MultiArray

class myDynamixelController(Node):
    def __init__(self) -> None:
        super().__init__("send_single_joint_command")
        self.sub = self.create_subscription(Int32MultiArray, 'dxl_actual_joint_cmd', self.listener_callback, 10)
        self.dxls = CustomDXL(dxl_ids=[2, 3, 4], profile_velocity=[50, 50, 50])
        self.dxls.open_port()
        self.dxls.send_goal(goal_pos=[2047, 2047, 2047])
        print("Created")
        print("Publish data between [1023, 3073] to the topic /dxl_actual_joint_cmd")
        self.prev_joint = [0] * len(self.dxls.dxl_ids)

    def listener_callback(self, msg):
        data = list(msg.data)
        if not data:
            return
        
        for i, item in enumerate(data):
            if i < len(self.prev_joint) and item != self.prev_joint[i]:
                self.prev_joint[i] = item
                self.dxls.send_single_goal(motor_order=i, goal_pos=item)
    
def main(args=None):
    rclpy.init(args=args)
    node = myDynamixelController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()