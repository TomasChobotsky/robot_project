#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from adatools import utils as utils

from std_msgs.msg import Int32MultiArray


class visualizerToRobot(Node):
    def __init__(self) -> None:
        super().__init__("visualizer_to_robot")
        self.sub = self.create_subscription(Int32MultiArray, 'dxl_joint_cmd', self.listener_callback, 10)
        self.pub = self.create_publisher(Int32MultiArray, 'dxl_actual_joint_cmd', 10)
        self.create_timer(0.25, self.timer_callback)
        self.received_data = []

        print("Created")

    def listener_callback(self, msg):
        self.received_data = msg.data

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.received_data
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = visualizerToRobot()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()