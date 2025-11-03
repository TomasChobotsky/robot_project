#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from adatools import config_generator as cg
from adatools import plotting_tools as pt
from adatools import utils as utils

from std_msgs.msg import Int32MultiArray


class myDynamixelVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("robot_visualizer")
        self.pub = self.create_publisher(Int32MultiArray, 'dxl_joint_cmd', 10)
        self.create_config()
        self.create_visualizer()
        self.create_timer(0.25, self.timer_callback)

        print("Created")

    def create_config(self):
        self.my_conf_robot = cg.get_robot_config_1(link1=0.145, link1_offset=0.0,
                                       link2=0.350, link2_offset=0.0,
                                       link3=0.330, link3_offset=0.0,
                                       link4=0.100, link4_offset=0.0)
        
    def create_visualizer(self):
        self.robot_teach = self.my_conf_robot.teach(self.my_conf_robot.q, backend='pyplot', block=False)
        self.plot = pt.plot_baseplate(self.robot_teach)

    def timer_callback(self):
        robot = self.my_conf_robot.q
        msg = Int32MultiArray()
        msg.data = [utils.rad2steps(robot[0]), utils.rad2steps(robot[1]), utils.rad2steps(robot[2])]
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.plot.step()
    
def main(args=None):
    rclpy.init(args=args)
    node = myDynamixelVisualizer()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()