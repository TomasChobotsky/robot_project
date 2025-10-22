#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class mySubscriberNode(Node):
    def __init__(self) -> None:
        self.bridge = CvBridge()
        super().__init__("my_subscriber")
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        print("Created")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        self.get_logger().info(f'The width is: {msg.width}, and height is: {msg.height}')

def main(args=None):
    rclpy.init(args=args)
    node = mySubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

