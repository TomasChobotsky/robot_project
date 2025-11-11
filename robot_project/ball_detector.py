#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import pyrealsense2 as rs
import math

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        # Launch parameters
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('processing_rate', 5.0)
        self.declare_parameter('min_area', 600)
        self.declare_parameter('morph_kernel', 5)
        self.declare_parameter('display_scale', 1.0)

        # HSV thresholds (adjustable)
        # Green
        self.declare_parameter('green_h_low', 65)
        self.declare_parameter('green_s_low', 120)
        self.declare_parameter('green_v_low', 80)
        self.declare_parameter('green_h_high', 90)
        self.declare_parameter('green_s_high', 255)
        self.declare_parameter('green_v_high', 255)

        # Orange
        self.declare_parameter('orange_h_low', 5)
        self.declare_parameter('orange_s_low', 140)
        self.declare_parameter('orange_v_low', 140)
        self.declare_parameter('orange_h_high', 25)
        self.declare_parameter('orange_s_high', 255)
        self.declare_parameter('orange_v_high', 255)


        # Load parameters
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.rate_hz = float(self.get_parameter('processing_rate').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.morph_kernel = int(self.get_parameter('morph_kernel').value)
        self.display_scale = float(self.get_parameter('display_scale').value)

        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics_valid = False
        self.intrinsics = rs.intrinsics()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))

        # Publishers
        self.green_pub = self.create_publisher(PointStamped, 'green_ball_position', 10)
        self.orange_pub = self.create_publisher(PointStamped, 'orange_ball_position', 10)

        # Subscribers
        self.create_subscription(CameraInfoMsg,
                                 '/camera/aligned_depth_to_color/camera_info',
                                 self.caminfo_cb, 10)
        self.create_subscription(ImageMsg,
                                 '/camera/color/image_raw',
                                 self.color_cb, 10)
        self.create_subscription(ImageMsg,
                                 '/camera/aligned_depth_to_color/image_raw',
                                 self.depth_cb, 10)

        # Main loop timer
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self.process_frame)

        self.get_logger().info(f'BallDetector started (tracking green & orange, {self.rate_hz} Hz)')

    def caminfo_cb(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        self.intrinsics.model = rs.distortion.brown_conrady
        self.intrinsics.coeffs = [i for i in msg.d]
        self.intrinsics_valid = True

    def color_cb(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Color conversion failed: {e}')

    def depth_cb(self, msg):
        try:
            if msg.encoding == '16UC1':
                dtype = np.uint16
                depth_image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
            elif msg.encoding in ['32FC1', 'float32']:
                depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                # fallback to cv_bridge for other encodings
                depth_image = self.bridge.imgmsg_to_cv2(msg)
            self.latest_depth = depth_image
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')


    def _threshold_hsv(self, bgr, low, high):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def _find_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None, None
        perimeter = cv2.arcLength(largest, True)
        if perimeter == 0:
            return None, None
        circularity = 4 * math.pi * (cv2.contourArea(largest) / (perimeter * perimeter))
        if circularity < 0.6:  # 1.0 = perfect circle
            return None, None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy), largest


    def _depth_at(self, cx, cy):
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return None
        d = float(self.latest_depth[cy, cx])
        if d <= 0:
            return None
        # convert mm→m if needed
        if d > 10:
            d /= 1000.0
        return d

    def _deproject(self, pixel, depth_m):
        try:
            p = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(pixel[0]), float(pixel[1])], float(depth_m))
            return np.array(p)
        except Exception as e:
            self.get_logger().error(f'Deprojection failed: {e}')
            return None

    def _publish_point(self, pub, point):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x, msg.point.y, msg.point.z = point
        pub.publish(msg)

    def _publish_lost(self, pub):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x = msg.point.y = msg.point.z = 0.0
        pub.publish(msg)

    def process_frame(self):
        if self.latest_color is None or self.latest_depth is None or not self.intrinsics_valid:
            return

        frame = self.latest_color.copy()

        # Load thresholds
        g_low = [self.get_parameter('green_h_low').value,
                 self.get_parameter('green_s_low').value,
                 self.get_parameter('green_v_low').value]
        g_high = [self.get_parameter('green_h_high').value,
                  self.get_parameter('green_s_high').value,
                  self.get_parameter('green_v_high').value]

        o_low = [self.get_parameter('orange_h_low').value,
                 self.get_parameter('orange_s_low').value,
                 self.get_parameter('orange_v_low').value]
        o_high = [self.get_parameter('orange_h_high').value,
                  self.get_parameter('orange_s_high').value,
                  self.get_parameter('orange_v_high').value]

        # Detect green
        mask_g = self._threshold_hsv(frame, g_low, g_high)
        c_g, contour_g = self._find_centroid(mask_g)
        found_g = False

        if c_g:
            d = self._depth_at(*c_g)
            if d and 0.1 < d < 5.0:
                p = self._deproject(c_g, d)
                if p is not None:
                    self._publish_point(self.green_pub, p)
                    found_g = True
                    cv2.circle(frame, c_g, 6, (0, 255, 0), -1)
                    cv2.putText(frame, f"G {d:.2f}m", (c_g[0]+8, c_g[1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                    # Draw enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour_g)
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        if not found_g:
            self._publish_lost(self.green_pub)

        # Detect orange
        mask_o = self._threshold_hsv(frame, o_low, o_high)
        c_o, contour_o = self._find_centroid(mask_o)
        found_o = False

        if c_o:
            d = self._depth_at(*c_o)
            if d:
                p = self._deproject(c_o, d)
                if p is not None:
                    self._publish_point(self.orange_pub, p)
                    found_o = True
                    cv2.circle(frame, c_o, 6, (0, 165, 255), -1)
                    cv2.putText(frame, f"O {d:.2f}m", (c_o[0]+8, c_o[1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 1)
                    # Draw enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour_o)
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 165, 255), 2)
        if not found_o:
            self._publish_lost(self.orange_pub)

        if self.enable_visualization:
            scaled = cv2.resize(frame, (0, 0), fx=self.display_scale, fy=self.display_scale)
            cv2.imshow('ball_detector', scaled)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()