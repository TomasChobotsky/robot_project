#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class BallDetector(Node):
    def __init__(self):
        super().__init__("ball_detector")
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Subscribers for RealSense topics
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.green_ball_pub = self.create_publisher(PointStamped, 'green_ball_position', 10)
        self.orange_ball_pub = self.create_publisher(PointStamped, 'orange_ball_position', 10)
        self.target_color_pub = self.create_publisher(String, 'target_ball_color', 10)
        
        # Storage for latest images
        self.color_image = None
        self.depth_image = None
        self.camera_intrinsics = None
        
        # Color ranges in HSV for detection
        # Green ball HSV range
        self.green_lower = np.array([35, 50, 50])
        self.green_upper = np.array([85, 255, 255])
        
        # Orange ball HSV range
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([25, 255, 255])
        
        # Minimum contour area to filter noise
        self.min_contour_area = 500
        
        # Timer for processing (10 Hz)
        self.create_timer(0.1, self.process_callback)
        
        self.get_logger().info("Ball Detector Node Started")
    
    def camera_info_callback(self, msg):
        """Store camera intrinsics for depth-to-3D conversion"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = rs.intrinsics()
            self.camera_intrinsics.width = msg.width
            self.camera_intrinsics.height = msg.height
            self.camera_intrinsics.ppx = msg.k[2]
            self.camera_intrinsics.ppy = msg.k[5]
            self.camera_intrinsics.fx = msg.k[0]
            self.camera_intrinsics.fy = msg.k[4]
            self.camera_intrinsics.model = rs.distortion.none
            self.camera_intrinsics.coeffs = [0, 0, 0, 0, 0]
            self.get_logger().info("Camera intrinsics received")
    
    def color_callback(self, msg):
        """Store latest color image"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
    
    def detect_ball(self, color_image, lower_hsv, upper_hsv, color_name):
        """
        Detect a ball of specified color in the image
        Returns: (center_x, center_y, radius) or None
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Create mask for specified color
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Check if contour is large enough
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            return None
        
        # Get minimum enclosing circle
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        # Calculate moments for better center
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
        else:
            center_x, center_y = int(x), int(y)
        
        self.get_logger().info(f"{color_name} ball detected at pixel ({center_x}, {center_y}), radius: {radius:.1f}")
        
        return (center_x, center_y, radius)
    
    def pixel_to_3d(self, pixel_x, pixel_y):
        """
        Convert 2D pixel coordinates to 3D coordinates using depth
        Returns: (x, y, z) in meters relative to camera frame
        """
        if self.depth_image is None or self.camera_intrinsics is None:
            return None
        
        # Get depth value at pixel (in millimeters, convert to meters)
        depth_value = self.depth_image[pixel_y, pixel_x] / 1000.0
        
        # Check for invalid depth
        if depth_value <= 0.0 or depth_value > 5.0:  # Max 5 meters
            self.get_logger().warn(f"Invalid depth value: {depth_value}")
            return None
        
        # Deproject pixel to 3D point
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [pixel_x, pixel_y], depth_value)
        
        return point_3d  # Returns [x, y, z] in meters
    
    def publish_ball_position(self, position_3d, color_name):
        """Publish 3D position of detected ball"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x = position_3d[0]
        msg.point.y = position_3d[1]
        msg.point.z = position_3d[2]
        
        if color_name == "green":
            self.green_ball_pub.publish(msg)
        elif color_name == "orange":
            self.orange_ball_pub.publish(msg)
        
        self.get_logger().info(f"{color_name.capitalize()} ball at: x={position_3d[0]:.3f}, y={position_3d[1]:.3f}, z={position_3d[2]:.3f}")
    
    def process_callback(self):
        """Main processing loop"""
        if self.color_image is None or self.depth_image is None:
            return
        
        # Create a copy for visualization
        display_image = self.color_image.copy()
        
        # Detect green ball
        green_detection = self.detect_ball(self.color_image, self.green_lower, self.green_upper, "green")
        if green_detection:
            cx, cy, radius = green_detection
            cv2.circle(display_image, (cx, cy), int(radius), (0, 255, 0), 2)
            cv2.circle(display_image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(display_image, "GREEN", (cx - 30, cy - int(radius) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Convert to 3D and publish
            pos_3d = self.pixel_to_3d(cx, cy)
            if pos_3d:
                self.publish_ball_position(pos_3d, "green")
        
        # Detect orange ball
        orange_detection = self.detect_ball(self.color_image, self.orange_lower, self.orange_upper, "orange")
        if orange_detection:
            cx, cy, radius = orange_detection
            cv2.circle(display_image, (cx, cy), int(radius), (0, 165, 255), 2)
            cv2.circle(display_image, (cx, cy), 5, (0, 165, 255), -1)
            cv2.putText(display_image, "ORANGE", (cx - 30, cy - int(radius) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            # Convert to 3D and publish
            pos_3d = self.pixel_to_3d(cx, cy)
            if pos_3d:
                self.publish_ball_position(pos_3d, "orange")
        
        # Display the image with detections
        cv2.imshow("Ball Detection", display_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()