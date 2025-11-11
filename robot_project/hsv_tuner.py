#!/usr/bin/env python3
"""
HSV Color Range Tuner for Ball Detection
Run this to find optimal HSV thresholds for your lighting conditions
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # Subscribe to color image
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Create windows
        cv2.namedWindow('Original')
        cv2.namedWindow('HSV Mask')
        cv2.namedWindow('Controls')
        
        # Create trackbars for GREEN ball
        cv2.createTrackbar('G_H_Low', 'Controls', 40, 179, lambda x: None)
        cv2.createTrackbar('G_S_Low', 'Controls', 60, 255, lambda x: None)
        cv2.createTrackbar('G_V_Low', 'Controls', 40, 255, lambda x: None)
        cv2.createTrackbar('G_H_High', 'Controls', 90, 179, lambda x: None)
        cv2.createTrackbar('G_S_High', 'Controls', 255, 255, lambda x: None)
        cv2.createTrackbar('G_V_High', 'Controls', 255, 255, lambda x: None)
        
        # Create trackbars for ORANGE ball
        cv2.createTrackbar('O_H_Low', 'Controls', 10, 179, lambda x: None)
        cv2.createTrackbar('O_S_Low', 'Controls', 150, 255, lambda x: None)
        cv2.createTrackbar('O_V_Low', 'Controls', 130, 255, lambda x: None)
        cv2.createTrackbar('O_H_High', 'Controls', 22, 179, lambda x: None)
        cv2.createTrackbar('O_S_High', 'Controls', 255, 255, lambda x: None)
        cv2.createTrackbar('O_V_High', 'Controls', 255, 255, lambda x: None)
        
        # Toggle between green and orange
        cv2.createTrackbar('Color (0=G, 1=O)', 'Controls', 0, 1, lambda x: None)
        
        # Morphology kernel size
        cv2.createTrackbar('Kernel Size', 'Controls', 5, 15, lambda x: None)
        
        # Min area
        cv2.createTrackbar('Min Area / 100', 'Controls', 8, 50, lambda x: None)
        
        self.create_timer(0.033, self.process_frame)  # ~30 Hz
        
        self.get_logger().info("HSV Tuner started!")
        self.get_logger().info("Adjust trackbars to find optimal values")
        self.get_logger().info("Press 'p' to print current values")
        self.get_logger().info("Press 'q' to quit")
    
    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
    
    def process_frame(self):
        if self.latest_frame is None:
            return
        
        frame = self.latest_frame.copy()
        
        # Get current trackbar values
        color_mode = cv2.getTrackbarPos('Color (0=G, 1=O)', 'Controls')
        kernel_size = cv2.getTrackbarPos('Kernel Size', 'Controls')
        if kernel_size < 1:
            kernel_size = 1
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        min_area = cv2.getTrackbarPos('Min Area / 100', 'Controls') * 100
        
        if color_mode == 0:  # Green
            h_low = cv2.getTrackbarPos('G_H_Low', 'Controls')
            s_low = cv2.getTrackbarPos('G_S_Low', 'Controls')
            v_low = cv2.getTrackbarPos('G_V_Low', 'Controls')
            h_high = cv2.getTrackbarPos('G_H_High', 'Controls')
            s_high = cv2.getTrackbarPos('G_S_High', 'Controls')
            v_high = cv2.getTrackbarPos('G_V_High', 'Controls')
            color_name = "GREEN"
            color_bgr = (0, 255, 0)
        else:  # Orange
            h_low = cv2.getTrackbarPos('O_H_Low', 'Controls')
            s_low = cv2.getTrackbarPos('O_S_Low', 'Controls')
            v_low = cv2.getTrackbarPos('O_V_Low', 'Controls')
            h_high = cv2.getTrackbarPos('O_H_High', 'Controls')
            s_high = cv2.getTrackbarPos('O_S_High', 'Controls')
            v_high = cv2.getTrackbarPos('O_V_High', 'Controls')
            color_name = "ORANGE"
            color_bgr = (0, 165, 255)
        
        # Convert to HSV and create mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw info on original frame
        info_frame = frame.copy()
        cv2.putText(info_frame, f"Tuning: {color_name}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color_bgr, 2)
        cv2.putText(info_frame, f"Contours found: {len(contours)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Process contours
        valid_contours = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= min_area:
                valid_contours += 1
                
                # Draw contour
                cv2.drawContours(info_frame, [cnt], -1, color_bgr, 2)
                
                # Calculate and draw centroid
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(info_frame, (cx, cy), 8, color_bgr, -1)
                    cv2.putText(info_frame, f"Area: {int(area)}", (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
        
        cv2.putText(info_frame, f"Valid (area>={min_area}): {valid_contours}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Show HSV values info
        cv2.putText(info_frame, f"H:[{h_low}-{h_high}] S:[{s_low}-{s_high}] V:[{v_low}-{v_high}]",
                    (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display
        cv2.imshow('Original', info_frame)
        cv2.imshow('HSV Mask', mask)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            raise KeyboardInterrupt
        elif key == ord('p'):
            self.print_values(color_mode, h_low, s_low, v_low, h_high, s_high, v_high,
                            kernel_size, min_area, color_name)
    
    def print_values(self, mode, h_low, s_low, v_low, h_high, s_high, v_high, 
                     kernel_size, min_area, color_name):
        """Print current parameter values"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"Current {color_name} Ball Parameters:")
        self.get_logger().info("="*60)
        
        prefix = "green" if mode == 0 else "orange"
        
        self.get_logger().info(f"# {color_name} HSV Thresholds:")
        self.get_logger().info(f"'{prefix}_h_low': {h_low}")
        self.get_logger().info(f"'{prefix}_s_low': {s_low}")
        self.get_logger().info(f"'{prefix}_v_low': {v_low}")
        self.get_logger().info(f"'{prefix}_h_high': {h_high}")
        self.get_logger().info(f"'{prefix}_s_high': {s_high}")
        self.get_logger().info(f"'{prefix}_v_high': {v_high}")
        self.get_logger().info(f"\n# Other parameters:")
        self.get_logger().info(f"'morph_kernel': {kernel_size}")
        self.get_logger().info(f"'min_area': {min_area}")
        self.get_logger().info("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()