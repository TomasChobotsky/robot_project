#!/usr/bin/env python3
"""
Simple test script to verify RealSense camera and ball detection without ROS2
Run this first to make sure your camera and color detection are working
"""
import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    print("="*60)
    print("RealSense Ball Detection Test")
    print("="*60)
    print("\nPress 'q' to quit")
    print("Press 'g' to toggle green detection")
    print("Press 'o' to toggle orange detection")
    print("Press 'h' to open HSV tuner")
    print("="*60)
    
    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable color and depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start streaming
    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"\n❌ Error: Could not start RealSense camera!")
        print(f"   {e}")
        print("\nTroubleshooting:")
        print("1. Make sure camera is plugged in")
        print("2. Run 'realsense-viewer' to test camera")
        print("3. Check permissions: sudo usermod -a -G plugdev $USER")
        return
    
    print("\n✓ Camera started successfully!")#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'robot_project'
    
    # Declare launch arguments for ball tracking controller

    
    # Get camera intrinsics
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    print(f"✓ Depth scale: {depth_scale}")
    print(f"✓ Camera resolution: {intrinsics.width}x{intrinsics.height}")
    
    # Align depth to color
    align = rs.align(rs.stream.color)
    
    # Color ranges (adjust these if needed)
    green_lower = np.array([35, 50, 50])
    green_upper = np.array([85, 255, 255])
    orange_lower = np.array([5, 100, 100])
    orange_upper = np.array([25, 255, 255])
    
    detect_green = True
    detect_orange = True
    
    try:
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Create display image
            display = color_image.copy()
            
            # Convert to HSV
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            
            # Detect green ball
            if detect_green:
                green_mask = cv2.inRange(hsv, green_lower, green_upper)
                kernel = np.ones((5, 5), np.uint8)
                green_mask = cv2.erode(green_mask, kernel, iterations=2)
                green_mask = cv2.dilate(green_mask, kernel, iterations=2)
                
                contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest) > 500:
                        ((x, y), radius) = cv2.minEnclosingCircle(largest)
                        M = cv2.moments(largest)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            # Get 3D position
                            depth = depth_frame.get_distance(cx, cy)
                            if depth > 0:
                                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                                
                                # Draw detection
                                cv2.circle(display, (cx, cy), int(radius), (0, 255, 0), 2)
                                cv2.circle(display, (cx, cy), 5, (0, 255, 0), -1)
                                
                                # Display 3D coordinates
                                text = f"GREEN: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})m"
                                cv2.putText(display, text, (cx - 100, cy - int(radius) - 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                                print(f"Green ball: x={point_3d[0]:.3f}m, y={point_3d[1]:.3f}m, z={point_3d[2]:.3f}m")
            
            # Detect orange ball
            if detect_orange:
                orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
                kernel = np.ones((5, 5), np.uint8)
                orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
                orange_mask = cv2.dilate(orange_mask, kernel, iterations=2)
                
                contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest) > 500:
                        ((x, y), radius) = cv2.minEnclosingCircle(largest)
                        M = cv2.moments(largest)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            # Get 3D position
                            depth = depth_frame.get_distance(cx, cy)
                            if depth > 0:
                                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                                
                                # Draw detection
                                cv2.circle(display, (cx, cy), int(radius), (0, 165, 255), 2)
                                cv2.circle(display, (cx, cy), 5, (0, 165, 255), -1)
                                
                                # Display 3D coordinates
                                text = f"ORANGE: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})m"
                                cv2.putText(display, text, (cx - 100, cy + int(radius) + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                                
                                print(f"Orange ball: x={point_3d[0]:.3f}m, y={point_3d[1]:.3f}m, z={point_3d[2]:.3f}m")
            
            # Display status
            status_text = []
            if detect_green:
                status_text.append("GREEN: ON")
            else:
                status_text.append("GREEN: OFF")
            if detect_orange:
                status_text.append("ORANGE: ON")
            else:
                status_text.append("ORANGE: OFF")
            
            cv2.putText(display, " | ".join(status_text), (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show image
            cv2.imshow('RealSense Ball Detection Test', display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                detect_green = not detect_green
                print(f"Green detection: {'ON' if detect_green else 'OFF'}")
            elif key == ord('o'):
                detect_orange = not detect_orange
                print(f"Orange detection: {'ON' if detect_orange else 'OFF'}")
            elif key == ord('h'):
                print("\nTo tune HSV values, use this script:")
                print("https://github.com/opencv/opencv/blob/master/samples/python/color_histogram.py")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n✓ Test completed")


if __name__ == '__main__':
    main()