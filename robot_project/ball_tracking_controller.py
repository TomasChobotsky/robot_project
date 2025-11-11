#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger
from adatools import config_generator as cg
from adatools import utils as utils
from spatialmath import SE3
import numpy as np

class BallTrackingController(Node):
    def __init__(self):
        super().__init__("ball_tracking_controller")
        
        self.green_ball_sub = self.create_subscription(
            PointStamped, 'green_ball_position', self.green_ball_callback, 10)
        self.orange_ball_sub = self.create_subscription(
            PointStamped, 'orange_ball_position', self.orange_ball_callback, 10)
        
        self.joint_cmd_pub = self.create_publisher(Int32MultiArray, 'dxl_joint_cmd', 10)
        self.target_pub = self.create_publisher(String, 'current_target', 10)
        
        # Services for manual control
        self.move_initial_srv = self.create_service(Trigger, 'move_to_initial', self.move_to_initial_callback)
        self.move_first_srv = self.create_service(Trigger, 'move_to_first_ball', self.move_to_first_callback)
        self.move_second_srv = self.create_service(Trigger, 'move_to_second_ball', self.move_to_second_callback)
        
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.200, link1_offset=0.0,
            link2=0.330, link2_offset=0.0,
            link3=0.335, link3_offset=0.0,
            link4=0.080, link4_offset=0.0
        )
        
        self.green_ball_pos = None
        self.orange_ball_pos = None
        
        # Initial position (home/ready position)
        self.declare_parameter('initial_q1', 0.0)
        self.declare_parameter('initial_q2', 0.0)
        self.declare_parameter('initial_q3', 0.0)
        self.declare_parameter('initial_q4', 0.0)
        
        self.initial_joints = np.array([
            self.get_parameter('initial_q1').value,
            self.get_parameter('initial_q2').value,
            self.get_parameter('initial_q3').value,
            self.get_parameter('initial_q4').value
        ])
        
        # Ball sequence: 'green_first' or 'orange_first'
        self.declare_parameter('ball_sequence', 'green_first')
        self.ball_sequence = self.get_parameter('ball_sequence').value
        
        # Camera to robot base transform
        self.declare_parameter('camera_x_offset', 1.0)
        self.declare_parameter('camera_y_offset', 0.0)
        self.declare_parameter('camera_z_offset', 0.1)
        self.declare_parameter('camera_rotation', 180.0)
        
        self.cam_x = self.get_parameter('camera_x_offset').value
        self.cam_y = self.get_parameter('camera_y_offset').value
        self.cam_z = self.get_parameter('camera_z_offset').value
        self.cam_rot = self.get_parameter('camera_rotation').value
        
        # Safe workspace limits
        self.workspace_x_min = 0.05
        self.workspace_x_max = 0.8
        self.workspace_y_min = -0.5
        self.workspace_y_max = 0.5
        self.workspace_z_min = 0.0
        self.workspace_z_max = 0.5
        
        # Offset above ball for approach
        self.declare_parameter('approach_height_offset', 0.1)
        self.approach_offset = self.get_parameter('approach_height_offset').value
        
        # Store last valid joint configuration
        self.last_q = self.initial_joints
        
        self.get_logger().info(f"Ball Tracking Controller Started (Sequential Manual Mode)")
        self.get_logger().info(f"Ball sequence: {self.ball_sequence}")
        self.get_logger().info(f"Initial joints: {self.initial_joints}")
        self.get_logger().info(f"Camera offset: x={self.cam_x}, y={self.cam_y}, z={self.cam_z}")
    
    def green_ball_callback(self, msg):
        if msg.point.x == 0.0 and msg.point.y == 0.0 and msg.point.z == 0.0:
            if self.green_ball_pos is not None:
                self.get_logger().info("Green ball lost")
            self.green_ball_pos = None
        else:
            self.green_ball_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.get_logger().debug(f"Green ball: {self.green_ball_pos}")

    def orange_ball_callback(self, msg):
        if msg.point.x == 0.0 and msg.point.y == 0.0 and msg.point.z == 0.0:
            if self.orange_ball_pos is not None:
                self.get_logger().info("Orange ball lost")
            self.orange_ball_pos = None
        else:
            self.orange_ball_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.get_logger().debug(f"Orange ball: {self.orange_ball_pos}")
    
    def camera_to_robot_frame(self, camera_pos):
        x_cam, y_cam, z_cam = camera_pos
        theta = np.radians(self.cam_rot)
        
        # Camera to standard frame conversion
        x_std = z_cam
        y_std = -x_cam
        z_std = -y_cam
        
        # Rotation around Z-axis
        x_rotated = x_std * np.cos(theta) - y_std * np.sin(theta)
        y_rotated = x_std * np.sin(theta) + y_std * np.cos(theta)
        z_rotated = z_std
        
        # Add offset
        x_robot = self.cam_x + x_rotated
        y_robot = self.cam_y + y_rotated
        z_robot = self.cam_z + z_rotated

        self.get_logger().info(f"Camera: [{x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}] → "
                              f"Robot: [{x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f}]")
        return np.array([x_robot, y_robot, z_robot])
    
    def is_in_workspace(self, position):
        x, y, z = position
        return (self.workspace_x_min <= x <= self.workspace_x_max and
                self.workspace_y_min <= y <= self.workspace_y_max and
                self.workspace_z_min <= z <= self.workspace_z_max)
    
    def compute_ik(self, target_position):
        x, y, z = target_position
        z_approach = z + self.approach_offset
        
        if not self.is_in_workspace([x, y, z_approach]):
            self.get_logger().warn(f"Target out of workspace: {target_position}")
            return None
        
        # CRITICAL: End effector pointing DOWN
        # Use SE3.Ry(180) for proper downward orientation
        Tgoal = SE3(x, y, z_approach) * SE3.Ry(180, 'deg')
        
        initial_guesses = [
            self.last_q,
            self.initial_joints,
            self.my_conf_robot.qr,
            np.array([0, np.pi/4, np.pi/4, 0]),
        ]
        
        for i, q0 in enumerate(initial_guesses):
            try:
                sol = self.my_conf_robot.ikine_LM(
                    Tgoal, 
                    q0=q0,
                    mask=[1, 1, 1, 1, 1, 1],
                    ilimit=1000,
                    slimit=200,
                    tol=1e-6,
                )
                
                if sol.success:
                    T_actual = self.my_conf_robot.fkine(sol.q)
                    trans_error = np.linalg.norm(Tgoal.t - T_actual.t)
                    
                    # Check end effector orientation (should point down = -Z direction)
                    z_desired = Tgoal.R[:3, 2]  # Z-axis of desired frame
                    z_actual = T_actual.R[:3, 2]  # Z-axis of actual frame
                    
                    # For pointing down, Z-axis should be close to [0, 0, -1]
                    orientation_error = np.arccos(np.clip(np.dot(z_desired, z_actual), -1, 1))
                    orientation_error_deg = np.degrees(orientation_error)
                    
                    self.get_logger().info(f"IK attempt {i+1}: pos_err={trans_error*1000:.1f}mm, "
                                          f"orient_err={orientation_error_deg:.1f}°")
                    self.get_logger().info(f"End effector Z-axis: {z_actual}")
                    
                    if trans_error < 0.02 and orientation_error_deg < 15:
                        self.get_logger().info(f"✓ IK successful")
                        self.last_q = sol.q
                        return sol.q
                        
            except Exception as e:
                self.get_logger().debug(f"IK attempt {i+1} failed: {e}")
                continue
        
        self.get_logger().error("All IK attempts failed")
        return None
    
    def send_joint_command(self, joint_angles):
        msg = Int32MultiArray()
        msg.data = [
            utils.rad2steps(joint_angles[0]),
            utils.rad2steps(joint_angles[1]),
            utils.rad2steps(joint_angles[2]),
            utils.rad2steps(joint_angles[3])
        ]
        self.joint_cmd_pub.publish(msg)
        
        joint_degrees = [np.degrees(q) for q in joint_angles]
        self.get_logger().info(f"Joint cmd (deg): {[f'{d:.1f}' for d in joint_degrees]}")
        self.get_logger().info(f"Joint cmd (steps): {msg.data}")
    
    def move_to_ball(self, ball_position, color_name):
        self.get_logger().info(f"→ Moving to {color_name} ball...")
        
        robot_pos = self.camera_to_robot_frame(ball_position)
        joint_angles = self.compute_ik(robot_pos)
        
        if joint_angles is None:
            return False
        
        self.send_joint_command(joint_angles)
        
        target_msg = String()
        target_msg.data = color_name
        self.target_pub.publish(target_msg)
        
        return True
    
    def move_to_initial_callback(self, request, response):
        self.get_logger().info("→ Moving to INITIAL position")
        self.send_joint_command(self.initial_joints)
        self.last_q = self.initial_joints
        response.success = True
        response.message = "Moved to initial position"
        return response
    
    def move_to_first_callback(self, request, response):
        if self.ball_sequence == 'green_first':
            if self.green_ball_pos is None:
                response.success = False
                response.message = "Green ball not detected!"
                self.get_logger().error("Green ball not detected")
                return response
            success = self.move_to_ball(self.green_ball_pos, 'green')
        else:  # orange_first
            if self.orange_ball_pos is None:
                response.success = False
                response.message = "Orange ball not detected!"
                self.get_logger().error("Orange ball not detected")
                return response
            success = self.move_to_ball(self.orange_ball_pos, 'orange')
        
        response.success = success
        response.message = "Moving to first ball" if success else "Failed to compute IK"
        return response
    
    def move_to_second_callback(self, request, response):
        if self.ball_sequence == 'green_first':
            if self.orange_ball_pos is None:
                response.success = False
                response.message = "Orange ball not detected!"
                self.get_logger().error("Orange ball not detected")
                return response
            success = self.move_to_ball(self.orange_ball_pos, 'orange')
        else:  # orange_first
            if self.green_ball_pos is None:
                response.success = False
                response.message = "Green ball not detected!"
                self.get_logger().error("Green ball not detected")
                return response
            success = self.move_to_ball(self.green_ball_pos, 'green')
        
        response.success = success
        response.message = "Moving to second ball" if success else "Failed to compute IK"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()