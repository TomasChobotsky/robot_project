#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool
from adatools import config_generator as cg
from adatools import utils as utils
from spatialmath import SE3
import numpy as np
import time

class PrecisionTestService(Node):
    def __init__(self):
        super().__init__("precision_test_service")
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(Int32MultiArray, 'dxl_joint_cmd', 10)
        
        # Services for step-by-step control
        self.go_home_srv = self.create_service(Trigger, 'precision_go_home', self.go_home_callback)
        self.execute_trial_srv = self.create_service(Trigger, 'precision_execute_trial', self.execute_trial_callback)
        
        # Robot configuration
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.200, link1_offset=0.0,
            link2=0.330, link2_offset=0.0,
            link3=0.335, link3_offset=0.0,
            link4=0.080, link4_offset=0.0
        )
        
        # Parameters
        self.declare_parameter('home_q1', 0.0)
        self.declare_parameter('home_q2', 0.0)
        self.declare_parameter('home_q3', 0.0)
        self.declare_parameter('home_q4', 0.0)
        
        self.home_joints = np.array([
            self.get_parameter('home_q1').value,
            self.get_parameter('home_q2').value,
            self.get_parameter('home_q3').value,
            self.get_parameter('home_q4').value
        ])
        
        # Target position (will be updated via parameters)
        self.declare_parameter('target_x', 0.3)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.1)
        
        # Pen offset from end effector
        self.declare_parameter('pen_z_offset', 0.0)
        self.pen_offset = self.get_parameter('pen_z_offset').value
        
        # Movement parameters
        self.declare_parameter('move_duration', 3.0)
        self.declare_parameter('marking_duration', 1.0)
        
        self.move_time = self.get_parameter('move_duration').value
        self.mark_time = self.get_parameter('marking_duration').value
        
        self.trial_count = 0
        
        self.get_logger().info("=== Precision Test Service Started ===")
        self.get_logger().info(f"Home joints: {self.home_joints}")
        self.get_logger().info("Ready for service calls")
    
    def compute_ik(self, target_x, target_y, target_z):
        # Adjust Z for pen offset
        z_adjusted = target_z - self.pen_offset
        
        # End effector pointing down (use Ry for proper orientation)
        Tgoal = SE3(target_x, target_y, z_adjusted) * SE3.Ry(180, 'deg')
        
        try:
            sol = self.my_conf_robot.ikine_LM(
                Tgoal, 
                q0=self.home_joints,
                mask=[1, 1, 1, 1, 1, 1],
                ilimit=1000,
                slimit=200,
            )
            
            if sol.success:
                T_actual = self.my_conf_robot.fkine(sol.q)
                trans_error = np.linalg.norm(Tgoal.t - T_actual.t)
                
                # Check orientation
                z_desired = Tgoal.R[:3, 2]
                z_actual = T_actual.R[:3, 2]
                orientation_error = np.arccos(np.clip(np.dot(z_desired, z_actual), -1, 1))
                orientation_error_deg = np.degrees(orientation_error)
                
                self.get_logger().info(f"IK: pos_err={trans_error*1000:.1f}mm, "
                                      f"orient_err={orientation_error_deg:.1f}°")
                
                if trans_error < 0.02 and orientation_error_deg < 15:
                    return sol.q
                else:
                    self.get_logger().warn("IK solution not accurate enough")
                    return None
            else:
                self.get_logger().error("IK failed to converge")
                return None
                
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
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
        self.get_logger().info(f"Sent: {[f'{d:.1f}°' for d in joint_degrees]}")
    
    def go_home_callback(self, request, response):
        self.get_logger().info("→ Moving to HOME position")
        self.send_joint_command(self.home_joints)
        
        # Wait for movement to complete
        time.sleep(self.move_time)
        
        response.success = True
        response.message = "Reached home position"
        return response
    
    def execute_trial_callback(self, request, response):
        self.trial_count += 1
        
        # Get current target from parameters
        target_x = self.get_parameter('target_x').value
        target_y = self.get_parameter('target_y').value
        target_z = self.get_parameter('target_z').value
        
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"TRIAL {self.trial_count}")
        self.get_logger().info(f"Target: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")
        self.get_logger().info(f"{'='*50}")
        
        # 1. Ensure at home
        self.get_logger().info("→ Starting from HOME")
        self.send_joint_command(self.home_joints)
        time.sleep(self.move_time)
        
        # 2. Compute IK
        self.get_logger().info("→ Computing IK...")
        joint_angles = self.compute_ik(target_x, target_y, target_z)
        
        if joint_angles is None:
            response.success = False
            response.message = f"Trial {self.trial_count} FAILED: Cannot reach target"
            self.get_logger().error("✗ FAILED: Cannot reach target")
            return response
        
        # 3. Move to target
        self.get_logger().info("→ Moving to TARGET")
        self.send_joint_command(joint_angles)
        time.sleep(self.move_time)
        
        # 4. Hold for marking
        self.get_logger().info(f"→ Marking (holding {self.mark_time}s)...")
        time.sleep(self.mark_time)
        self.get_logger().info("✓ Mark complete")
        
        # 5. Return home
        self.get_logger().info("→ Returning to HOME")
        self.send_joint_command(self.home_joints)
        time.sleep(self.move_time)
        
        self.get_logger().info(f"✓ TRIAL {self.trial_count} COMPLETE\n")
        
        response.success = True
        response.message = f"Trial {self.trial_count} completed successfully"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionTestService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()