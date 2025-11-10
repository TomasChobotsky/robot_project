#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from adatools import config_generator as cg
from adatools import utils as utils
from spatialmath import SE3
import numpy as np
import time

class PrecisionChallenge(Node):
    def __init__(self):
        super().__init__("precision_challenge")
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(Int32MultiArray, 'dxl_joint_cmd', 10)
        
        # Robot configuration
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.155, link1_offset=0.0,
            link2=0.250, link2_offset=0.0,
            link3=0.250, link3_offset=0.0,
            link4=0.100, link4_offset=0.0
        )
        
        # Home pose (define this once, can't change after target is announced)
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
        
        # Pen offset from end effector (measure this carefully!)
        self.declare_parameter('pen_z_offset', 0.0)  # pen tip below end effector
        self.pen_offset = self.get_parameter('pen_z_offset').value
        
        # Movement parameters
        self.declare_parameter('move_duration', 3.0)  # seconds to reach target
        self.declare_parameter('marking_duration', 1.0)  # seconds to hold for marking
        
        self.move_time = self.get_parameter('move_duration').value
        self.mark_time = self.get_parameter('marking_duration').value
        
        self.get_logger().info("=== Precision Challenge Node Started ===")
        self.get_logger().info(f"Home joints: {self.home_joints}")
        self.get_logger().info(f"Pen offset: {self.pen_offset}m")
    
    def compute_ik(self, target_x, target_y, target_z):
        """
        Compute inverse kinematics for target position
        Returns: joint angles in radians or None if failed
        """
        # Adjust Z for pen offset
        z_adjusted = target_z - self.pen_offset
        
        # Create target pose (pen pointing down)
        Tgoal = SE3(target_x, target_y, z_adjusted) * SE3.Rx(180, 'deg')
        
        try:
            sol = self.my_conf_robot.ikine_LM(
                Tgoal, 
                q0=self.home_joints,  # Start from home position
                mask=[1, 1, 1, 0.2, 0.2, 0.3]
            )
            
            if sol.success:
                T_actual = self.my_conf_robot.fkine(sol.q)
                trans_error = np.linalg.norm(Tgoal.t - T_actual.t)
                
                if trans_error < 0.02:  # 2cm tolerance
                    self.get_logger().info(f"IK success! Error: {trans_error*1000:.2f}mm")
                    return sol.q
                else:
                    self.get_logger().warn(f"IK inaccurate: {trans_error*1000:.2f}mm")
                    return None
            else:
                self.get_logger().error("IK failed to converge")
                return None
                
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            return None
    
    def send_joint_command(self, joint_angles):
        """Send joint command to robot"""
        msg = Int32MultiArray()
        msg.data = [
            utils.rad2steps(joint_angles[0]),
            utils.rad2steps(joint_angles[1]),
            utils.rad2steps(joint_angles[2]),
            utils.rad2steps(joint_angles[3])
        ]
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")
    
    def go_home(self):
        """Move to home position"""
        self.get_logger().info("→ Moving to HOME position")
        self.send_joint_command(self.home_joints)
        time.sleep(self.move_time)
        self.get_logger().info("✓ HOME position reached")
    
    def mark_target(self, target_x, target_y, target_z, trial_number):
        """
        Execute one trial: HOME → TARGET → MARK → HOME
        """
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"TRIAL {trial_number}")
        self.get_logger().info(f"Target: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")
        self.get_logger().info(f"{'='*50}")
        
        # 1. Ensure we're at home
        self.go_home()
        
        # 2. Compute IK for target
        self.get_logger().info("→ Computing IK...")
        joint_angles = self.compute_ik(target_x, target_y, target_z)
        
        if joint_angles is None:
            self.get_logger().error("✗ FAILED: Cannot reach target")
            return False
        
        # 3. Move to target
        self.get_logger().info("→ Moving to TARGET")
        self.send_joint_command(joint_angles)
        time.sleep(self.move_time)
        
        # 4. Hold for marking
        self.get_logger().info(f"→ Marking (holding {self.mark_time}s)...")
        time.sleep(self.mark_time)
        self.get_logger().info("✓ Mark complete")
        
        # 5. Return home
        self.go_home()
        
        self.get_logger().info(f"✓ TRIAL {trial_number} COMPLETE\n")
        return True
    
    def run_challenge(self, target_x, target_y, target_z, num_trials=5):
        """
        Run the full precision challenge: 5 trials to same target
        """
        self.get_logger().info(f"\n{'#'*50}")
        self.get_logger().info(f"STARTING PRECISION CHALLENGE")
        self.get_logger().info(f"Target: X={target_x:.4f}m, Y={target_y:.4f}m, Z={target_z:.4f}m")
        self.get_logger().info(f"Trials: {num_trials}")
        self.get_logger().info(f"{'#'*50}\n")
        
        # Initial home position
        self.get_logger().info("Initial setup...")
        self.go_home()
        
        input("\nPress ENTER to start challenge...")
        
        success_count = 0
        
        for trial in range(1, num_trials + 1):
            input(f"\nTrial {trial}/{num_trials} - Add/remove payload if needed, then press ENTER...")
            
            if self.mark_target(target_x, target_y, target_z, trial):
                success_count += 1
            else:
                self.get_logger().error(f"Trial {trial} FAILED!")
        
        # Final summary
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"CHALLENGE COMPLETE")
        self.get_logger().info(f"Success: {success_count}/{num_trials} trials")
        self.get_logger().info(f"{'='*50}")
        self.get_logger().info("\nNow measure the 5 dots on paper:")
        self.get_logger().info("1. Measure distance from each dot to target center")
        self.get_logger().info("2. Calculate mean distance and standard deviation")
        self.get_logger().info("3. Score = f(mean_distance, std_deviation)")
        
        return success_count == num_trials


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionChallenge()
    
    # Wait for target coordinates
    print("\n" + "="*60)
    print("PRECISION CHALLENGE SETUP")
    print("="*60)
    print("Enter target coordinates (in robot base frame):")
    
    target_x = float(input("Target X (meters, forward): "))
    target_y = float(input("Target Y (meters, left): "))
    target_z = float(input("Target Z (meters, up): "))
    
    # Run the challenge
    try:
        node.run_challenge(target_x, target_y, target_z, num_trials=5)
    except KeyboardInterrupt:
        print("\nChallenge interrupted!")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()