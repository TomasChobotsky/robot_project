#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from adatools import config_generator as cg
from adatools import utils as utils
from spatialmath import SE3
import numpy as np

class PrecisionTestService(Node):
    def __init__(self):
        super().__init__("precision_test_service")
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(Int32MultiArray, 'dxl_joint_cmd', 10)
        
        # Robot configuration
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.200, link1_offset=0.0,
            link2=0.270, link2_offset=0.0,
            link3=0.275, link3_offset=0.0,
            link4=0.240, link4_offset=0.0
        )
        
        # Target position (will be updated via parameters)
        self.declare_parameter('target_x', 0.4)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.1)
        
        self.get_logger().info("=== Precision Test Service Started ===")
        joint_angles = self.compute_ik(0.37, 0.0, 0.04)
        self.send_joint_command(joint_angles)
    
    def compute_ik(self, target_x, target_y, target_z):        
        
        # Create target pose
        # End effector pointing down
        Tgoal = SE3(target_x, target_y, target_z) * SE3.Rx(180, 'deg')
        
        # Compute inverse kinematics
        try:
            sol = self.my_conf_robot.ikine_LM(
                Tgoal, 
                q0=self.my_conf_robot.qr,  # Start from ready position
                mask=[1, 1, 1, 0.5, 0.5, 1.0]  # Prioritize position over orientation
            )
            
            if sol.success:
                # Verify solution with forward kinematics
                T_actual = self.my_conf_robot.fkine(sol.q)
                trans_error = np.linalg.norm(Tgoal.t - T_actual.t)
                
                if trans_error < 0.02:  # 2cm tolerance
                    self.get_logger().info(f"IK successful! Position error: {trans_error*1000:.1f}mm")
                    return sol.q
                else:
                    self.get_logger().warn(f"IK solution inaccurate: error={trans_error*1000:.1f}mm")
                    return None
            else:
                self.get_logger().warn("IK failed to converge")
                return None
                
        except Exception as e:
            self.get_logger().error(f"IK computation error: {e}")
            return None
    
    def send_joint_command(self, joint_angles):
        msg = Int32MultiArray()
        msg.data = [
            utils.rad2steps(joint_angles[0]),
            utils.rad2steps(joint_angles[1]),
            utils.rad2steps(joint_angles[2]),
            utils.rad2steps(joint_angles[3]),
            utils.rad2steps(joint_angles[4])
        ]
        self.joint_cmd_pub.publish(msg)
        
        joint_degrees = [np.degrees(q) for q in joint_angles]
        self.get_logger().info(f"Sent: {[f'{d:.1f}°' for d in joint_degrees]}")


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