#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32MultiArray, String
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
        
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.200, link1_offset=0.0,
            link2=0.270, link2_offset=0.0,
            link3=0.275, link3_offset=0.0,
            link4=0.240, link4_offset=0.0
        )
        
        self.green_ball_pos = None
        self.orange_ball_pos = None

        self.sequence_state = 0
        self.sequence_done = False

        self._seq_timer = None
        self._attempted_green = False
        self._attempted_orange = False


        # Camera to robot base transform
        self.declare_parameter('camera_x_offset', 1.0)  # meters
        self.declare_parameter('camera_y_offset', 0.0)  # meters
        self.declare_parameter('camera_z_offset', 0.2)  # meters
        self.declare_parameter('camera_rotation', 180.0)  # degrees around Z
        
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
        
        self.create_timer(1.0, self.auto_track_callback)
        
        self.last_target = None
        
        self.get_logger().info(f"Ball Tracking Controller Started")
        self.get_logger().info(f"Camera offset: x={self.cam_x}, y={self.cam_y}, z={self.cam_z}")
    
    def green_ball_callback(self, msg):
        """Store or clear green ball position"""
        if msg.point.x == 0.0 and msg.point.y == 0.0 and msg.point.z == 0.0:
            if self.green_ball_pos is not None:
                self.get_logger().info("Green ball lost")
            self.green_ball_pos = None
        else:
            self.green_ball_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.get_logger().debug(f"Green ball updated: {self.green_ball_pos}")

    def orange_ball_callback(self, msg):
        """Store or clear orange ball position"""
        if msg.point.x == 0.0 and msg.point.y == 0.0 and msg.point.z == 0.0:
            if self.orange_ball_pos is not None:
                self.get_logger().info("Orange ball lost")
            self.orange_ball_pos = None
        else:
            self.orange_ball_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.get_logger().debug(f"Orange ball updated: {self.orange_ball_pos}")
    
    def camera_to_robot_frame(self, camera_pos):
        """
        Transform position from camera frame to robot base frame
        Camera frame: X right, Y down, Z forward
        Robot frame: X forward, Y left, Z up
        
        Applies camera position offset and rotation around vertical axis
        """
        
        # Camera coordinates
        x_cam, y_cam, z_cam = camera_pos
        
        # Convert camera rotation from degrees to radians
        theta = np.radians(self.cam_rot)
        
        x_std = z_cam      # Camera forward (Z) = standard forward (X)
        y_std = -x_cam     # Camera right (X) = standard left (-Y)
        z_std = -y_cam     # Camera down (Y) = standard up (-Z)
        
        # Rotation matrix around Z-axis:
        # [cos(θ)  -sin(θ)   0]
        # [sin(θ)   cos(θ)   0]
        # [  0        0      1]
        x_rotated = x_std * np.cos(theta) - y_std * np.sin(theta)
        y_rotated = x_std * np.sin(theta) + y_std * np.cos(theta)
        z_rotated = z_std
        
        # Step 3: Add camera mounting position offset
        x_robot = self.cam_x + x_rotated
        y_robot = self.cam_y + y_rotated
        z_robot = self.cam_z + z_rotated

        self.get_logger().info(f"Camera pos: [{x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}]")
        self.get_logger().info(f"Robot pos: [{x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f}]")
        self.get_logger().info(
            f"Cam offset: ({self.cam_x:.3f}, {self.cam_y:.3f}, {self.cam_z:.3f}), "
            f"Rotated: ({x_rotated:.3f}, {y_rotated:.3f}, {z_rotated:.3f})"
        )
    
        return np.array([x_robot, y_robot, z_robot])
    
    def is_in_workspace(self, position):
        """Check if position is within safe workspace"""
        x, y, z = position
        
        if not (self.workspace_x_min <= x <= self.workspace_x_max):
            return False
        if not (self.workspace_y_min <= y <= self.workspace_y_max):
            return False
        if not (self.workspace_z_min <= z <= self.workspace_z_max):
            return False
        
        return True
    
    def compute_ik(self, target_position):
        """
        Compute inverse kinematics for target position
        Returns: joint angles in radians or None if failed
        """
        x, y, z = target_position
        
        # Check workspace limits
        if not self.is_in_workspace([x, y, z]):
            self.get_logger().warn(f"Target position out of workspace: {target_position}")
            return None
        
        # Create target pose
        # End effector pointing down
        Tgoal = SE3(x, y, z) * SE3.Rx(180, 'deg')
        
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

    def auto_track_callback(self):
        """Sequence-only mode: compute IK & move to green once, wait 1s, then compute IK & move to orange once"""

        # Already finished
        if self.sequence_done:
            return

        # STATE 0: attempt green once
        if self.sequence_state == 0 and not self._attempted_green:
            if self.green_ball_pos is None:
                # still waiting for detection
                return

            self._attempted_green = True  # mark we won't attempt green again

            self.get_logger().info("Sequence: preparing IK for GREEN ball...")
            robot_pos = self.camera_to_robot_frame(self.green_ball_pos)

            joint_angles = self.compute_ik(robot_pos)
            if joint_angles is None:
                self.get_logger().error("Sequence: IK failed for GREEN ball — aborting sequence.")
                self.sequence_done = True
                return

            # publish joint command once
            msg = Int32MultiArray()
            msg.data = [
                utils.rad2steps(joint_angles[0]),
                utils.rad2steps(joint_angles[1]),
                utils.rad2steps(joint_angles[2]),
                utils.rad2steps(joint_angles[3]),
                utils.rad2steps(joint_angles[4])
            ]
            self.joint_cmd_pub.publish(msg)
            self.get_logger().info(f"Sequence: published joint command to reach GREEN: {msg.data}")

            # publish target label
            tmsg = String()
            tmsg.data = 'green'
            self.target_pub.publish(tmsg)

            # advance state and schedule single-shot 1s timer for step2
            self.sequence_state = 1
            self.get_logger().info("Reached GREEN (commanded). Waiting 1 second before ORANGE...")
            # create timer and store handle so it can be cancelled inside callback
            self._seq_timer = self.create_timer(1.0, self._step2_once)
            return

        # If we've already attempted green, just wait for timer to fire
        return


    def _step2_once(self):
        """Fires once after 1s delay to attempt orange. Cancels its own timer immediately."""
        # cancel timer so it won't run again
        try:
            if self._seq_timer is not None:
                self._seq_timer.cancel()
                self._seq_timer = None
        except Exception:
            pass

        if self.sequence_state != 1 or self.sequence_done:
            return

        # Only attempt orange once
        if self._attempted_orange:
            return
        self._attempted_orange = True

        if self.orange_ball_pos is None:
            self.get_logger().warn("Sequence: ORANGE ball not detected — sequence ends.")
            self.sequence_done = True
            self.sequence_state = 2
            return

        self.get_logger().info("Sequence: preparing IK for ORANGE ball...")
        robot_pos = self.camera_to_robot_frame(self.orange_ball_pos)

        joint_angles = self.compute_ik(robot_pos)
        if joint_angles is None:
            self.get_logger().error("Sequence: IK failed for ORANGE ball — sequence ends.")
            self.sequence_done = True
            self.sequence_state = 2
            return

        # publish joint command once
        msg = Int32MultiArray()
        msg.data = [
            utils.rad2steps(joint_angles[0]),
            utils.rad2steps(joint_angles[1]),
            utils.rad2steps(joint_angles[2]),
            utils.rad2steps(joint_angles[3]),
            utils.rad2steps(joint_angles[4])
        ]
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(f"Sequence: published joint command to reach ORANGE: {msg.data}")

        # publish target label
        tmsg = String()
        tmsg.data = 'orange'
        self.target_pub.publish(tmsg)

        # Done
        self.sequence_done = True
        self.sequence_state = 2
        self.get_logger().info("Sequence completed. Stopping tracking.")

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