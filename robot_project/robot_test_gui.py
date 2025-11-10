#!/usr/bin/env python3
"""
Robot Test Suite GUI Application
Integrates all robot testing functionality with ROS2
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys
import os
import subprocess
import signal
import time
from datetime import datetime

# Import the UI class (generated from Qt Designer)
from .main_ui import Ui_RobotTestSuite


class RobotTestGUI(QMainWindow, Ui_RobotTestSuite):
    def __init__(self, parent=None):
        super(RobotTestGUI, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle("Robot Test UI")
        
        # Process tracking
        self.system_process = None
        self.ball_tracking_process = None
        self.precision_test_process = None
        self.weight_test_process = None
        self.visualizer_process = None
        self.joint_cmd_process = None
        
        # Test state
        self.precision_test_running = False
        self.precision_current_trial = 0
        self.weight_test_running = False
        
        # Connect system control signals
        self.startSystemBtn.clicked.connect(self.start_complete_system)
        self.stopSystemBtn.clicked.connect(self.stop_all_nodes)
        
        # Connect ball tracking signals
        self.startBallTrackingBtn.clicked.connect(self.start_ball_tracking)
        self.stopBallTrackingBtn.clicked.connect(self.stop_ball_tracking)
        
        # Connect precision test signals
        self.startPrecisionBtn.clicked.connect(self.start_precision_test)
        self.nextTrialBtn.clicked.connect(self.next_precision_trial)
        self.stopPrecisionBtn.clicked.connect(self.stop_precision_test)
        
        # Connect weight test signals
        self.startWeightBtn.clicked.connect(self.start_weight_test)
        self.moveToPositionBtn.clicked.connect(self.move_to_weight_position)
        self.stopWeightBtn.clicked.connect(self.stop_weight_test)
        
        # Connect manual control signals
        self.sendJointCmdBtn.clicked.connect(self.send_joint_command)
        self.goHomeBtn.clicked.connect(self.go_to_home)
        self.resetJointsBtn.clicked.connect(self.reset_joints)
        
        # Connect sliders to labels
        self.joint1Slider.valueChanged.connect(lambda v: self.joint1Label.setText(f"{v/100:.2f} rad"))
        self.joint2Slider.valueChanged.connect(lambda v: self.joint2Label.setText(f"{v/100:.2f} rad"))
        self.joint3Slider.valueChanged.connect(lambda v: self.joint3Label.setText(f"{v/100:.2f} rad"))
        self.joint4Slider.valueChanged.connect(lambda v: self.joint4Label.setText(f"{v/100:.2f} rad"))
        
        self.log_message("GUI initialized. Ready to start testing.", "system")
        self.show()
    
    # ==================== UTILITY FUNCTIONS ====================
    
    def log_message(self, message, target="system"):
        """Log message to appropriate text widget"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {message}\n"
        
        if target == "system":
            self.systemStatusLabel.setText(f"Status: {message}")
        elif target == "ball":
            self.ballStatusText.appendPlainText(formatted_msg)
        elif target == "precision":
            self.precisionStatusText.appendPlainText(formatted_msg)
        elif target == "weight":
            self.weightStatusText.appendPlainText(formatted_msg)
        elif target == "manual":
            self.manualStatusText.appendPlainText(formatted_msg)
        
        QApplication.processEvents()
    
    def kill_process(self, process, name="process"):
        """Safely kill a subprocess"""
        if process is not None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                time.sleep(0.5)
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                self.log_message(f"{name} stopped", "system")
                return True
            except Exception as e:
                self.log_message(f"Error stopping {name}: {e}", "system")
                return False
        return True
    
    # ==================== SYSTEM CONTROL ====================
    
    def start_complete_system(self):
        """Start the complete ball tracking system"""
        if self.system_process is not None:
            self.log_message("System already running!", "system")
            return
        
        self.log_message("Starting complete system...", "system")
        
        try:
            # Build launch arguments
            launch_args = [
                "ros2", "launch", "robot_project", "complete_system.launch.py",
                f"enable_visualization:={str(self.enableVizCheckBox.isChecked()).lower()}",
                f"processing_rate:={self.processingRateSpin.value()}",
                f"target_mode:={self.targetModeCombo.currentText()}",
                f"control_mode:={self.controlModeCombo.currentText()}",
                f"camera_x_offset:={self.cameraXSpin.value()}",
                f"camera_y_offset:={self.cameraYSpin.value()}",
                f"camera_z_offset:={self.cameraZSpin.value()}",
                f"camera_rotation:={self.cameraRotSpin.value()}"
            ]
            
            self.system_process = subprocess.Popen(
                launch_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )
            
            self.log_message("Complete system started successfully!", "system")
            self.startSystemBtn.setEnabled(False)
            self.stopSystemBtn.setEnabled(True)
            
        except Exception as e:
            self.log_message(f"Failed to start system: {e}", "system")
    
    def stop_all_nodes(self):
        """Stop all running ROS nodes"""
        self.log_message("Stopping all nodes...", "system")
        
        self.kill_process(self.system_process, "Complete system")
        self.kill_process(self.ball_tracking_process, "Ball tracking")
        self.kill_process(self.precision_test_process, "Precision test")
        self.kill_process(self.weight_test_process, "Weight test")
        
        self.system_process = None
        self.ball_tracking_process = None
        self.precision_test_process = None
        self.weight_test_process = None
        
        self.startSystemBtn.setEnabled(True)
        self.stopSystemBtn.setEnabled(False)
        
        self.log_message("All nodes stopped", "system")
    
    # ==================== BALL TRACKING ====================
    
    def start_ball_tracking(self):
        """Start ball tracking nodes individually"""
        if self.ball_tracking_process is not None:
            self.log_message("Ball tracking already running!", "ball")
            return
        
        self.log_message("Starting ball tracking system...", "ball")
        
        try:
            launch_args = [
                "ros2", "launch", "robot_project", "complete_system.launch.py",
                f"enable_visualization:={str(self.enableVizCheckBox.isChecked()).lower()}",
                f"processing_rate:={self.processingRateSpin.value()}",
                f"target_mode:={self.targetModeCombo.currentText()}",
                f"control_mode:={self.controlModeCombo.currentText()}",
                f"camera_x_offset:={self.cameraXSpin.value()}",
                f"camera_y_offset:={self.cameraYSpin.value()}",
                f"camera_z_offset:={self.cameraZSpin.value()}",
                f"camera_rotation:={self.cameraRotSpin.value()}"
            ]
            
            self.ball_tracking_process = subprocess.Popen(
                launch_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )
            
            self.log_message("Ball tracking started!", "ball")
            self.startBallTrackingBtn.setEnabled(False)
            self.stopBallTrackingBtn.setEnabled(True)
            
        except Exception as e:
            self.log_message(f"Failed to start ball tracking: {e}", "ball")
    
    def stop_ball_tracking(self):
        """Stop ball tracking"""
        self.log_message("Stopping ball tracking...", "ball")
        self.kill_process(self.ball_tracking_process, "Ball tracking")
        self.ball_tracking_process = None
        self.startBallTrackingBtn.setEnabled(True)
        self.stopBallTrackingBtn.setEnabled(False)
    
    # ==================== PRECISION TEST ====================
    
    def start_precision_test(self):
        """Start precision test"""
        if self.precision_test_running:
            self.log_message("Precision test already running!", "precision")
            return
        
        target_x = self.targetXSpin.value()
        target_y = self.targetYSpin.value()
        target_z = self.targetZSpin.value()
        num_trials = self.numTrialsSpin.value()
        
        self.log_message("="*50, "precision")
        self.log_message("STARTING PRECISION TEST", "precision")
        self.log_message(f"Target: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}", "precision")
        self.log_message(f"Number of trials: {num_trials}", "precision")
        self.log_message("="*50, "precision")
        
        # Start precision test node with parameters
        try:
            # Note: This uses a service-based approach for interactive control
            # You'll need to modify precision_test.py to support this
            self.precision_test_running = True
            self.precision_current_trial = 0
            
            self.startPrecisionBtn.setEnabled(False)
            self.nextTrialBtn.setEnabled(True)
            self.stopPrecisionBtn.setEnabled(True)
            
            self.log_message("Click 'Next Trial' to start each trial", "precision")
            
        except Exception as e:
            self.log_message(f"Failed to start precision test: {e}", "precision")
    
    def next_precision_trial(self):
        """Execute next trial in precision test"""
        if not self.precision_test_running:
            return
        
        self.precision_current_trial += 1
        num_trials = self.numTrialsSpin.value()
        
        if self.precision_current_trial > num_trials:
            self.log_message("All trials completed!", "precision")
            self.stop_precision_test()
            return
        
        self.log_message(f"\n--- TRIAL {self.precision_current_trial}/{num_trials} ---", "precision")
        
        # Send command to precision test node
        try:
            cmd = [
                "ros2", "run", "robot_project", "precision_test",
                "--ros-args",
                "-p", f"target_x:={self.targetXSpin.value()}",
                "-p", f"target_y:={self.targetYSpin.value()}",
                "-p", f"target_z:={self.targetZSpin.value()}",
                "-p", f"move_duration:={self.moveDurationSpin.value()}",
                "-p", f"marking_duration:={self.markDurationSpin.value()}",
                "-p", f"pen_z_offset:={self.penOffsetSpin.value()}",
                "-p", f"home_q1:={self.homeQ1Spin.value()}",
                "-p", f"home_q2:={self.homeQ2Spin.value()}",
                "-p", f"home_q3:={self.homeQ3Spin.value()}",
                "-p", f"home_q4:={self.homeQ4Spin.value()}"
            ]
            
            # Run synchronously for this trial
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                self.log_message(f"Trial {self.precision_current_trial} completed successfully", "precision")
            else:
                self.log_message(f"Trial {self.precision_current_trial} failed!", "precision")
                self.log_message(result.stderr, "precision")
            
        except Exception as e:
            self.log_message(f"Error in trial {self.precision_current_trial}: {e}", "precision")
    
    def stop_precision_test(self):
        """Stop precision test"""
        self.precision_test_running = False
        self.precision_current_trial = 0
        
        self.startPrecisionBtn.setEnabled(True)
        self.nextTrialBtn.setEnabled(False)
        self.stopPrecisionBtn.setEnabled(False)
        
        self.log_message("Precision test stopped", "precision")
    
    # ==================== WEIGHT TEST ====================
    
    def start_weight_test(self):
        """Start weight test"""
        self.log_message("Starting weight test...", "weight")
        self.log_message(f"Current payload: {self.weightValueSpin.value():.2f} kg", "weight")
        self.log_message(f"Trials: {self.weightNumTrialsSpin.value()}", "weight")
        
        self.weight_test_running = True
        self.startWeightBtn.setEnabled(False)
        self.stopWeightBtn.setEnabled(True)
        
        # For now, just move to position - you'll implement the full test logic
        self.move_to_weight_position()
    
    def move_to_weight_position(self):
        """Move to specified weight test position"""
        x = self.weightPosXSpin.value()
        y = self.weightPosYSpin.value()
        z = self.weightPosZSpin.value()
        
        self.log_message(f"Moving to position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}", "weight")
        
        # TODO: Implement weight test movement command
        # This would send a command to a weight test node or service
        
    def stop_weight_test(self):
        """Stop weight test"""
        self.weight_test_running = False
        self.startWeightBtn.setEnabled(True)
        self.stopWeightBtn.setEnabled(False)
        self.log_message("Weight test stopped", "weight")
    
    # ==================== MANUAL CONTROL ====================
    
    def send_joint_command(self):
        """Send joint command from sliders"""
        j1 = self.joint1Slider.value() / 100.0
        j2 = self.joint2Slider.value() / 100.0
        j3 = self.joint3Slider.value() / 100.0
        j4 = self.joint4Slider.value() / 100.0
        
        self.log_message(f"Sending joint command: [{j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}]", "manual")
        
        # TODO: Implement joint command publishing via service or topic
        # For now, we'll use ros2 topic pub
        try:
            # Convert radians to steps (assuming utils.rad2steps conversion)
            # You might need to adjust this based on your robot's conversion
            cmd = [
                "ros2", "topic", "pub", "--once", "/dxl_joint_cmd",
                "std_msgs/msg/Int32MultiArray",
                f"{{data: [{int(j1*651.74)}, {int(j2*651.74)}, {int(j3*651.74)}, {int(j4*651.74)}]}}"
            ]
            
            subprocess.run(cmd, capture_output=True, text=True)
            self.log_message("Command sent!", "manual")
            
        except Exception as e:
            self.log_message(f"Failed to send command: {e}", "manual")
    
    def go_to_home(self):
        """Move to home position"""
        home_q = [
            self.homeQ1Spin.value(),
            self.homeQ2Spin.value(),
            self.homeQ3Spin.value(),
            self.homeQ4Spin.value()
        ]
        
        self.log_message(f"Moving to home: {home_q}", "manual")
        
        # Update sliders
        self.joint1Slider.setValue(int(home_q[0] * 100))
        self.joint2Slider.setValue(int(home_q[1] * 100))
        self.joint3Slider.setValue(int(home_q[2] * 100))
        self.joint4Slider.setValue(int(home_q[3] * 100))
        
        # Send command
        self.send_joint_command()
    
    def reset_joints(self):
        """Reset all joints to zero"""
        self.log_message("Resetting joints to zero", "manual")
        self.joint1Slider.setValue(0)
        self.joint2Slider.setValue(0)
        self.joint3Slider.setValue(0)
        self.joint4Slider.setValue(0)
    
    # ==================== CLEANUP ====================
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.log_message("Shutting down...", "system")
        
        # Stop all processes
        self.kill_process(self.system_process, "System")
        self.kill_process(self.ball_tracking_process, "Ball tracking")
        self.kill_process(self.precision_test_process, "Precision test")
        self.kill_process(self.weight_test_process, "Weight test")
        
        time.sleep(1)
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show GUI
    window = RobotTestGUI()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()