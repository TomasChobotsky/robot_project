#!/usr/bin/env python3
"""
Generated UI file for Robot Test Suite
This file should be generated from Qt Designer using:
pyuic5 -x robot_test_suite.ui -o main_ui.py
"""

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_RobotTestSuite(object):
    def setupUi(self, RobotTestSuite):
        RobotTestSuite.setObjectName("RobotTestSuite")
        RobotTestSuite.resize(1200, 800)
        
        # Main layout
        self.centralwidget = QtWidgets.QWidget(RobotTestSuite)
        self.mainLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        
        # Title
        self.titleLabel = QtWidgets.QLabel(self.centralwidget)
        self.titleLabel.setObjectName("titleLabel")
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.titleLabel.setFont(font)
        self.titleLabel.setText("Robot Test Suite Control Panel")
        self.titleLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.mainLayout.addWidget(self.titleLabel)
        
        # ============ SYSTEM CONTROL SECTION ============
        self.systemGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.systemGroupBox.setTitle("System Control")
        self.systemLayout = QtWidgets.QHBoxLayout(self.systemGroupBox)
        
        self.startSystemBtn = QtWidgets.QPushButton(self.systemGroupBox)
        self.startSystemBtn.setObjectName("startSystemBtn")
        self.startSystemBtn.setText("Start Complete System")
        self.startSystemBtn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.systemLayout.addWidget(self.startSystemBtn)
        
        self.stopSystemBtn = QtWidgets.QPushButton(self.systemGroupBox)
        self.stopSystemBtn.setObjectName("stopSystemBtn")
        self.stopSystemBtn.setText("Stop All Nodes")
        self.stopSystemBtn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
        self.systemLayout.addWidget(self.stopSystemBtn)
        
        self.systemStatusLabel = QtWidgets.QLabel(self.systemGroupBox)
        self.systemStatusLabel.setObjectName("systemStatusLabel")
        self.systemStatusLabel.setText("Status: Idle")
        self.systemStatusLabel.setStyleSheet("padding: 10px; background-color: #f0f0f0;")
        self.systemLayout.addWidget(self.systemStatusLabel)
        
        self.mainLayout.addWidget(self.systemGroupBox)
        
        # ============ TABBED INTERFACE ============
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        
        # --- TAB 1: Ball Tracking Test ---
        self.ballTrackingTab = QtWidgets.QWidget()
        self.ballTrackingLayout = QtWidgets.QVBoxLayout(self.ballTrackingTab)
        
        # Ball tracking parameters
        self.ballParamsGroupBox = QtWidgets.QGroupBox(self.ballTrackingTab)
        self.ballParamsGroupBox.setTitle("Ball Tracking Parameters")
        self.ballParamsLayout = QtWidgets.QFormLayout(self.ballParamsGroupBox)
        
        self.targetModeCombo = QtWidgets.QComboBox(self.ballParamsGroupBox)
        self.targetModeCombo.addItems(["closest", "green", "orange", "alternate"])
        self.ballParamsLayout.addRow("Target Mode:", self.targetModeCombo)
        
        self.controlModeCombo = QtWidgets.QComboBox(self.ballParamsGroupBox)
        self.controlModeCombo.addItems(["auto", "manual"])
        self.ballParamsLayout.addRow("Control Mode:", self.controlModeCombo)
        
        self.processingRateSpin = QtWidgets.QDoubleSpinBox(self.ballParamsGroupBox)
        self.processingRateSpin.setRange(1.0, 30.0)
        self.processingRateSpin.setValue(5.0)
        self.processingRateSpin.setSuffix(" Hz")
        self.ballParamsLayout.addRow("Processing Rate:", self.processingRateSpin)
        
        self.enableVizCheckBox = QtWidgets.QCheckBox(self.ballParamsGroupBox)
        self.enableVizCheckBox.setChecked(True)
        self.ballParamsLayout.addRow("Enable Visualization:", self.enableVizCheckBox)
        
        self.ballTrackingLayout.addWidget(self.ballParamsGroupBox)
        
        # Camera calibration
        self.cameraCalibGroupBox = QtWidgets.QGroupBox(self.ballTrackingTab)
        self.cameraCalibGroupBox.setTitle("Camera Position (meters)")
        self.cameraCalibLayout = QtWidgets.QFormLayout(self.cameraCalibGroupBox)
        
        self.cameraXSpin = QtWidgets.QDoubleSpinBox(self.cameraCalibGroupBox)
        self.cameraXSpin.setRange(-2.0, 2.0)
        self.cameraXSpin.setValue(1.0)
        self.cameraXSpin.setSingleStep(0.01)
        self.cameraCalibLayout.addRow("X Offset (forward):", self.cameraXSpin)
        
        self.cameraYSpin = QtWidgets.QDoubleSpinBox(self.cameraCalibGroupBox)
        self.cameraYSpin.setRange(-2.0, 2.0)
        self.cameraYSpin.setValue(0.0)
        self.cameraYSpin.setSingleStep(0.01)
        self.cameraCalibLayout.addRow("Y Offset (left):", self.cameraYSpin)
        
        self.cameraZSpin = QtWidgets.QDoubleSpinBox(self.cameraCalibGroupBox)
        self.cameraZSpin.setRange(-2.0, 2.0)
        self.cameraZSpin.setValue(0.1)
        self.cameraZSpin.setSingleStep(0.01)
        self.cameraCalibLayout.addRow("Z Offset (up):", self.cameraZSpin)
        
        self.cameraRotSpin = QtWidgets.QDoubleSpinBox(self.cameraCalibGroupBox)
        self.cameraRotSpin.setRange(-180.0, 180.0)
        self.cameraRotSpin.setValue(180.0)
        self.cameraRotSpin.setSuffix(" °")
        self.cameraCalibLayout.addRow("Rotation (Z-axis):", self.cameraRotSpin)
        
        self.ballTrackingLayout.addWidget(self.cameraCalibGroupBox)
        
        # Ball tracking control buttons
        self.ballControlLayout = QtWidgets.QHBoxLayout()
        
        self.startBallTrackingBtn = QtWidgets.QPushButton(self.ballTrackingTab)
        self.startBallTrackingBtn.setText("Start Ball Tracking")
        self.startBallTrackingBtn.setStyleSheet("background-color: #2196F3; color: white; padding: 10px;")
        self.ballControlLayout.addWidget(self.startBallTrackingBtn)
        
        self.stopBallTrackingBtn = QtWidgets.QPushButton(self.ballTrackingTab)
        self.stopBallTrackingBtn.setText("Stop Ball Tracking")
        self.stopBallTrackingBtn.setStyleSheet("background-color: #FF9800; color: white; padding: 10px;")
        self.ballControlLayout.addWidget(self.stopBallTrackingBtn)
        
        self.ballTrackingLayout.addLayout(self.ballControlLayout)
        
        # Ball tracking status
        self.ballStatusText = QtWidgets.QPlainTextEdit(self.ballTrackingTab)
        self.ballStatusText.setReadOnly(True)
        self.ballStatusText.setMaximumHeight(150)
        self.ballStatusText.setPlaceholderText("Ball tracking status will appear here...")
        self.ballTrackingLayout.addWidget(self.ballStatusText)
        
        self.ballTrackingLayout.addStretch()
        self.tabWidget.addTab(self.ballTrackingTab, "Ball Tracking Test")
        
        # --- TAB 2: Precision Test ---
        self.precisionTab = QtWidgets.QWidget()
        self.precisionLayout = QtWidgets.QVBoxLayout(self.precisionTab)
        
        # Target coordinates
        self.targetGroupBox = QtWidgets.QGroupBox(self.precisionTab)
        self.targetGroupBox.setTitle("Target Coordinates (meters)")
        self.targetLayout = QtWidgets.QFormLayout(self.targetGroupBox)
        
        self.targetXSpin = QtWidgets.QDoubleSpinBox(self.targetGroupBox)
        self.targetXSpin.setRange(-1.0, 1.0)
        self.targetXSpin.setValue(0.3)
        self.targetXSpin.setSingleStep(0.001)
        self.targetXSpin.setDecimals(4)
        self.targetLayout.addRow("Target X:", self.targetXSpin)
        
        self.targetYSpin = QtWidgets.QDoubleSpinBox(self.targetGroupBox)
        self.targetYSpin.setRange(-1.0, 1.0)
        self.targetYSpin.setValue(0.0)
        self.targetYSpin.setSingleStep(0.001)
        self.targetYSpin.setDecimals(4)
        self.targetLayout.addRow("Target Y:", self.targetYSpin)
        
        self.targetZSpin = QtWidgets.QDoubleSpinBox(self.targetGroupBox)
        self.targetZSpin.setRange(-1.0, 1.0)
        self.targetZSpin.setValue(0.1)
        self.targetZSpin.setSingleStep(0.001)
        self.targetZSpin.setDecimals(4)
        self.targetLayout.addRow("Target Z:", self.targetZSpin)
        
        self.precisionLayout.addWidget(self.targetGroupBox)
        
        # Precision test parameters
        self.precisionParamsGroupBox = QtWidgets.QGroupBox(self.precisionTab)
        self.precisionParamsGroupBox.setTitle("Test Parameters")
        self.precisionParamsLayout = QtWidgets.QFormLayout(self.precisionParamsGroupBox)
        
        self.numTrialsSpin = QtWidgets.QSpinBox(self.precisionParamsGroupBox)
        self.numTrialsSpin.setRange(1, 20)
        self.numTrialsSpin.setValue(5)
        self.precisionParamsLayout.addRow("Number of Trials:", self.numTrialsSpin)
        
        self.moveDurationSpin = QtWidgets.QDoubleSpinBox(self.precisionParamsGroupBox)
        self.moveDurationSpin.setRange(0.5, 10.0)
        self.moveDurationSpin.setValue(3.0)
        self.moveDurationSpin.setSuffix(" s")
        self.precisionParamsLayout.addRow("Move Duration:", self.moveDurationSpin)
        
        self.markDurationSpin = QtWidgets.QDoubleSpinBox(self.precisionParamsGroupBox)
        self.markDurationSpin.setRange(0.1, 5.0)
        self.markDurationSpin.setValue(1.0)
        self.markDurationSpin.setSuffix(" s")
        self.precisionParamsLayout.addRow("Marking Duration:", self.markDurationSpin)
        
        self.penOffsetSpin = QtWidgets.QDoubleSpinBox(self.precisionParamsGroupBox)
        self.penOffsetSpin.setRange(-0.2, 0.2)
        self.penOffsetSpin.setValue(0.0)
        self.penOffsetSpin.setSingleStep(0.001)
        self.penOffsetSpin.setDecimals(4)
        self.precisionParamsLayout.addRow("Pen Z Offset:", self.penOffsetSpin)
        
        self.precisionLayout.addWidget(self.precisionParamsGroupBox)
        
        # Home position
        self.homeGroupBox = QtWidgets.QGroupBox(self.precisionTab)
        self.homeGroupBox.setTitle("Home Joint Positions (radians)")
        self.homeLayout = QtWidgets.QFormLayout(self.homeGroupBox)
        
        self.homeQ1Spin = QtWidgets.QDoubleSpinBox(self.homeGroupBox)
        self.homeQ1Spin.setRange(-3.14, 3.14)
        self.homeQ1Spin.setValue(0.0)
        self.homeQ1Spin.setSingleStep(0.01)
        self.homeLayout.addRow("Joint 1:", self.homeQ1Spin)
        
        self.homeQ2Spin = QtWidgets.QDoubleSpinBox(self.homeGroupBox)
        self.homeQ2Spin.setRange(-3.14, 3.14)
        self.homeQ2Spin.setValue(0.0)
        self.homeQ2Spin.setSingleStep(0.01)
        self.homeLayout.addRow("Joint 2:", self.homeQ2Spin)
        
        self.homeQ3Spin = QtWidgets.QDoubleSpinBox(self.homeGroupBox)
        self.homeQ3Spin.setRange(-3.14, 3.14)
        self.homeQ3Spin.setValue(0.0)
        self.homeQ3Spin.setSingleStep(0.01)
        self.homeLayout.addRow("Joint 3:", self.homeQ3Spin)
        
        self.homeQ4Spin = QtWidgets.QDoubleSpinBox(self.homeGroupBox)
        self.homeQ4Spin.setRange(-3.14, 3.14)
        self.homeQ4Spin.setValue(0.0)
        self.homeQ4Spin.setSingleStep(0.01)
        self.homeLayout.addRow("Joint 4:", self.homeQ4Spin)
        
        self.precisionLayout.addWidget(self.homeGroupBox)
        
        # Precision test control
        self.precisionControlLayout = QtWidgets.QHBoxLayout()
        
        self.startPrecisionBtn = QtWidgets.QPushButton(self.precisionTab)
        self.startPrecisionBtn.setText("Start Precision Test")
        self.startPrecisionBtn.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px; font-weight: bold;")
        self.precisionControlLayout.addWidget(self.startPrecisionBtn)
        
        self.nextTrialBtn = QtWidgets.QPushButton(self.precisionTab)
        self.nextTrialBtn.setText("Next Trial")
        self.nextTrialBtn.setEnabled(False)
        self.precisionControlLayout.addWidget(self.nextTrialBtn)
        
        self.stopPrecisionBtn = QtWidgets.QPushButton(self.precisionTab)
        self.stopPrecisionBtn.setText("Stop Test")
        self.stopPrecisionBtn.setStyleSheet("background-color: #f44336; color: white; padding: 10px;")
        self.precisionControlLayout.addWidget(self.stopPrecisionBtn)
        
        self.precisionLayout.addLayout(self.precisionControlLayout)
        
        # Precision test status
        self.precisionStatusText = QtWidgets.QPlainTextEdit(self.precisionTab)
        self.precisionStatusText.setReadOnly(True)
        self.precisionStatusText.setMaximumHeight(150)
        self.precisionStatusText.setPlaceholderText("Precision test status will appear here...")
        self.precisionLayout.addWidget(self.precisionStatusText)
        
        self.precisionLayout.addStretch()
        self.tabWidget.addTab(self.precisionTab, "Precision Test")
        
        # --- TAB 3: Weight Test ---
        self.weightTab = QtWidgets.QWidget()
        self.weightLayout = QtWidgets.QVBoxLayout(self.weightTab)
        
        self.weightInfoLabel = QtWidgets.QLabel(self.weightTab)
        self.weightInfoLabel.setText("Weight Test: Assess robot performance under various payload conditions")
        self.weightInfoLabel.setWordWrap(True)
        self.weightInfoLabel.setStyleSheet("padding: 10px; background-color: #E3F2FD; border-radius: 5px;")
        self.weightLayout.addWidget(self.weightInfoLabel)
        
        # Weight test parameters
        self.weightParamsGroupBox = QtWidgets.QGroupBox(self.weightTab)
        self.weightParamsGroupBox.setTitle("Test Parameters")
        self.weightParamsLayout = QtWidgets.QFormLayout(self.weightParamsGroupBox)
        
        self.weightValueSpin = QtWidgets.QDoubleSpinBox(self.weightParamsGroupBox)
        self.weightValueSpin.setRange(0.0, 5.0)
        self.weightValueSpin.setValue(0.0)
        self.weightValueSpin.setSingleStep(0.1)
        self.weightValueSpin.setSuffix(" kg")
        self.weightParamsLayout.addRow("Current Payload:", self.weightValueSpin)
        
        self.weightNumTrialsSpin = QtWidgets.QSpinBox(self.weightParamsGroupBox)
        self.weightNumTrialsSpin.setRange(1, 20)
        self.weightNumTrialsSpin.setValue(3)
        self.weightParamsLayout.addRow("Trials per Weight:", self.weightNumTrialsSpin)
        
        self.weightLayout.addWidget(self.weightParamsGroupBox)
        
        # Test positions for weight test
        self.weightPositionsGroupBox = QtWidgets.QGroupBox(self.weightTab)
        self.weightPositionsGroupBox.setTitle("Test Positions (meters)")
        self.weightPositionsLayout = QtWidgets.QFormLayout(self.weightPositionsGroupBox)
        
        self.weightPosXSpin = QtWidgets.QDoubleSpinBox(self.weightPositionsGroupBox)
        self.weightPosXSpin.setRange(-1.0, 1.0)
        self.weightPosXSpin.setValue(0.3)
        self.weightPosXSpin.setSingleStep(0.01)
        self.weightPositionsLayout.addRow("Position X:", self.weightPosXSpin)
        
        self.weightPosYSpin = QtWidgets.QDoubleSpinBox(self.weightPositionsGroupBox)
        self.weightPosYSpin.setRange(-1.0, 1.0)
        self.weightPosYSpin.setValue(0.0)
        self.weightPosYSpin.setSingleStep(0.01)
        self.weightPositionsLayout.addRow("Position Y:", self.weightPosYSpin)
        
        self.weightPosZSpin = QtWidgets.QDoubleSpinBox(self.weightPositionsGroupBox)
        self.weightPosZSpin.setRange(-1.0, 1.0)
        self.weightPosZSpin.setValue(0.2)
        self.weightPosZSpin.setSingleStep(0.01)
        self.weightPositionsLayout.addRow("Position Z:", self.weightPosZSpin)
        
        self.weightLayout.addWidget(self.weightPositionsGroupBox)
        
        # Weight test control
        self.weightControlLayout = QtWidgets.QHBoxLayout()
        
        self.startWeightBtn = QtWidgets.QPushButton(self.weightTab)
        self.startWeightBtn.setText("Start Weight Test")
        self.startWeightBtn.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px; font-weight: bold;")
        self.weightControlLayout.addWidget(self.startWeightBtn)
        
        self.moveToPositionBtn = QtWidgets.QPushButton(self.weightTab)
        self.moveToPositionBtn.setText("Move to Position")
        self.weightControlLayout.addWidget(self.moveToPositionBtn)
        
        self.stopWeightBtn = QtWidgets.QPushButton(self.weightTab)
        self.stopWeightBtn.setText("Stop Test")
        self.stopWeightBtn.setStyleSheet("background-color: #f44336; color: white; padding: 10px;")
        self.weightControlLayout.addWidget(self.stopWeightBtn)
        
        self.weightLayout.addLayout(self.weightControlLayout)
        
        # Weight test status
        self.weightStatusText = QtWidgets.QPlainTextEdit(self.weightTab)
        self.weightStatusText.setReadOnly(True)
        self.weightStatusText.setMaximumHeight(150)
        self.weightStatusText.setPlaceholderText("Weight test status will appear here...")
        self.weightLayout.addWidget(self.weightStatusText)
        
        self.weightLayout.addStretch()
        self.tabWidget.addTab(self.weightTab, "Weight Test")
        
        # --- TAB 4: Manual Control ---
        self.manualTab = QtWidgets.QWidget()
        self.manualLayout = QtWidgets.QVBoxLayout(self.manualTab)
        
        self.manualInfoLabel = QtWidgets.QLabel(self.manualTab)
        self.manualInfoLabel.setText("Manual Control: Set custom joint positions directly")
        self.manualInfoLabel.setStyleSheet("padding: 10px; background-color: #FFF3E0; border-radius: 5px;")
        self.manualLayout.addWidget(self.manualInfoLabel)
        
        # Joint control
        self.jointControlGroupBox = QtWidgets.QGroupBox(self.manualTab)
        self.jointControlGroupBox.setTitle("Joint Positions")
        self.jointControlLayout = QtWidgets.QFormLayout(self.jointControlGroupBox)
        
        self.joint1Slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.manualTab)
        self.joint1Slider.setRange(-314, 314)  # -3.14 to 3.14 * 100
        self.joint1Slider.setValue(0)
        self.joint1Label = QtWidgets.QLabel("0.00 rad")
        joint1Layout = QtWidgets.QHBoxLayout()
        joint1Layout.addWidget(self.joint1Slider)
        joint1Layout.addWidget(self.joint1Label)
        self.jointControlLayout.addRow("Joint 1:", joint1Layout)
        
        self.joint2Slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.manualTab)
        self.joint2Slider.setRange(-314, 314)
        self.joint2Slider.setValue(0)
        self.joint2Label = QtWidgets.QLabel("0.00 rad")
        joint2Layout = QtWidgets.QHBoxLayout()
        joint2Layout.addWidget(self.joint2Slider)
        joint2Layout.addWidget(self.joint2Label)
        self.jointControlLayout.addRow("Joint 2:", joint2Layout)
        
        self.joint3Slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.manualTab)
        self.joint3Slider.setRange(-314, 314)
        self.joint3Slider.setValue(0)
        self.joint3Label = QtWidgets.QLabel("0.00 rad")
        joint3Layout = QtWidgets.QHBoxLayout()
        joint3Layout.addWidget(self.joint3Slider)
        joint3Layout.addWidget(self.joint3Label)
        self.jointControlLayout.addRow("Joint 3:", joint3Layout)
        
        self.joint4Slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.manualTab)
        self.joint4Slider.setRange(-314, 314)
        self.joint4Slider.setValue(0)
        self.joint4Label = QtWidgets.QLabel("0.00 rad")
        joint4Layout = QtWidgets.QHBoxLayout()
        joint4Layout.addWidget(self.joint4Slider)
        joint4Layout.addWidget(self.joint4Label)
        self.jointControlLayout.addRow("Joint 4:", joint4Layout)
        
        self.manualLayout.addWidget(self.jointControlGroupBox)
        
        # Manual control buttons
        self.manualControlLayout = QtWidgets.QHBoxLayout()
        
        self.sendJointCmdBtn = QtWidgets.QPushButton(self.manualTab)
        self.sendJointCmdBtn.setText("Send Joint Command")
        self.sendJointCmdBtn.setStyleSheet("background-color: #2196F3; color: white; padding: 10px; font-weight: bold;")
        self.manualControlLayout.addWidget(self.sendJointCmdBtn)
        
        self.goHomeBtn = QtWidgets.QPushButton(self.manualTab)
        self.goHomeBtn.setText("Go to Home Position")
        self.manualControlLayout.addWidget(self.goHomeBtn)
        
        self.resetJointsBtn = QtWidgets.QPushButton(self.manualTab)
        self.resetJointsBtn.setText("Reset to Zero")
        self.manualControlLayout.addWidget(self.resetJointsBtn)
        
        self.manualLayout.addLayout(self.manualControlLayout)
        
        # Manual control status
        self.manualStatusText = QtWidgets.QPlainTextEdit(self.manualTab)
        self.manualStatusText.setReadOnly(True)
        self.manualStatusText.setMaximumHeight(150)
        self.manualStatusText.setPlaceholderText("Manual control status will appear here...")
        self.manualLayout.addWidget(self.manualStatusText)
        
        self.manualLayout.addStretch()
        self.tabWidget.addTab(self.manualTab, "Manual Control")
        
        self.mainLayout.addWidget(self.tabWidget)
        
        RobotTestSuite.setCentralWidget(self.centralwidget)
        
        # Menu bar
        self.menubar = QtWidgets.QMenuBar(RobotTestSuite)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1200, 22))
        RobotTestSuite.setMenuBar(self.menubar)
        
        # Status bar
        self.statusbar = QtWidgets.QStatusBar(RobotTestSuite)
        RobotTestSuite.setStatusBar(self.statusbar)
        
        QtCore.QMetaObject.connectSlotsByName(RobotTestSuite)


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    RobotTestSuite = QtWidgets.QMainWindow()
    ui = Ui_RobotTestSuite()
    ui.setupUi(RobotTestSuite)
    RobotTestSuite.show()
    sys.exit(app.exec_())