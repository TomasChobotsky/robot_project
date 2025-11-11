from setuptools import setup
import os
from glob import glob

package_name = 'robot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='tomas.chobotsky1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector = robot_project.ball_detector:main',
            'ball_tracking_controller = robot_project.ball_tracking_controller:main',
            'send_single_joint_command = robot_project.send_single_joint_command:main',
            'robot_visualizer = robot_project.robot_visualizer:main',
            'precision_test = robot_project.precision_test:main',
            'robot_test_gui = robot_project.robot_test_gui:main',
            'hsv_tuner = robot_project.hsv_tuner:main',
        ],
    },
)
