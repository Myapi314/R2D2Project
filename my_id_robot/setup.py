import os
from glob import glob
from setuptools import setup

package_name = 'my_id_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grimmettr',
    maintainer_email='grimmettr@byui.edu',
    description='Simple Robotic Arm',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_publisher = my_id_robot.voice_publisher:main',
            'main_subscriber = my_id_robot.main_subscriber:main',
            'servo_subscriber = my_id_robot.servo_subscriber:main',
            'opencv_subscriber = my_id_robot.opencv_subscriber:main',
            'arduino_subscriber = my_id_robot.arduino_subscriber:main',
            'ir_publisher = my_id_robot.ir_publisher:main',
            'ultrasonic_publisher = my_id_robot.ultrasonic_publisher:main',
            'my_robot_launch = my_id_robot.my_robot_launch:main'
        ],
    },
)
