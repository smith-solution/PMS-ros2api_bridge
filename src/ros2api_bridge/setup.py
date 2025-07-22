#!/usr/bin/env python3
from setuptools import find_packages, setup
import glob
import os

package_name = 'ros2api_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools','fastapi','uvicorn','rclpy',],
    zip_safe=True,
    maintainer='ddokkon',
    maintainer_email='pddj21@knu.ac.kr',
    description='Publish ROS topic and services with FastAPI and ROS2 DDS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts' : [
    	'ros2api_bridge = ros2api_bridge.main:main',
    	# FastAPI는 uvicorn으로 직접 실행 예정, entry point 불필요. 위는 테스트용임.
        ],
    },
)
