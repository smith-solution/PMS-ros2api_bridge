# launch/ros2api_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2api_bridge',
            executable='ros2api_bridge',
            name='ros2api_bridge_node',
            output='screen'
        )
    ])

