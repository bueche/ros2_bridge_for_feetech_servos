import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Automatically track down the path to your working URDF package file
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare("ros2_waveshare_bridge"), "urdf", "arm.urdf"
    ])

    waveshare_bridge_node = Node(
        package='ros2_waveshare_bridge',
        executable='bridge_node',
        name='ros2_waveshare_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 115200,
            'urdf_path': urdf_file_path  # The single dynamic source of truth parameter
        }]
    )

    return LaunchDescription([waveshare_bridge_node])
