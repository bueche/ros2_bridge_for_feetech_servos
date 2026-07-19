import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_waveshare_bridge')
    
    # Establish dynamic defaults located inside package share
    default_urdf = os.path.join(pkg_share, 'urdf', 'so-arm101.sample.urdf')
    default_yaml = os.path.join(pkg_share, 'config', 'so-arm101.yaml')

    return LaunchDescription([
        # Declare command-line parameters allowing easy overrides
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='Absolute path to robot URDF file'
        ),
        DeclareLaunchArgument(
            'joint_config_file',
            default_value=default_yaml,
            description='Path to calibration YAML file'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyWaveshare',
            description='Serial port softlink'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='1000000',
            description='Serial baud rate'
        ),

        # Initialize the Node with parameter evaluations mapped
        Node(
            package='ros2_waveshare_bridge',
            executable='bridge_node',
            name='ros2_waveshare_bridge',
            output='screen',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf_path'),
                'joint_config_file': LaunchConfiguration('joint_config_file'),
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }]
        )
    ])
