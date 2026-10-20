import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_waveshare_bridge')
    
    # Paths to URDF and controllers configuration
    default_urdf = os.path.join(pkg_share, 'urdf', 'so-arm101.sample.urdf')
    default_controllers = os.path.join(pkg_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        # Arguments for path overrides
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='Absolute path to robot URDF file'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=default_controllers,
            description='Path to ros2_control YAML configuration file'
        ),

        # 1. Robot State Publisher (publishes robot_description topic from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', LaunchConfiguration('urdf_path')])
            }]
        ),

        # 2. ros2_control Main Node (Loads feetech_ros2_driver plugin)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command(['cat ', LaunchConfiguration('urdf_path')])},
                LaunchConfiguration('controllers_file')
            ],
            output='screen'
        ),

        # 3. Spawn Joint State Broadcaster (Publishes /joint_states)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # 4. Spawn Position Controller (Accepts position command inputs)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])