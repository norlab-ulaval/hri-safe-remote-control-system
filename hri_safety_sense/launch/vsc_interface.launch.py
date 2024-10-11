from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS3', description='Port for the serial interface'),

        Node(
            package='hri_safety_sense',
            executable='safe_remote_control',
            name='vsc_serial_interface',
            output='screen',
            parameters=[{'port': '/dev/ttyS3'}],
            respawn=True
        )
    ])
