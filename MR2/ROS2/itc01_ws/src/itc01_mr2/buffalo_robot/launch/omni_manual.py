from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='buffalo_robot',
            executable='omni_pidv4',
            output='screen',
        ),
        Node(
            package='buffalo_robot',
            executable='can_buffaloV3',
            output='screen',
        ),
        Node(
            package='buffalo_robot',
            executable='ps4v2',
            output='screen',
        ),
    ])