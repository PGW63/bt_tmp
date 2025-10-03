from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vosper_ros',
            executable='vosper_node_receptionist',
            name='vosper_node_receptionist',
            output='screen'
        ),
        Node(
            package='vosper_ros',
            executable='speech_to_tts',
            name='speech_to_tts',
            output='screen'
        ),
    ])

