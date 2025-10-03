from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vosper_ros',
            executable='vosper_node2',
            name='vosper_node',
            output='screen'
        ),
        Node(
            package='vosper_ros',
            executable='speech_to_tts',
            name='speech_to_tts',
            output='screen'
        ),
    ])

