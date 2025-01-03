from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_pkg',
            namespace='example1',
            executable='talker',
            output='screen',
            name='sim'
        ),
        Node(
            package='my_pkg',
            namespace='example1',
            executable='listener',
            output='screen',
            name='sim'
        )
    ])
