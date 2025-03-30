from launch import LaunchDescription # type: ignore
from launch_ros.actions import Node  # type: ignore

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='',
            node_namespace='',
            node_executable = 'publisher',
            output = 'screen',
            node_name = 'sim_1'
        ),
        Node(
            package='',
            node_namespace='',
            node_executable = 'subscriber',
            output = 'screen',
            node_name = 'sim_2'
        )
    ])