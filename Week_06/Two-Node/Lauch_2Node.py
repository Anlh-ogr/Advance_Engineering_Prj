from lauch import LaunchDescription # type: ignore
from lauch_ros.actions import Node  # type: ignore

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='',
            node_namespace='',
            node_executable = 'talker',
            output = 'screen',
            node_name = 'sim_1'
        ),
        Node(
            package='',
            node_namespace='',
            node_executable = 'listener',
            output = 'screen',
            node_name = 'sim_2'
        )
    ])