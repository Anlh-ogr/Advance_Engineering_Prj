from lauch import LaunchDescription # type: ignore
from lauch_ros.actions import Node  # type: ignore

def generate_launch_description():
    return LaunchDescription([
        # Node 1 : Gamepad Data Publisher
        Node (
            package = '',
            node_namespace='',
            node_executable = 'talker',
            output = 'screen',
            node_name = 'sim_1'
        ),
        
        # Node 2 : Gamepad Data Subscriber
        Node (
            package = '',
            node_namespace='',
            node_executable = 'listener',
            output = 'screen',
            node_name = 'sim_2'
        ),

        # Node 3 : Gamepad Data Subscriber
        Node (
            package = '',
            node_namespace='',
            node_executable = 'listener',
            output = 'screen',
            node_name = 'sim_3'
        )
    ])