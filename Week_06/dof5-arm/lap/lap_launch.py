from launch import LaunchDescription # type: ignore
from launch_ros.actions import Node  # type: ignore

def generate_launch_description():
    return LaunchDescription([
        # node GamePad Publisher
        Node (package = '',
              node_namespace = 'lap',
              node_executable = 'gamepad_publisher',
              output = 'screen',
              node_name = 'gamepad_publisher'),
        
        # node Color Detection Publisher
        Node (package = '',
              node_namespace = 'lap',
              node_executable = 'color_detection_publisher',
              output = 'screen',
              node_name = 'color_detection_publisher')
    ])