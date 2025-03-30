from launch import LaunchDescription # type: ignore
from launch_ros.actions import Node  # type: ignore

def generate_launch_description():
    return LaunchDescription([
        # node Robot Arm Subscriber
        Node (package = '',
              node_namespace = 'jet',
              node_executable = 'robot_arm_subscriber',
              output = 'screen',
              node_name = 'robot_arm_subscriber')
    ])