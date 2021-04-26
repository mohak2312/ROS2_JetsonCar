from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosim',
            node_namespace='Controller',
            node_executable='talker',
            output='screen'
        ),
        Node(
            package='robosim',
            node_namespace='JetsonCar',
            node_executable='listener',
            output='screen'
        )
    ])