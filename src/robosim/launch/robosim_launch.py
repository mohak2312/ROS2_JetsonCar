from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosim',
            node_namespace='RoboSim',
            node_executable='talker',
            #output='screen'
            node_name='controller'
        ),
        Node(
            package='robosim',
            node_namespace='RoboSim',
            node_executable='listener',
            #output='screen'
            node_name='rc_jcar'
        )
    ])