from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Turtlesim Node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Turtle close loop Node
        Node(
            package='my_robot_controller',
            executable='close_loop',
            name='close_loop'
        ),
    ])