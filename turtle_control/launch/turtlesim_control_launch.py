from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='sim'),
        Node(
            package='turtle_control', 
            executable='pose_server',
            parameters=[{'max_linear_speed': 2.5, 'max_angular_speed': 1.8}]
        ),
        Node(package='turtle_control', executable='reset_server'),
        Node(package='turtle_control', executable='square_server'),
    ])
