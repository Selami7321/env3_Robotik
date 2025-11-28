from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_publisher_pkg',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen'
        ),
        Node(
            package='data_processor_pkg',
            executable='data_processor',
            name='data_processor',
            output='screen'
        ),
        Node(
            package='command_server_pkg',
            executable='command_server',
            name='command_server',
            output='screen'
        ),
    ])


