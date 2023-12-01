from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_publishers',
            executable='random_string_publisher',
            name='random_string_publisher',
            output='screen'
        ),
        Node(
            package='web_publishers',
            executable='image_publisher',
            name='image_publisher',
            output='screen'
        )
    ])