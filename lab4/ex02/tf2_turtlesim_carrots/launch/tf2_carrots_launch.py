from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='tf2_turtlesim_carrots',
            executable='turtle_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='tf2_turtlesim_carrots',
            executable='turtle_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='tf2_turtlesim_carrots',
            executable='turtle_listener',
            name='listener',
            parameters=[
                {'target_frame': "carrot1",
                 'turtle_name': "turtle2"}
            ]
        ),
        Node(
            package='tf2_turtlesim_carrots',
            executable='carrot',
            name='dynamic_broadcaster',
             parameters=[
                {'radius': 1.0}
            ]
        ),
    ])
