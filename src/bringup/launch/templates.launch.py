from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    transmitter_node = Node(
        package='templates_cpp',
        executable='transmitter',
        name='transmitter',
        parameters=[{'period': 1.0}],
        remappings=[('/example_topic', '/my_topic')]
    )

    listener_node = Node(
        package='templates_py',
        executable='receiver',
        name='listener',
        remappings=[('/example_topic', '/my_topic')]
    )

    ld.add_action(transmitter_node)
    ld.add_action(listener_node)
    return ld