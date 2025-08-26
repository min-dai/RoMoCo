from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="h1_simulation",
            executable="h1_state_machine",
            name="h1_state_machine",
            output="screen"
        ),
        Node(
            package="screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node",
            output="screen"
        )
    ])
