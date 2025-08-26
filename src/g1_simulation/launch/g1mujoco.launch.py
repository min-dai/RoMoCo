from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="g1_simulation",
            executable="g1_state_machine",
            name="g1_state_machine",
            output="screen"
        ),
        Node(
            package="screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node",
            output="screen"
        )
    ])
