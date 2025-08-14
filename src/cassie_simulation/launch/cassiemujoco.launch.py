from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="cassie_simulation",
            executable="cassie_state_machine",
            name="cassie_state_machine",
            output="screen"
        ),
        Node(
            package="biped_command",
            executable="fake_radio_node",
            name="fake_radio_node",
            output="screen"
        )
    ])
