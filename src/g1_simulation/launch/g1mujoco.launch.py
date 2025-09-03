from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="g1_simulation",
            executable="g1_interface_node_main",
            name="g1_interface_node_main",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="g1_simulation",
            executable="g1_controller_node_main",
            name="g1_controller_node_main",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node"
        )
    ])
