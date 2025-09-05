from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import datetime
def generate_launch_description() -> LaunchDescription:
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_folder_timestamp = f"logs_{timestamp}"
    return LaunchDescription([
        SetEnvironmentVariable('LOG_FOLDER_TIMESTAMP', log_folder_timestamp),
        Node(
            package="cassie_simulation",
            executable="cassie_mujoco_interface_node_main",
            name="cassie_mujoco_interface_node_main",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="cassie_simulation",
            executable="cassie_controller_node_main",
            name="cassie_controller_node_main",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node"
        )
    ])
