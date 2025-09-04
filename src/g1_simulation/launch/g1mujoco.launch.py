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
            package="g1_simulation",
            executable="g1_mujoco_interface_node_main",
            name="g1_mujoco_interface_node_main",
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
