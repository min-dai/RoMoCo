from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import datetime
def generate_launch_description() -> LaunchDescription:
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_folder_timestamp = f"logs_{timestamp}"
    use_estimation_arg = DeclareLaunchArgument(
        'use_estimation',
        default_value='false',
        description='Whether to use estimation in the interface'
    )
    return LaunchDescription([
        SetEnvironmentVariable('LOG_FOLDER_TIMESTAMP', log_folder_timestamp),
        use_estimation_arg,
        Node(
            package="h1_stack",
            executable="h1_state_machine",
            name="h1_state_machine",
            arguments=[LaunchConfiguration('use_estimation')],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="romoco_screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node"
        )
    ])