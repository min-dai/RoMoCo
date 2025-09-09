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
        default_value='true',
        description='Whether to use estimation in the interface'
    )
    use_sim_gains_arg = DeclareLaunchArgument(
        'use_sim_gains',
        default_value='true',
        description='Whether to use simulation gains'
    )
    return LaunchDescription([
        SetEnvironmentVariable('LOG_FOLDER_TIMESTAMP', log_folder_timestamp),
        use_estimation_arg,
        use_sim_gains_arg,
        Node(
            package="g1_simulation",
            executable="g1_mujoco_interface_node_main",
            name="g1_mujoco_interface_node_main",
            arguments=[LaunchConfiguration('use_estimation')],
            # output="screen",
            # emulate_tty=True
        ),
        Node(
            package="g1_simulation",
            executable="g1_controller_node_main",
            name="g1_controller_node_main",
            arguments=[LaunchConfiguration('use_sim_gains')],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="romoco_screen_radio",
            executable="screen_radio_node",
            name="screen_radio_node"
        )
    ])
