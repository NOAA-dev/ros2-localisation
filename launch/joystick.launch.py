from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    program_bringup_pkg = get_package_share_directory("program_bringup")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",description="Use simulated time")

    joystick_teleop_config = os.path.join(
        program_bringup_pkg,
        'config',
        'joystick_teleop.yaml'
    )

    joystick_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[joystick_teleop_config]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        output='screen',
        parameters = [os.path.join(
            get_package_share_directory('program_bringup'),
            'config',
            'joystick_config.yaml')]
    )

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "/bumper_bot/cmd_vel_unstamped",
            "config_locks": os.path.join(program_bringup_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(program_bringup_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(program_bringup_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="my_py_pkg1",
        executable="cmd_relay",
        name="cmd_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    return LaunchDescription([
        use_sim_time_arg,
        joystick_node,
        joy_node,
        twist_mux_launch,
        twist_relay_node
    ]
    )