import os

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction  # noqa: F401
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity


def generate_launch_description():
    ldes: list[LaunchDescriptionEntity] = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument('robot_name', default_value='robot', description='The unique name for the robot'),
        ########################################################################
        # Parameters
        ########################################################################
        DeclareLaunchArgument(
            'joy_linux_params_file',
            default_value=os.path.join(
                get_package_share_directory('gamepad_teleoperation'), 'config', 'example_joy_linux.yaml'
            ),
            description='Path to the joy_linux parameters file',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_params_file',
            default_value=os.path.join(
                get_package_share_directory('gamepad_teleoperation'), 'config', 'example_robotnik_pad.yaml'
            ),
            description='Path to the robotnik_pad parameters file',
        ),
        ########################################################################
        # Remappings
        ########################################################################
        DeclareLaunchArgument('joy_linux_remappings', default_value='', description=rlh.REMAPPINGS_DESC),
        ########################################################################
        # Logging options
        ########################################################################
        DeclareLaunchArgument(
            'joy_linux_log_options', default_value=rlh.default_log_options_str(), description=rlh.LOG_OPTIONS_DESC
        ),
        DeclareLaunchArgument(
            'robotnik_pad_log_options', default_value=rlh.default_log_options_str(), description=rlh.LOG_OPTIONS_DESC
        ),
        ########################################################################
        # Node options
        ########################################################################
        DeclareLaunchArgument(
            'joy_linux_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        DeclareLaunchArgument(
            'robotnik_pad_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        ########################################################################
        # Launch nodes
        ########################################################################
        OpaqueFunction(function=rlh.set_robot_namespace, args=['namespace', 'robot_name']),
        OpaqueFunction(function=launch_joy_linux),
        OpaqueFunction(function=launch_robotnik_pad),
    ]

    return LaunchDescription(ldes)


def launch_joy_linux(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    node_options = rlh.parse_cli_node_opts(LaunchConfiguration('joy_linux_node_options').perform(ctx))

    return [
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name=str(node_options['name']) or 'joy_linux',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[
                ParameterFile(LaunchConfiguration('joy_linux_params_file').perform(ctx), allow_substs=True),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            remappings=rlh.parse_cli_remappings(LaunchConfiguration('joy_linux_remappings').perform(ctx)),
            ros_arguments=rlh.parse_cli_log_opts(LaunchConfiguration('joy_linux_log_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]


def launch_robotnik_pad(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    node_options = rlh.parse_cli_node_opts(LaunchConfiguration('robotnik_pad_node_options').perform(ctx))

    return [
        Node(
            package='robotnik_pad',
            executable='robotnik_pad',
            name=str(node_options['name']) or 'robotnik_pad',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[
                ParameterFile(LaunchConfiguration('robotnik_pad_params_file').perform(ctx), allow_substs=True),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            ros_arguments=rlh.parse_cli_log_opts(LaunchConfiguration('robotnik_pad_log_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
