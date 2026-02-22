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
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('gamepad_teleoperation'), 'config', 'example_gamepad_teleoperation.yaml'
            ),
            description='Path to the gamepad_teleoperation parameter file',
        ),
        ########################################################################
        # Remappings
        ########################################################################
        # Name of the parameter is: <executable_name>_topic_remappings
        DeclareLaunchArgument(
            'joy_linux_node_topic_remappings', default_value='', description=rlh.TOPIC_REMAPPINGS_DESC
        ),
        ########################################################################
        # Node options
        ########################################################################
        # Name of the parameter is: <executable_name>_node_options
        # If <executable_name> already ends with "_node", do not duplicate "_node"
        # (example: joy_linux_node -> joy_linux_node_options).
        DeclareLaunchArgument(
            'joy_linux_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        DeclareLaunchArgument(
            'robotnik_pad_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        ########################################################################
        # Logging options
        ########################################################################
        # Name of the parameter is: <executable_name>_logging_options
        DeclareLaunchArgument(
            'joy_linux_node_logging_options',
            default_value=rlh.default_logging_options_str(),
            description=rlh.LOGGING_OPTIONS_DESC,
        ),
        DeclareLaunchArgument(
            'robotnik_pad_logging_options',
            default_value=rlh.default_logging_options_str(),
            description=rlh.LOGGING_OPTIONS_DESC,
        ),
        ########################################################################
        # Launch nodes
        ########################################################################
        OpaqueFunction(function=launch_joy_linux_node),
        OpaqueFunction(function=launch_robotnik_pad),
    ]

    return LaunchDescription(ldes)


def launch_joy_linux_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    node_options = rlh.process_node_options(LaunchConfiguration('joy_linux_node_options').perform(ctx))
    node_name = str(node_options['name']) or 'joy_linux'

    robot_ns = rlh.create_robot_namespace(
        LaunchConfiguration('namespace').perform(ctx), LaunchConfiguration('robot_name').perform(ctx)
    )

    return [
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name=node_name,
            # Insert the node into the robot_ns.
            namespace=robot_ns,
            parameters=parameters,
            remappings=rlh.process_topic_remappings(
                LaunchConfiguration('joy_linux_node_topic_remappings').perform(ctx)
            ),
            ros_arguments=rlh.process_logging_options(
                LaunchConfiguration('joy_linux_node_logging_options').perform(ctx)
            ),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]


def launch_robotnik_pad(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('robotnik_pad_node_options').perform(ctx))
    node_name = str(node_options['name']) or 'robotnik_pad'

    robot_ns = rlh.create_robot_namespace(
        LaunchConfiguration('namespace').perform(ctx), LaunchConfiguration('robot_name').perform(ctx)
    )

    return [
        Node(
            package='robotnik_pad',
            executable='robotnik_pad',
            name=node_name,
            # Insert the node into the robot_ns.
            namespace=robot_ns,
            parameters=parameters,
            # Topic definition is done in the parameter file.
            ros_arguments=rlh.process_logging_options(LaunchConfiguration('robotnik_pad_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
