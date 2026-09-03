"""Launch joy_linux and robotnik_pad with one shared parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
import ros2_launch_helpers as rlh

DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


def generate_launch_description() -> LaunchDescription:
    """Declare the shared configuration and node arguments for gamepad teleoperation."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace shared by the gamepad driver and teleoperation node.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('teleop_twist_gamepad'),
                    'config',
                    'example_logitech_f710_teleoperation.yaml',
                ),
                description='Complete YAML parameter file for both nodes.',
            ),
            DeclareLaunchArgument(
                'params_file_allow_substs',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in params_file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS simulation time when true.',
            ),
            DeclareLaunchArgument(
                'joy_linux_node_args',
                default_value=DEFAULT_NODE_ARGS,
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            DeclareLaunchArgument(
                'robotnik_pad_node_args',
                default_value=DEFAULT_NODE_ARGS,
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            OpaqueFunction(function=_launch_nodes),
        ]
    )


def _launch_nodes(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Create both teleoperation nodes from one validated shared parameter file."""
    allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )

    # Reuse one ParameterFile object so launch performs substitutions at most once.
    parameter_file = ParameterFile(LaunchConfiguration('params_file'), allow_substs=allow_substs)

    return [
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                parameter_file,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    )
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('joy_linux_node_args').perform(ctx),
                default_arguments={'name': 'joy_linux'},
                extra_rejected_arguments={'namespace'},
            ),
        ),
        Node(
            package='robotnik_pad',
            executable='robotnik_pad',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                parameter_file,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    )
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('robotnik_pad_node_args').perform(ctx),
                default_arguments={'name': 'robotnik_pad'},
                extra_rejected_arguments={'namespace'},
            ),
        ),
    ]
