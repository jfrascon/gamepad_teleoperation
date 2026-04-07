import os
from pathlib import Path

import ros2_launch_helpers as rlh
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity


def generate_launch_description():
    ldes: list[LaunchDescriptionEntity] = [
        DeclareLaunchArgument('namespace', default_value='', description='Namespace'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('gamepad_teleoperation'),
                'config',
                'example_logitech_f710_teleoperation.yaml',
            ),
            description='Path to the gamepad_teleoperation parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument('joy_linux_dev', default_value='', description='Path to the joystick device'),
        # If `dev_name` exists in the system, `joy_linux_node` opens that device by name.
        # Otherwise, `joy_linux_node` falls back to the `dev` path.
        DeclareLaunchArgument('joy_linux_dev_name', default_value='', description='Name of the joystick device to use'),
        DeclareLaunchArgument(
            'joy_linux_deadzone', default_value='', description='Joystick axis deadzone in range [0.0, 1.0]'
        ),
        DeclareLaunchArgument(
            'joy_linux_autorepeat_rate',
            default_value='',
            description='Republish rate in Hz when joystick state does not change (0 disables it)',
        ),
        DeclareLaunchArgument(
            'joy_linux_coalesce_interval',
            default_value='',
            description='Time in seconds used to coalesce joystick events',
        ),
        DeclareLaunchArgument(
            'joy_linux_default_trig_val',
            default_value='',
            choices=['True', 'true', 'False', 'false', ''],
            description='Publish a default trigger value when the joystick does not provide one',
        ),
        DeclareLaunchArgument(
            'joy_linux_sticky_buttons',
            default_value='',
            choices=['True', 'true', 'False', 'false', ''],
            description='Publish sticky button states that stay True briefly after release',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_desired_freq', default_value='', description='Republish rate in Hz used by robotnik_pad'
        ),
        DeclareLaunchArgument(
            'robotnik_pad_num_of_buttons', default_value='', description='Number of buttons on the gamepad'
        ),
        DeclareLaunchArgument(
            'robotnik_pad_num_of_axes', default_value='', description='Number of axes on the gamepad'
        ),
        DeclareLaunchArgument(
            'robotnik_pad_joy_topic', default_value='', description='Topic used by robotnik_pad for Joy messages'
        ),
        DeclareLaunchArgument(
            'robotnik_pad_joy_timeout',
            default_value='',
            description='Timeout in seconds before robotnik_pad considers the joystick disconnected',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_max_linear_speed',
            default_value='',
            description='Maximum linear speed in m/s published by the Movement plugin',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_max_angular_speed',
            default_value='',
            description='Maximum angular speed in rad/s published by the Movement plugin',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_cmd_topic_vel',
            default_value='',
            description='Topic used by the Movement plugin for Twist commands',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_button_deadman',
            default_value='',
            description='Button index used as deadman switch in the Movement plugin',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_axis_linear_x',
            default_value='',
            description='Axis index used for the linear x command',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_axis_linear_y',
            default_value='',
            description='Axis index used for the linear y command',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_axis_angular_z',
            default_value='',
            description='Axis index used for the angular z command',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_button_speed_up',
            default_value='',
            description='Button index used to increase the Movement plugin speed limit',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_button_speed_down',
            default_value='',
            description='Button index used to decrease the Movement plugin speed limit',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_button_kinematic_mode',
            default_value='',
            description='Button index used to switch the Movement plugin kinematic mode',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_use_accel_watchdog',
            default_value='',
            choices=['True', 'true', 'False', 'false', ''],
            description='Enable the Movement plugin acceleration watchdog',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_axis_watchdog',
            default_value='',
            description='Axis indices monitored by the Movement plugin watchdog',
        ),
        DeclareLaunchArgument(
            'robotnik_pad_movement_config_watchdog_duration',
            default_value='',
            description='Stop duration in seconds used when the Movement plugin watchdog triggers',
        ),
        ########################################################################
        # NODE REMAPPINGS
        ########################################################################
        DeclareLaunchArgument('joy_linux_node_remappings', default_value='', description=rlh.REMAPPINGS_DESC),
        # Robotnik pad defines the topics it uses in its parameter file, so no remapping launch
        # argument is needed for it.
        ########################################################################
        # NODE OPTIONS
        ########################################################################
        DeclareLaunchArgument(
            'joy_linux_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        DeclareLaunchArgument(
            'robotnik_pad_node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
        ),
        ########################################################################
        # NODE LOGGING OPTIONS
        ########################################################################
        DeclareLaunchArgument(
            'joy_linux_node_logging_options',
            default_value=rlh.default_logging_options_str(),
            description=rlh.LOGGING_OPTIONS_DESC,
        ),
        DeclareLaunchArgument(
            'robotnik_pad_node_logging_options',
            default_value=rlh.default_logging_options_str(),
            description=rlh.LOGGING_OPTIONS_DESC,
        ),
        ########################################################################
        # NODES
        ########################################################################
        OpaqueFunction(function=_launch_joy_linux_node),
        OpaqueFunction(function=_launch_robotnik_pad_node),
    ]

    return LaunchDescription(ldes)


def _launch_joy_linux_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # If the params_file exists, load it as a ParameterFile.
    # If any parameter is also provided to this launch file, it takes precedence over the
    # params_file.
    # This allows to override specific parameters in the params_file without having to create a new
    # params file.
    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx)
    joy_linux_dev = LaunchConfiguration('joy_linux_dev').perform(ctx)
    joy_linux_dev_name = LaunchConfiguration('joy_linux_dev_name').perform(ctx)
    joy_linux_deadzone = LaunchConfiguration('joy_linux_deadzone').perform(ctx)
    joy_linux_autorepeat_rate = LaunchConfiguration('joy_linux_autorepeat_rate').perform(ctx)
    joy_linux_coalesce_interval = LaunchConfiguration('joy_linux_coalesce_interval').perform(ctx)
    joy_linux_default_trig_val = LaunchConfiguration('joy_linux_default_trig_val').perform(ctx)
    joy_linux_sticky_buttons = LaunchConfiguration('joy_linux_sticky_buttons').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if joy_linux_dev:
        parameters.append({'dev': joy_linux_dev})

    if joy_linux_dev_name:
        parameters.append({'dev_name': joy_linux_dev_name})

    if joy_linux_deadzone:
        try:
            parameters.append({'deadzone': float(joy_linux_deadzone)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'joy_linux_deadzone': '{joy_linux_deadzone}'. Must be a float."
            ) from exc

    if joy_linux_autorepeat_rate:
        try:
            parameters.append({'autorepeat_rate': float(joy_linux_autorepeat_rate)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'joy_linux_autorepeat_rate': '{joy_linux_autorepeat_rate}'. Must be a float."
            ) from exc

    if joy_linux_coalesce_interval:
        try:
            parameters.append({'coalesce_interval': float(joy_linux_coalesce_interval)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'joy_linux_coalesce_interval': '{joy_linux_coalesce_interval}'. Must be a float."
            ) from exc

    if joy_linux_default_trig_val:
        parameters.append({'default_trig_val': joy_linux_default_trig_val.lower() == 'true'})

    if joy_linux_sticky_buttons:
        parameters.append({'sticky_buttons': joy_linux_sticky_buttons.lower() == 'true'})

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('joy_linux_node_options').perform(ctx))
    node_name = str(node_options['name']) or 'joy_linux'

    return [
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            remappings=rlh.process_remappings(LaunchConfiguration('joy_linux_node_remappings').perform(ctx)),
            ros_arguments=rlh.process_node_logging_options(
                LaunchConfiguration('joy_linux_node_logging_options').perform(ctx)
            ),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]


def _launch_robotnik_pad_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # If the params_file exists, load it as a ParameterFile.
    # If any parameter is also provided to this launch file, it takes precedence over the
    # params_file.
    # This allows to override specific parameters in the params_file without having to create a new
    # params file.
    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx)
    rpad_desired_freq = LaunchConfiguration('robotnik_pad_desired_freq').perform(ctx)
    rpad_num_of_buttons = LaunchConfiguration('robotnik_pad_num_of_buttons').perform(ctx)
    rpad_num_of_axes = LaunchConfiguration('robotnik_pad_num_of_axes').perform(ctx)
    rpad_joy_topic = LaunchConfiguration('robotnik_pad_joy_topic').perform(ctx)
    rpad_joy_timeout = LaunchConfiguration('robotnik_pad_joy_timeout').perform(ctx)
    rpad_movement_max_linear_speed = LaunchConfiguration('robotnik_pad_movement_max_linear_speed').perform(ctx)
    rpad_movement_max_angular_speed = LaunchConfiguration('robotnik_pad_movement_max_angular_speed').perform(ctx)
    rpad_movement_cmd_topic_vel = LaunchConfiguration('robotnik_pad_movement_cmd_topic_vel').perform(ctx)
    rpad_config_button_deadman = LaunchConfiguration('robotnik_pad_movement_config_button_deadman').perform(ctx)
    rpad_config_axis_linear_x = LaunchConfiguration('robotnik_pad_movement_config_axis_linear_x').perform(ctx)
    rpad_config_axis_linear_y = LaunchConfiguration('robotnik_pad_movement_config_axis_linear_y').perform(ctx)
    rpad_config_axis_angular_z = LaunchConfiguration('robotnik_pad_movement_config_axis_angular_z').perform(ctx)
    rpad_config_button_speed_up = LaunchConfiguration('robotnik_pad_movement_config_button_speed_up').perform(ctx)
    rpad_config_button_speed_down = LaunchConfiguration('robotnik_pad_movement_config_button_speed_down').perform(ctx)
    rpad_config_button_kinematic_mode = LaunchConfiguration(
        'robotnik_pad_movement_config_button_kinematic_mode'
    ).perform(ctx)
    rpad_config_use_accel_watchdog = LaunchConfiguration('robotnik_pad_movement_config_use_accel_watchdog').perform(ctx)
    rpad_config_axis_watchdog = LaunchConfiguration('robotnik_pad_movement_config_axis_watchdog').perform(ctx)
    rpad_config_watchdog_duration = LaunchConfiguration('robotnik_pad_movement_config_watchdog_duration').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if rpad_desired_freq:
        try:
            parameters.append({'desired_freq': float(rpad_desired_freq)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'robotnik_pad_desired_freq': '{rpad_desired_freq}'. Must be a float."
            ) from exc

    if rpad_num_of_buttons:
        try:
            parameters.append({'pad': {'num_of_buttons': int(rpad_num_of_buttons)}})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'robotnik_pad_num_of_buttons': '{rpad_num_of_buttons}'. Must be an integer."
            ) from exc

    if rpad_num_of_axes:
        try:
            parameters.append({'pad': {'num_of_axes': int(rpad_num_of_axes)}})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'robotnik_pad_num_of_axes': '{rpad_num_of_axes}'. Must be an integer."
            ) from exc

    if rpad_joy_topic:
        parameters.append({'pad': {'joy_topic': rpad_joy_topic}})

    if rpad_joy_timeout:
        try:
            parameters.append({'pad': {'joy_timeout': float(rpad_joy_timeout)}})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for 'robotnik_pad_joy_timeout': '{rpad_joy_timeout}'. Must be a float."
            ) from exc

    if rpad_movement_max_linear_speed:
        try:
            parameters.append({'Movement': {'max_linear_speed': float(rpad_movement_max_linear_speed)}})
        except ValueError as exc:
            raise ValueError(
                "Invalid value for 'robotnik_pad_movement_max_linear_speed': "
                f"'{rpad_movement_max_linear_speed}'. Must be a float."
            ) from exc

    if rpad_movement_max_angular_speed:
        try:
            parameters.append({'Movement': {'max_angular_speed': float(rpad_movement_max_angular_speed)}})
        except ValueError as exc:
            raise ValueError(
                "Invalid value for 'robotnik_pad_movement_max_angular_speed': "
                f"'{rpad_movement_max_angular_speed}'. Must be a float."
            ) from exc

    if rpad_movement_cmd_topic_vel:
        parameters.append({'Movement': {'cmd_topic_vel': rpad_movement_cmd_topic_vel}})

    int_overrides = [
        ('robotnik_pad_movement_config_button_deadman', rpad_config_button_deadman, 'button_deadman'),
        ('robotnik_pad_movement_config_axis_linear_x', rpad_config_axis_linear_x, 'axis_linear_x'),
        ('robotnik_pad_movement_config_axis_linear_y', rpad_config_axis_linear_y, 'axis_linear_y'),
        ('robotnik_pad_movement_config_axis_angular_z', rpad_config_axis_angular_z, 'axis_angular_z'),
        ('robotnik_pad_movement_config_button_speed_up', rpad_config_button_speed_up, 'button_speed_up'),
        ('robotnik_pad_movement_config_button_speed_down', rpad_config_button_speed_down, 'button_speed_down'),
        (
            'robotnik_pad_movement_config_button_kinematic_mode',
            rpad_config_button_kinematic_mode,
            'button_kinematic_mode',
        ),
    ]

    for launch_arg_name, raw_value, parameter_name in int_overrides:
        if raw_value:
            try:
                parameters.append({'Movement': {'config': {parameter_name: int(raw_value)}}})
            except ValueError as exc:
                raise ValueError(f"Invalid value for '{launch_arg_name}': '{raw_value}'. Must be an integer.") from exc

    if rpad_config_use_accel_watchdog:
        parameters.append(
            {'Movement': {'config': {'use_accel_watchdog': rpad_config_use_accel_watchdog.lower() == 'true'}}}
        )

    if rpad_config_axis_watchdog:
        try:
            axis_watchdog_values = yaml.safe_load(rpad_config_axis_watchdog)
        except yaml.YAMLError as exc:
            raise ValueError(
                "Invalid value for 'robotnik_pad_movement_config_axis_watchdog': "
                f"'{rpad_config_axis_watchdog}'. Must be a YAML list of integers."
            ) from exc

        if not isinstance(axis_watchdog_values, list) or not all(
            isinstance(value, int) for value in axis_watchdog_values
        ):
            raise ValueError(
                "Invalid value for 'robotnik_pad_movement_config_axis_watchdog': "
                f"'{rpad_config_axis_watchdog}'. Must be a YAML list of integers."
            )

        parameters.append({'Movement': {'config': {'axis_watchdog': axis_watchdog_values}}})

    if rpad_config_watchdog_duration:
        try:
            parameters.append({'Movement': {'config': {'watchdog_duration': float(rpad_config_watchdog_duration)}}})
        except ValueError as exc:
            raise ValueError(
                "Invalid value for 'robotnik_pad_movement_config_watchdog_duration': "
                f"'{rpad_config_watchdog_duration}'. Must be a float."
            ) from exc

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('robotnik_pad_node_options').perform(ctx))
    node_name = str(node_options['name']) or 'robotnik_pad'

    return [
        Node(
            package='robotnik_pad',
            executable='robotnik_pad',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            ros_arguments=rlh.process_node_logging_options(
                LaunchConfiguration('robotnik_pad_node_logging_options').perform(ctx)
            ),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
