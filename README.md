# gamepad_teleoperation

This package provides a wrapper for gamepad-based teleoperation in ROS 2, built on top of the
original package `robotnik_pad` developed by **Robotnik**.
The underlying `robotnik_pad` package allows teleoperation of **differential**, **ackermann**, and **omnidirectional** robot platforms.

For more details and supported features, see the official Robotnik repository:<br/>
<https://github.com/RobotnikAutomation/robotnik_pad>

## What this package launches

The launch file starts these two nodes in the same namespace:

- `joy_linux_node`, which reads joystick events from the gamepad device.
- `robotnik_pad`, which converts joystick messages into velocity commands.

The `namespace` launch argument sets that shared ROS namespace. Use a different namespace for each
robot instance in multirobot scenarios so topics, node names, and parameters do not collide.

The package also installs `config/example_gamepad_teleoperation.yaml`, which configures both nodes
and uses `$(var ...)` substitutions so launch arguments can override selected parameter values.

## Configuration model

`params_file` is always loaded as the node parameter file.

If you do not pass `params_file`, the default value points to the example YAML installed by this
package. If you pass your own YAML file, that file is used instead.

Inside the YAML file, values written as `$(var <launch_argument_name>)` are resolved from the
current launch context. Values written as literals are used as-is.

The launch file also exposes these helper-based arguments:

- Topic remappings (`joy_linux_node_remappings`)
- Logging options (`joy_linux_node_logging_options`, `robotnik_pad_node_logging_options`)
- Node options (`joy_linux_node_options`, `robotnik_pad_node_options`)

Use the logging options arguments to change the node log level, for example `log-level=debug`.

## Examples

### Example 1: launch with the default parameter file

```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py
```

This command uses the default `params_file`, which points to
`config/example_gamepad_teleoperation.yaml`.

That file defines the parameter tree, but each value is delegated to a launch argument through
`$(var ...)`. If you launch this command without overriding those arguments, the
`DeclareLaunchArgument` defaults in `gamepad_teleoperation.launch.py` provide the values. If you
override a launch argument from the CLI or from a parent launch file, the YAML file picks up that
value.

```yaml
/**/joy_linux:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    dev: $(var joy_linux_dev)
    dev_name: $(var joy_linux_dev_name)
    deadzone: $(var joy_linux_deadzone)
    autorepeat_rate: $(var joy_linux_autorepeat_rate)
    coalesce_interval: $(var joy_linux_coalesce_interval)
    default_trig_val: $(var joy_linux_default_trig_val)
    sticky_buttons: $(var joy_linux_sticky_buttons)

/**/robotnik_pad:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    desired_freq: $(var robotnik_pad_desired_freq)
    pad:
      num_of_buttons: $(var robotnik_pad_num_of_buttons)
      num_of_axes: $(var robotnik_pad_num_of_axes)
      joy_topic: $(var robotnik_pad_joy_topic)
      joy_timeout: $(var robotnik_pad_joy_timeout)
    plugins:
      - Movement
    Movement:
      type: robotnik_pad_plugins/Movement
      max_linear_speed: $(var robotnik_pad_movement_max_linear_speed)
      max_angular_speed: $(var robotnik_pad_movement_max_angular_speed)
      cmd_topic_vel: $(var robotnik_pad_movement_cmd_topic_vel)
      config:
        button_deadman: $(var robotnik_pad_movement_config_button_deadman)
        axis_linear_x: $(var robotnik_pad_movement_config_axis_linear_x)
        axis_linear_y: $(var robotnik_pad_movement_config_axis_linear_y)
        axis_angular_z: $(var robotnik_pad_movement_config_axis_angular_z)
        button_speed_up: $(var robotnik_pad_movement_config_button_speed_up)
        button_speed_down: $(var robotnik_pad_movement_config_button_speed_down)
        button_kinematic_mode: $(var robotnik_pad_movement_config_button_kinematic_mode)
        use_accel_watchdog: $(var robotnik_pad_movement_config_use_accel_watchdog)
        axis_watchdog: $(var robotnik_pad_movement_config_axis_watchdog)
        watchdog_duration: $(var robotnik_pad_movement_config_watchdog_duration)
```

### Example 2: launch with a custom parameter file and CLI overrides

```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py \
 use_sim_time:=true \
 namespace:=myproject \
 params_file:=/path/to/my_gamepad_teleoperation.yaml \
 joy_linux_node_remappings:="joy:=joypad" \
 joy_linux_node_logging_options:="log-level=debug,disable-stdout-logs=False" \
 robotnik_pad_node_logging_options:="log-level=debug" \
 joy_linux_node_options:="name=joy_linux_custom,output=screen,emulate_tty=True,respawn=True,respawn_delay=2.0" \
 robotnik_pad_node_options:="name=robotnik_pad_custom,output=screen,emulate_tty=True"
```

You can override any launch file argument as needed. See the launch file for all available options.

The custom `params_file` can mix literal values and `$(var ...)` substitutions. For example, this
file hardcodes the joystick device and several movement parameters, but it still keeps
`use_sim_time`, `desired_freq`, and `cmd_topic_vel` configurable through launch arguments.

The top-level YAML keys in this example are `/**/joy_linux_custom` and `/**/robotnik_pad_custom`
because the command above sets those node names through `joy_linux_node_options` and
`robotnik_pad_node_options`. If you use different node names, update those YAML keys accordingly.

```yaml
/**/joy_linux_custom:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    dev: /dev/input/js1
    dev_name: _not_provided_
    deadzone: 0.08
    autorepeat_rate: 40.0
    coalesce_interval: 0.001
    default_trig_val: false
    sticky_buttons: false

/**/robotnik_pad_custom:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    desired_freq: $(var robotnik_pad_desired_freq)
    pad:
      num_of_buttons: 11
      num_of_axes: 8
      joy_topic: joy
      joy_timeout: 1.0
    plugins:
      - Movement
    Movement:
      type: robotnik_pad_plugins/Movement
      max_linear_speed: 1.2
      max_angular_speed: 1.8
      cmd_topic_vel: $(var robotnik_pad_movement_cmd_topic_vel)
      config:
        button_deadman: 5
        axis_linear_x: 1
        axis_linear_y: 0
        axis_angular_z: 3
        button_speed_up: 3
        button_speed_down: 0
        button_kinematic_mode: 2
        use_accel_watchdog: false
        axis_watchdog: [6, 7, 8]
        watchdog_duration: 0.0
```

### Example 3: launch with a mostly literal parameter file

In this style, the YAML file fully defines the node parameters except `use_sim_time`, which remains
connected to the launch argument so the same file can be used in both real and simulated runs.

Launch arguments still provide `namespace`, remappings, logging options, and node options, but not
the node parameter values listed below.

```yaml
/**/joy_linux:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    dev: /dev/input/js0
    dev_name: _not_provided_
    deadzone: 0.05
    autorepeat_rate: 30.0
    coalesce_interval: 0.001
    default_trig_val: false
    sticky_buttons: false

/**/robotnik_pad:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    desired_freq: 30.0
    pad:
      num_of_buttons: 11
      num_of_axes: 8
      joy_topic: joy
      joy_timeout: 1.0
    plugins:
      - Movement
    Movement:
      type: robotnik_pad_plugins/Movement
      max_linear_speed: 1.0
      max_angular_speed: 1.5
      cmd_topic_vel: twist_cmd/joypad
      config:
        button_deadman: 5
        axis_linear_x: 1
        axis_linear_y: 0
        axis_angular_z: 3
        button_speed_up: 3
        button_speed_down: 0
        button_kinematic_mode: 2
        use_accel_watchdog: false
        axis_watchdog: [6, 7, 8]
        watchdog_duration: 0.0
```
