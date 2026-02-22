# gamepad_teleoperation

This package provides a wrapper for gamepad-based teleoperation in ROS 2, built on top of the original package `robotnik_pad` developed by **Robotnik**.
The underlying `robotnik_pad` package allows teleoperation of **differential**, **ackermann**, and **omnidirectional** robot platforms.

For more details and supported features, see the official Robotnik repository:<br/>
<https://github.com/RobotnikAutomation/robotnik_pad>

## Purpose

It allows easy launching of gamepad teleoperation, integrating the `joy_linux` and `robotnik_pad` nodes with example configurations and a ready-to-use Python launch file for mobile robots.

## Multirobot compatibility

The `robot_name` argument has a default value (`robot`), and `namespace` is optional but *highly recommended* for multirobot scenarios. This ensures that all resources (topics, nodes, parameters) are properly namespaced, avoiding conflicts between robots. In multirobot setups, provide a unique `robot_name` for each robot instance. The effective namespace for all nodes is the combination of `namespace` (if provided) and `robot_name`.

## Main Features

- Based on Robotnik's `robotnik_pad` package.
- Provides a unified example configuration file in `config/` for both `joy_linux` and `robotnik_pad`.
- Includes a launch file (`gamepad_teleoperation.launch.py`) that simplifies integration and launching of teleoperation.
- Uses the `ros2_launch_helpers` utility to make it easy to pass arguments and advanced configuration from the launch file.

## Use of ros2_launch_helpers

The launch file uses `ros2_launch_helpers` to define and process launch arguments in a consistent way for:

- Topic remappings (`joy_linux_node_topic_remappings`)
- Logging options (`joy_linux_node_logging_options`, `robotnik_pad_logging_options`)
- Node options (`joy_linux_node_options`, `robotnik_pad_node_options`)
- Node parameter file (`params_file`)

This enables flexible customization of the teleoperation nodes' behavior from the command line or from other launch files.

## Example usage

Basic usage:

```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py
```

Advanced usage with all relevant arguments:

```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py \
 use_sim_time:=true \
 namespace:=myproject \
 robot_name:=myrobot \
 params_file:=/path/to/my_gamepad_teleoperation.yaml \
 joy_linux_node_topic_remappings:="joy:=joypad" \
 joy_linux_node_logging_options:="log-level=debug,disable-stdout-logs=False" \
 robotnik_pad_logging_options:="log-level=info" \
 joy_linux_node_options:="name=joy_linux_custom,output=screen,emulate_tty=True,respawn=True,respawn_delay=2.0" \
 robotnik_pad_node_options:="name=robotnik_pad_custom,output=screen,emulate_tty=True"
```

You can override any launch file argument as needed. See the launch file for all available options.
