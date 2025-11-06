
# gamepad_teleoperation

This package provides a wrapper for gamepad-based teleoperation in ROS 2, built on top of the original package `robotnik_pad` developed by **Robotnik**.
The underlying `robotnik_pad` package allows teleoperation of **differential**, **ackermann**, and **omnidirectional** robot platforms.

For more details and supported features, see the official Robotnik repository:<br>
https://github.com/RobotnikAutomation/robotnik_pad

## Purpose
It allows easy launching of gamepad teleoperation, integrating the `joy_linux` and `robotnik_pad` nodes with example configurations and a ready-to-use Python launch file for mobile robots.

## Multirobot compatibility

The `robot_name` argument is required in the launch file, while `namespace` is optional but *highly recommended* for multirobot scenarios. This ensures that all resources (topics, nodes, parameters) are properly namespaced, avoiding conflicts between robots. Always provide a unique `robot_name` for each robot instance. Using a meaningful `namespace` is recommended for clarity and better organization of your robot resources. The effective namespace for all nodes will be the combination of the given `namespace` (if any) and the `robot_name`.

## Main Features
- Based on Robotnik's `robotnik_pad` package.
- Provides example configuration files for `joy_linux` and `robotnik_pad` in the `config/` folder.
- Includes a launch file (`gamepad_teleoperation.launch.py`) that simplifies integration and launching of teleoperation.
- Uses the `ros2_launch_helpers` utility to make it easy to pass arguments and advanced configuration from the launch file.

## Use of ros2_launch_helpers
The launch file leverages `ros2_launch_helpers` to easily pass:
- Topic remappings (`joy_linux_remappings`)
- Logging options (`joy_linux_log_options`, `robotnik_pad_log_options`)
- Node options (`joy_linux_node_options`, `robotnik_pad_node_options`)
- Node parameter files (`joy_linux_params_file`, `robotnik_pad_params_file`)

This enables flexible customization of the teleoperation nodes' behavior from the command line or from other launch files.

## Example usage

Basic usage:
```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py
```

Advanced usage with all relevant arguments (note: `robot_name` is required, `namespace` is optional but recommended):
```bash
ros2 launch gamepad_teleoperation gamepad_teleoperation.launch.py \
	use_sim_time:=true \
	namespace:=myproject \
	robot_name:=myrobot \
	joy_linux_params_file:=/path/to/my_joy_linux.yaml \
	robotnik_pad_params_file:=/path/to/my_robotnik_pad.yaml \
	joy_linux_remappings:="joy:=joypad" \
	joy_linux_log_options:="log-level=debug,disable-stdout-logs=False" \
	robotnik_pad_log_options:="log-level=info" \
	joy_linux_node_options:="name=joy_linux_custom,output=screen,emulate_tty=True,respawn=True,respawn_delay=2.0" \
	robotnik_pad_node_options:="name=robotnik_pad_custom,output=screen,emulate_tty=True"
```

You can override any launch file argument as needed. See the launch file for all available options.
