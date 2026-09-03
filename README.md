# [teleop_twist_gamepad](https://github.com/jfrascon/teleop_twist_gamepad)

`teleop_twist_gamepad` is a ROS 2 integration package for gamepad teleoperation.
It starts `joy_linux` to read the gamepad and `robotnik_pad` to convert `sensor_msgs/msg/Joy` messages into velocity commands.

The Robotnik Movement plugin supports differential, Ackermann and omnidirectional command modes.
The button configured as `button_kinematic_mode` changes mode at runtime.
See the [Robotnik package](https://github.com/RobotnikAutomation/robotnik_pad) for its plugin behavior.

## Architecture

The package owns configuration and orchestration, not joystick or motion-control algorithms:

```text
Linux gamepad
    |
    v
joy_linux_node
    |  sensor_msgs/msg/Joy
    v
robotnik_pad + Movement plugin
    |  geometry_msgs/msg/Twist
    v
configured command topic
```

Only the Movement plugin is enabled by the installed example.
The Robotnik charge plugin and its navigation-message dependency are not part of this package's configured runtime behavior.
This does not remove dependencies that the external `robotnik_pad` repository requires to build all its plugins.

## Launch contract

`teleop_twist_gamepad.launch.py` starts both nodes in one explicit namespace.

| Argument | Default | Responsibility |
| --- | --- | --- |
| `namespace` | empty | Namespace shared by both nodes. |
| `params_file` | installed Logitech F710 YAML | Complete functional configuration for both nodes. |
| `params_file_allow_substs` | `False` | Allow ROS launch substitutions inside the YAML file. |
| `use_sim_time` | `False` | Select the ROS simulation clock for both nodes. |
| `joy_linux_node_args` | standard JSON | Configure the `joy_linux` Node action. |
| `robotnik_pad_node_args` | standard JSON | Configure the `robotnik_pad` Node action. |

Both node-argument defaults are:

```json
{"output":"both","ros_arguments":["--log-level","info"]}
```

The launch file creates one `ParameterFile` and passes that same object to both nodes.
When substitutions are enabled, the shared object ensures that launch renders the YAML at most once.
The launch-owned `use_sim_time` parameter is appended after the YAML for both nodes.

The YAML keys must match the effective node names.
If a caller changes a node name through `node_args`, that caller must update the YAML node key.

## Destructive API changes

The launch file no longer exposes individual node parameters or the old helper arguments.
Configure functional behavior in the YAML file and action behavior in the corresponding `node_args`.

Removed groups include:

- `joy_linux_dev`, `joy_linux_deadzone` and the other `joy_linux_*` parameter overrides.
- `robotnik_pad_desired_freq` and the other `robotnik_pad_*` parameter overrides.
- `joy_linux_node_remappings`.
- `joy_linux_node_options` and `robotnik_pad_node_options`.
- `joy_linux_node_logging_options` and `robotnik_pad_node_logging_options`.

A structured remapping now belongs in `joy_linux_node_args`:

```json
{
  "output": "both",
  "remappings": [["joy", "joypad"]],
  "ros_arguments": ["--log-level", "debug"]
}
```

## Installed Logitech F710 configuration

The installed `config/example_logitech_f710_teleoperation.yaml` configures:

- `/dev/input/js0` as the joystick device.
- A `0.05` axis deadzone.
- A 30 Hz joystick autorepeat rate.
- The Robotnik Movement plugin.
- Linear and angular speed limits.
- Deadman, speed and kinematic-mode buttons.
- The command topic `twist_cmd/joypad`.

The parameter file intentionally omits `use_sim_time`.
Clock selection belongs to the launch file.

Launch the default configuration:

```bash
ros2 launch teleop_twist_gamepad teleop_twist_gamepad.launch.py
```

Use another gamepad configuration:

```bash
ros2 launch teleop_twist_gamepad teleop_twist_gamepad.launch.py \
  namespace:=robot_01 \
  params_file:=/absolute/path/to/gamepad.yaml
```

Change the joystick topic and enable debug logging:

```bash
ros2 launch teleop_twist_gamepad teleop_twist_gamepad.launch.py \
  joy_linux_node_args:='{"output":"both","remappings":[["joy","joypad"]],"ros_arguments":["--log-level","debug"]}'
```

## Dependencies

Runtime dependencies are:

- `joy_linux`
- `robotnik_pad`
- `robotnik_pad_plugins`
- `ros2_launch_helpers`
- ROS 2 launch libraries

`deps.repos` pins the external Robotnik package and the source repository used for
`ros2_launch_helpers`.
The repository file complements `package.xml`; it does not replace runtime dependency declarations.

## Build and test

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install --packages-select teleop_twist_gamepad
source install/setup.bash
colcon test --merge-install --packages-select teleop_twist_gamepad
colcon test-result --test-result-base build/teleop_twist_gamepad --verbose
```
