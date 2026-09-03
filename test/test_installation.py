"""Test resources installed by teleop_twist_gamepad."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def test_documentation_and_runtime_resources_are_installed() -> None:
    """Keep package-index resources aligned with the documented launch contract."""
    share_directory = Path(get_package_share_directory('teleop_twist_gamepad'))

    assert (share_directory / 'README.md').is_file()
    assert (share_directory / 'LICENSE').is_file()
    assert (share_directory / 'launch' / 'teleop_twist_gamepad.launch.py').is_file()
    assert (share_directory / 'config' / 'example_logitech_f710_teleoperation.yaml').is_file()
