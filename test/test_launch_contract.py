"""Test the public teleop_twist_gamepad launch contract."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


def _load_launch_module() -> ModuleType:
    """Load the source launch file as a Python module."""
    path = PACKAGE_ROOT / 'launch' / 'teleop_twist_gamepad.launch.py'
    spec = importlib.util.spec_from_file_location('teleop_twist_gamepad_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_exposes_only_shared_configuration_and_node_arguments() -> None:
    """Protect the compact launch API and prevent functional parameter overrides."""
    module = _load_launch_module()
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert set(declarations) == {
        'namespace',
        'params_file',
        'params_file_allow_substs',
        'use_sim_time',
        'joy_linux_node_args',
        'robotnik_pad_node_args',
    }

    for argument_name in ('joy_linux_node_args', 'robotnik_pad_node_args'):
        context = LaunchContext()
        declarations[argument_name].visit(context)
        assert context.launch_configurations[argument_name] == DEFAULT_NODE_ARGS


@pytest.mark.parametrize('allow_substs', ['True', 'False'])
def test_launch_uses_one_parameter_file_for_both_nodes(
    allow_substs: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Render one shared YAML file and append the launch-owned clock to both nodes."""
    module = _load_launch_module()
    created_nodes: list[dict[str, object]] = []
    created_parameter_files: list[object] = []

    class FakeParameterFile:
        def __init__(self, path: object, *, allow_substs: bool) -> None:
            self.path = path
            self.allow_substs = allow_substs
            created_parameter_files.append(self)

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            created_nodes.append(kwargs)

    monkeypatch.setattr(module, 'ParameterFile', FakeParameterFile)
    monkeypatch.setattr(module, 'Node', FakeNode)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': 'robot',
            'params_file': '/tmp/teleop.yaml',
            'params_file_allow_substs': allow_substs,
            'use_sim_time': 'False',
            'joy_linux_node_args': DEFAULT_NODE_ARGS,
            'robotnik_pad_node_args': DEFAULT_NODE_ARGS,
        }
    )

    actions = module._launch_nodes(context)

    assert len(actions) == 2
    assert len(created_parameter_files) == 1
    assert created_parameter_files[0].allow_substs is (allow_substs == 'True')
    assert [node['name'] for node in created_nodes] == ['joy_linux', 'robotnik_pad']
    assert [node['package'] for node in created_nodes] == ['joy_linux', 'robotnik_pad']
    assert all(node['parameters'][0] is created_parameter_files[0] for node in created_nodes)
    assert all(set(node['parameters'][1]) == {'use_sim_time'} for node in created_nodes)
