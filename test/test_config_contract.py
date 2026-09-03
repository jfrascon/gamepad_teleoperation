"""Test the installed example teleoperation configuration."""

from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_example_configures_both_nodes_without_owning_the_clock() -> None:
    """Keep functional parameters in YAML and leave use_sim_time to launch."""
    configuration = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'example_logitech_f710_teleoperation.yaml').read_text(
            encoding='utf-8'
        )
    )

    assert set(configuration) == {'/**/joy_linux', '/**/robotnik_pad'}
    joy_parameters = configuration['/**/joy_linux']['ros__parameters']
    pad_parameters = configuration['/**/robotnik_pad']['ros__parameters']
    assert 'use_sim_time' not in joy_parameters
    assert 'use_sim_time' not in pad_parameters
    assert joy_parameters['default_trig_val'] is False
    assert joy_parameters['sticky_buttons'] is False
    assert pad_parameters['plugins'] == ['Movement']
    assert pad_parameters['Movement']['type'] == 'robotnik_pad_plugins/Movement'
