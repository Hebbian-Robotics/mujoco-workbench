"""Behavior tests for parsing dynamic scene modules into `LoadedScene`."""

from __future__ import annotations

import sys
from types import ModuleType

import pytest

from mujoco_workbench.arm_handles import ArmSide
from mujoco_workbench.runtime import load_scene


def _install_scene_module(module_name: str, **attrs: object) -> None:
    module = ModuleType(module_name)

    def build_spec() -> object:
        raise NotImplementedError

    def apply_initial_state() -> None:
        return None

    module.build_spec = build_spec
    module.apply_initial_state = apply_initial_state
    for attr_name, attr_value in attrs.items():
        setattr(module, attr_name, attr_value)
    sys.modules[module_name] = module


def test_load_scene_parses_finite_scene_metadata() -> None:
    module_name = "tests._fake_valid_loaded_scene"
    _install_scene_module(
        module_name,
        NAME="Fake scene",
        ARM_PREFIXES=("left/", "right/"),
        ROBOT_KIND="ur10e",
        GRIPPABLES=("server",),
        AUX_ACTUATOR_NAMES=("base_x",),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.display_name == "Fake scene"
    assert loaded_scene.arm_sides == (ArmSide.LEFT, ArmSide.RIGHT)
    assert loaded_scene.robot_kind == "ur10e"
    assert loaded_scene.grippable_names == ("server",)
    assert loaded_scene.aux_actuator_names == ("base_x",)


def test_load_scene_rejects_unknown_robot_kind() -> None:
    module_name = "tests._fake_bad_robot_kind_scene"
    _install_scene_module(module_name, ROBOT_KIND="quadruped")

    with pytest.raises(ValueError, match="unsupported ROBOT_KIND"):
        load_scene(module_name)


def test_load_scene_rejects_unknown_arm_side() -> None:
    module_name = "tests._fake_bad_arm_side_scene"
    _install_scene_module(module_name, ARM_PREFIXES=("middle/",))

    with pytest.raises(ValueError, match="unsupported ARM_PREFIXES"):
        load_scene(module_name)
