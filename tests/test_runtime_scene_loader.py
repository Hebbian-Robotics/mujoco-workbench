"""Behavior tests for parsing dynamic scene modules into `LoadedScene`."""

from __future__ import annotations

import sys
from types import ModuleType

import pytest

from mujoco_workbench.arm_handles import ArmSide, ManipulatorSpec, RobotKind
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
        BASE_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
        LIFT_ACTUATOR_NAME="torso_lift",
        AUX_ACTUATOR_NAMES=("indicator_light",),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.display_name == "Fake scene"
    assert loaded_scene.manipulators == (
        ManipulatorSpec(side=ArmSide.LEFT, robot_kind=RobotKind.UR10E),
        ManipulatorSpec(side=ArmSide.RIGHT, robot_kind=RobotKind.UR10E),
    )
    assert loaded_scene.arm_sides == (ArmSide.LEFT, ArmSide.RIGHT)
    assert loaded_scene.robot_kind == RobotKind.UR10E
    assert loaded_scene.grippable_names == ("server",)
    assert loaded_scene.aux_actuator_names == ("indicator_light",)
    assert loaded_scene.mobile_base is not None
    assert loaded_scene.mobile_base.actuator_names == ("base_x", "base_y", "base_yaw")
    assert loaded_scene.lift is not None
    assert loaded_scene.lift.actuator_name == "torso_lift"


def test_load_scene_accepts_per_manipulator_robot_kinds() -> None:
    module_name = "tests._fake_per_manipulator_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(
            {"side": "left/", "robot_kind": "piper"},
            {"side": "right/", "robot_kind": "ur10e"},
        ),
        BASE_ACTUATOR_NAMES=("drive_x", "drive_y", "drive_yaw"),
        LIFT_ACTUATOR_NAME="lift",
        AUX_ACTUATOR_NAMES=("server_latch",),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.manipulators == (
        ManipulatorSpec(side=ArmSide.LEFT, robot_kind=RobotKind.PIPER),
        ManipulatorSpec(side=ArmSide.RIGHT, robot_kind=RobotKind.UR10E),
    )
    assert loaded_scene.mobile_base is not None
    assert loaded_scene.mobile_base.actuator_names == ("drive_x", "drive_y", "drive_yaw")
    assert loaded_scene.lift is not None
    assert loaded_scene.lift.actuator_name == "lift"
    assert loaded_scene.aux_actuator_names == ("server_latch",)


def test_load_scene_does_not_infer_base_from_aux_actuators() -> None:
    module_name = "tests._fake_aux_only_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=({"side": "left/", "robot_kind": "piper"},),
        AUX_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.mobile_base is None
    assert loaded_scene.aux_actuator_names == ("base_x", "base_y", "base_yaw")


def test_load_scene_rejects_component_actuators_in_aux_actuators() -> None:
    module_name = "tests._fake_duplicate_component_actuator_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=({"side": "left/", "robot_kind": "piper"},),
        BASE_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
        LIFT_ACTUATOR_NAME="lift",
        AUX_ACTUATOR_NAMES=("lift",),
    )

    with pytest.raises(ValueError, match="again in AUX_ACTUATOR_NAMES"):
        load_scene(module_name)


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


def test_load_scene_rejects_more_than_two_manipulators() -> None:
    module_name = "tests._fake_too_many_manipulators_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(
            {"side": "left/", "robot_kind": "piper"},
            {"side": "right/", "robot_kind": "piper"},
            {"side": "left/", "robot_kind": "ur10e"},
        ),
    )

    with pytest.raises(ValueError, match="only unimanual and bimanual"):
        load_scene(module_name)
