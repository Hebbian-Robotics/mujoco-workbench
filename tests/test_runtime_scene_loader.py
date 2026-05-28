"""Behavior tests for parsing dynamic scene modules into `LoadedScene`."""

from __future__ import annotations

import sys
from types import ModuleType

import pytest

from mujoco_workbench.arm_handles import (
    ArmSide,
    franka_panda_manipulator_spec,
    piper_manipulator_spec,
    ur10e_robotiq_manipulator_spec,
)
from mujoco_workbench.runtime import load_scene
from mujoco_workbench.scene_base import ViserCameraPose


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
        MANIPULATORS=(
            ur10e_robotiq_manipulator_spec(ArmSide.LEFT),
            ur10e_robotiq_manipulator_spec(ArmSide.RIGHT),
        ),
        GRIPPABLES=("server",),
        BASE_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
        LIFT_ACTUATOR_NAME="torso_lift",
        AUX_ACTUATOR_NAMES=("indicator_light",),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.display_name == "Fake scene"
    assert loaded_scene.manipulators == (
        ur10e_robotiq_manipulator_spec(ArmSide.LEFT),
        ur10e_robotiq_manipulator_spec(ArmSide.RIGHT),
    )
    assert loaded_scene.arm_sides == (ArmSide.LEFT, ArmSide.RIGHT)
    assert loaded_scene.grippable_names == ("server",)
    assert loaded_scene.aux_actuator_names == ("indicator_light",)
    assert loaded_scene.mobile_base is not None
    assert loaded_scene.mobile_base.actuator_names == ("base_x", "base_y", "base_yaw")
    assert loaded_scene.lift is not None
    assert loaded_scene.lift.actuator_name == "torso_lift"


def test_load_scene_accepts_explicit_per_manipulator_specs() -> None:
    module_name = "tests._fake_explicit_manipulator_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(
            piper_manipulator_spec(ArmSide.LEFT),
            ur10e_robotiq_manipulator_spec(ArmSide.RIGHT),
        ),
        BASE_ACTUATOR_NAMES=("drive_x", "drive_y", "drive_yaw"),
        LIFT_ACTUATOR_NAME="lift",
        AUX_ACTUATOR_NAMES=("server_latch",),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.manipulators == (
        piper_manipulator_spec(ArmSide.LEFT),
        ur10e_robotiq_manipulator_spec(ArmSide.RIGHT),
    )
    assert loaded_scene.mobile_base is not None
    assert loaded_scene.mobile_base.actuator_names == ("drive_x", "drive_y", "drive_yaw")
    assert loaded_scene.lift is not None
    assert loaded_scene.lift.actuator_name == "lift"
    assert loaded_scene.aux_actuator_names == ("server_latch",)


def test_load_scene_parses_policy_free_play_factory() -> None:
    module_name = "tests._fake_policy_factory_scene"

    def make_step_free_play(**_kwargs: object) -> object:
        return object()

    _install_scene_module(
        module_name,
        MANIPULATORS=(franka_panda_manipulator_spec(ArmSide.LEFT),),
        make_step_free_play=make_step_free_play,
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.make_step_free_play is make_step_free_play


def test_load_scene_parses_camera_feed_preprocessing() -> None:
    module_name = "tests._fake_camera_feed_scene"

    def preprocess_camera_feed(_camera_name: str, image: object) -> object:
        return image

    _install_scene_module(
        module_name,
        MANIPULATORS=(piper_manipulator_spec(ArmSide.LEFT),),
        CAMERA_FEED_RENDER_WIDTH=320,
        CAMERA_FEED_RENDER_HEIGHT=180,
        preprocess_camera_feed=preprocess_camera_feed,
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.camera_feed.render_width == 320
    assert loaded_scene.camera_feed.render_height == 180
    assert loaded_scene.camera_feed.preprocess is preprocess_camera_feed


def test_load_scene_parses_default_viser_camera_pose() -> None:
    module_name = "tests._fake_default_viser_camera_scene"
    pose = ViserCameraPose(
        position=(0.274, -1.800, 1.795),
        lookat=(1.477, -0.625, 0.977),
    )
    _install_scene_module(
        module_name,
        MANIPULATORS=(piper_manipulator_spec(ArmSide.LEFT),),
        DEFAULT_VISER_CAMERA_POSE=pose,
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.default_viser_camera_pose == pose


def test_load_scene_rejects_missing_manipulators() -> None:
    module_name = "tests._fake_missing_manipulators_scene"
    _install_scene_module(module_name)

    with pytest.raises(ValueError, match="must declare MANIPULATORS"):
        load_scene(module_name)


def test_load_scene_does_not_infer_base_from_aux_actuators() -> None:
    module_name = "tests._fake_aux_only_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(piper_manipulator_spec(ArmSide.LEFT),),
        AUX_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
    )

    loaded_scene = load_scene(module_name)

    assert loaded_scene.mobile_base is None
    assert loaded_scene.aux_actuator_names == ("base_x", "base_y", "base_yaw")


def test_load_scene_rejects_component_actuators_in_aux_actuators() -> None:
    module_name = "tests._fake_duplicate_component_actuator_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(piper_manipulator_spec(ArmSide.LEFT),),
        BASE_ACTUATOR_NAMES=("base_x", "base_y", "base_yaw"),
        LIFT_ACTUATOR_NAME="lift",
        AUX_ACTUATOR_NAMES=("lift",),
    )

    with pytest.raises(ValueError, match="again in AUX_ACTUATOR_NAMES"):
        load_scene(module_name)


def test_load_scene_rejects_more_than_two_manipulators() -> None:
    module_name = "tests._fake_too_many_manipulators_scene"
    _install_scene_module(
        module_name,
        MANIPULATORS=(
            piper_manipulator_spec(ArmSide.LEFT),
            piper_manipulator_spec(ArmSide.RIGHT),
            ur10e_robotiq_manipulator_spec(ArmSide.LEFT),
        ),
    )

    with pytest.raises(ValueError, match="only unimanual and bimanual"):
        load_scene(module_name)
