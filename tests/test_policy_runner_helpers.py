"""Behavior tests for runner policy-mode helper functions."""

from __future__ import annotations

import sys
from types import ModuleType

import mujoco

from mujoco_workbench.arm_handles import ArmSide, franka_panda_manipulator_spec
from mujoco_workbench.policy_types import PolicyEndpoint, PolicyPrompt, make_policy_endpoint
from mujoco_workbench.runner import (
    build_policy_step_free_play,
    clear_step_free_play_action_buffer,
    close_step_free_play,
    prewarm_step_free_play,
    reset_step_free_play,
    set_step_free_play_prompt,
)
from mujoco_workbench.runtime import load_scene


class _FakePolicyFreePlay:
    def __init__(self) -> None:
        self.prewarm_count = 0
        self.reset_count = 0
        self.close_count = 0
        self.clear_action_buffer_count = 0
        self.tick_count = 0
        self.prompt: PolicyPrompt | None = None

    def prewarm(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        del model, data
        self.prewarm_count += 1

    def reset(self) -> None:
        self.reset_count += 1

    def close(self) -> None:
        self.close_count += 1

    def set_prompt(self, prompt: PolicyPrompt) -> None:
        self.prompt = prompt

    def clear_action_buffer(self) -> None:
        self.clear_action_buffer_count += 1

    def __call__(self, t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        del t, model, data
        self.tick_count += 1


def _make_model_and_data() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="body">
              <geom type="sphere" size="0.01"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    return model, mujoco.MjData(model)


def _install_policy_scene_module(module_name: str, fake_free_play: _FakePolicyFreePlay) -> None:
    module = ModuleType(module_name)
    module.MANIPULATORS = (franka_panda_manipulator_spec(ArmSide.LEFT),)

    def build_spec() -> tuple[mujoco.MjModel, mujoco.MjData]:
        return _make_model_and_data()

    def apply_initial_state() -> None:
        return None

    def make_step_free_play(
        *,
        policy_endpoint: PolicyEndpoint,
        prompt: PolicyPrompt,
    ) -> _FakePolicyFreePlay:
        assert policy_endpoint == make_policy_endpoint(
            host="127.0.0.1",
            port=5555,
            local_port=5557,
        )
        assert prompt == PolicyPrompt("pick")
        return fake_free_play

    module.build_spec = build_spec
    module.apply_initial_state = apply_initial_state
    module.make_step_free_play = make_step_free_play
    sys.modules[module_name] = module


def test_policy_step_free_play_factory_and_lifecycle_hooks() -> None:
    module_name = "tests._fake_runner_policy_scene"
    fake_free_play = _FakePolicyFreePlay()
    _install_policy_scene_module(module_name, fake_free_play)
    loaded_scene = load_scene(module_name)
    model, data = _make_model_and_data()

    active_step_free_play = build_policy_step_free_play(
        loaded_scene,
        scene_module_name=module_name,
        policy_endpoint=make_policy_endpoint(host="127.0.0.1", port=5555, local_port=5557),
        policy_prompt=PolicyPrompt("pick"),
    )

    assert prewarm_step_free_play(active_step_free_play, model, data)
    assert reset_step_free_play(active_step_free_play)
    assert set_step_free_play_prompt(active_step_free_play, PolicyPrompt("place"))
    assert clear_step_free_play_action_buffer(active_step_free_play)
    assert close_step_free_play(active_step_free_play)
    active_step_free_play(0.0, model, data)

    assert fake_free_play.prewarm_count == 1
    assert fake_free_play.reset_count == 1
    assert fake_free_play.clear_action_buffer_count == 1
    assert fake_free_play.close_count == 1
    assert fake_free_play.tick_count == 1
    assert fake_free_play.prompt == PolicyPrompt("place")


def test_policy_lifecycle_helpers_ignore_plain_free_play_callbacks() -> None:
    model, data = _make_model_and_data()

    def plain_step_free_play(t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        del t, model, data

    assert not prewarm_step_free_play(plain_step_free_play, model, data)
    assert not reset_step_free_play(plain_step_free_play)
    assert not set_step_free_play_prompt(plain_step_free_play, PolicyPrompt("place"))
    assert not clear_step_free_play_action_buffer(plain_step_free_play)
    assert not close_step_free_play(plain_step_free_play)
