"""Contract tests for hosted policy plumbing that does not hit the network."""

from __future__ import annotations

from collections.abc import Mapping

import mujoco
import numpy as np
import pytest

from examples.scenes import franka_libero_pi
from mujoco_workbench.arm_handles import ArmHandles, ArmSide, RobotKind, get_arm_handles
from mujoco_workbench.embodiments.droid import (
    DROID_OBSERVATION_KEYS,
    DroidCameraName,
    build_droid_observation,
    resize_droid_policy_image,
)
from mujoco_workbench.embodiments.libero import (
    LIBERO_OBSERVATION_KEYS,
    LiberoCameraName,
    build_libero_observation,
    libero_action_to_actuator_ctrl,
    resize_libero_policy_image,
)
from mujoco_workbench.policy_client import (
    HostedPolicyActionChunkClient,
    HostedPolicyClient,
    parse_policy_inference_response,
)
from mujoco_workbench.policy_types import (
    PolicyActionInterpretation,
    make_policy_endpoint,
    make_policy_prompt,
    parse_policy_action_interpretation,
)


def _make_three_actuator_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="body">
              <joint name="joint0" type="hinge"/>
              <joint name="joint1" type="hinge"/>
              <joint name="joint2" type="hinge"/>
              <geom type="sphere" size="0.01"/>
            </body>
          </worldbody>
          <actuator>
            <position name="actuator0" joint="joint0" ctrlrange="0 1"/>
            <position name="actuator1" joint="joint1" ctrlrange="-1 1"/>
            <position name="actuator2" joint="joint2" ctrlrange="0 255"/>
          </actuator>
        </mujoco>
        """
    )


def _make_droid_action_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="body0">
              <joint name="joint0" type="hinge"/>
              <geom type="sphere" size="0.01"/>
              <body name="body1">
                <joint name="joint1" type="hinge"/>
                <geom type="sphere" size="0.01"/>
                <body name="body2">
                  <joint name="joint2" type="hinge"/>
                  <geom type="sphere" size="0.01"/>
                  <body name="body3">
                    <joint name="joint3" type="hinge"/>
                    <geom type="sphere" size="0.01"/>
                    <body name="body4">
                      <joint name="joint4" type="hinge"/>
                      <geom type="sphere" size="0.01"/>
                      <body name="body5">
                        <joint name="joint5" type="hinge"/>
                        <geom type="sphere" size="0.01"/>
                        <body name="body6">
                          <joint name="joint6" type="hinge"/>
                          <joint name="finger_joint" type="slide"/>
                          <geom type="sphere" size="0.01"/>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </worldbody>
          <actuator>
            <position name="actuator0" joint="joint0" ctrlrange="-1 1"/>
            <position name="actuator1" joint="joint1" ctrlrange="-2 2"/>
            <position name="actuator2" joint="joint2" ctrlrange="-3 3"/>
            <position name="actuator3" joint="joint3" ctrlrange="-4 4"/>
            <position name="actuator4" joint="joint4" ctrlrange="-5 5"/>
            <position name="actuator5" joint="joint5" ctrlrange="-6 6"/>
            <position name="actuator6" joint="joint6" ctrlrange="-7 7"/>
            <position name="actuator7" joint="finger_joint" ctrlrange="0 255"/>
          </actuator>
        </mujoco>
        """
    )


class _FakeHostedPolicy:
    def __init__(self) -> None:
        self.infer_calls = 0
        self.reset_calls = 0
        self.close_calls = 0

    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        assert obs == {"prompt": "pick"}
        self.infer_calls += 1
        return {
            "actions": np.array(
                [
                    [2.0, -2.0, 300.0],
                    [0.25, 0.5, 128.0],
                ],
                dtype=float,
            ),
            "server_timing": {"infer_ms": 42.0},
        }

    def get_server_metadata(self) -> Mapping[str, object]:
        return {"action_horizon": 2}

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.close_calls += 1


class _DroidFakeHostedPolicy(_FakeHostedPolicy):
    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        assert obs["prompt"] == "pick"
        self.infer_calls += 1
        return {
            "actions": np.array(
                [
                    [2.0, -2.5, 0.25, -0.5, 1.0, 1.5, -1.5, 1.2],
                    [0.25, 0.5, -0.25, -0.75, 0.0, 0.25, -0.25, 0.5],
                ],
                dtype=float,
            ),
        }


class _DroidDeltaFakeHostedPolicy(_FakeHostedPolicy):
    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        assert obs["prompt"] == "pick"
        self.infer_calls += 1
        return {
            "actions": np.array(
                [
                    [-0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.7, 0.25],
                ],
                dtype=float,
            ),
        }


class _DroidVelocityFakeHostedPolicy(_FakeHostedPolicy):
    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        assert obs["prompt"] == "pick"
        self.infer_calls += 1
        return {
            "actions": np.array(
                [
                    [2.0, -2.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.49],
                    [0.3, 0.6, -0.9, 1.2, 0.0, 0.0, 0.0, 0.51],
                    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.7],
                ],
                dtype=float,
            ),
        }


class _LiberoFakeHostedPolicy(_FakeHostedPolicy):
    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        assert obs["prompt"] == "pick"
        self.infer_calls += 1
        return {
            "actions": np.array(
                [
                    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -1.0],
                    [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, 1.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                ],
                dtype=float,
            ),
        }


def test_policy_client_buffers_and_clamps_action_chunks() -> None:
    model = _make_three_actuator_model()
    fake_policy = _FakeHostedPolicy()
    endpoint = make_policy_endpoint(host="127.0.0.1", port=5555)

    client = HostedPolicyClient(
        endpoint=endpoint,
        model=model,
        policy_factory=lambda _endpoint: fake_policy,
    )

    first_action = client.next_actuator_ctrl({"prompt": "pick"})
    second_action = client.next_actuator_ctrl({"prompt": "pick"})

    np.testing.assert_allclose(first_action, np.array([1.0, -1.0, 255.0]))
    np.testing.assert_allclose(second_action, np.array([0.25, 0.5, 128.0]))
    assert fake_policy.infer_calls == 1
    assert client.metadata == {"action_horizon": 2}
    assert client.last_server_timing == {"infer_ms": 42.0}


def test_droid_action_interpretation_scales_normalized_gripper() -> None:
    model = _make_droid_action_model()
    fake_policy = _DroidFakeHostedPolicy()
    endpoint = make_policy_endpoint(host="127.0.0.1", port=5555)

    client = HostedPolicyClient(
        endpoint=endpoint,
        model=model,
        action_interpretation=PolicyActionInterpretation.DROID_ABSOLUTE_JOINT_POSITION,
        policy_factory=lambda _endpoint: fake_policy,
    )

    first_action = client.next_actuator_ctrl({"prompt": "pick"})
    second_action = client.next_actuator_ctrl({"prompt": "pick"})

    np.testing.assert_allclose(
        first_action,
        np.array([1.0, -2.0, 0.25, -0.5, 1.0, 1.5, -1.5, 255.0]),
    )
    np.testing.assert_allclose(
        second_action,
        np.array([0.25, 0.5, -0.25, -0.75, 0.0, 0.25, -0.25, 127.5]),
    )
    np.testing.assert_allclose(client.last_raw_action_batch[0, -1], 1.2)
    np.testing.assert_allclose(client.last_actuator_ctrl_batch[0, -1], 255.0)


def test_droid_joint_delta_action_interpretation_adds_observed_joint_position() -> None:
    model = _make_droid_action_model()
    fake_policy = _DroidDeltaFakeHostedPolicy()
    endpoint = make_policy_endpoint(host="127.0.0.1", port=5555)
    client = HostedPolicyClient(
        endpoint=endpoint,
        model=model,
        action_interpretation=PolicyActionInterpretation.DROID_JOINT_DELTA,
        policy_factory=lambda _endpoint: fake_policy,
    )
    obs = {
        "prompt": "pick",
        "observation/joint_position": np.array([0.0, -0.8, 0.0, -2.3, 0.0, 1.5, 0.7]),
    }

    actuator_ctrl = client.next_actuator_ctrl(obs)

    np.testing.assert_allclose(
        actuator_ctrl,
        np.array([-0.1, -0.6, 0.3, -2.7, 0.5, 0.9, 1.4, 63.75]),
    )


def test_droid_joint_velocity_action_interpretation_integrates_at_control_rate() -> None:
    model = _make_droid_action_model()
    fake_policy = _DroidVelocityFakeHostedPolicy()
    endpoint = make_policy_endpoint(host="127.0.0.1", port=5555)
    client = HostedPolicyClient(
        endpoint=endpoint,
        model=model,
        action_interpretation=PolicyActionInterpretation.DROID_JOINT_VELOCITY,
        max_buffered_actions=2,
        droid_velocity_time_step_seconds=0.1,
        policy_factory=lambda _endpoint: fake_policy,
    )
    obs = {
        "prompt": "pick",
        "observation/joint_position": np.array([0.0, -0.8, 0.0, -2.3, 0.0, 1.5, 0.7]),
    }

    first_action = client.next_actuator_ctrl(obs)
    second_action = client.next_actuator_ctrl(obs)

    np.testing.assert_allclose(
        first_action,
        np.array([0.1, -0.9, 0.05, -2.3, 0.0, 1.5, 0.7, 0.0]),
    )
    np.testing.assert_allclose(
        second_action,
        np.array([0.13, -0.84, -0.04, -2.2, 0.0, 1.5, 0.7, 255.0]),
    )
    assert fake_policy.infer_calls == 1
    assert client.last_raw_action_batch.shape == (3, 8)
    assert client.last_actuator_ctrl_batch.shape == (2, 8)


def test_raw_policy_action_chunk_client_buffers_non_actuator_actions() -> None:
    fake_policy = _LiberoFakeHostedPolicy()
    endpoint = make_policy_endpoint(host="127.0.0.1", port=5555)
    client = HostedPolicyActionChunkClient(
        endpoint=endpoint,
        expected_action_width=7,
        max_buffered_actions=2,
        policy_factory=lambda _endpoint: fake_policy,
    )
    obs = {"prompt": "pick"}

    first_action = client.next_action(obs)
    second_action = client.next_action(obs)

    np.testing.assert_allclose(first_action, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -1.0]))
    np.testing.assert_allclose(
        second_action,
        np.array([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, 1.0]),
    )
    assert fake_policy.infer_calls == 1
    assert client.last_raw_action_batch.shape == (3, 7)


def test_policy_action_interpretation_is_explicit() -> None:
    assert (
        parse_policy_action_interpretation("direct_actuator_ctrl")
        is PolicyActionInterpretation.DIRECT_ACTUATOR_CTRL
    )
    assert (
        parse_policy_action_interpretation("droid_absolute_joint_position")
        is PolicyActionInterpretation.DROID_ABSOLUTE_JOINT_POSITION
    )
    assert (
        parse_policy_action_interpretation("droid_joint_delta")
        is PolicyActionInterpretation.DROID_JOINT_DELTA
    )
    assert (
        parse_policy_action_interpretation("droid_joint_velocity")
        is PolicyActionInterpretation.DROID_JOINT_VELOCITY
    )
    with pytest.raises(ValueError, match="unsupported policy action interpretation"):
        parse_policy_action_interpretation("joint_delta")


def test_policy_endpoint_rejects_invalid_cli_values() -> None:
    with pytest.raises(ValueError, match="policy host"):
        make_policy_endpoint(host="  ", port=5555)
    with pytest.raises(ValueError, match="policy port"):
        make_policy_endpoint(host="127.0.0.1", port=0)
    with pytest.raises(ValueError, match="policy local port"):
        make_policy_endpoint(host="127.0.0.1", port=5555, local_port=0)
    with pytest.raises(ValueError, match="policy prompt"):
        make_policy_prompt("  ")


def test_policy_response_parser_rejects_malformed_actions() -> None:
    with pytest.raises(KeyError, match="actions"):
        parse_policy_inference_response({}, expected_action_width=8)
    with pytest.raises(ValueError, match="action width"):
        parse_policy_inference_response({"actions": np.zeros((2, 7))}, expected_action_width=8)
    with pytest.raises(ValueError, match="empty action chunk"):
        parse_policy_inference_response({"actions": np.zeros((0, 8))}, expected_action_width=8)


def _make_droid_observation_model() -> mujoco.MjModel:
    chain_xml = """
              <body name="left/link1">
                <joint name="left/joint1" type="hinge"/>
                <geom type="sphere" size="0.01"/>
                <body name="left/link2">
                  <joint name="left/joint2" type="hinge"/>
                  <geom type="sphere" size="0.01"/>
                  <body name="left/link3">
                    <joint name="left/joint3" type="hinge"/>
                    <geom type="sphere" size="0.01"/>
                    <body name="left/link4">
                      <joint name="left/joint4" type="hinge"/>
                      <geom type="sphere" size="0.01"/>
                      <body name="left/link5">
                        <joint name="left/joint5" type="hinge"/>
                        <geom type="sphere" size="0.01"/>
                        <body name="left/link6">
                          <joint name="left/joint6" type="hinge"/>
                          <geom type="sphere" size="0.01"/>
                          <body name="left/link7">
                            <joint name="left/joint7" type="hinge"/>
                            <joint name="left/finger_joint" type="slide"/>
                            <geom type="sphere" size="0.01"/>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
    """
    return mujoco.MjModel.from_xml_string(
        f"""
        <mujoco>
          <worldbody>
            <camera name="cam_exterior" pos="0 -1 1"/>
            <camera name="left/gripper/cam_wrist" pos="0 0 1"/>
            {chain_xml}
          </worldbody>
          <actuator>
            <position
              name="left/gripper/fingers_actuator"
              joint="left/finger_joint"
              ctrlrange="0 255"
            />
          </actuator>
        </mujoco>
        """
    )


def test_droid_observation_matches_hosted_warmup_schema() -> None:
    model = _make_droid_observation_model()
    data = mujoco.MjData(model)
    expected_joint_positions = np.arange(7, dtype=float)
    for joint_index, joint_position in enumerate(expected_joint_positions, start=1):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"left/joint{joint_index}")
        data.qpos[int(model.jnt_qposadr[joint_id])] = joint_position
    data.ctrl[0] = 127.5

    def render_camera(camera_name: DroidCameraName) -> np.ndarray:
        if camera_name is DroidCameraName.EXTERIOR:
            return np.zeros((224, 224, 3), dtype=np.uint8)
        if camera_name is DroidCameraName.LEFT_WRIST:
            return np.ones((224, 224, 3), dtype=np.uint8)
        raise AssertionError(f"unexpected camera {camera_name}")

    observation = build_droid_observation(
        model,
        data,
        render_camera,
        "pick up the red cube",
    )

    assert tuple(observation) == DROID_OBSERVATION_KEYS
    assert observation["observation/exterior_image_1_left"].shape == (224, 224, 3)
    assert observation["observation/exterior_image_1_left"].dtype == np.uint8
    assert observation["observation/wrist_image_left"].shape == (224, 224, 3)
    assert observation["observation/wrist_image_left"].dtype == np.uint8
    np.testing.assert_allclose(
        observation["observation/joint_position"],
        expected_joint_positions,
    )
    np.testing.assert_allclose(observation["observation/gripper_position"], np.array([0.5]))
    assert observation["prompt"] == "pick up the red cube"
    assert observation["mode"] == "action_only"


def test_droid_policy_image_resize_pads_sixteen_by_nine_frames() -> None:
    rendered_image = np.full((180, 320, 3), 127, dtype=np.uint8)

    policy_image = resize_droid_policy_image(rendered_image)

    assert policy_image.shape == (224, 224, 3)
    assert policy_image.dtype == np.uint8
    np.testing.assert_array_equal(policy_image[:49], 0)
    np.testing.assert_array_equal(policy_image[49:175], 127)
    np.testing.assert_array_equal(policy_image[175:], 0)


def test_droid_observation_rejects_wrong_image_shape() -> None:
    model = _make_droid_observation_model()
    data = mujoco.MjData(model)

    with pytest.raises(ValueError, match="observation/exterior_image_1_left"):
        build_droid_observation(
            model,
            data,
            lambda _camera_name: np.zeros((64, 64, 3), dtype=np.uint8),
            "prompt",
        )


def _make_libero_observation_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="left/hand" pos="0.1 0.2 0.3">
              <joint name="left/finger_joint1" type="slide" range="0 0.04"/>
              <joint name="left/finger_joint2" type="slide" range="0 0.04"/>
              <site name="left/tcp" pos="0 0 0.1"/>
              <geom type="sphere" size="0.01"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def _fake_libero_arm(model: mujoco.MjModel) -> ArmHandles:
    tcp_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left/tcp")
    return ArmHandles(
        side=ArmSide.LEFT,
        robot_kind=RobotKind.FRANKA_PANDA,
        qpos_idx=np.zeros(7, dtype=np.int64),
        dof_idx=np.zeros(7, dtype=np.int64),
        jnt_ids=np.zeros(7, dtype=np.int64),
        arm_dof_idx=np.zeros(7, dtype=np.int64),
        act_arm_ids=np.zeros(7, dtype=np.int64),
        act_gripper_id=0,
        link6_id=0,
        tcp_site_id=tcp_site_id,
        gripper_open=255.0,
        gripper_closed=0.0,
        weld_ids=np.zeros(0, dtype=np.int64),
    )


def test_libero_observation_matches_openpi_schema() -> None:
    model = _make_libero_observation_model()
    data = mujoco.MjData(model)
    first_finger_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left/finger_joint1")
    second_finger_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left/finger_joint2")
    data.qpos[int(model.jnt_qposadr[first_finger_id])] = 0.04
    data.qpos[int(model.jnt_qposadr[second_finger_id])] = 0.02
    mujoco.mj_forward(model, data)

    def render_camera(camera_name: LiberoCameraName) -> np.ndarray:
        if camera_name is LiberoCameraName.AGENTVIEW:
            return np.zeros((224, 224, 3), dtype=np.uint8)
        if camera_name is LiberoCameraName.WRIST:
            return np.ones((224, 224, 3), dtype=np.uint8)
        raise AssertionError(f"unexpected camera {camera_name}")

    observation = build_libero_observation(
        model,
        data,
        _fake_libero_arm(model),
        render_camera,
        "pick up the red cube",
    )

    assert tuple(observation) == LIBERO_OBSERVATION_KEYS
    assert observation["observation/image"].shape == (224, 224, 3)
    assert observation["observation/wrist_image"].shape == (224, 224, 3)
    assert observation["observation/state"].shape == (8,)
    expected_tcp_position = np.asarray(data.site_xpos[_fake_libero_arm(model).tcp_site_id])
    np.testing.assert_allclose(observation["observation/state"][:3], expected_tcp_position)
    np.testing.assert_allclose(observation["observation/state"][3:6], np.zeros(3))
    np.testing.assert_allclose(observation["observation/state"][6:], np.array([0.04, 0.02]))
    assert observation["prompt"] == "pick up the red cube"


def test_libero_policy_image_resize_rotates_training_frames() -> None:
    rendered_image = np.zeros((224, 224, 3), dtype=np.uint8)
    rendered_image[0, 0] = np.array([255, 0, 0], dtype=np.uint8)
    rendered_image[-1, -1] = np.array([0, 255, 0], dtype=np.uint8)

    policy_image = resize_libero_policy_image(rendered_image)

    assert policy_image.shape == (224, 224, 3)
    assert policy_image.dtype == np.uint8
    np.testing.assert_array_equal(policy_image[0, 0], np.array([0, 255, 0], dtype=np.uint8))
    np.testing.assert_array_equal(policy_image[-1, -1], np.array([255, 0, 0], dtype=np.uint8))


def test_libero_action_mapping_applies_incremental_robosuite_gripper_commands() -> None:
    model, data = franka_libero_pi.build_spec()
    arm = get_arm_handles(model, franka_libero_pi.MANIPULATORS[0], franka_libero_pi.N_CUBES)
    franka_libero_pi.apply_initial_state(model, data, {ArmSide.LEFT: arm}, [])
    scratch_data = mujoco.MjData(model)

    hold_action = np.zeros(7, dtype=float)
    hold_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, hold_action)
    np.testing.assert_allclose(hold_ctrl[arm.act_gripper_id], arm.gripper_open)

    close_action = np.zeros(7, dtype=float)
    close_action[-1] = 1.0
    close_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, close_action)
    expected_close_step = arm.gripper_open + 0.1 * (arm.gripper_closed - arm.gripper_open)
    np.testing.assert_allclose(close_ctrl[arm.act_gripper_id], expected_close_step)

    data.ctrl[arm.act_gripper_id] = arm.gripper_closed
    open_action = np.zeros(7, dtype=float)
    open_action[-1] = -1.0
    open_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, open_action)
    expected_open_step = arm.gripper_open + 0.9 * (arm.gripper_closed - arm.gripper_open)
    np.testing.assert_allclose(open_ctrl[arm.act_gripper_id], expected_open_step)
