"""Contract tests for hosted policy plumbing that does not hit the network."""

from __future__ import annotations

from collections.abc import Mapping

import mujoco
import numpy as np
import pytest

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
