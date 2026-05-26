"""Hosted OpenPI policy client wrapper for MuJoCo control loops."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from importlib import import_module
from typing import Protocol, cast

import mujoco
import numpy as np

from mujoco_workbench.policy_types import PolicyActionInterpretation, PolicyEndpoint

DROID_DEFAULT_CONTROL_PERIOD_SECONDS = 1.0 / 15.0


class HostedPolicy(Protocol):
    """Subset of `FlashTransportPolicy` used by the simulator."""

    def infer(self, obs: Mapping[str, object]) -> Mapping[str, object]:
        """Run policy inference and return a response containing `actions`."""

    def get_server_metadata(self) -> Mapping[str, object]:
        """Return server-advertised metadata."""

    def reset(self) -> None:
        """Reset server-side policy state."""

    def close(self) -> None:
        """Release local transport resources."""


PolicyFactory = Callable[[PolicyEndpoint], HostedPolicy]


@dataclass(frozen=True)
class PolicyActionBatch:
    """Parsed 2D action chunk returned by the hosted policy."""

    rows: np.ndarray


@dataclass(frozen=True)
class PolicyInferenceResult:
    """Parsed policy response with external shapes checked at the boundary."""

    action_batch: PolicyActionBatch
    server_timing: Mapping[str, object]
    policy_timing: Mapping[str, object]


def _default_policy_factory(endpoint: PolicyEndpoint) -> HostedPolicy:
    try:
        flash_transport_policy_module = import_module("openpi_flash_client.flash_transport_policy")
    except ModuleNotFoundError as err:
        raise RuntimeError(
            "Policy mode requires `openpi-flash-client`. Install the hosted OpenPI "
            "client package or run from an environment where "
            "`openpi_flash_client.flash_transport_policy` is importable."
        ) from err
    flash_transport_policy = flash_transport_policy_module.FlashTransportPolicy
    return cast(
        HostedPolicy,
        flash_transport_policy(
            host=endpoint.host,
            port=endpoint.port,
            local_port=endpoint.local_port,
        ),
    )


class HostedPolicyClient:
    """Owns a hosted policy connection and action-chunk buffer."""

    def __init__(
        self,
        *,
        endpoint: PolicyEndpoint,
        model: mujoco.MjModel,
        action_interpretation: PolicyActionInterpretation = (
            PolicyActionInterpretation.DIRECT_ACTUATOR_CTRL
        ),
        max_buffered_actions: int | None = None,
        droid_velocity_time_step_seconds: float = DROID_DEFAULT_CONTROL_PERIOD_SECONDS,
        policy_factory: PolicyFactory = _default_policy_factory,
    ) -> None:
        if max_buffered_actions is not None and max_buffered_actions <= 0:
            raise ValueError(
                f"max_buffered_actions must be positive when set, got {max_buffered_actions}"
            )
        if droid_velocity_time_step_seconds <= 0:
            raise ValueError(
                "droid_velocity_time_step_seconds must be positive, "
                f"got {droid_velocity_time_step_seconds}"
            )
        self._policy = policy_factory(endpoint)
        self._action_interpretation = action_interpretation
        self._actuator_ctrlrange = np.asarray(model.actuator_ctrlrange, dtype=float).copy()
        self._max_buffered_actions = max_buffered_actions
        self._droid_velocity_time_step_seconds = droid_velocity_time_step_seconds
        self._action_buffer: deque[np.ndarray] = deque()
        self.metadata = dict(self._policy.get_server_metadata())
        self.last_server_timing: Mapping[str, object] = {}
        self.last_policy_timing: Mapping[str, object] = {}
        self.last_raw_action_batch = np.zeros((0, self._actuator_ctrlrange.shape[0]), dtype=float)
        self.last_actuator_ctrl_batch = np.zeros_like(self.last_raw_action_batch)

    def prewarm(self, obs: Mapping[str, object]) -> None:
        """Force one inference before the simulation starts ticking."""
        if not self._action_buffer:
            self._fetch_action_chunk(obs)

    def next_actuator_ctrl(self, obs: Mapping[str, object]) -> np.ndarray:
        """Return the next actuator ctrl row, fetching a new chunk when needed."""
        if not self._action_buffer:
            self._fetch_action_chunk(obs)
        if not self._action_buffer:
            raise RuntimeError("policy returned an empty action chunk")
        return self._action_buffer.popleft().copy()

    def clear_action_buffer(self) -> None:
        self._action_buffer.clear()

    def reset(self) -> None:
        self._action_buffer.clear()
        self._policy.reset()

    def close(self) -> None:
        self._policy.close()

    def _fetch_action_chunk(self, obs: Mapping[str, object]) -> None:
        policy_inference_result = parse_policy_inference_response(
            self._policy.infer(obs),
            expected_action_width=self._actuator_ctrlrange.shape[0],
            action_interpretation=self._action_interpretation,
        )
        self.last_server_timing = policy_inference_result.server_timing
        self.last_policy_timing = policy_inference_result.policy_timing
        raw_action_rows = policy_inference_result.action_batch.rows
        self.last_raw_action_batch = raw_action_rows.copy()
        buffered_raw_action_rows = self._buffered_action_rows(raw_action_rows)
        actuator_ctrl_rows = self._action_chunk_to_actuator_ctrl(buffered_raw_action_rows, obs)
        self.last_actuator_ctrl_batch = actuator_ctrl_rows.copy()
        for actuator_ctrl_row in actuator_ctrl_rows:
            self._action_buffer.append(actuator_ctrl_row.copy())

    def _buffered_action_rows(self, raw_action_rows: np.ndarray) -> np.ndarray:
        if self._max_buffered_actions is None:
            return raw_action_rows
        return raw_action_rows[: self._max_buffered_actions]

    def _action_chunk_to_actuator_ctrl(
        self,
        raw_action_rows: np.ndarray,
        obs: Mapping[str, object],
    ) -> np.ndarray:
        current_droid_joint_position: np.ndarray | None = None
        if self._action_interpretation in (
            PolicyActionInterpretation.DROID_JOINT_DELTA,
            PolicyActionInterpretation.DROID_JOINT_VELOCITY,
        ):
            current_droid_joint_position = _parse_droid_joint_position(
                obs,
                action_interpretation=self._action_interpretation,
            )
        if self._action_interpretation is PolicyActionInterpretation.DROID_JOINT_VELOCITY:
            if current_droid_joint_position is None:
                raise RuntimeError("DROID joint-velocity action mapping requires joint position")
            return self._droid_joint_velocity_chunk_to_actuator_ctrl(
                raw_action_rows,
                current_droid_joint_position=current_droid_joint_position,
            )
        return np.asarray(
            [
                self._action_to_actuator_ctrl(
                    action_row,
                    current_droid_joint_position=current_droid_joint_position,
                )
                for action_row in raw_action_rows
            ],
            dtype=float,
        )

    def _action_to_actuator_ctrl(
        self,
        action: np.ndarray,
        *,
        current_droid_joint_position: np.ndarray | None,
    ) -> np.ndarray:
        match self._action_interpretation:
            case PolicyActionInterpretation.DIRECT_ACTUATOR_CTRL:
                actuator_ctrl = np.asarray(action, dtype=float).copy()
            case PolicyActionInterpretation.DROID_ABSOLUTE_JOINT_POSITION:
                actuator_ctrl = np.asarray(action, dtype=float).copy()
                actuator_ctrl[-1] = self._normalized_gripper_to_actuator_ctrl(
                    normalized_gripper=float(action[-1])
                )
            case PolicyActionInterpretation.DROID_JOINT_DELTA:
                if current_droid_joint_position is None:
                    raise RuntimeError("DROID joint-delta action mapping requires joint position")
                actuator_ctrl = np.asarray(action, dtype=float).copy()
                actuator_ctrl[: current_droid_joint_position.shape[0]] = (
                    current_droid_joint_position
                    + actuator_ctrl[: current_droid_joint_position.shape[0]]
                )
                actuator_ctrl[-1] = self._normalized_gripper_to_actuator_ctrl(
                    normalized_gripper=float(action[-1])
                )
            case PolicyActionInterpretation.DROID_JOINT_VELOCITY:
                raise RuntimeError(
                    "DROID joint-velocity action mapping must be handled at chunk level"
                )
        return self._clamped_actuator_ctrl(actuator_ctrl)

    def _droid_joint_velocity_chunk_to_actuator_ctrl(
        self,
        raw_action_rows: np.ndarray,
        *,
        current_droid_joint_position: np.ndarray,
    ) -> np.ndarray:
        actuator_ctrl_rows: list[np.ndarray] = []
        target_joint_position = current_droid_joint_position.copy()
        for raw_action_row in raw_action_rows:
            actuator_ctrl = np.asarray(raw_action_row, dtype=float).copy()
            clipped_joint_velocity = np.clip(actuator_ctrl[:7], -1.0, 1.0)
            target_joint_position = (
                target_joint_position
                + clipped_joint_velocity * self._droid_velocity_time_step_seconds
            )
            actuator_ctrl[:7] = target_joint_position
            actuator_ctrl[-1] = self._thresholded_gripper_to_actuator_ctrl(
                normalized_gripper=float(raw_action_row[-1])
            )
            actuator_ctrl_rows.append(self._clamped_actuator_ctrl(actuator_ctrl))
        return np.asarray(actuator_ctrl_rows, dtype=float)

    def _normalized_gripper_to_actuator_ctrl(self, *, normalized_gripper: float) -> float:
        gripper_ctrl_min = float(self._actuator_ctrlrange[-1, 0])
        gripper_ctrl_max = float(self._actuator_ctrlrange[-1, 1])
        clipped_gripper = float(np.clip(normalized_gripper, 0.0, 1.0))
        return gripper_ctrl_min + clipped_gripper * (gripper_ctrl_max - gripper_ctrl_min)

    def _thresholded_gripper_to_actuator_ctrl(self, *, normalized_gripper: float) -> float:
        return self._normalized_gripper_to_actuator_ctrl(
            normalized_gripper=1.0 if normalized_gripper > 0.5 else 0.0
        )

    def _clamped_actuator_ctrl(self, actuator_ctrl: np.ndarray) -> np.ndarray:
        lower = self._actuator_ctrlrange[:, 0]
        upper = self._actuator_ctrlrange[:, 1]
        finite_ctrlrange = np.isfinite(lower) & np.isfinite(upper) & (lower < upper)
        clamped_actuator_ctrl = np.asarray(actuator_ctrl, dtype=float).copy()
        clamped_actuator_ctrl[finite_ctrlrange] = np.clip(
            clamped_actuator_ctrl[finite_ctrlrange],
            lower[finite_ctrlrange],
            upper[finite_ctrlrange],
        )
        return clamped_actuator_ctrl


class HostedPolicyActionChunkClient:
    """Owns a hosted policy connection and buffers raw action rows.

    Use this for embodiments whose policy output is not already actuator
    control space. The scene remains responsible for converting each raw action
    row into controls using its own kinematics.
    """

    def __init__(
        self,
        *,
        endpoint: PolicyEndpoint,
        expected_action_width: int,
        max_buffered_actions: int | None = None,
        policy_factory: PolicyFactory = _default_policy_factory,
    ) -> None:
        if expected_action_width <= 0:
            raise ValueError(f"expected_action_width must be positive, got {expected_action_width}")
        if max_buffered_actions is not None and max_buffered_actions <= 0:
            raise ValueError(
                f"max_buffered_actions must be positive when set, got {max_buffered_actions}"
            )
        self._policy = policy_factory(endpoint)
        self._expected_action_width = expected_action_width
        self._max_buffered_actions = max_buffered_actions
        self._action_buffer: deque[np.ndarray] = deque()
        self.metadata = dict(self._policy.get_server_metadata())
        self.last_server_timing: Mapping[str, object] = {}
        self.last_policy_timing: Mapping[str, object] = {}
        self.last_raw_action_batch = np.zeros((0, expected_action_width), dtype=float)

    def prewarm(self, obs: Mapping[str, object]) -> None:
        """Force one inference before the simulation starts ticking."""
        if not self._action_buffer:
            self._fetch_action_chunk(obs)

    def next_action(self, obs: Mapping[str, object]) -> np.ndarray:
        """Return the next raw policy action row, fetching a new chunk when needed."""
        if not self._action_buffer:
            self._fetch_action_chunk(obs)
        if not self._action_buffer:
            raise RuntimeError("policy returned an empty action chunk")
        return self._action_buffer.popleft().copy()

    def clear_action_buffer(self) -> None:
        self._action_buffer.clear()

    def reset(self) -> None:
        self._action_buffer.clear()
        self._policy.reset()

    def close(self) -> None:
        self._policy.close()

    def _fetch_action_chunk(self, obs: Mapping[str, object]) -> None:
        policy_inference_result = parse_policy_inference_response(
            self._policy.infer(obs),
            expected_action_width=self._expected_action_width,
            action_width_label="raw policy action width",
        )
        self.last_server_timing = policy_inference_result.server_timing
        self.last_policy_timing = policy_inference_result.policy_timing
        raw_action_rows = policy_inference_result.action_batch.rows
        buffered_raw_action_rows = self._buffered_action_rows(raw_action_rows)
        self.last_raw_action_batch = raw_action_rows.copy()
        for raw_action_row in buffered_raw_action_rows:
            self._action_buffer.append(raw_action_row.copy())

    def _buffered_action_rows(self, raw_action_rows: np.ndarray) -> np.ndarray:
        if self._max_buffered_actions is None:
            return raw_action_rows
        return raw_action_rows[: self._max_buffered_actions]


def parse_policy_inference_response(
    response: Mapping[str, object],
    *,
    expected_action_width: int,
    action_interpretation: PolicyActionInterpretation = PolicyActionInterpretation.DIRECT_ACTUATOR_CTRL,
    action_width_label: str | None = None,
) -> PolicyInferenceResult:
    """Parse the untyped hosted-policy response into checked domain data."""
    if action_width_label is None:
        match action_interpretation:
            case PolicyActionInterpretation.DIRECT_ACTUATOR_CTRL:
                action_width_label = "model.nu"
            case PolicyActionInterpretation.DROID_ABSOLUTE_JOINT_POSITION:
                action_width_label = "DROID action width"
            case PolicyActionInterpretation.DROID_JOINT_DELTA:
                action_width_label = "DROID action width"
            case PolicyActionInterpretation.DROID_JOINT_VELOCITY:
                action_width_label = "DROID action width"
    if "actions" not in response:
        raise KeyError("policy response missing required `actions` key")
    actions = np.asarray(response["actions"], dtype=float)
    if actions.ndim == 1:
        actions = actions.reshape(1, -1)
    if actions.ndim != 2:
        raise ValueError(f"policy actions must be 1D or 2D, got shape {actions.shape}")
    if actions.shape[1] != expected_action_width:
        raise ValueError(
            f"policy action width {actions.shape[1]} does not match {action_width_label} "
            f"{expected_action_width}"
        )
    if actions.shape[0] == 0:
        raise ValueError("policy returned an empty action chunk")

    return PolicyInferenceResult(
        action_batch=PolicyActionBatch(rows=actions),
        server_timing=_parse_timing_mapping(response.get("server_timing")),
        policy_timing=_parse_timing_mapping(response.get("policy_timing")),
    )


def _parse_droid_joint_position(
    obs: Mapping[str, object],
    *,
    action_interpretation: PolicyActionInterpretation,
) -> np.ndarray:
    if "observation/joint_position" not in obs:
        raise KeyError(
            f"DROID {action_interpretation.value} action mapping requires "
            "`observation/joint_position`"
        )
    joint_position = np.asarray(obs["observation/joint_position"], dtype=float)
    if joint_position.ndim != 1:
        raise ValueError(
            "DROID `observation/joint_position` must be a 1D array for "
            f"{action_interpretation.value} action mapping, got shape {joint_position.shape}"
        )
    if joint_position.shape[0] != 7:
        raise ValueError(
            "DROID `observation/joint_position` must contain 7 arm joints, "
            f"got shape {joint_position.shape}"
        )
    return joint_position


def _parse_timing_mapping(raw_timing: object) -> Mapping[str, object]:
    if raw_timing is None:
        return {}
    if not isinstance(raw_timing, Mapping):
        raise ValueError(
            f"policy timing metadata must be a mapping, got {type(raw_timing).__name__}"
        )
    parsed_timing: dict[str, object] = {}
    for timing_key, timing_value in raw_timing.items():
        if not isinstance(timing_key, str):
            raise ValueError(f"policy timing key must be str, got {type(timing_key).__name__}")
        parsed_timing[timing_key] = timing_value
    return parsed_timing
