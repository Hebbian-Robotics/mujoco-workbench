"""LIBERO observation and action adapter for hosted pi0.5 policy mode."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from typing import TypedDict

import mujoco
import numpy as np
from openpi_client import image_tools

from mujoco_workbench.arm_handles import ArmHandles
from mujoco_workbench.ik import FullPose, solve_ik


class LiberoCameraName(StrEnum):
    AGENTVIEW = "agentview"
    WRIST = "left/robot0_eye_in_hand"


class LiberoGripperCommand(StrEnum):
    OPEN = "open"
    CLOSE = "close"
    HOLD = "hold"


@dataclass(frozen=True)
class LiberoOscPoseAction:
    """Parsed 7-D LIBERO `OSC_POSE` action row."""

    position_delta_normalized: np.ndarray
    rotation_delta_normalized: np.ndarray
    gripper_command: LiberoGripperCommand
    gripper_action_normalized: float


LIBERO_AGENTVIEW_CAMERA_NAME = LiberoCameraName.AGENTVIEW
LIBERO_WRIST_CAMERA_NAME = LiberoCameraName.WRIST

LIBERO_OBSERVATION_KEYS: tuple[str, ...] = (
    "observation/image",
    "observation/wrist_image",
    "observation/state",
    "prompt",
)

LIBERO_POLICY_IMAGE_SIZE = 224
LIBERO_POLICY_CAMERA_RENDER_HEIGHT = 256
LIBERO_POLICY_CAMERA_RENDER_WIDTH = 256
LIBERO_CONTROL_FREQUENCY_HZ = 20.0
LIBERO_CONTROL_PERIOD_SECONDS = 1.0 / LIBERO_CONTROL_FREQUENCY_HZ
LIBERO_DEFAULT_OPEN_LOOP_HORIZON = 5
LIBERO_ACTION_WIDTH = 7

# Robosuite's default OSC_POSE controller scales normalized policy actions to
# +/-5 cm translation and +/-0.5 rad rotation deltas.
LIBERO_POSITION_DELTA_SCALE_M = 0.05
LIBERO_ROTATION_DELTA_SCALE_RAD = 0.5
LIBERO_GRIPPER_NORMALIZED_STEP = 0.2

_LIBERO_IMAGE_SHAPE: tuple[int, int, int] = (
    LIBERO_POLICY_IMAGE_SIZE,
    LIBERO_POLICY_IMAGE_SIZE,
    3,
)
_PANDA_FINGER_JOINT_SUFFIXES: tuple[str, str] = ("finger_joint1", "finger_joint2")

RenderCamera = Callable[[LiberoCameraName], np.ndarray]
LiberoObservation = TypedDict(
    "LiberoObservation",
    {
        "observation/image": np.ndarray,
        "observation/wrist_image": np.ndarray,
        "observation/state": np.ndarray,
        "prompt": str,
    },
)


def build_libero_observation(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmHandles,
    render_fn: RenderCamera,
    prompt: str,
) -> LiberoObservation:
    """Build the OpenPI LIBERO observation dict used by `pi05_libero`."""
    agentview_image = _validate_image(
        render_fn(LIBERO_AGENTVIEW_CAMERA_NAME),
        observation_key="observation/image",
    )
    wrist_image = _validate_image(
        render_fn(LIBERO_WRIST_CAMERA_NAME),
        observation_key="observation/wrist_image",
    )
    return {
        "observation/image": agentview_image,
        "observation/wrist_image": wrist_image,
        "observation/state": _libero_state(model, data, arm),
        "prompt": prompt,
    }


def resize_libero_policy_image(rendered_image: np.ndarray) -> np.ndarray:
    """Apply OpenPI's LIBERO image preprocessing to one rendered camera frame."""
    image_array = np.asarray(rendered_image)
    if image_array.ndim != 3 or image_array.shape[-1] != 3:
        raise ValueError(
            f"LIBERO rendered image must have shape (H, W, 3), got {image_array.shape}"
        )
    if image_array.dtype != np.uint8:
        raise ValueError(f"LIBERO rendered image must have dtype uint8, got {image_array.dtype}")

    # OpenPI's LIBERO eval rotates both robosuite images by 180 degrees before
    # resizing. Preserve that training-time convention for native MuJoCo scenes.
    rotated_image = np.ascontiguousarray(image_array[::-1, ::-1])
    resized_image = image_tools.resize_with_pad(
        rotated_image,
        height=LIBERO_POLICY_IMAGE_SIZE,
        width=LIBERO_POLICY_IMAGE_SIZE,
    )
    uint8_image = image_tools.convert_to_uint8(resized_image)
    return _validate_image(uint8_image, observation_key="LIBERO policy image")


def libero_action_to_actuator_ctrl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scratch_data: mujoco.MjData,
    arm: ArmHandles,
    raw_action: np.ndarray,
) -> np.ndarray:
    """Map a 7-D LIBERO action row to this MuJoCo model's actuator controls.

    LIBERO actions are robosuite `OSC_POSE` deltas plus one Panda gripper
    command. Native workbench scenes expose joint-position actuators, so this
    adapter solves a target TCP pose in scratch data and returns the matching
    actuator targets without teleporting the live simulation state.
    """
    action = _parse_libero_action(raw_action)
    mujoco.mj_forward(model, data)
    _copy_runtime_state(data, scratch_data)
    mujoco.mj_forward(model, scratch_data)

    current_tcp_position = np.asarray(data.site_xpos[arm.tcp_site_id], dtype=float)
    target_tcp_position = (
        current_tcp_position + action.position_delta_normalized * LIBERO_POSITION_DELTA_SCALE_M
    )

    current_tcp_quat_wxyz = _site_quat_wxyz(data, arm.tcp_site_id)
    delta_quat_wxyz = _quat_wxyz_from_axis_angle(
        action.rotation_delta_normalized * LIBERO_ROTATION_DELTA_SCALE_RAD
    )
    target_tcp_quat_wxyz = _normalised_quat_wxyz(
        _quat_multiply_wxyz(delta_quat_wxyz, current_tcp_quat_wxyz)
    )

    current_arm_q = np.asarray(data.qpos[arm.arm_qpos_idx], dtype=float).copy()
    solved_arm_q, _err = solve_ik(
        model,
        scratch_data,
        arm,
        target_tcp_position,
        orientation=FullPose(target_tcp_quat_wxyz),
        seed_q=current_arm_q,
        max_iters=120,
        rate_dt=0.02,
        pos_tol=0.004,
        rot_tol=0.05,
        solver="daqp",
    )

    actuator_ctrl = np.asarray(data.ctrl, dtype=float).copy()
    actuator_ctrl[arm.act_arm_ids] = solved_arm_q
    actuator_ctrl[arm.act_gripper_id] = _libero_gripper_command_to_ctrl(
        arm,
        gripper_command=action.gripper_command,
        gripper_action_normalized=action.gripper_action_normalized,
        current_gripper_ctrl=float(data.ctrl[arm.act_gripper_id]),
    )
    return _clamped_actuator_ctrl(model, actuator_ctrl)


def _validate_image(image: np.ndarray, *, observation_key: str) -> np.ndarray:
    image_array = np.asarray(image)
    if image_array.shape != _LIBERO_IMAGE_SHAPE:
        raise ValueError(
            f"{observation_key} must have shape {_LIBERO_IMAGE_SHAPE}, got {image_array.shape}"
        )
    if image_array.dtype != np.uint8:
        raise ValueError(f"{observation_key} must have dtype uint8, got {image_array.dtype}")
    return image_array


def _libero_state(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmHandles) -> np.ndarray:
    end_effector_position = np.asarray(data.site_xpos[arm.tcp_site_id], dtype=float).copy()
    end_effector_axis_angle = _axis_angle_from_quat_wxyz(_site_quat_wxyz(data, arm.tcp_site_id))
    gripper_qpos = _panda_finger_qpos(model, data, arm)
    return np.concatenate((end_effector_position, end_effector_axis_angle, gripper_qpos))


def _panda_finger_qpos(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmHandles) -> np.ndarray:
    finger_qpos: list[float] = []
    for joint_suffix in _PANDA_FINGER_JOINT_SUFFIXES:
        joint_name = f"{arm.side}{joint_suffix}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"LIBERO Panda finger joint {joint_name!r} not found in model")
        finger_qpos.append(float(data.qpos[model.jnt_qposadr[joint_id]]))
    return np.asarray(finger_qpos, dtype=float)


def _parse_libero_action(raw_action: np.ndarray) -> LiberoOscPoseAction:
    action_row = np.asarray(raw_action, dtype=float)
    if action_row.shape != (LIBERO_ACTION_WIDTH,):
        raise ValueError(
            f"LIBERO action must have shape ({LIBERO_ACTION_WIDTH},), got {action_row.shape}"
        )
    clipped_action_row = np.clip(action_row, -1.0, 1.0)
    return LiberoOscPoseAction(
        position_delta_normalized=clipped_action_row[:3],
        rotation_delta_normalized=clipped_action_row[3:6],
        gripper_command=_parse_libero_gripper_command(float(clipped_action_row[-1])),
        gripper_action_normalized=float(clipped_action_row[-1]),
    )


def _parse_libero_gripper_command(gripper_action: float) -> LiberoGripperCommand:
    if np.isclose(gripper_action, 0.0):
        return LiberoGripperCommand.HOLD
    if gripper_action > 0.0:
        return LiberoGripperCommand.CLOSE
    return LiberoGripperCommand.OPEN


def _copy_runtime_state(source_data: mujoco.MjData, target_data: mujoco.MjData) -> None:
    target_data.qpos[:] = source_data.qpos
    target_data.qvel[:] = source_data.qvel
    target_data.ctrl[:] = source_data.ctrl
    target_data.act[:] = source_data.act
    target_data.time = source_data.time


def _site_quat_wxyz(data: mujoco.MjData, site_id: int) -> np.ndarray:
    quat_wxyz = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat_wxyz, data.site_xmat[site_id])
    return _normalised_quat_wxyz(quat_wxyz)


def _axis_angle_from_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = _normalised_quat_wxyz(quat_wxyz)
    if quat[0] < 0.0:
        quat = -quat
    scalar = float(np.clip(quat[0], -1.0, 1.0))
    denominator = float(np.sqrt(max(0.0, 1.0 - scalar * scalar)))
    if np.isclose(denominator, 0.0):
        return np.zeros(3, dtype=float)
    return quat[1:] * (2.0 * np.arccos(scalar)) / denominator


def _quat_wxyz_from_axis_angle(axis_angle: np.ndarray) -> np.ndarray:
    axis_angle_array = np.asarray(axis_angle, dtype=float)
    angle = float(np.linalg.norm(axis_angle_array))
    if np.isclose(angle, 0.0):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = axis_angle_array / angle
    half_angle = 0.5 * angle
    return _normalised_quat_wxyz(
        np.array(
            [
                np.cos(half_angle),
                axis[0] * np.sin(half_angle),
                axis[1] * np.sin(half_angle),
                axis[2] * np.sin(half_angle),
            ],
            dtype=float,
        )
    )


def _quat_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return np.array(
        [
            left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
            left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
            left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
            left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
        ],
        dtype=float,
    )


def _normalised_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=float)
    norm = float(np.linalg.norm(quat))
    if np.isclose(norm, 0.0):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


def _libero_gripper_command_to_ctrl(
    arm: ArmHandles,
    *,
    gripper_command: LiberoGripperCommand,
    gripper_action_normalized: float,
    current_gripper_ctrl: float,
) -> float:
    match gripper_command:
        case LiberoGripperCommand.HOLD:
            return current_gripper_ctrl
        case LiberoGripperCommand.OPEN | LiberoGripperCommand.CLOSE:
            pass
    current_normalized = _gripper_ctrl_to_libero_normalized(arm, current_gripper_ctrl)
    next_normalized = float(
        np.clip(
            current_normalized
            + LIBERO_GRIPPER_NORMALIZED_STEP * np.sign(gripper_action_normalized),
            -1.0,
            1.0,
        )
    )
    return _libero_normalized_gripper_to_ctrl(arm, next_normalized)


def _gripper_ctrl_to_libero_normalized(arm: ArmHandles, gripper_ctrl: float) -> float:
    """Map simulator gripper ctrl to robosuite's -1=open, +1=closed space."""
    gripper_span = arm.gripper_closed - arm.gripper_open
    if np.isclose(gripper_span, 0.0):
        return -1.0
    normalized_gripper = -1.0 + 2.0 * (gripper_ctrl - arm.gripper_open) / gripper_span
    return float(np.clip(normalized_gripper, -1.0, 1.0))


def _libero_normalized_gripper_to_ctrl(arm: ArmHandles, normalized_gripper: float) -> float:
    clipped_normalized_gripper = float(np.clip(normalized_gripper, -1.0, 1.0))
    interpolation = 0.5 * (clipped_normalized_gripper + 1.0)
    return arm.gripper_open + interpolation * (arm.gripper_closed - arm.gripper_open)


def _clamped_actuator_ctrl(model: mujoco.MjModel, actuator_ctrl: np.ndarray) -> np.ndarray:
    lower = np.asarray(model.actuator_ctrlrange[:, 0], dtype=float)
    upper = np.asarray(model.actuator_ctrlrange[:, 1], dtype=float)
    finite_ctrlrange = np.isfinite(lower) & np.isfinite(upper) & (lower < upper)
    clamped_actuator_ctrl = np.asarray(actuator_ctrl, dtype=float).copy()
    clamped_actuator_ctrl[finite_ctrlrange] = np.clip(
        clamped_actuator_ctrl[finite_ctrlrange],
        lower[finite_ctrlrange],
        upper[finite_ctrlrange],
    )
    return clamped_actuator_ctrl
