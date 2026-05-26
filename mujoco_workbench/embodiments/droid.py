"""DROID observation builder for the hosted pi0.5 policy path."""

from __future__ import annotations

from collections.abc import Callable
from enum import StrEnum
from typing import Literal, TypedDict

import mujoco
import numpy as np
from openpi_client import image_tools


class DroidCameraName(StrEnum):
    EXTERIOR = "cam_exterior"
    LEFT_WRIST = "left/gripper/cam_wrist"


DROID_EXTERIOR_CAMERA_NAME = DroidCameraName.EXTERIOR
DROID_WRIST_CAMERA_NAME = DroidCameraName.LEFT_WRIST

DROID_OBSERVATION_KEYS: tuple[str, ...] = (
    "observation/exterior_image_1_left",
    "observation/wrist_image_left",
    "observation/joint_position",
    "observation/gripper_position",
    "prompt",
    "mode",
)

DroidInferenceMode = Literal["action_only"]
DROID_ACTION_INFERENCE_MODE: DroidInferenceMode = "action_only"
DROID_POLICY_CAMERA_RENDER_HEIGHT = 180
DROID_POLICY_CAMERA_RENDER_WIDTH = 320
# Median vertical FOV from DROID calibration intrinsics; MuJoCo cameras take vertical fovy.
DROID_DATASET_EXTERIOR_CAMERA_FOVY_DEG = 68.86
DROID_DATASET_WRIST_CAMERA_FOVY_DEG = 52.39
DROID_CONTROL_FREQUENCY_HZ = 15.0
DROID_CONTROL_PERIOD_SECONDS = 1.0 / DROID_CONTROL_FREQUENCY_HZ
DROID_DEFAULT_OPEN_LOOP_HORIZON = 8

_DROID_IMAGE_SHAPE: tuple[int, int, int] = (224, 224, 3)
_DROID_JOINT_NAMES: tuple[str, ...] = tuple(f"left/joint{i}" for i in range(1, 8))
_DROID_GRIPPER_ACTUATOR_NAME = "left/gripper/fingers_actuator"

RenderCamera = Callable[[DroidCameraName], np.ndarray]
DroidObservation = TypedDict(
    "DroidObservation",
    {
        "observation/exterior_image_1_left": np.ndarray,
        "observation/wrist_image_left": np.ndarray,
        "observation/joint_position": np.ndarray,
        "observation/gripper_position": np.ndarray,
        "prompt": str,
        "mode": DroidInferenceMode,
    },
)


def build_droid_observation(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    render_fn: RenderCamera,
    prompt: str,
) -> DroidObservation:
    """Build the single-arm DROID action observation used by hosted policy mode."""
    exterior_image = _validate_image(
        render_fn(DROID_EXTERIOR_CAMERA_NAME),
        observation_key="observation/exterior_image_1_left",
    )
    wrist_image = _validate_image(
        render_fn(DROID_WRIST_CAMERA_NAME),
        observation_key="observation/wrist_image_left",
    )
    return {
        "observation/exterior_image_1_left": exterior_image,
        "observation/wrist_image_left": wrist_image,
        "observation/joint_position": _joint_positions(model, data),
        "observation/gripper_position": np.array([_normalised_gripper_position(model, data)]),
        "prompt": prompt,
        "mode": DROID_ACTION_INFERENCE_MODE,
    }


def resize_droid_policy_image(rendered_image: np.ndarray) -> np.ndarray:
    """Apply OpenPI's DROID image preprocessing to one rendered camera frame."""
    image_array = np.asarray(rendered_image)
    if image_array.ndim != 3 or image_array.shape[-1] != 3:
        raise ValueError(f"DROID rendered image must have shape (H, W, 3), got {image_array.shape}")
    if image_array.dtype != np.uint8:
        raise ValueError(f"DROID rendered image must have dtype uint8, got {image_array.dtype}")
    resized_image = image_tools.resize_with_pad(
        image_array,
        height=_DROID_IMAGE_SHAPE[0],
        width=_DROID_IMAGE_SHAPE[1],
    )
    return _validate_image(resized_image, observation_key="DROID policy image")


def _validate_image(image: np.ndarray, *, observation_key: str) -> np.ndarray:
    image_array = np.asarray(image)
    if image_array.shape != _DROID_IMAGE_SHAPE:
        raise ValueError(
            f"{observation_key} must have shape {_DROID_IMAGE_SHAPE}, got {image_array.shape}"
        )
    if image_array.dtype != np.uint8:
        raise ValueError(f"{observation_key} must have dtype uint8, got {image_array.dtype}")
    return image_array


def _joint_positions(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    qpos_indices = []
    for joint_name in _DROID_JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"DROID observation joint {joint_name!r} not found in model")
        qpos_indices.append(int(model.jnt_qposadr[joint_id]))
    return np.asarray([data.qpos[qpos_index] for qpos_index in qpos_indices], dtype=float)


def _normalised_gripper_position(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        _DROID_GRIPPER_ACTUATOR_NAME,
    )
    if actuator_id < 0:
        raise ValueError(
            f"DROID observation gripper actuator {_DROID_GRIPPER_ACTUATOR_NAME!r} not found"
        )
    raw_gripper_position = float(data.ctrl[actuator_id])
    ctrl_min = float(model.actuator_ctrlrange[actuator_id, 0])
    ctrl_max = float(model.actuator_ctrlrange[actuator_id, 1])
    if ctrl_max <= ctrl_min:
        return raw_gripper_position
    return float(np.clip((raw_gripper_position - ctrl_min) / (ctrl_max - ctrl_min), 0.0, 1.0))
