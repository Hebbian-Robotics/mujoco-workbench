"""Explicit manipulator handle resolution.

Scene modules declare each manipulator with compiled MJCF element names. The
runtime treats `ArmSide` as a logical key for task plans and UI grouping only;
it does not derive MuJoCo names from the side.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import StrEnum

import mujoco
import numpy as np


class ArmSide(StrEnum):
    """Logical manipulator identity used by task plans and controls."""

    LEFT = "left/"
    RIGHT = "right/"

    @property
    def label(self) -> str:
        return self.value.rstrip("/")


@dataclass(frozen=True)
class GripperPuppetJoint:
    """Joint qpos to mirror when a gripper target changes."""

    joint_name: str
    scale: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class GripperControlSpec:
    """Declarative gripper control convention for one manipulator."""

    actuator_name: str
    open_ctrl: float
    closed_ctrl: float
    puppet_joints: tuple[GripperPuppetJoint, ...] = ()


@dataclass(frozen=True)
class ManipulatorSpec:
    """Explicit compiled names for one manipulator."""

    side: ArmSide
    name: str
    joint_names: tuple[str, ...]
    arm_actuator_names: tuple[str, ...]
    gripper: GripperControlSpec
    wrist_body_name: str
    tcp_site_name: str
    base_body_name: str
    joint_labels: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(self.arm_actuator_names):
            raise ValueError(
                f"manipulator {self.name!r} has {len(self.joint_names)} joints "
                f"but {len(self.arm_actuator_names)} arm actuators"
            )
        if self.joint_labels and len(self.joint_labels) != len(self.joint_names):
            raise ValueError(
                f"manipulator {self.name!r} has {len(self.joint_labels)} joint labels "
                f"but {len(self.joint_names)} joints"
            )

    @property
    def resolved_joint_labels(self) -> tuple[str, ...]:
        return self.joint_labels or self.joint_names


@dataclass(frozen=True)
class GripperPuppetJointHandle:
    """Resolved qpos/dof target for a gripper puppet joint."""

    qpos_idx: int
    dof_idx: int
    scale: float
    offset: float

    def qpos_for_ctrl(self, ctrl: float) -> float:
        return self.offset + self.scale * ctrl


@dataclass
class ArmHandles:
    side: ArmSide
    name: str
    joint_names: tuple[str, ...]
    joint_labels: tuple[str, ...]
    qpos_idx: np.ndarray
    dof_idx: np.ndarray
    jnt_ids: np.ndarray
    arm_dof_idx: np.ndarray
    act_arm_ids: np.ndarray
    act_gripper_id: int
    grasp_body_id: int
    base_body_id: int
    tcp_site_id: int
    tcp_site_name: str
    gripper_open: float
    gripper_closed: float
    gripper_puppet_joints: tuple[GripperPuppetJointHandle, ...]
    weld_ids: np.ndarray

    @property
    def arm_qpos_idx(self) -> np.ndarray:
        """qpos indices for controlled arm DoFs."""
        return self.qpos_idx


def namespaced_name(side: ArmSide, suffix: str) -> str:
    """Return the dm_control attach namespace name for a single-arm subtree."""
    return f"{side.value}{suffix}"


def _namespaced_tuple(side: ArmSide, suffixes: Iterable[str]) -> tuple[str, ...]:
    return tuple(namespaced_name(side, suffix) for suffix in suffixes)


def _joint_labels(prefix: str, count: int) -> tuple[str, ...]:
    return tuple(f"{prefix}{joint_index}" for joint_index in range(1, count + 1))


def piper_manipulator_spec(side: ArmSide) -> ManipulatorSpec:
    joint_names = _namespaced_tuple(side, (f"joint{i}" for i in range(1, 7)))
    return ManipulatorSpec(
        side=side,
        name="piper",
        joint_names=joint_names,
        arm_actuator_names=joint_names,
        gripper=GripperControlSpec(
            actuator_name=namespaced_name(side, "gripper"),
            open_ctrl=0.035,
            closed_ctrl=0.0,
            puppet_joints=(
                GripperPuppetJoint(namespaced_name(side, "joint7"), scale=1.0),
                GripperPuppetJoint(namespaced_name(side, "joint8"), scale=-1.0),
            ),
        ),
        wrist_body_name=namespaced_name(side, "link6"),
        tcp_site_name=namespaced_name(side, "tcp"),
        base_body_name=namespaced_name(side, "base_link"),
        joint_labels=_joint_labels("joint", 6),
    )


def ur10e_robotiq_manipulator_spec(side: ArmSide) -> ManipulatorSpec:
    joint_suffixes = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    actuator_suffixes = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    )
    return ManipulatorSpec(
        side=side,
        name="ur10e_robotiq_2f85",
        joint_names=_namespaced_tuple(side, joint_suffixes),
        arm_actuator_names=_namespaced_tuple(side, actuator_suffixes),
        gripper=GripperControlSpec(
            actuator_name=namespaced_name(side, "gripper/fingers_actuator"),
            open_ctrl=0.0,
            closed_ctrl=255.0,
        ),
        wrist_body_name=namespaced_name(side, "wrist_3_link"),
        tcp_site_name=namespaced_name(side, "tcp"),
        base_body_name=namespaced_name(side, "base"),
        joint_labels=joint_suffixes,
    )


def franka_panda_manipulator_spec(side: ArmSide) -> ManipulatorSpec:
    joint_suffixes = tuple(f"joint{i}" for i in range(1, 8))
    return ManipulatorSpec(
        side=side,
        name="franka_panda",
        joint_names=_namespaced_tuple(side, joint_suffixes),
        arm_actuator_names=_namespaced_tuple(side, (f"actuator{i}" for i in range(1, 8))),
        gripper=GripperControlSpec(
            actuator_name=namespaced_name(side, "actuator8"),
            open_ctrl=255.0,
            closed_ctrl=0.0,
        ),
        wrist_body_name=namespaced_name(side, "hand"),
        tcp_site_name=namespaced_name(side, "tcp"),
        base_body_name=namespaced_name(side, "link0"),
        joint_labels=joint_suffixes,
    )


def franka_panda_robotiq_manipulator_spec(side: ArmSide) -> ManipulatorSpec:
    joint_suffixes = tuple(f"joint{i}" for i in range(1, 8))
    return ManipulatorSpec(
        side=side,
        name="franka_panda_robotiq_2f85",
        joint_names=_namespaced_tuple(side, joint_suffixes),
        arm_actuator_names=_namespaced_tuple(side, (f"actuator{i}" for i in range(1, 8))),
        gripper=GripperControlSpec(
            actuator_name=namespaced_name(side, "gripper/fingers_actuator"),
            open_ctrl=0.0,
            closed_ctrl=255.0,
        ),
        wrist_body_name=namespaced_name(side, "gripper/base"),
        tcp_site_name=namespaced_name(side, "gripper/pinch"),
        base_body_name=namespaced_name(side, "link0"),
        joint_labels=joint_suffixes,
    )


def openarm_v1_manipulator_spec(side: ArmSide) -> ManipulatorSpec:
    joint_suffixes = tuple(f"openarm_joint{i}" for i in range(1, 8))
    return ManipulatorSpec(
        side=side,
        name="openarm_v1",
        joint_names=_namespaced_tuple(side, joint_suffixes),
        arm_actuator_names=_namespaced_tuple(side, (f"joint{i}_ctrl" for i in range(1, 8))),
        gripper=GripperControlSpec(
            actuator_name=namespaced_name(side, "finger_ctrl"),
            open_ctrl=0.044,
            closed_ctrl=0.0,
            puppet_joints=(
                GripperPuppetJoint(namespaced_name(side, "openarm_finger_joint1"), scale=1.0),
                GripperPuppetJoint(namespaced_name(side, "openarm_finger_joint2"), scale=1.0),
            ),
        ),
        wrist_body_name=namespaced_name(side, "openarm_link7"),
        tcp_site_name=namespaced_name(side, "tcp"),
        base_body_name=namespaced_name(side, "openarm_link1"),
        joint_labels=_joint_labels("openarm_joint", 7),
    )


def openarm_v2_manipulator_spec(side: ArmSide, *, namespace: str = "") -> ManipulatorSpec:
    def v2_name(name: str) -> str:
        return f"{namespace}/{name}" if namespace else name

    side_label = side.label
    finger_open = 0.7854 if side is ArmSide.LEFT else -0.7854
    return ManipulatorSpec(
        side=side,
        name="openarm_v2",
        joint_names=tuple(v2_name(f"openarm_{side_label}_joint{i}") for i in range(1, 8)),
        arm_actuator_names=tuple(v2_name(f"{side_label}_joint{i}_ctrl") for i in range(1, 8)),
        gripper=GripperControlSpec(
            actuator_name=v2_name(f"{side_label}_finger1_ctrl"),
            open_ctrl=finger_open,
            closed_ctrl=0.0,
            puppet_joints=(
                GripperPuppetJoint(v2_name(f"openarm_{side_label}_finger_joint1"), scale=1.0),
                GripperPuppetJoint(v2_name(f"openarm_{side_label}_finger_joint2"), scale=1.0),
            ),
        ),
        wrist_body_name=v2_name(f"openarm_{side_label}_ee_base_link"),
        tcp_site_name=v2_name(f"openarm_{side_label}_tcp"),
        base_body_name=v2_name(f"openarm_{side_label}_base_link"),
        joint_labels=_joint_labels(f"openarm_{side_label}_joint", 7),
    )


def arm_joint_labels(manipulator_or_arm: ManipulatorSpec | ArmHandles) -> tuple[str, ...]:
    """Return canonical arm joint labels for UI and scalar logging."""
    return manipulator_or_arm.joint_labels


def _resolve_id(model: mujoco.MjModel, obj_type: int, name: str, kind: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise RuntimeError(f"{kind} {name!r} not found in compiled model")
    return int(obj_id)


def _resolve_gripper_puppet_joints(
    model: mujoco.MjModel,
    specs: tuple[GripperPuppetJoint, ...],
) -> tuple[GripperPuppetJointHandle, ...]:
    handles: list[GripperPuppetJointHandle] = []
    for spec in specs:
        joint_id = _resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, spec.joint_name, "gripper joint")
        handles.append(
            GripperPuppetJointHandle(
                qpos_idx=int(model.jnt_qposadr[joint_id]),
                dof_idx=int(model.jnt_dofadr[joint_id]),
                scale=spec.scale,
                offset=spec.offset,
            )
        )
    return tuple(handles)


def get_arm_handles(
    model: mujoco.MjModel,
    manipulator: ManipulatorSpec,
    n_cubes: int,
) -> ArmHandles:
    """Resolve compiled MuJoCo ids for one explicit manipulator declaration."""
    joint_ids = np.array(
        [
            _resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name, "joint")
            for joint_name in manipulator.joint_names
        ],
        dtype=np.int64,
    )
    qpos_idx = np.array([int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids])
    dof_idx = np.array([int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids])
    actuator_ids = np.array(
        [
            _resolve_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name, "actuator")
            for actuator_name in manipulator.arm_actuator_names
        ],
        dtype=np.int64,
    )
    weld_ids = np.array(
        [
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                f"{manipulator.side.value.replace('/', '_')}grasp_cube{i}",
            )
            for i in range(n_cubes)
        ],
        dtype=np.int64,
    )
    return ArmHandles(
        side=manipulator.side,
        name=manipulator.name,
        joint_names=manipulator.joint_names,
        joint_labels=manipulator.resolved_joint_labels,
        qpos_idx=qpos_idx,
        dof_idx=dof_idx,
        jnt_ids=joint_ids,
        arm_dof_idx=dof_idx,
        act_arm_ids=actuator_ids,
        act_gripper_id=_resolve_id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            manipulator.gripper.actuator_name,
            "gripper actuator",
        ),
        grasp_body_id=_resolve_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            manipulator.wrist_body_name,
            "grasp body",
        ),
        base_body_id=_resolve_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            manipulator.base_body_name,
            "base body",
        ),
        tcp_site_id=_resolve_id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            manipulator.tcp_site_name,
            "TCP site",
        ),
        tcp_site_name=manipulator.tcp_site_name,
        gripper_open=manipulator.gripper.open_ctrl,
        gripper_closed=manipulator.gripper.closed_ctrl,
        gripper_puppet_joints=_resolve_gripper_puppet_joints(
            model, manipulator.gripper.puppet_joints
        ),
        weld_ids=weld_ids,
    )


def write_gripper_target(data: mujoco.MjData, arm: ArmHandles, target_ctrl: float) -> None:
    """Write gripper actuator ctrl and any declared puppet-joint qpos."""
    for puppet_joint in arm.gripper_puppet_joints:
        data.qpos[puppet_joint.qpos_idx] = puppet_joint.qpos_for_ctrl(target_ctrl)
        data.qvel[puppet_joint.dof_idx] = 0.0
    data.ctrl[arm.act_gripper_id] = target_ctrl
