"""Per-arm handle resolution, dispatched by robot kind.

`get_arm_handles(model, manipulator, n_cubes)` returns an `ArmHandles`
carrying the qpos/dof/actuator/body indices the runner + IK code use to
drive one prefixed arm. Robot-specific naming lives in `ROBOT_ADAPTERS`.
Two robot families are currently supported:

* `"piper"` — AgileX Piper 6-DoF arm + parallel-jaw with two
  tendon-coupled finger slide joints. `qpos_idx` / `dof_idx` are
  length-8 (joints 1..8).
* `"ur10e"` — Universal Robots UR10e + Robotiq 2F-85. The 2F-85's
  4-bar linkage is tendon-driven by a single `fingers_actuator`
  (ctrl 0..255); finger qpos is NOT puppet-written — the actuator
  pushes the tendon equality and the linkage settles.
  `qpos_idx` / `dof_idx` are length-6 (no finger entries).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum

import mujoco
import numpy as np


class RobotKind(StrEnum):
    """Supported mobile-manipulator arm families.

    Keep this set finite: the workbench is intentionally scoped to one or two
    robot arms on optional mobile bases. New arms should enter through a
    `RobotAdapter` registration instead of ad-hoc name checks in runtime code.
    """

    PIPER = "piper"
    UR10E = "ur10e"


class ArmSide(StrEnum):
    """Bimanual arm identity; value is the MuJoCo body-name prefix.

    The trailing `/` is dm_control.mjcf's namespace separator: when a
    sub-MJCF (Piper or UR10e) is attached with `model="left"`, every
    body/joint/actuator inside it is renamed `left/<original>`.
    f-string concatenation (`f"{side}link6"` /
    `f"{side}wrist_3_link"`) naturally produces the slash-namespaced
    compiled name.
    """

    LEFT = "left/"
    RIGHT = "right/"


@dataclass(frozen=True)
class RobotAdapter:
    """Names and control conventions for one supported robot-arm family."""

    robot_kind: RobotKind
    joint_suffixes: tuple[str, ...]
    controlled_arm_joint_count: int
    arm_actuator_suffixes: tuple[str, ...]
    gripper_actuator_suffix: str
    wrist_body_suffix: str
    tcp_site_suffix: str
    piper_mirrored_gripper_joint_suffixes: tuple[str, str] | None = None

    @property
    def arm_joint_suffixes(self) -> tuple[str, ...]:
        return self.joint_suffixes[: self.controlled_arm_joint_count]


@dataclass(frozen=True)
class ManipulatorSpec:
    """Parsed manipulator declaration for a scene.

    `side` is still restricted to left/right for now, but `robot_kind` is no
    longer scene-global. That lets a supported unimanual or asymmetric bimanual
    scene declare the actual robot family per manipulator.
    """

    side: ArmSide
    robot_kind: RobotKind

    @property
    def prefix(self) -> str:
        return self.side.value


_PIPER_ARM_JOINT_SUFFIXES: tuple[str, ...] = tuple(f"joint{i}" for i in range(1, 9))
"""Piper joints 1..8 — first 6 are arm DoFs, last 2 are tendon-coupled
finger slides."""

_UR10E_ARM_JOINT_SUFFIXES: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
"""UR10e arm DoFs in canonical chain order. 2F-85 finger joints are
tendon-driven and not addressed through `qpos_idx` / `dof_idx`."""


ROBOT_ADAPTERS: Mapping[RobotKind, RobotAdapter] = {
    RobotKind.PIPER: RobotAdapter(
        robot_kind=RobotKind.PIPER,
        joint_suffixes=_PIPER_ARM_JOINT_SUFFIXES,
        controlled_arm_joint_count=6,
        arm_actuator_suffixes=tuple(f"joint{i}" for i in range(1, 7)),
        gripper_actuator_suffix="gripper",
        wrist_body_suffix="link6",
        tcp_site_suffix="tcp",
        piper_mirrored_gripper_joint_suffixes=("joint7", "joint8"),
    ),
    RobotKind.UR10E: RobotAdapter(
        robot_kind=RobotKind.UR10E,
        joint_suffixes=_UR10E_ARM_JOINT_SUFFIXES,
        controlled_arm_joint_count=6,
        arm_actuator_suffixes=(
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ),
        gripper_actuator_suffix="gripper/fingers_actuator",
        wrist_body_suffix="wrist_3_link",
        tcp_site_suffix="tcp",
    ),
}


def parse_robot_kind(raw_robot_kind: object) -> RobotKind:
    try:
        return RobotKind(str(raw_robot_kind))
    except ValueError as err:
        valid = ", ".join(robot_kind.value for robot_kind in RobotKind)
        raise ValueError(
            f"unsupported robot kind {raw_robot_kind!r}; expected one of: {valid}"
        ) from err


def robot_adapter(robot_kind: RobotKind | str) -> RobotAdapter:
    parsed_robot_kind = (
        robot_kind if isinstance(robot_kind, RobotKind) else parse_robot_kind(robot_kind)
    )
    return ROBOT_ADAPTERS[parsed_robot_kind]


def arm_joint_suffixes(robot_kind: RobotKind | str) -> tuple[str, ...]:
    """Return canonical-order arm joint suffixes for a robot kind.

    Single source of truth shared by `get_arm_handles`, the runner's
    rerun scalar logging, and teleop's per-joint slider labels.
    Callers must NOT redefine this list locally.
    """
    return robot_adapter(robot_kind).arm_joint_suffixes


@dataclass
class ArmHandles:
    side: ArmSide
    robot_kind: RobotKind
    # piper=8 (joints 1..8), ur10e=6.
    qpos_idx: np.ndarray
    dof_idx: np.ndarray
    jnt_ids: np.ndarray
    arm_dof_idx: np.ndarray
    # Position-actuator ids for the 6 arm DoFs. The runner mirrors
    # puppet qpos into ctrl so position servos don't fight back with
    # stale targets.
    act_arm_ids: np.ndarray
    act_gripper_id: int
    # Grasp-weld parent. Piper: link6. UR10e: wrist_3_link (the 2F-85
    # attachment frame).
    link6_id: int
    tcp_site_id: int
    gripper_open: float
    gripper_closed: float
    weld_ids: np.ndarray  # equality ids, one per cube (may be empty)
    piper_mirrored_gripper_qpos_idx: tuple[int, int] | None = None
    piper_mirrored_gripper_dof_idx: tuple[int, int] | None = None

    @property
    def arm_qpos_idx(self) -> np.ndarray:
        """qpos indices for controlled arm DoFs only."""
        return self.qpos_idx[: len(self.act_arm_ids)]

    @property
    def tcp_site_name(self) -> str:
        return f"{self.side}tcp"


def _resolve_id(model: mujoco.MjModel, obj_type: int, name: str, kind: str) -> int:
    """Look up an MJCF element id by name; raise with context if missing."""
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise RuntimeError(
            f"{kind} {name!r} not found in compiled model. "
            "Check the scene module's `ROBOT_KIND` matches the actually-loaded robot."
        )
    return obj_id


def _coerce_manipulator_spec(
    side_or_spec: ArmSide | ManipulatorSpec,
    robot_kind: RobotKind | str,
) -> ManipulatorSpec:
    if isinstance(side_or_spec, ManipulatorSpec):
        return side_or_spec
    return ManipulatorSpec(side=side_or_spec, robot_kind=parse_robot_kind(robot_kind))


def get_arm_handles(
    model: mujoco.MjModel,
    side: ArmSide | ManipulatorSpec,
    n_cubes: int,
    robot_kind: RobotKind | str = RobotKind.PIPER,
) -> ArmHandles:
    """Resolve all per-arm handles. The `robot_kind` argument selects
    which joint-name convention + gripper layout to use. Scenes
    declare this via a module-level `ROBOT_KIND` attribute that the
    runner reads with `getattr`."""
    manipulator_spec = _coerce_manipulator_spec(side, robot_kind)
    adapter = robot_adapter(manipulator_spec.robot_kind)
    side = manipulator_spec.side

    jnt_names = [f"{side}{suffix}" for suffix in adapter.joint_suffixes]
    jnt_ids = np.array(
        [_resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, n, "joint") for n in jnt_names]
    )
    qpos_idx = np.array([model.jnt_qposadr[j] for j in jnt_ids])
    dof_idx = np.array([model.jnt_dofadr[j] for j in jnt_ids])

    act_arm_names = [f"{side}{suffix}" for suffix in adapter.arm_actuator_suffixes]
    act_arm_ids = np.array(
        [_resolve_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n, "actuator") for n in act_arm_names]
    )
    act_gripper_id = _resolve_id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        f"{side}{adapter.gripper_actuator_suffix}",
        "gripper actuator",
    )
    link6_id = _resolve_id(
        model, mujoco.mjtObj.mjOBJ_BODY, f"{side}{adapter.wrist_body_suffix}", "wrist body"
    )
    tcp_site_id = _resolve_id(
        model, mujoco.mjtObj.mjOBJ_SITE, f"{side}{adapter.tcp_site_suffix}", "TCP site"
    )

    piper_mirrored_gripper_qpos_idx: tuple[int, int] | None = None
    piper_mirrored_gripper_dof_idx: tuple[int, int] | None = None
    if adapter.piper_mirrored_gripper_joint_suffixes is not None:
        # Piper joint7 is a slide (range 0..0.035 m); joint8 mirrors via
        # tendon. Read the range rather than hardcoding numerics.
        gripper_jnt_id = _resolve_id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}joint7", "Piper joint7"
        )
        lo, hi = model.jnt_range[gripper_jnt_id]
        gripper_open = float(hi)
        gripper_closed = float(lo)
        first_gripper_joint_suffix, second_gripper_joint_suffix = (
            adapter.piper_mirrored_gripper_joint_suffixes
        )
        mirrored_joint_ids = (
            _resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}{suffix}", "Piper gripper joint")
            for suffix in (first_gripper_joint_suffix, second_gripper_joint_suffix)
        )
        first_gripper_joint_id, second_gripper_joint_id = mirrored_joint_ids
        piper_mirrored_gripper_qpos_idx = (
            int(model.jnt_qposadr[first_gripper_joint_id]),
            int(model.jnt_qposadr[second_gripper_joint_id]),
        )
        piper_mirrored_gripper_dof_idx = (
            int(model.jnt_dofadr[first_gripper_joint_id]),
            int(model.jnt_dofadr[second_gripper_joint_id]),
        )
    else:
        # Robotiq 2F-85 ctrlrange: 0 fully open, 255 fully closed.
        gripper_open = 0.0
        gripper_closed = 255.0

    weld_ids = np.array(
        [
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                f"{side.replace('/', '_')}grasp_cube{i}",
            )
            for i in range(n_cubes)
        ],
        dtype=np.int64,
    )

    return ArmHandles(
        side=side,
        robot_kind=adapter.robot_kind,
        qpos_idx=qpos_idx,
        dof_idx=dof_idx,
        jnt_ids=jnt_ids,
        arm_dof_idx=dof_idx[: adapter.controlled_arm_joint_count],
        act_arm_ids=act_arm_ids,
        act_gripper_id=act_gripper_id,
        link6_id=link6_id,
        tcp_site_id=tcp_site_id,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        weld_ids=weld_ids,
        piper_mirrored_gripper_qpos_idx=piper_mirrored_gripper_qpos_idx,
        piper_mirrored_gripper_dof_idx=piper_mirrored_gripper_dof_idx,
    )


def manipulator_specs_from_legacy_arm_sides(
    arm_sides: Sequence[ArmSide],
    robot_kind: RobotKind,
) -> tuple[ManipulatorSpec, ...]:
    return tuple(ManipulatorSpec(side=side, robot_kind=robot_kind) for side in arm_sides)
