"""OpenArm robot loaders for workbench scenes.

The OpenArm repository is not vendored here. By default this loader looks for
the sibling checkout used during development:

    /Users/<user>/openarm/openarm_mujoco

Set `OPENARM_MUJOCO_PATH` to point at a different checkout.

OpenArm v1 ships torque motors. The workbench timeline runner puppets arm qpos
and mirrors targets into position-style `data.ctrl`, so this loader replaces
the upstream motor actuators with position actuators while keeping names stable.

OpenArm v2 ships a pedestal/body XML that uses MuJoCo's `<asset><model ...>`
include feature, which dm-control's MJCF parser cannot read. The v2 pedestal
loader below mirrors that upstream body explicitly and attaches the bimanual
MJCF that dm-control can parse.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from dm_control import mjcf

from mujoco_workbench.arm_handles import ArmSide

OPENARM_V1_ARM_JOINT_NAMES: tuple[str, ...] = tuple(
    f"openarm_joint{joint_index}" for joint_index in range(1, 8)
)
OPENARM_V1_ARM_ACTUATOR_NAMES: tuple[str, ...] = tuple(
    f"joint{joint_index}_ctrl" for joint_index in range(1, 8)
)
OPENARM_V1_GRIPPER_ACTUATOR_NAME = "finger_ctrl"
OPENARM_V1_TCP_SITE_NAME = "tcp"
OPENARM_V2_MODEL_NAMESPACE = "openarm_v2"
OPENARM_V2_BODY_NAME = "openarm_body_link0"
OPENARM_V2_BASE_X_JOINT_NAME = "base_x"
OPENARM_V2_BASE_Y_JOINT_NAME = "base_y"
OPENARM_V2_BASE_YAW_JOINT_NAME = "base_yaw"
OPENARM_V2_TOP_CAMERA_MOUNT_SITE = "openarm_top_cam_mount"
OPENARM_V2_TOP_CAMERA_BODY_NAME = "openarm_top_camera_body"
OPENARM_V2_BIMANUAL_MOUNT_SITE = "openarm_v2_bimanual_mount"

_OPENARM_V2_BIMANUAL_MOUNT_Z = 0.698
_OPENARM_V2_TOP_CAMERA_POS = [0.04, 0.0, 0.785]
_OPENARM_V2_BASE_PLATE_HALF_EXTENTS = [0.125, 0.095, 0.004]
_OPENARM_V2_POST_HALF_EXTENTS = [0.035, 0.035, _OPENARM_V2_BIMANUAL_MOUNT_Z / 2.0]
_OPENARM_V2_POST_CENTER_Z = _OPENARM_V2_BIMANUAL_MOUNT_Z / 2.0
_OPENARM_V2_ARM_ROOT_HOUSING_HALF_EXTENTS = [0.049, 0.0935, 0.061]

_OPENARM_V1_REQUIRED_BODIES: tuple[str, ...] = (
    "openarm_link1",
    "openarm_link2",
    "openarm_link3",
    "openarm_link4",
    "openarm_link5",
    "openarm_link6",
    "openarm_link7",
    "openarm_right_finger",
    "openarm_left_finger",
)
_OPENARM_V1_FINGER_JOINT_NAMES: tuple[str, str] = (
    "openarm_finger_joint1",
    "openarm_finger_joint2",
)


@dataclass(frozen=True)
class OpenArmPositionActuatorGain:
    """Position actuator gains for one upstream OpenArm joint."""

    actuator_name: str
    joint_name: str
    kp: float
    kv: float
    forcerange: tuple[float, float]


_DEFAULT_OPENARM_V1_POSITION_GAINS: tuple[OpenArmPositionActuatorGain, ...] = (
    OpenArmPositionActuatorGain("joint1_ctrl", "openarm_joint1", 220.0, 18.0, (-120.0, 120.0)),
    OpenArmPositionActuatorGain("joint2_ctrl", "openarm_joint2", 220.0, 18.0, (-120.0, 120.0)),
    OpenArmPositionActuatorGain("joint3_ctrl", "openarm_joint3", 180.0, 14.0, (-90.0, 90.0)),
    OpenArmPositionActuatorGain("joint4_ctrl", "openarm_joint4", 180.0, 14.0, (-90.0, 90.0)),
    OpenArmPositionActuatorGain("joint5_ctrl", "openarm_joint5", 80.0, 8.0, (-40.0, 40.0)),
    OpenArmPositionActuatorGain("joint6_ctrl", "openarm_joint6", 80.0, 8.0, (-40.0, 40.0)),
    OpenArmPositionActuatorGain("joint7_ctrl", "openarm_joint7", 80.0, 8.0, (-40.0, 40.0)),
    OpenArmPositionActuatorGain("finger_ctrl", "openarm_finger_joint1", 80.0, 4.0, (-20.0, 20.0)),
)


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _resolve_openarm_mujoco_root() -> Path:
    env_path = os.environ.get("OPENARM_MUJOCO_PATH")
    if env_path:
        return Path(env_path).expanduser()
    return _project_root().parents[1] / "openarm" / "openarm_mujoco"


def _openarm_v1_xml() -> Path:
    path = _resolve_openarm_mujoco_root() / "v1" / "openarm.xml"
    if not path.is_file():
        raise FileNotFoundError(
            f"OpenArm v1 MJCF not found: {path}. "
            "Clone https://github.com/enactic/openarm_mujoco next to this "
            "workspace or set OPENARM_MUJOCO_PATH."
        )
    return path


def _openarm_v2_xml() -> Path:
    path = _resolve_openarm_mujoco_root() / "v2" / "openarm_v20_bimanual.xml"
    if not path.is_file():
        raise FileNotFoundError(
            f"OpenArm v2 MJCF not found: {path}. "
            "Clone https://github.com/enactic/openarm_mujoco next to this "
            "workspace or set OPENARM_MUJOCO_PATH."
        )
    return path


def _assert_openarm_v1_shape(openarm: mjcf.RootElement) -> None:
    for body_name in _OPENARM_V1_REQUIRED_BODIES:
        if openarm.find("body", body_name) is None:
            raise RuntimeError(f"OpenArm v1 XML missing expected body {body_name!r}.")
    for joint_name in (*OPENARM_V1_ARM_JOINT_NAMES, *_OPENARM_V1_FINGER_JOINT_NAMES):
        if openarm.find("joint", joint_name) is None:
            raise RuntimeError(f"OpenArm v1 XML missing expected joint {joint_name!r}.")
    for actuator_name in (*OPENARM_V1_ARM_ACTUATOR_NAMES, OPENARM_V1_GRIPPER_ACTUATOR_NAME):
        if openarm.find("actuator", actuator_name) is None:
            raise RuntimeError(f"OpenArm v1 XML missing expected actuator {actuator_name!r}.")


def _replace_motors_with_position_actuators(openarm: mjcf.RootElement) -> None:
    for actuator in list(openarm.find_all("actuator")):
        actuator.remove()

    for gain in _DEFAULT_OPENARM_V1_POSITION_GAINS:
        joint = openarm.find("joint", gain.joint_name)
        if joint is None:
            raise RuntimeError(f"OpenArm v1 XML missing expected joint {gain.joint_name!r}.")
        if gain.actuator_name == OPENARM_V1_GRIPPER_ACTUATOR_NAME:
            ctrlrange = [0.0, 0.044]
        else:
            raw_range = getattr(joint, "range", None)
            if raw_range is None:
                raise RuntimeError(f"OpenArm joint {gain.joint_name!r} has no range.")
            ctrlrange = [float(raw_range[0]), float(raw_range[1])]

        openarm.actuator.add(
            "position",
            name=gain.actuator_name,
            joint=gain.joint_name,
            kp=gain.kp,
            kv=gain.kv,
            ctrllimited="true",
            ctrlrange=ctrlrange,
            forcelimited="true",
            forcerange=list(gain.forcerange),
        )


def _add_tcp_site(openarm: mjcf.RootElement) -> None:
    wrist = openarm.find("body", "openarm_link7")
    if wrist is None:
        raise RuntimeError("OpenArm v1 XML missing openarm_link7; cannot add TCP site.")
    wrist.add(
        "site",
        name=OPENARM_V1_TCP_SITE_NAME,
        pos=[0.0, 0.0, 0.18],
        size=[0.006, 0.006, 0.006],
        rgba=[1.0, 0.0, 1.0, 0.0],
    )


def _assert_openarm_v2_shape(openarm: mjcf.RootElement) -> None:
    for side_label in ("left", "right"):
        required_bodies = (
            f"openarm_{side_label}_base_link",
            f"openarm_{side_label}_link6",
            f"openarm_{side_label}_ee_base_link",
        )
        for body_name in required_bodies:
            if openarm.find("body", body_name) is None:
                raise RuntimeError(f"OpenArm v2 XML missing expected body {body_name!r}.")
        for joint_index in range(1, 8):
            joint_name = f"openarm_{side_label}_joint{joint_index}"
            actuator_name = f"{side_label}_joint{joint_index}_ctrl"
            if openarm.find("joint", joint_name) is None:
                raise RuntimeError(f"OpenArm v2 XML missing expected joint {joint_name!r}.")
            if openarm.find("actuator", actuator_name) is None:
                raise RuntimeError(f"OpenArm v2 XML missing expected actuator {actuator_name!r}.")
        if openarm.find("actuator", f"{side_label}_finger1_ctrl") is None:
            raise RuntimeError(
                f"OpenArm v2 XML missing expected gripper actuator {side_label}_finger1_ctrl."
            )


def _set_openarm_v2_base_spacing(openarm: mjcf.RootElement, *, y_abs: float) -> None:
    left_base = openarm.find("body", "openarm_left_base_link")
    right_base = openarm.find("body", "openarm_right_base_link")
    if left_base is None or right_base is None:
        raise RuntimeError("OpenArm v2 XML missing base bodies; cannot set mount spacing.")
    left_base.pos = [0.0, y_abs, 0.0]
    right_base.pos = [0.0, -y_abs, 0.0]


def _add_openarm_v2_tcp_sites(openarm: mjcf.RootElement) -> None:
    for side_label in ("left", "right"):
        wrist = openarm.find("body", f"openarm_{side_label}_ee_base_link")
        if wrist is None:
            raise RuntimeError(f"OpenArm v2 missing {side_label} wrist body; cannot add TCP site.")
        wrist.add(
            "site",
            name=f"openarm_{side_label}_tcp",
            pos=[0.0, 0.0, -0.11],
            size=[0.006, 0.006, 0.006],
            rgba=[1.0, 0.0, 1.0, 0.0],
        )


def _remove_openarm_v2_arm_root_housing_meshes(openarm: mjcf.RootElement) -> None:
    """Remove non-actuated root housing meshes from the bimanual arm subtree.

    The upstream root housing meshes are decorative shells around each arm's
    first joint. Keeping the body and child joint chain preserves kinematics.
    The pedestal base supplies a single primitive replacement visual.
    """
    for side_label in ("left", "right"):
        base_body = openarm.find("body", f"openarm_{side_label}_base_link")
        if base_body is None:
            raise RuntimeError(f"OpenArm v2 XML missing {side_label} base body.")
        for geom_name in (
            f"link0_0_{side_label}_vis",
            f"link0_1_{side_label}_vis",
            f"link0_2_{side_label}_vis",
            f"base_link_{side_label}_collision_00",
        ):
            geom = openarm.find("geom", geom_name)
            if geom is None:
                raise RuntimeError(f"OpenArm v2 XML missing expected geom {geom_name!r}.")
            geom.remove()


def load_openarm_v1(side: ArmSide) -> mjcf.RootElement:
    """Load OpenArm v1, namespace it under `side`, and adapt controls.

    Returns the un-attached `mjcf.RootElement` so scenes can add cameras or
    visual payloads before attaching it to a mobile-base mount site.
    """
    openarm = mjcf.from_path(str(_openarm_v1_xml()))
    openarm.model = side.rstrip("/")
    _assert_openarm_v1_shape(openarm)
    _replace_motors_with_position_actuators(openarm)
    _add_tcp_site(openarm)
    return openarm


def load_openarm_v2_bimanual(*, base_y_abs: float = 0.031) -> mjcf.RootElement:
    """Load OpenArm v2 as one native bimanual MJCF subtree.

    `base_y_abs=0.031` matches upstream `pedestal.xml`. Larger values can be
    used only by scenes that intentionally mount the arms onto another body.
    """
    openarm = mjcf.from_path(str(_openarm_v2_xml()))
    openarm.model = OPENARM_V2_MODEL_NAMESPACE
    _assert_openarm_v2_shape(openarm)
    _set_openarm_v2_base_spacing(openarm, y_abs=base_y_abs)
    _add_openarm_v2_tcp_sites(openarm)
    _remove_openarm_v2_arm_root_housing_meshes(openarm)
    return openarm


def load_openarm_v2_pedestal_base() -> mjcf.RootElement:
    """Return the movable OpenArm v2 pedestal/body base.

    The returned root keeps OpenArm's native pedestal dimensions with simple
    primitive geometry, planar base joints used by workbench scripted
    timelines, and mount sites for the bimanual arms plus a forward camera.
    """
    root = mjcf.RootElement(model="openarm_v2_pedestal_base")

    body = root.worldbody.add("body", name=OPENARM_V2_BODY_NAME, pos=[0.0, 0.0, 0.0])
    body.add(
        "joint",
        name=OPENARM_V2_BASE_X_JOINT_NAME,
        type="slide",
        axis=[1.0, 0.0, 0.0],
        damping=50.0,
        limited="false",
    )
    body.add(
        "joint",
        name=OPENARM_V2_BASE_Y_JOINT_NAME,
        type="slide",
        axis=[0.0, 1.0, 0.0],
        damping=50.0,
        limited="false",
    )
    body.add(
        "joint",
        name=OPENARM_V2_BASE_YAW_JOINT_NAME,
        type="hinge",
        axis=[0.0, 0.0, 1.0],
        damping=20.0,
        limited="false",
    )
    body.add(
        "geom",
        name="openarm_body_base_plate",
        type="box",
        size=_OPENARM_V2_BASE_PLATE_HALF_EXTENTS,
        pos=[-0.03, 0.0, _OPENARM_V2_BASE_PLATE_HALF_EXTENTS[2]],
        rgba=[0.42, 0.42, 0.44, 1.0],
        contype=0,
        conaffinity=0,
        mass=0.0,
    )
    body.add(
        "geom",
        name="openarm_body_post",
        type="box",
        size=_OPENARM_V2_POST_HALF_EXTENTS,
        pos=[0.0, 0.0, _OPENARM_V2_POST_CENTER_Z],
        rgba=[0.12, 0.12, 0.13, 1.0],
        contype=0,
        conaffinity=0,
        mass=0.0,
    )
    body.add(
        "geom",
        name="openarm_arm_root_housing",
        type="box",
        size=_OPENARM_V2_ARM_ROOT_HOUSING_HALF_EXTENTS,
        pos=[0.0, 0.0, _OPENARM_V2_BIMANUAL_MOUNT_Z],
        rgba=[0.16, 0.16, 0.17, 1.0],
        contype=0,
        conaffinity=0,
        mass=0.0,
    )
    body.add(
        "site",
        name=OPENARM_V2_BIMANUAL_MOUNT_SITE,
        pos=[0.0, 0.0, _OPENARM_V2_BIMANUAL_MOUNT_Z],
        quat=[1.0, 0.0, 0.0, 0.0],
        size=[0.001, 0.001, 0.001],
    )
    top_camera_body = body.add(
        "body",
        name=OPENARM_V2_TOP_CAMERA_BODY_NAME,
        pos=_OPENARM_V2_TOP_CAMERA_POS,
    )
    top_camera_body.add(
        "site",
        name=OPENARM_V2_TOP_CAMERA_MOUNT_SITE,
        pos=[0.0, 0.0, 0.0],
        quat=[0.65328148, 0.27059805, -0.27059805, -0.65328148],
        size=[0.001, 0.001, 0.001],
    )
    return root
