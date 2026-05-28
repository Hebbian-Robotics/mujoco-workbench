"""Indicator-check scene — full OpenArm v2 embodiment.

This is the OpenArm embodiment port of
`examples.scenes.mobile_aloha_piper_indicator_check`. It keeps the same
data-center layout, base path, phase contracts, forward camera, and indicator
light behavior, but replaces the whole robot body with OpenArm v2's pedestal
body plus native bimanual arms loaded from a local `openarm_mujoco` checkout.

Set `OPENARM_MUJOCO_PATH` if the OpenArm checkout is not at the default
`/Users/<user>/openarm/openarm_mujoco` sibling path.
"""

from __future__ import annotations

import math
from enum import StrEnum

import mujoco
import numpy as np
from dm_control import mjcf

from examples.paths import D405_MESH_STL, D435I_XML
from examples.robots.openarm import (
    OPENARM_V2_BASE_X_JOINT_NAME,
    OPENARM_V2_BASE_Y_JOINT_NAME,
    OPENARM_V2_BASE_YAW_JOINT_NAME,
    OPENARM_V2_BIMANUAL_MOUNT_SITE,
    OPENARM_V2_BODY_NAME,
    OPENARM_V2_MODEL_NAMESPACE,
    OPENARM_V2_TOP_CAMERA_BODY_NAME,
    OPENARM_V2_TOP_CAMERA_MOUNT_SITE,
    load_openarm_v2_bimanual,
    load_openarm_v2_pedestal_base,
)
from examples.scenes import mobile_aloha_piper_indicator_check as indicator_base
from examples.scenes.mobile_aloha_piper_indicator_check_layout import (
    ALERT_LIGHT_GEOM_NAME,
    LAYOUT,
)
from mujoco_workbench.arm_handles import (
    ArmHandles,
    ArmSide,
    ManipulatorSpec,
    openarm_v2_manipulator_spec,
    write_gripper_target,
)
from mujoco_workbench.cameras import CameraRole
from mujoco_workbench.scene_base import (
    GripperState,
    JointSetStatic,
    MobileBaseTarget,
    PhaseContract,
    PhaseState,
    QaccSentinel,
    Step,
    TaskPhase,
)
from mujoco_workbench.scene_check import (
    AttachmentConstraint,
    CameraInvariant,
    FixedCameraInvariant,
)

NAME = "openarm_v2_indicator_check"
DEFAULT_VISER_CAMERA_POSE = indicator_base.DEFAULT_VISER_CAMERA_POSE
IK_LOCKED_JOINT_NAMES: tuple[str, ...] = (
    OPENARM_V2_BASE_X_JOINT_NAME,
    OPENARM_V2_BASE_Y_JOINT_NAME,
    OPENARM_V2_BASE_YAW_JOINT_NAME,
)
ARM_SIDES: tuple[ArmSide, ...] = (ArmSide.LEFT, ArmSide.RIGHT)
MANIPULATORS: tuple[ManipulatorSpec, ...] = tuple(
    openarm_v2_manipulator_spec(side, namespace=OPENARM_V2_MODEL_NAMESPACE) for side in ARM_SIDES
)
GRIPPABLES: tuple[str, ...] = ()
N_CUBES = 0
ATTACHMENTS: tuple[AttachmentConstraint, ...] = ()
AUX_ACTUATOR_NAMES: tuple[str, ...] = ()


class IndicatorBaseActuator(StrEnum):
    """Planar mobile-base actuators."""

    BASE_X = OPENARM_V2_BASE_X_JOINT_NAME
    BASE_Y = OPENARM_V2_BASE_Y_JOINT_NAME
    BASE_YAW = OPENARM_V2_BASE_YAW_JOINT_NAME


BASE_ACTUATOR_NAMES: tuple[str, ...] = (
    IndicatorBaseActuator.BASE_X.value,
    IndicatorBaseActuator.BASE_Y.value,
    IndicatorBaseActuator.BASE_YAW.value,
)


def base_target(*, x: float, y: float, yaw: float) -> MobileBaseTarget:
    """Canonical phase-boundary representation for planar base targets."""
    return MobileBaseTarget(x=x, y=y, yaw=yaw)


CAMERAS: tuple[tuple[str, CameraRole], ...] = (
    ("forward_cam", CameraRole.TOP),
    (f"{OPENARM_V2_MODEL_NAMESPACE}/camera_wrist_left", CameraRole.WRIST),
    (f"{OPENARM_V2_MODEL_NAMESPACE}/camera_wrist_right", CameraRole.WRIST),
)

CAMERA_INVARIANTS: tuple[CameraInvariant, ...] = (
    FixedCameraInvariant(name="forward_cam", parent_body=OPENARM_V2_TOP_CAMERA_BODY_NAME),
    FixedCameraInvariant(
        name=f"{OPENARM_V2_MODEL_NAMESPACE}/camera_wrist_left",
        parent_body=f"{OPENARM_V2_MODEL_NAMESPACE}/openarm_left_ee_base_link",
    ),
    FixedCameraInvariant(
        name=f"{OPENARM_V2_MODEL_NAMESPACE}/camera_wrist_right",
        parent_body=f"{OPENARM_V2_MODEL_NAMESPACE}/openarm_right_ee_base_link",
    ),
)

_ALERT_SERVER_TOUCH_GEOM_NAMES: tuple[str, ...] = (
    ALERT_LIGHT_GEOM_NAME,
    *(
        f"server_{LAYOUT.alert.row}_r{LAYOUT.alert.rack_index}_s{slot_index:02d}"
        for slot_index in range(
            max(0, LAYOUT.alert.slot_index - 1),
            min(LAYOUT.servers.n_per_rack, LAYOUT.alert.slot_index + 3),
        )
    ),
)
_FINGERTIP_BODY_NAMES: tuple[str, ...] = tuple(
    f"{OPENARM_V2_MODEL_NAMESPACE}/openarm_{side.label}_{finger_body_name}"
    for side in ARM_SIDES
    for finger_body_name in ("ee_inner_finger", "ee_outer_finger")
)
ALLOWED_STATIC_OVERLAPS: tuple[tuple[str, str], ...] = tuple(
    (finger_body_name, target_geom_name)
    for finger_body_name in _FINGERTIP_BODY_NAMES
    for target_geom_name in _ALERT_SERVER_TOUCH_GEOM_NAMES
)

HOME_ARM_Q_BY_SIDE: dict[ArmSide, np.ndarray] = {
    ArmSide.LEFT: np.array([0.0, -0.35, 0.0, 1.25, 0.0, 0.0, 0.0], dtype=float),
    ArmSide.RIGHT: np.array([0.0, 0.35, 0.0, 1.25, 0.0, 0.0, 0.0], dtype=float),
}
REACH_ARM_Q_BY_SIDE: dict[ArmSide, np.ndarray] = {
    ArmSide.LEFT: np.array([-0.45, -0.90, -0.35, 1.55, 0.20, 0.0, 0.0], dtype=float),
    ArmSide.RIGHT: np.array([0.45, 0.90, 0.35, 1.55, -0.20, 0.0, 0.0], dtype=float),
}

_QACC_SENTINEL = QaccSentinel(max_increase=0)
_ARM_JOINT_NAMES: tuple[str, ...] = tuple(
    joint_name for manipulator in MANIPULATORS for joint_name in manipulator.joint_names
)
_ARMS_STATIC = JointSetStatic(joint_names=_ARM_JOINT_NAMES, label="arms")
_BASE_STATIC = JointSetStatic(joint_names=IK_LOCKED_JOINT_NAMES, label="base")

_CLICK_X, _CLICK_Y = LAYOUT.click_chassis_xy
_CLICK_YAW = math.pi / 2.0 if LAYOUT.alert.row == "left" else -math.pi / 2.0
_BASE_ORIGIN = base_target(x=0.0, y=0.0, yaw=0.0)
_BASE_AT_TRAVERSE_END = base_target(x=_CLICK_X, y=0.0, yaw=0.0)
_BASE_AT_CLICK = base_target(x=_CLICK_X, y=_CLICK_Y, yaw=_CLICK_YAW)

PHASE_CONTRACTS: tuple[PhaseContract, ...] = (
    PhaseContract(
        phase=TaskPhase.SETUP,
        starts=PhaseState(description="Robot at world origin.", base_target=_BASE_ORIGIN),
        ends=PhaseState(description="Ready to drive into aisle.", base_target=_BASE_ORIGIN),
        invariants=(_QACC_SENTINEL, _ARMS_STATIC, _BASE_STATIC),
    ),
    PhaseContract(
        legal_predecessors=(TaskPhase.SETUP,),
        phase=TaskPhase.TRAVERSE_INTO_AISLE,
        starts=PhaseState(description="Robot at origin facing +X.", base_target=_BASE_ORIGIN),
        ends=PhaseState(
            description="Robot aligned with alert rack column.",
            base_target=_BASE_AT_TRAVERSE_END,
        ),
        invariants=(_QACC_SENTINEL, _ARMS_STATIC),
    ),
    PhaseContract(
        legal_predecessors=(TaskPhase.TRAVERSE_INTO_AISLE,),
        phase=TaskPhase.ALIGN_TO_TARGET,
        starts=PhaseState(description="Robot in aisle.", base_target=_BASE_AT_TRAVERSE_END),
        ends=PhaseState(description="Robot at click pose.", base_target=_BASE_AT_CLICK),
        invariants=(_QACC_SENTINEL, _ARMS_STATIC),
    ),
    PhaseContract(
        legal_predecessors=(TaskPhase.ALIGN_TO_TARGET,),
        phase=TaskPhase.REACH_TO_SERVER,
        starts=PhaseState(description="Arms at home.", base_target=_BASE_AT_CLICK),
        ends=PhaseState(description="Arms extended near alert server.", base_target=_BASE_AT_CLICK),
        invariants=(_QACC_SENTINEL, _BASE_STATIC),
    ),
    PhaseContract(
        legal_predecessors=(TaskPhase.REACH_TO_SERVER,),
        phase=TaskPhase.WAIT_AT_SERVER,
        starts=PhaseState(description="Light still red.", base_target=_BASE_AT_CLICK),
        ends=PhaseState(description="Light flipped green.", base_target=_BASE_AT_CLICK),
        invariants=(_QACC_SENTINEL, _ARMS_STATIC, _BASE_STATIC),
    ),
    PhaseContract(
        legal_predecessors=(TaskPhase.WAIT_AT_SERVER,),
        phase=TaskPhase.RETRACT,
        starts=PhaseState(description="Arms extended.", base_target=_BASE_AT_CLICK),
        ends=PhaseState(description="Arms at home.", base_target=_BASE_AT_CLICK),
        invariants=(_QACC_SENTINEL, _BASE_STATIC),
    ),
)


def _add_openarm_wrist_camera_visual(
    openarm: mjcf.RootElement,
    d405_mesh: mjcf.Element,
    *,
    side: ArmSide,
) -> None:
    wrist = openarm.find("body", f"openarm_{side.label}_ee_base_link")
    if wrist is None:
        raise RuntimeError(f"OpenArm v2 missing {side.label} wrist body; cannot add wrist camera.")
    camera = openarm.find("camera", f"camera_wrist_{side.label}")
    if camera is None:
        raise RuntimeError(f"OpenArm v2 missing native {side.label} wrist camera.")
    wrist.add(
        "geom",
        name=f"openarm_{side.label}_wrist_d405_visual",
        type="mesh",
        mesh=d405_mesh,
        pos=list(camera.pos),
        euler=list(camera.euler),
        rgba=[0.08, 0.08, 0.08, 1.0],
        contype=0,
        conaffinity=0,
        group=1,
        density=0,
    )


def _configure_scene_root(root: mjcf.RootElement) -> None:
    root.option.integrator = "implicitfast"
    root.option.cone = "elliptic"
    root.option.impratio = 10.0
    root.option.timestep = 0.002
    root.option.flag.contact = "disable"
    root.option.gravity = [0.0, 0.0, 0.0]

    visual_global = getattr(root.visual, "global")
    visual_global.offwidth = 1920
    visual_global.offheight = 1080
    root.visual.map.znear = 0.010
    root.visual.quality.offsamples = 8

    visual = root.default.add("default", dclass="visual")
    visual.geom.contype = 0
    visual.geom.conaffinity = 0
    visual.geom.mass = 0.0

    root.visual.headlight.diffuse = [0.4, 0.4, 0.4]
    root.visual.headlight.ambient = [0.25, 0.25, 0.25]
    root.visual.headlight.specular = [0.0, 0.0, 0.0]
    root.worldbody.add(
        "light",
        name="key_directional",
        pos=[3.0, 0.0, 4.0],
        dir=[0.0, 0.0, -1.0],
        directional="true",
        diffuse=[0.4, 0.4, 0.4],
        specular=[0.05, 0.05, 0.05],
    )
    root.worldbody.add(
        "geom",
        type="plane",
        size=[8.0, 4.0, 0.1],
        pos=[3.0, 0.0, 0.0],
        rgba=[0.45, 0.45, 0.48, 1.0],
    )


def build_spec() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Assemble the OpenArm v2 indicator-check scene."""
    root = load_openarm_v2_pedestal_base()
    _configure_scene_root(root)
    indicator_base._add_data_center(root)

    openarm_body = root.find("body", OPENARM_V2_BODY_NAME)
    if openarm_body is None:
        raise RuntimeError(f"OpenArm v2 body {OPENARM_V2_BODY_NAME!r} not found")

    d405_mesh = root.asset.add("mesh", name="d405", file=str(D405_MESH_STL))
    openarm = load_openarm_v2_bimanual()
    for side in ARM_SIDES:
        _add_openarm_wrist_camera_visual(openarm, d405_mesh, side=side)
    bimanual_mount = root.find("site", OPENARM_V2_BIMANUAL_MOUNT_SITE)
    if bimanual_mount is None:
        raise RuntimeError(
            f"OpenArm v2 bimanual mount {OPENARM_V2_BIMANUAL_MOUNT_SITE!r} not found"
        )
    bimanual_mount.attach(openarm)

    top_cam_mount = root.find("site", OPENARM_V2_TOP_CAMERA_MOUNT_SITE)
    if top_cam_mount is None:
        raise RuntimeError(
            f"OpenArm v2 camera mount {OPENARM_V2_TOP_CAMERA_MOUNT_SITE!r} not found"
        )
    top_camera_body = root.find("body", OPENARM_V2_TOP_CAMERA_BODY_NAME)
    if top_camera_body is None:
        raise RuntimeError(f"OpenArm v2 camera body {OPENARM_V2_TOP_CAMERA_BODY_NAME!r} not found")
    top_d435i = mjcf.from_path(str(D435I_XML))
    top_d435i.model = "top"
    top_cam_mount.attach(top_d435i)

    tilt_rad = math.radians(indicator_base._TOP_CAM_TILT_DOWN_DEG)
    sin_tilt = math.sin(tilt_rad)
    cos_tilt = math.cos(tilt_rad)
    top_camera_body.add(
        "camera",
        name="forward_cam",
        pos=[0.0, 0.0, 0.0],
        xyaxes=[0.0, -1.0, 0.0, sin_tilt, 0.0, cos_tilt],
        mode="fixed",
        fovy=indicator_base._TOP_CAM_FOVY_DEG,
    )

    root.actuator.add(
        "position",
        name=IndicatorBaseActuator.BASE_X,
        joint=IndicatorBaseActuator.BASE_X,
        kp=20000.0,
        kv=400.0,
        ctrllimited="true",
        ctrlrange=[-1.0, 7.0],
    )
    root.actuator.add(
        "position",
        name=IndicatorBaseActuator.BASE_Y,
        joint=IndicatorBaseActuator.BASE_Y,
        kp=20000.0,
        kv=400.0,
        ctrllimited="true",
        ctrlrange=[-1.0, 1.0],
    )
    root.actuator.add(
        "position",
        name=IndicatorBaseActuator.BASE_YAW,
        joint=IndicatorBaseActuator.BASE_YAW,
        kp=8000.0,
        kv=200.0,
        ctrllimited="true",
        ctrlrange=[-3.5, 3.5],
    )

    xml_str = root.to_xml_string()
    assets = dict(root.get_assets())
    model = mujoco.MjModel.from_xml_string(xml_str, assets)
    return model, mujoco.MjData(model)


def apply_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    cube_body_ids: list[int],
    *,
    start_phase: TaskPhase | None = None,
) -> None:
    """Reset to OpenArm home and optional phase-seeded base pose."""
    del cube_body_ids
    base_pose = _BASE_ORIGIN.as_tuple()
    if start_phase is not None and start_phase is not TaskPhase.SETUP:
        contract = next((c for c in PHASE_CONTRACTS if c.phase is start_phase), None)
        if contract is not None and contract.starts.base_target is not None:
            base_pose = contract.starts.base_target.as_tuple()

    mujoco.mj_resetData(model, data)
    for arm in arms.values():
        home_q = HOME_ARM_Q_BY_SIDE[arm.side]
        data.qpos[arm.arm_qpos_idx] = home_q
        data.ctrl[arm.act_arm_ids] = home_q
        write_gripper_target(data, arm, arm.gripper_open)

    for joint_name, value in zip(
        (
            IndicatorBaseActuator.BASE_X,
            IndicatorBaseActuator.BASE_Y,
            IndicatorBaseActuator.BASE_YAW,
        ),
        base_pose,
        strict=True,
    ):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
        data.qvel[int(model.jnt_dofadr[joint_id])] = 0.0
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
        if actuator_id >= 0:
            data.ctrl[actuator_id] = value

    mujoco.mj_forward(model, data)


def make_task_plan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    cube_body_ids: list[int],
) -> dict[ArmSide, list[Step]]:
    """Scripted OpenArm indicator-check choreography."""
    del model, data, arms, cube_body_ids
    scripts: dict[ArmSide, list[Step]] = {side: [] for side in ARM_SIDES}

    def push_both(
        label: str,
        duration: float,
        phase: TaskPhase,
        base: MobileBaseTarget,
        *,
        gripper: GripperState = "open",
        set_geom_rgba: tuple[tuple[str, tuple[float, float, float, float]], ...] = (),
    ) -> None:
        for side in ARM_SIDES:
            scripts[side].append(
                Step(
                    label=label,
                    arm_q=HOME_ARM_Q_BY_SIDE[side].copy(),
                    gripper=gripper,
                    duration=duration,
                    phase=phase,
                    base_target=base,
                    set_geom_rgba=set_geom_rgba,
                )
            )

    def push_arms(
        label: str,
        duration: float,
        phase: TaskPhase,
        arm_q_by_side: dict[ArmSide, np.ndarray],
        base: MobileBaseTarget,
        *,
        gripper: GripperState = "open",
    ) -> None:
        for side in ARM_SIDES:
            scripts[side].append(
                Step(
                    label=f"{label} {side.rstrip('_/')}",
                    arm_q=arm_q_by_side[side].copy(),
                    gripper=gripper,
                    duration=duration,
                    phase=phase,
                    base_target=base,
                )
            )

    push_both("home", 1.0, TaskPhase.SETUP, _BASE_ORIGIN)
    push_both("settle", 0.5, TaskPhase.SETUP, _BASE_ORIGIN)
    push_both("drive into aisle", 10.0, TaskPhase.TRAVERSE_INTO_AISLE, _BASE_AT_TRAVERSE_END)
    push_both(
        "yaw to face alert row",
        4.0,
        TaskPhase.ALIGN_TO_TARGET,
        base_target(x=_CLICK_X, y=0.0, yaw=_CLICK_YAW),
    )
    push_both("step toward rack front", 2.0, TaskPhase.ALIGN_TO_TARGET, _BASE_AT_CLICK)
    push_arms(
        "reach to server",
        3.0,
        TaskPhase.REACH_TO_SERVER,
        REACH_ARM_Q_BY_SIDE,
        _BASE_AT_CLICK,
    )
    for cycle_index in range(3):
        scripts[ArmSide.LEFT].append(
            Step(
                label=f"L close #{cycle_index}",
                arm_q=REACH_ARM_Q_BY_SIDE[ArmSide.LEFT].copy(),
                gripper="closed",
                duration=0.5,
                phase=TaskPhase.WAIT_AT_SERVER,
                base_target=_BASE_AT_CLICK,
            )
        )
        scripts[ArmSide.RIGHT].append(
            Step(
                label=f"R open #{cycle_index}",
                arm_q=REACH_ARM_Q_BY_SIDE[ArmSide.RIGHT].copy(),
                gripper="open",
                duration=0.5,
                phase=TaskPhase.WAIT_AT_SERVER,
                base_target=_BASE_AT_CLICK,
            )
        )
        scripts[ArmSide.LEFT].append(
            Step(
                label=f"L open #{cycle_index}",
                arm_q=REACH_ARM_Q_BY_SIDE[ArmSide.LEFT].copy(),
                gripper="open",
                duration=0.5,
                phase=TaskPhase.WAIT_AT_SERVER,
                base_target=_BASE_AT_CLICK,
            )
        )
        scripts[ArmSide.RIGHT].append(
            Step(
                label=f"R close #{cycle_index}",
                arm_q=REACH_ARM_Q_BY_SIDE[ArmSide.RIGHT].copy(),
                gripper="closed",
                duration=0.5,
                phase=TaskPhase.WAIT_AT_SERVER,
                base_target=_BASE_AT_CLICK,
            )
        )
    for side in ARM_SIDES:
        scripts[side].append(
            Step(
                label=f"flip indicator {side.rstrip('_/')}",
                arm_q=REACH_ARM_Q_BY_SIDE[side].copy(),
                gripper="open",
                duration=1.5,
                phase=TaskPhase.WAIT_AT_SERVER,
                base_target=_BASE_AT_CLICK,
                set_geom_rgba=((ALERT_LIGHT_GEOM_NAME, LAYOUT.light.rgba_green),)
                if side is ArmSide.LEFT
                else (),
            )
        )
    push_both("arms back to home", 2.0, TaskPhase.RETRACT, _BASE_AT_CLICK)

    for side in ARM_SIDES:
        print(f"  [{side}] {len(scripts[side])} steps planned")
    return scripts
