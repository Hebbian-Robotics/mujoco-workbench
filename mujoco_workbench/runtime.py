"""Shared helpers for non-interactive debug commands.

Owns the "import scene module, compile, advance to t, render" plumbing so
each tool stays a thin CLI wrapper. Step-boundary semantics (weld activate,
attach deactivate, ctrl interpolation) are replicated here once and mirror
`runner.advance_arm`.
"""

from __future__ import annotations

import importlib
import inspect
import math
import os
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Literal, NewType, cast

# Pick a safe MuJoCo GL backend before importing mujoco. Linux EC2 needs
# EGL for headless rendering; macOS rejects `egl`, so use GLFW there.
os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "darwin" else "egl")

# Add the repository root to sys.path so path-executed tools can import the
# examples package regardless of cwd.
_REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
if str(_REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPOSITORY_ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from mujoco_workbench.arm_handles import (  # noqa: E402
    ArmHandles,
    ArmSide,
    ManipulatorSpec,
    RobotKind,
    get_arm_handles,
    manipulator_specs_from_legacy_arm_sides,
    parse_robot_kind,
)
from mujoco_workbench.cameras import CameraRole  # noqa: E402
from mujoco_workbench.scene_base import (  # noqa: E402
    MobileBaseTarget,
    PhaseContract,
    Step,
    TaskPhase,
)
from mujoco_workbench.scene_check import AttachmentConstraint, CameraInvariant  # noqa: E402
from mujoco_workbench.welds import (  # noqa: E402
    activate_attachment_weld,
    activate_grasp_weld,
    deactivate_grasp_weld,
)

SceneName = NewType("SceneName", str)
Seconds = NewType("Seconds", float)
AzimuthDeg = NewType("AzimuthDeg", float)
ElevationDeg = NewType("ElevationDeg", float)
Metres = NewType("Metres", float)
WorldPoint = tuple[float, float, float]
BuildSpec = Callable[[], tuple[mujoco.MjModel, mujoco.MjData]]
ApplyInitialState = Callable[..., None]
MakeTaskPlan = Callable[
    [mujoco.MjModel, mujoco.MjData, dict[ArmSide, ArmHandles], list[int]],
    dict[ArmSide, list[Step]],
]
StepFreePlay = Callable[[float, mujoco.MjModel, mujoco.MjData], None]


@dataclass(frozen=True)
class MobileBaseSpec:
    """Scene-owned mobile-base actuator names parsed at the module boundary."""

    actuator_names: tuple[str, ...]


@dataclass(frozen=True)
class LiftSpec:
    """Scene-owned vertical lift actuator parsed at the module boundary."""

    actuator_name: str


@dataclass(frozen=True)
class TimelineActuatorMaps:
    """Resolved actuator ids and qpos/dof addresses for timeline components."""

    base_name_to_id: dict[str, int]
    base_qposadr: dict[str, int]
    base_dofadr: dict[str, int]
    lift_name_to_id: dict[str, int]
    lift_qposadr: dict[str, int]
    lift_dofadr: dict[str, int]
    aux_name_to_id: dict[str, int]


@dataclass(frozen=True)
class LoadedScene:
    """Parsed scene module boundary.

    Scene files are intentionally lightweight Python modules, which means their
    public contract is dynamic at import time. `load_scene` parses that module
    once and returns this typed object so runner/debug code can consume explicit
    fields and callbacks instead of repeating `getattr` checks.
    """

    module_name: SceneName
    display_name: str
    module: ModuleType
    build_spec: BuildSpec
    apply_initial_state_callback: ApplyInitialState
    supports_start_phase: bool
    make_task_plan: MakeTaskPlan | None
    step_free_play: StepFreePlay | None
    manipulators: tuple[ManipulatorSpec, ...]
    arm_sides: tuple[ArmSide, ...]
    n_cubes: int
    robot_kind: RobotKind
    mobile_base: MobileBaseSpec | None
    lift: LiftSpec | None
    grippable_names: tuple[str, ...]
    aux_actuator_names: tuple[str, ...]
    attachment_constraints: tuple[AttachmentConstraint, ...]
    allowed_static_overlaps: tuple[tuple[str, str], ...]
    camera_invariants: tuple[CameraInvariant, ...]
    phase_contracts: tuple[PhaseContract, ...]
    cameras: tuple[tuple[str, CameraRole], ...]
    ik_locked_joint_names: tuple[str, ...]
    ik_seed_q: np.ndarray | None

    def apply_initial_state(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        arms: dict[ArmSide, ArmHandles],
        cube_body_ids: list[int],
        *,
        start_phase: TaskPhase | None = None,
    ) -> None:
        if self.supports_start_phase:
            self.apply_initial_state_callback(
                model,
                data,
                arms,
                cube_body_ids,
                start_phase=start_phase,
            )
            return
        if start_phase is not None:
            print(
                f"[runner] scene {self.display_name!r} does not support --start-phase; "
                f"booting at scene home."
            )
        self.apply_initial_state_callback(model, data, arms, cube_body_ids)


def _required_callable(module: ModuleType, scene_name: SceneName, attr_name: str) -> Any:
    value = getattr(module, attr_name, None)
    if not callable(value):
        raise TypeError(f"scene {scene_name!r} must define callable {attr_name}()")
    return value


def _optional_callable(module: ModuleType, attr_name: str) -> Any | None:
    value = getattr(module, attr_name, None)
    if value is None:
        return None
    if not callable(value):
        raise TypeError(f"scene attribute {attr_name!r} must be callable when present")
    return value


def _tuple_attr(module: ModuleType, attr_name: str) -> tuple[Any, ...]:
    value = getattr(module, attr_name, ())
    if value is None:
        return ()
    return tuple(value)


def _parse_scene_robot_kind(raw_robot_kind: object, scene_name: SceneName) -> RobotKind:
    try:
        return parse_robot_kind(raw_robot_kind)
    except ValueError as err:
        raise ValueError(
            f"scene {scene_name!r} has unsupported ROBOT_KIND={raw_robot_kind!r}"
        ) from err


def _parse_arm_sides(raw_arm_sides: tuple[Any, ...], scene_name: SceneName) -> tuple[ArmSide, ...]:
    try:
        return tuple(ArmSide(raw_arm_side) for raw_arm_side in raw_arm_sides)
    except ValueError as err:
        valid = ", ".join(side.value for side in ArmSide)
        raise ValueError(
            f"scene {scene_name!r} has unsupported ARM_PREFIXES entry; expected one of: {valid}"
        ) from err


def _parse_manipulator_side(raw_side: object, scene_name: SceneName) -> ArmSide:
    try:
        return ArmSide(str(raw_side))
    except ValueError as err:
        valid = ", ".join(side.value for side in ArmSide)
        raise ValueError(
            f"scene {scene_name!r} has unsupported manipulator side {raw_side!r}; "
            f"expected one of: {valid}"
        ) from err


def _parse_manipulator_spec(
    raw_manipulator: object,
    scene_name: SceneName,
    default_robot_kind: RobotKind,
) -> ManipulatorSpec:
    if isinstance(raw_manipulator, ManipulatorSpec):
        return raw_manipulator
    if isinstance(raw_manipulator, ArmSide):
        return ManipulatorSpec(side=raw_manipulator, robot_kind=default_robot_kind)
    if isinstance(raw_manipulator, Mapping):
        raw_manipulator_mapping = cast(Mapping[str, object], raw_manipulator)
        raw_side = raw_manipulator_mapping.get("side")
        raw_robot_kind = raw_manipulator_mapping.get("robot_kind", default_robot_kind)
        return ManipulatorSpec(
            side=_parse_manipulator_side(raw_side, scene_name),
            robot_kind=_parse_scene_robot_kind(raw_robot_kind, scene_name),
        )
    raise ValueError(
        f"scene {scene_name!r} has unsupported MANIPULATORS entry {raw_manipulator!r}; "
        "expected ManipulatorSpec or {'side': ..., 'robot_kind': ...}"
    )


def _parse_manipulators(
    module: ModuleType,
    scene_name: SceneName,
    default_robot_kind: RobotKind,
) -> tuple[ManipulatorSpec, ...]:
    raw_manipulators = _tuple_attr(module, "MANIPULATORS")
    if raw_manipulators:
        manipulators = tuple(
            _parse_manipulator_spec(raw_manipulator, scene_name, default_robot_kind)
            for raw_manipulator in raw_manipulators
        )
    else:
        arm_sides = _parse_arm_sides(_tuple_attr(module, "ARM_PREFIXES"), scene_name)
        manipulators = manipulator_specs_from_legacy_arm_sides(arm_sides, default_robot_kind)

    if len(manipulators) > 2:
        raise ValueError(
            f"scene {scene_name!r} declares {len(manipulators)} manipulators; "
            "only unimanual and bimanual scenes are supported"
        )
    sides = [manipulator.side for manipulator in manipulators]
    if len(set(sides)) != len(sides):
        raise ValueError(f"scene {scene_name!r} declares duplicate manipulator sides")
    return manipulators


def _parse_mobile_base(
    module: ModuleType,
    *,
    scene_name: SceneName,
) -> MobileBaseSpec | None:
    raw_base_actuator_names = _tuple_attr(module, "BASE_ACTUATOR_NAMES")
    if not raw_base_actuator_names:
        return None

    actuator_names = tuple(str(name) for name in raw_base_actuator_names)
    if len(actuator_names) != 3:
        raise ValueError(
            f"scene {scene_name!r} BASE_ACTUATOR_NAMES must be exactly "
            "(x, y, yaw); got {actuator_names!r}"
        )
    if len(set(actuator_names)) != len(actuator_names):
        raise ValueError(
            f"scene {scene_name!r} BASE_ACTUATOR_NAMES contains duplicate names: {actuator_names!r}"
        )
    return MobileBaseSpec(actuator_names=actuator_names)


def _parse_lift(module: ModuleType, *, scene_name: SceneName) -> LiftSpec | None:
    raw_lift_actuator_name = getattr(module, "LIFT_ACTUATOR_NAME", None)
    if raw_lift_actuator_name is None:
        return None
    actuator_name = str(raw_lift_actuator_name)
    if not actuator_name:
        raise ValueError(f"scene {scene_name!r} LIFT_ACTUATOR_NAME must not be empty")
    return LiftSpec(actuator_name=actuator_name)


def _validate_scene_owned_actuator_names(
    *,
    scene_name: SceneName,
    mobile_base: MobileBaseSpec | None,
    lift: LiftSpec | None,
    aux_actuator_names: tuple[str, ...],
) -> None:
    component_names: dict[str, str] = {}
    if mobile_base is not None:
        for actuator_name in mobile_base.actuator_names:
            component_names[actuator_name] = "BASE_ACTUATOR_NAMES"
    if lift is not None:
        if lift.actuator_name in component_names:
            raise ValueError(
                f"scene {scene_name!r} declares lift actuator {lift.actuator_name!r} "
                "inside BASE_ACTUATOR_NAMES"
            )
        component_names[lift.actuator_name] = "LIFT_ACTUATOR_NAME"

    duplicated_names = sorted(name for name in aux_actuator_names if name in component_names)
    if duplicated_names:
        owner_list = ", ".join(f"{name} ({component_names[name]})" for name in duplicated_names)
        raise ValueError(
            f"scene {scene_name!r} declares {owner_list} again in AUX_ACTUATOR_NAMES; "
            "aux actuators must exclude arm, base, and lift components"
        )


def _ensure_target_not_in_aux_ctrl(
    *,
    scene: LoadedScene,
    step: Step,
    forbidden_names: set[str],
    component_name: str,
) -> None:
    if not step.aux_ctrl:
        return
    duplicated_names = sorted(str(name) for name in step.aux_ctrl if str(name) in forbidden_names)
    if duplicated_names:
        raise ValueError(
            f"scene {scene.module_name!r} step {step.label!r} puts {component_name} "
            f"actuator(s) {duplicated_names} in aux_ctrl; use the typed target field instead"
        )


def validate_task_plan_targets(
    scene: LoadedScene,
    arms: dict[ArmSide, ArmHandles],
    task_plan: dict[ArmSide, list[Step]],
) -> None:
    """Validate task-plan shape after robot adapters have refined arm metadata."""
    expected_sides = set(scene.arm_sides)
    actual_sides = set(task_plan)
    if actual_sides != expected_sides:
        raise ValueError(
            f"scene {scene.module_name!r} task plan sides {sorted(side.value for side in actual_sides)} "
            f"do not match manipulators {sorted(side.value for side in expected_sides)}"
        )

    for side, script in task_plan.items():
        expected_joint_count = len(arms[side].arm_qpos_idx)
        for step in script:
            if step.arm_q.shape != (expected_joint_count,):
                raise ValueError(
                    f"scene {scene.module_name!r} step {step.label!r} for {side.value} "
                    f"has arm_q shape {step.arm_q.shape}; expected ({expected_joint_count},) "
                    f"for {arms[side].robot_kind.value}"
                )
            if step.base_target is not None and scene.mobile_base is None:
                raise ValueError(
                    f"scene {scene.module_name!r} step {step.label!r} has base_target "
                    "but the scene declares no BASE_ACTUATOR_NAMES"
                )
            if step.lift_target is not None and scene.lift is None:
                raise ValueError(
                    f"scene {scene.module_name!r} step {step.label!r} has lift_target "
                    "but the scene declares no LIFT_ACTUATOR_NAME"
                )
            if scene.mobile_base is not None:
                _ensure_target_not_in_aux_ctrl(
                    scene=scene,
                    step=step,
                    forbidden_names=set(scene.mobile_base.actuator_names),
                    component_name="mobile base",
                )
            if scene.lift is not None:
                _ensure_target_not_in_aux_ctrl(
                    scene=scene,
                    step=step,
                    forbidden_names={scene.lift.actuator_name},
                    component_name="lift",
                )


def load_scene(name: SceneName | str) -> LoadedScene:
    """Import and parse a fully qualified scene module."""
    scene_name = SceneName(str(name))
    module = importlib.import_module(str(scene_name))
    build_spec = _required_callable(module, scene_name, "build_spec")
    apply_initial_state = _required_callable(module, scene_name, "apply_initial_state")
    make_task_plan = _optional_callable(module, "make_task_plan")
    step_free_play = _optional_callable(module, "step_free_play")
    supports_start_phase = "start_phase" in inspect.signature(apply_initial_state).parameters
    robot_kind = _parse_scene_robot_kind(getattr(module, "ROBOT_KIND", RobotKind.PIPER), scene_name)
    manipulators = _parse_manipulators(module, scene_name, robot_kind)
    aux_actuator_names = tuple(str(name) for name in _tuple_attr(module, "AUX_ACTUATOR_NAMES"))
    mobile_base = _parse_mobile_base(module, scene_name=scene_name)
    lift = _parse_lift(module, scene_name=scene_name)
    _validate_scene_owned_actuator_names(
        scene_name=scene_name,
        mobile_base=mobile_base,
        lift=lift,
        aux_actuator_names=aux_actuator_names,
    )
    return LoadedScene(
        module_name=scene_name,
        display_name=str(getattr(module, "NAME", scene_name)),
        module=module,
        build_spec=build_spec,
        apply_initial_state_callback=apply_initial_state,
        supports_start_phase=supports_start_phase,
        make_task_plan=make_task_plan,
        step_free_play=step_free_play,
        manipulators=manipulators,
        arm_sides=tuple(manipulator.side for manipulator in manipulators),
        n_cubes=int(getattr(module, "N_CUBES", 0)),
        robot_kind=robot_kind,
        mobile_base=mobile_base,
        lift=lift,
        grippable_names=tuple(str(name) for name in _tuple_attr(module, "GRIPPABLES")),
        aux_actuator_names=aux_actuator_names,
        attachment_constraints=tuple(_tuple_attr(module, "ATTACHMENTS")),
        allowed_static_overlaps=tuple(_tuple_attr(module, "ALLOWED_STATIC_OVERLAPS")),
        camera_invariants=tuple(_tuple_attr(module, "CAMERA_INVARIANTS")),
        phase_contracts=tuple(_tuple_attr(module, "PHASE_CONTRACTS")),
        cameras=tuple(_tuple_attr(module, "CAMERAS")),
        ik_locked_joint_names=tuple(
            str(name) for name in _tuple_attr(module, "IK_LOCKED_JOINT_NAMES")
        ),
        ik_seed_q=(
            np.asarray(module.IK_SEED_Q, dtype=float) if hasattr(module, "IK_SEED_Q") else None
        ),
    )


def parse_world_point(raw: str, *, field_name: str) -> WorldPoint:
    """Parse `'x,y,z'` into a 3-tuple; raise on anything else.
    Parse-don't-validate at the CLI boundary."""
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(
            f"--{field_name} must be 'x,y,z' (three comma-separated numbers); got {raw!r}"
        )
    try:
        vals = [float(p) for p in parts]
    except ValueError as err:
        raise ValueError(f"--{field_name}: every component must be numeric; got {raw!r}") from err
    return (vals[0], vals[1], vals[2])


@dataclass(frozen=True)
class FreeCameraPose:
    """Orbit-camera pose in the convention the viser viewer uses."""

    azimuth_deg: AzimuthDeg
    elevation_deg: ElevationDeg
    distance_m: Metres
    lookat: WorldPoint


def build_free_cam(pose: FreeCameraPose) -> mujoco.MjvCamera:
    """Materialise a `FreeCameraPose` into MuJoCo's `MjvCamera` type."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = float(pose.azimuth_deg)
    cam.elevation = float(pose.elevation_deg)
    cam.distance = float(pose.distance_m)
    cam.lookat[:] = pose.lookat
    return cam


CameraSpec = mujoco.MjvCamera | str | None
"""Union the tools pass to `render_frame`:
 - MjvCamera: a free-cam pose
 - str: a named scene camera (e.g. 'top_d435i_cam')
 - None: MuJoCo's default free camera
"""


def render_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    camera: CameraSpec = None,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """Render a single (H, W, 3) uint8 frame.

    No explicit `renderer.close()`: MuJoCo 3.7's `Renderer.__del__` already
    calls close() at GC, and a manual close() + __del__'s call raises
    `_mjr_context` AttributeError on the second pass.
    """
    renderer = mujoco.Renderer(model, height=height, width=width)
    if isinstance(camera, str):
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cam_id < 0:
            available = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i) or f"cam{i}"
                for i in range(model.ncam)
            ]
            raise ValueError(f"unknown camera {camera!r}; available: {available}")
        renderer.update_scene(data, camera=cam_id)
    elif isinstance(camera, mujoco.MjvCamera):
        renderer.update_scene(data, camera=camera)
    else:
        renderer.update_scene(data)
    return renderer.render()


@dataclass
class _ArmTimelineState:
    step: int
    t: float
    start_q: np.ndarray
    start_g: float
    start_base: dict[str, float]
    start_lift: float | None
    start_aux: dict[str, float]


TimelineState = dict[ArmSide, _ArmTimelineState]


def make_timeline_state(data: mujoco.MjData, arms: dict[ArmSide, ArmHandles]) -> TimelineState:
    """Capture per-arm interpolation state before replay. Callers advancing
    in chunks must reuse this object; otherwise every chunk restarts at step 0."""
    return {
        side: _ArmTimelineState(
            step=0,
            t=0.0,
            start_q=np.array([data.qpos[i] for i in arms[side].arm_qpos_idx]),
            start_g=float(data.ctrl[arms[side].act_gripper_id]),
            start_base={},
            start_lift=None,
            start_aux={},
        )
        for side in arms
    }


def _resolve_position_actuator_qpos_dof(
    model: mujoco.MjModel,
    actuator_id: int,
) -> tuple[int, int]:
    joint_id = int(model.actuator_trnid[actuator_id][0])
    return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])


def _mobile_base_target_values(target: MobileBaseTarget) -> tuple[float, float, float]:
    return target.as_tuple()


def resolve_timeline_actuator_maps(
    model: mujoco.MjModel,
    scene: LoadedScene,
) -> TimelineActuatorMaps:
    aux_name_to_id = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in scene.aux_actuator_names
    }
    base_name_to_id = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in (scene.mobile_base.actuator_names if scene.mobile_base is not None else ())
    }
    base_qposadr: dict[str, int] = {}
    base_dofadr: dict[str, int] = {}
    for name, actuator_id in base_name_to_id.items():
        base_qposadr[name], base_dofadr[name] = _resolve_position_actuator_qpos_dof(
            model, actuator_id
        )

    lift_name_to_id = (
        {
            scene.lift.actuator_name: mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                scene.lift.actuator_name,
            )
        }
        if scene.lift is not None
        else {}
    )
    lift_qposadr: dict[str, int] = {}
    lift_dofadr: dict[str, int] = {}
    for name, actuator_id in lift_name_to_id.items():
        lift_qposadr[name], lift_dofadr[name] = _resolve_position_actuator_qpos_dof(
            model, actuator_id
        )

    return TimelineActuatorMaps(
        base_name_to_id=base_name_to_id,
        base_qposadr=base_qposadr,
        base_dofadr=base_dofadr,
        lift_name_to_id=lift_name_to_id,
        lift_qposadr=lift_qposadr,
        lift_dofadr=lift_dofadr,
        aux_name_to_id=aux_name_to_id,
    )


def _advance_one_arm(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmHandles,
    script: list[Step],
    st: _ArmTimelineState,
    sim_dt: float,
    base_name_to_id: dict[str, int],
    base_qposadr: dict[str, int],
    base_dofadr: dict[str, int],
    lift_name_to_id: dict[str, int],
    lift_qposadr: dict[str, int],
    lift_dofadr: dict[str, int],
    aux_name_to_id: dict[str, int],
    cube_body_ids: list[int],
) -> None:
    """Headless mirror of `runner.advance_arm`."""
    if st.step >= len(script):
        return
    step = script[st.step]
    first_tick = st.t == 0.0
    st.t += sim_dt

    if first_tick:
        if step.weld_activate is not None:
            activate_grasp_weld(
                model,
                data,
                int(arm.weld_ids[step.weld_activate]),
                arm.link6_id,
                cube_body_ids[step.weld_activate],
                arm.tcp_site_id,
            )
        if step.weld_deactivate is not None:
            deactivate_grasp_weld(data, int(arm.weld_ids[step.weld_deactivate]))
        for weld_name in step.attach_activate:
            eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
            if int(model.eq_type[eq_id]) == int(mujoco.mjtEq.mjEQ_CONNECT):
                data.eq_active[eq_id] = 1
            else:
                activate_attachment_weld(
                    model,
                    data,
                    eq_id,
                    int(model.eq_obj1id[eq_id]),
                    int(model.eq_obj2id[eq_id]),
                )
        for weld_name, target_xyz, target_quat in step.attach_activate_at or ():
            eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
            activate_attachment_weld(
                model,
                data,
                eq_id,
                int(model.eq_obj1id[eq_id]),
                int(model.eq_obj2id[eq_id]),
                target_world_pose=(target_xyz, target_quat),
            )
        for weld_name in step.attach_deactivate:
            eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
            data.eq_active[eq_id] = 0
        # Headless mirror of mujoco_workbench.runner's `set_geom_rgba` block. The live
        # runner also pushes the new colour to viser via update_geom_rgba;
        # for the offline renderer that's not relevant — `model.geom_rgba`
        # is the only thing MuJoCo's `Renderer.render()` reads, and writing
        # it here is enough for the indicator flip to land in the rendered
        # video. Without this the headless render kept showing the alert
        # red even after the WAIT-phase flip step fired.
        for geom_name, rgba in step.set_geom_rgba:
            geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name))
            if geom_id < 0:
                raise ValueError(
                    f"step {step.label!r} references unknown geom {geom_name!r} in set_geom_rgba"
                )
            model.geom_rgba[geom_id] = rgba

    alpha = min(1.0, st.t / max(step.duration, 1e-3))
    alpha_s = 0.5 - 0.5 * math.cos(math.pi * alpha)

    # Puppet mode: write qpos directly (mirrors runner.advance_arm).
    curr_q = (1.0 - alpha_s) * st.start_q + alpha_s * step.arm_q
    data.qpos[arm.arm_qpos_idx] = curr_q
    data.qvel[arm.arm_dof_idx] = 0.0
    data.ctrl[arm.act_arm_ids] = curr_q

    target_gripper = arm.gripper_open if step.gripper == "open" else arm.gripper_closed
    curr_g = (1.0 - alpha_s) * st.start_g + alpha_s * target_gripper
    if (
        arm.piper_mirrored_gripper_qpos_idx is not None
        and arm.piper_mirrored_gripper_dof_idx is not None
    ):
        left_gripper_qpos_idx, right_gripper_qpos_idx = arm.piper_mirrored_gripper_qpos_idx
        left_gripper_dof_idx, right_gripper_dof_idx = arm.piper_mirrored_gripper_dof_idx
        data.qpos[left_gripper_qpos_idx] = curr_g
        data.qpos[right_gripper_qpos_idx] = -curr_g
        data.qvel[left_gripper_dof_idx] = 0.0
        data.qvel[right_gripper_dof_idx] = 0.0
    # UR10e + 2F-85: actuator drives the tendon equality; finger joints settle.
    data.ctrl[arm.act_gripper_id] = curr_g

    if step.base_target is not None:
        if not base_name_to_id:
            raise ValueError(
                f"step {step.label!r} has base_target but no base actuators are resolved"
            )
        for base_name, base_target in zip(
            base_name_to_id,
            _mobile_base_target_values(step.base_target),
            strict=True,
        ):
            actuator_id = base_name_to_id[base_name]
            start = st.start_base.get(base_name, float(data.qpos[base_qposadr[base_name]]))
            current_base_value = (1.0 - alpha_s) * start + alpha_s * base_target
            data.qpos[base_qposadr[base_name]] = current_base_value
            data.qvel[base_dofadr[base_name]] = 0.0
            data.ctrl[actuator_id] = current_base_value

    if step.lift_target is not None:
        if not lift_name_to_id:
            raise ValueError(
                f"step {step.label!r} has lift_target but no lift actuator is resolved"
            )
        for lift_name, actuator_id in lift_name_to_id.items():
            start = st.start_lift
            if start is None:
                start = float(data.qpos[lift_qposadr[lift_name]])
            current_lift_value = (1.0 - alpha_s) * start + alpha_s * step.lift_target.position
            data.qpos[lift_qposadr[lift_name]] = current_lift_value
            data.qvel[lift_dofadr[lift_name]] = 0.0
            data.ctrl[actuator_id] = current_lift_value

    if step.aux_ctrl:
        for aux_name, aux_target in step.aux_ctrl.items():
            aux_key = str(aux_name)
            aid = aux_name_to_id[aux_key]
            jnt_id = int(model.actuator_trnid[aid][0])
            qadr = int(model.jnt_qposadr[jnt_id])
            dadr = int(model.jnt_dofadr[jnt_id])
            start = st.start_aux.get(aux_key, float(data.qpos[qadr]))
            curr_aux = (1.0 - alpha_s) * start + alpha_s * aux_target
            data.qpos[qadr] = curr_aux
            data.qvel[dadr] = 0.0
            data.ctrl[aid] = curr_aux

    if alpha >= 1.0:
        st.start_q = step.arm_q.copy()
        st.start_g = target_gripper
        if step.base_target is not None:
            for base_name, base_target in zip(
                base_name_to_id,
                _mobile_base_target_values(step.base_target),
                strict=True,
            ):
                st.start_base[base_name] = base_target
        if step.lift_target is not None:
            st.start_lift = step.lift_target.position
        if step.aux_ctrl:
            for aux_name, aux_target in step.aux_ctrl.items():
                st.start_aux[str(aux_name)] = aux_target
        st.step += 1
        st.t = 0.0


def advance_timeline(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    task_plan: dict[ArmSide, list[Step]],
    base_name_to_id: dict[str, int],
    base_qposadr: dict[str, int],
    base_dofadr: dict[str, int],
    lift_name_to_id: dict[str, int],
    lift_qposadr: dict[str, int],
    lift_dofadr: dict[str, int],
    aux_name_to_id: dict[str, int],
    cube_body_ids: list[int],
    sim_dt: float,
    until_s: Seconds,
) -> TimelineState:
    """Replay the per-arm step loop to `until_s` (one `mj_step` per `sim_dt`).
    Mutates `data` in place."""
    state = make_timeline_state(data, arms)
    return advance_timeline_with_state(
        model,
        data,
        arms,
        task_plan,
        state,
        base_name_to_id,
        base_qposadr,
        base_dofadr,
        lift_name_to_id,
        lift_qposadr,
        lift_dofadr,
        aux_name_to_id,
        cube_body_ids,
        sim_dt,
        until_s,
    )


def advance_timeline_with_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    task_plan: dict[ArmSide, list[Step]],
    state: TimelineState,
    base_name_to_id: dict[str, int],
    base_qposadr: dict[str, int],
    base_dofadr: dict[str, int],
    lift_name_to_id: dict[str, int],
    lift_qposadr: dict[str, int],
    lift_dofadr: dict[str, int],
    aux_name_to_id: dict[str, int],
    cube_body_ids: list[int],
    sim_dt: float,
    until_s: Seconds,
) -> TimelineState:
    """Advance a task plan while preserving caller-owned timeline state."""
    n_steps = int(float(until_s) / sim_dt)
    for _ in range(n_steps):
        for side, arm in arms.items():
            _advance_one_arm(
                model,
                data,
                arm,
                task_plan[side],
                state[side],
                sim_dt,
                base_name_to_id,
                base_qposadr,
                base_dofadr,
                lift_name_to_id,
                lift_qposadr,
                lift_dofadr,
                aux_name_to_id,
                cube_body_ids,
            )
        mujoco.mj_step(model, data)
    return state


@dataclass
class SceneContext:
    """`(model, data, arms, task_plan)` quadruple returned by
    `build_scene_and_advance`."""

    model: mujoco.MjModel
    data: mujoco.MjData
    arms: dict[ArmSide, ArmHandles]
    cube_body_ids: list[int]
    task_plan: dict[ArmSide, list[Step]] | None
    scene: LoadedScene


def build_scene_and_advance(scene_name: SceneName | str, t: Seconds | float = 0.0) -> SceneContext:
    """Load + compile the scene, apply initial state, advance task plan to `t`.
    `t=0` skips task-plan construction (avoids the IK cost for tools that
    only need the home pose)."""
    scene = load_scene(scene_name)
    model, data = scene.build_spec()

    cube_body_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in scene.grippable_names
    ]
    arms: dict[ArmSide, ArmHandles] = {
        manipulator.side: get_arm_handles(model, manipulator, scene.n_cubes)
        for manipulator in scene.manipulators
    }
    timeline_actuators = resolve_timeline_actuator_maps(model, scene)
    scene.apply_initial_state(model, data, arms, cube_body_ids)

    task_plan: dict[ArmSide, list[Step]] | None = None
    if scene.make_task_plan is not None:
        # Re-apply initial state after `snap` so t=0 renders show home pose,
        # not whatever IK seeding state the planner left behind.
        task_plan = scene.make_task_plan(model, data, arms, cube_body_ids)
        validate_task_plan_targets(scene, arms, task_plan)
        scene.apply_initial_state(model, data, arms, cube_body_ids)
    if task_plan is not None and float(t) > 0:
        sim_dt = float(model.opt.timestep)
        advance_timeline(
            model,
            data,
            arms,
            task_plan,
            timeline_actuators.base_name_to_id,
            timeline_actuators.base_qposadr,
            timeline_actuators.base_dofadr,
            timeline_actuators.lift_name_to_id,
            timeline_actuators.lift_qposadr,
            timeline_actuators.lift_dofadr,
            timeline_actuators.aux_name_to_id,
            cube_body_ids,
            sim_dt,
            Seconds(float(t)),
        )
    elif float(t) > 0:
        raise RuntimeError(
            f"scene {scene.module_name!r} has no make_task_plan, can't advance past t=0"
        )

    mujoco.mj_forward(model, data)
    return SceneContext(
        model=model,
        data=data,
        arms=arms,
        cube_body_ids=cube_body_ids,
        task_plan=task_plan,
        scene=scene,
    )


# Single boundary for filename-suffix → format. Downstream video code takes
# `VideoFormat` and doesn't re-inspect the path.
VideoFormat = Literal["mp4", "gif"]


def parse_video_format(out_path: Path) -> VideoFormat:
    suffix = out_path.suffix.lower().lstrip(".")
    if suffix == "mp4":
        return "mp4"
    if suffix == "gif":
        return "gif"
    raise ValueError(f"unsupported video suffix {out_path.suffix!r}; use .mp4 or .gif")
