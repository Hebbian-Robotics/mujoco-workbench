"""Franka Panda tabletop scene shaped for `pi05_droid` policy inference.

Purpose:
* Single-arm Franka mounted at the rear edge of a tabletop with a DROID-like
  red can target in front of it.
* Two cameras with placement matching the DROID observation schema the
  policy expects: a fixed external view (`cam_exterior`) and a wrist-mounted
  view on the Robotiq gripper (`cam_wrist`). The DROID observation builder
  reads them by their compiled MJCF names, so they must not be renamed without
  updating `mujoco_workbench.embodiments.droid`.
* Currently exposes a no-op `step_free_play`. Once the policy client is
  wired in, this module additionally exports `make_step_free_play`,
  which the runner detects and uses when `--policy-host` is set. Without
  that flag, the no-op free-play hook keeps the scene bootable for layout
  verification.

Conventions inherited from the workbench:
* Single arm uses `ArmSide.LEFT` (the "left" namespace) — same convention
  as bilateral scenes' left arm. Keeps grasp-weld naming uniform
  (`left_grasp_cubeN`) even when the grippable is not literally a cube.
* Pre-declared mjEQ_WELD equalities for each (left arm, grippable) pair
  are named `left_grasp_cubeN` for compatibility with the shared grasp
  adapter. Inactive at compile time; the
  state-based grasp detector (Phase 8 of the integration plan) will
  flip them on when the gripper closes around the target.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from dm_control import mjcf

from examples.robots.franka_panda import (
    ROBOTIQ_GRIPPER_BASE_BODY_NAME,
    load_franka_panda_with_robotiq_2f85,
)
from mujoco_workbench.arm_handles import (
    ArmHandles,
    ArmSide,
    ManipulatorSpec,
    franka_panda_robotiq_manipulator_spec,
)
from mujoco_workbench.cameras import CameraRole
from mujoco_workbench.embodiments.droid import (
    DROID_CONTROL_PERIOD_SECONDS,
    DROID_DATASET_EXTERIOR_CAMERA_FOVY_DEG,
    DROID_DATASET_WRIST_CAMERA_FOVY_DEG,
    DROID_DEFAULT_OPEN_LOOP_HORIZON,
    DROID_POLICY_CAMERA_RENDER_HEIGHT,
    DROID_POLICY_CAMERA_RENDER_WIDTH,
    DroidObservation,
    build_droid_observation,
    resize_droid_policy_image,
)
from mujoco_workbench.headless_renderer import NamedCameraRenderer
from mujoco_workbench.placement import camera_xyaxes_for_look_at
from mujoco_workbench.policy_client import HostedPolicyClient
from mujoco_workbench.policy_types import (
    PolicyActionInterpretation,
    PolicyEndpoint,
    PolicyPrompt,
)


@dataclass(frozen=True)
class _CanTargetSpec:
    body_name: str
    grippable_name: str
    x: float
    y: float
    radius: float
    height: float
    mass: float
    shell_rgba: tuple[float, float, float, float]
    label_rgba: tuple[float, float, float, float]


_RED_CAN_TARGET = _CanTargetSpec(
    body_name="red_can",
    grippable_name="red_can",
    x=0.22,
    y=0.0,
    radius=0.035,
    height=0.13,
    mass=0.10,
    shell_rgba=(0.95, 0.05, 0.04, 1.0),
    label_rgba=(0.96, 0.90, 0.70, 1.0),
)

# ---------------------------------------------------------------------------
# Scene-module public attributes (introspected by `runtime.load_scene`).
# ---------------------------------------------------------------------------
NAME = "Franka DROID (pi05)"
MANIPULATORS: tuple[ManipulatorSpec, ...] = (franka_panda_robotiq_manipulator_spec(ArmSide.LEFT),)
# Legacy name used by shared ArmHandles/grasp-weld indexing; this is the
# number of grippable targets, not necessarily literal cubes.
N_CUBES = 1
GRIPPABLES: tuple[str, ...] = (_RED_CAN_TARGET.grippable_name,)
CAMERAS: tuple[tuple[str, CameraRole], ...] = (
    ("cam_exterior", CameraRole.TOP),
    ("left/gripper/cam_wrist", CameraRole.WRIST),
)
CAMERA_FEED_RENDER_HEIGHT = DROID_POLICY_CAMERA_RENDER_HEIGHT
CAMERA_FEED_RENDER_WIDTH = DROID_POLICY_CAMERA_RENDER_WIDTH

# Franka base flange sits flush on the table top; the upstream link0
# collision geoms (mujoco_menagerie names them anonymously) inevitably
# overlap the table surface. Declare the pair so scene_check doesn't
# flag the legitimate "robot mounted on table" placement as a bug.
ALLOWED_STATIC_OVERLAPS: tuple[tuple[str, str], ...] = (
    ("table_top", "left//unnamed_geom_7"),
    ("table_top", "left//unnamed_geom_11"),
    ("table_top", "camera_stand_post"),
)

# ---------------------------------------------------------------------------
# Layout constants.
# ---------------------------------------------------------------------------
_TABLE_HALF_X: float = 0.65
_TABLE_HALF_Y: float = 0.30
_TABLE_TOP_Z: float = 0.40

# Franka mount point: rear edge of the table, centred laterally. Base
# sits on the table top so its z=0 lands at `_TABLE_TOP_Z`.
_ARM_MOUNT_X: float = -_TABLE_HALF_X + 0.15
_ARM_MOUNT_Y: float = 0.0
_ARM_MOUNT_Z: float = _TABLE_TOP_Z

# Franka home pose — the canonical "neutral" configuration used in
# robosuite + dm_control franka demos. Elbow up, slight downward
# pitch on joint5, gripper pointing along +x above the table.
_FRANKA_HOME_Q: tuple[float, ...] = (0.0, -0.8, 0.0, -2.0, 0.0, 1.5, 2.0)

_WRIST_CAMERA_POSITION_IN_GRIPPER_BASE_FRAME: tuple[float, float, float] = (0.05, 0.0, 0.025)
_WRIST_CAMERA_TARGET_IN_GRIPPER_BASE_FRAME: tuple[float, float, float] = (0.0, 0.0, 0.18)
_WRIST_CAMERA_UP_HINT_IN_GRIPPER_BASE_FRAME: tuple[float, float, float] = (1.0, 0.0, 0.0)

_EXTERIOR_CAMERA_STAND_XY: tuple[float, float] = (_ARM_MOUNT_X - 0.12, -_TABLE_HALF_Y - 0.12)
_EXTERIOR_CAMERA_HEIGHT_ABOVE_TABLE: float = 0.50
_EXTERIOR_CAMERA_TARGET: tuple[float, float, float] = (0.30, 0.02, _TABLE_TOP_Z + 0.035)


# ---------------------------------------------------------------------------
# Scene construction.
# ---------------------------------------------------------------------------
def _add_can_target(parent: mjcf.Element, target: _CanTargetSpec) -> None:
    """Add an upright can-like grippable with a single collision shell."""
    target_body = parent.add(
        "body",
        name=target.body_name,
        pos=[target.x, target.y, _TABLE_TOP_Z + target.height * 0.5 + 0.001],
    )
    target_body.add("freejoint", name=f"{target.body_name}_freejoint")
    target_body.add(
        "geom",
        name=f"{target.body_name}_shell",
        type="cylinder",
        size=[target.radius, target.height * 0.5],
        rgba=list(target.shell_rgba),
        mass=target.mass,
        friction=[1.8, 0.01, 0.0001],
        contype=1,
        conaffinity=1,
    )

    cap_half_height = 0.002
    cap_z_clearance = 0.0005
    cap_rgba = [0.82, 0.82, 0.78, 1.0]
    target_body.add(
        "geom",
        name=f"{target.body_name}_top_cap",
        type="cylinder",
        pos=[0.0, 0.0, target.height * 0.5 + cap_half_height + cap_z_clearance],
        size=[target.radius * 0.96, cap_half_height],
        rgba=cap_rgba,
        contype=0,
        conaffinity=0,
        mass=0.0,
    )
    target_body.add(
        "geom",
        name=f"{target.body_name}_front_label",
        type="box",
        pos=[0.0, -target.radius - 0.001, 0.004],
        size=[target.radius * 0.58, 0.001, target.height * 0.30],
        rgba=list(target.label_rgba),
        contype=0,
        conaffinity=0,
        mass=0.0,
    )


def _build_root() -> mjcf.RootElement:
    """Compose the scene MJCF: world, floor, table, arm, target, cameras, welds."""
    root = mjcf.RootElement(model="franka_droid_pi")

    # Visual / option defaults — DROID was collected under varied indoor lab
    # lighting, so use diffuse, low-glare illumination rather than a single
    # hard overhead light.
    root.option.timestep = 0.002
    root.option.gravity = [0.0, 0.0, -9.81]
    root.option.integrator = "implicitfast"
    root.visual.headlight.diffuse = [0.36, 0.36, 0.36]
    root.visual.headlight.ambient = [0.38, 0.38, 0.38]
    root.visual.headlight.specular = [0.0, 0.0, 0.0]
    root.visual.rgba.haze = [0.72, 0.76, 0.80, 1.0]
    root.visual.__getattr__("global").azimuth = 120
    root.visual.__getattr__("global").elevation = -20

    # Matte floor and table surfaces keep colored objects visible in both
    # DROID policy cameras without relying on camera auto-exposure.
    root.asset.add(
        "texture",
        name="studio_skybox",
        type="skybox",
        builtin="gradient",
        rgb1=[0.78, 0.80, 0.82],
        rgb2=[0.58, 0.60, 0.62],
        width=512,
        height=3072,
    )
    root.asset.add(
        "texture",
        name="grid",
        type="2d",
        builtin="checker",
        rgb1=[0.48, 0.50, 0.52],
        rgb2=[0.62, 0.64, 0.65],
        width=300,
        height=300,
        mark="edge",
        markrgb=[0.42, 0.43, 0.44],
    )
    root.asset.add(
        "material",
        name="grid",
        texture="grid",
        texrepeat=[5, 5],
        reflectance=0.0,
    )
    root.asset.add(
        "material",
        name="matte_charcoal_blue_table",
        rgba=[0.24, 0.29, 0.32, 1.0],
        specular=0.02,
        shininess=0.08,
        reflectance=0.0,
    )
    root.worldbody.add(
        "geom",
        name="floor",
        type="plane",
        size=[5.0, 5.0, 0.05],
        material="grid",
        contype=1,
        conaffinity=1,
    )
    root.worldbody.add(
        "light",
        name="overhead_key_light",
        pos=[0.2, -0.1, 2.4],
        dir=[0.0, 0.0, -1.0],
        directional="true",
        diffuse=[0.34, 0.34, 0.34],
        specular=[0.04, 0.04, 0.04],
    )
    root.worldbody.add(
        "light",
        name="camera_side_fill_light",
        pos=[-0.8, -0.7, 1.2],
        dir=[0.6, 0.5, -0.8],
        diffuse=[0.18, 0.18, 0.20],
        specular=[0.0, 0.0, 0.0],
        castshadow="false",
    )
    root.worldbody.add(
        "light",
        name="robot_side_fill_light",
        pos=[0.6, 0.7, 1.4],
        dir=[-0.5, -0.6, -0.8],
        diffuse=[0.13, 0.13, 0.15],
        specular=[0.0, 0.0, 0.0],
        castshadow="false",
    )

    # Table — a single static box geom rather than a sub-MJCF (no
    # articulation needed). Centred at world origin with top surface at
    # `_TABLE_TOP_Z`.
    table_body = root.worldbody.add("body", name="table", pos=[0.0, 0.0, _TABLE_TOP_Z * 0.5])
    table_body.add(
        "geom",
        name="table_top",
        type="box",
        size=[_TABLE_HALF_X, _TABLE_HALF_Y, _TABLE_TOP_Z * 0.5],
        material="matte_charcoal_blue_table",
        contype=1,
        conaffinity=1,
    )

    # Arm mount site — the Franka attaches here. Adding the site to a
    # named body (rather than directly to worldbody) keeps the scene
    # graph readable in viser.
    mount_body = root.worldbody.add(
        "body",
        name="arm_mount",
        pos=[_ARM_MOUNT_X, _ARM_MOUNT_Y, _ARM_MOUNT_Z],
    )
    mount_site = mount_body.add("site", name="arm_mount_site", size=[0.005, 0.005, 0.005])

    # Load Franka+Robotiq under the "left/" namespace and attach. Add an
    # off-axis wrist camera before attach so it inherits the compiled name
    # `left/gripper/cam_wrist`.
    franka_root = load_franka_panda_with_robotiq_2f85(ArmSide.LEFT)
    gripper_base = franka_root.find("body", ROBOTIQ_GRIPPER_BASE_BODY_NAME)
    if gripper_base is None:
        raise RuntimeError("Franka+Robotiq loader returned a robot without a gripper base body.")
    gripper_base.add(
        "camera",
        name="cam_wrist",
        pos=list(_WRIST_CAMERA_POSITION_IN_GRIPPER_BASE_FRAME),
        xyaxes=list(
            camera_xyaxes_for_look_at(
                np.asarray(_WRIST_CAMERA_POSITION_IN_GRIPPER_BASE_FRAME, dtype=float),
                np.asarray(_WRIST_CAMERA_TARGET_IN_GRIPPER_BASE_FRAME, dtype=float),
                up_hint=np.asarray(_WRIST_CAMERA_UP_HINT_IN_GRIPPER_BASE_FRAME, dtype=float),
            )
        ),
        mode="fixed",
        fovy=DROID_DATASET_WRIST_CAMERA_FOVY_DEG,
    )
    mount_site.attach(franka_root)

    # External DROID camera on a side stand. The eye stays outboard enough to
    # keep the arm base near the image edge while sitting close enough to show
    # gripper/object contact clearly.
    stand_x, stand_y = _EXTERIOR_CAMERA_STAND_XY
    camera_eye_z = _TABLE_TOP_Z + _EXTERIOR_CAMERA_HEIGHT_ABOVE_TABLE
    camera_stand_half_height = camera_eye_z * 0.5
    camera_eye = (
        stand_x,
        stand_y,
        camera_eye_z,
    )
    camera_target = _EXTERIOR_CAMERA_TARGET
    camera_stand = root.worldbody.add(
        "body",
        name="camera_stand",
        pos=[
            stand_x,
            stand_y,
            camera_stand_half_height,
        ],
    )
    camera_stand.add(
        "geom",
        name="camera_stand_post",
        type="cylinder",
        size=[0.01, camera_stand_half_height],
        rgba=[0.08, 0.08, 0.08, 1.0],
        contype=0,
        conaffinity=0,
    )
    camera_stand.add(
        "geom",
        name="camera_stand_head",
        type="box",
        pos=[0.0, 0.0, camera_stand_half_height + 0.01],
        size=[0.025, 0.018, 0.015],
        rgba=[0.02, 0.02, 0.02, 1.0],
        contype=0,
        conaffinity=0,
    )
    root.worldbody.add(
        "camera",
        name="cam_exterior",
        pos=list(camera_eye),
        xyaxes=list(camera_xyaxes_for_look_at(np.asarray(camera_eye), np.asarray(camera_target))),
        fovy=DROID_DATASET_EXTERIOR_CAMERA_FOVY_DEG,
    )

    _add_can_target(root.worldbody, _RED_CAN_TARGET)

    # Pre-declared grasp welds — one per (left arm, grippable) pair.
    # Inactive by default; the state-based grasp detector flips them
    # at runtime when the gripper closes around the target.
    for i, grippable_name in enumerate(GRIPPABLES):
        root.equality.add(
            "weld",
            name=f"left_grasp_cube{i}",
            body1="left/gripper/base",
            body2=grippable_name,
            active="false",
            relpose=[0, 0, 0, 1, 0, 0, 0],
        )

    return root


def build_spec() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Compile the scene MJCF and return (model, data)."""
    root = _build_root()
    model = mjcf.Physics.from_mjcf_model(root).model.ptr
    data = mujoco.MjData(model)
    return model, data


# ---------------------------------------------------------------------------
# Initial state.
# ---------------------------------------------------------------------------
def apply_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    grippable_body_ids: list[int],
) -> None:
    """Set Franka to its home pose, gripper open, target resting on the table.

    `grippable_body_ids` is unused — the target freejoint qpos is already set by
    the MJCF compile (the body `pos` we declared in `_build_root` becomes
    the freejoint's initial qpos), so no override needed at runtime.
    """
    del grippable_body_ids  # unused

    arm = arms[ArmSide.LEFT]
    home_q = np.asarray(_FRANKA_HOME_Q, dtype=float)
    data.qpos[arm.arm_qpos_idx] = home_q
    data.qvel[arm.arm_dof_idx] = 0.0
    # Position-actuator ctrl mirrors qpos so the controller doesn't fight
    # the initial pose with stale targets.
    data.ctrl[arm.act_arm_ids] = home_q

    # Gripper open. With our adapter convention `gripper_open=0.0`
    # (Robotiq-style default branch in get_arm_handles); if the
    # Franka convention turns out to be inverted during action
    # calibration, we'll swap this and the adapter values together.
    data.ctrl[arm.act_gripper_id] = arm.gripper_open

    mujoco.mj_forward(model, data)


# ---------------------------------------------------------------------------
# Free-play hook (placeholder until policy factory is wired in).
# ---------------------------------------------------------------------------
def step_free_play(t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """No-op fallback for plain `mwb run` without a hosted policy."""
    del t, model, data


def preprocess_camera_feed(_camera_name: str, rendered_image: np.ndarray) -> np.ndarray:
    """Show the same padded DROID image that policy mode sends to OpenPI."""
    return resize_droid_policy_image(rendered_image)


class _FrankaDroidPolicyFreePlay:
    """Callable free-play controller backed by hosted DROID policy inference."""

    def __init__(self, *, policy_endpoint: PolicyEndpoint, prompt: PolicyPrompt) -> None:
        self._policy_endpoint = policy_endpoint
        self._prompt = prompt
        self._renderer: NamedCameraRenderer | None = None
        self._policy_client: HostedPolicyClient | None = None
        self._next_policy_tick_time: float | None = None

    def prewarm(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._ensure_started(model)
        policy_client = self._require_policy_client()
        policy_client.prewarm(self._build_observation(model, data))

    def reset(self) -> None:
        self._next_policy_tick_time = None
        if self._policy_client is not None:
            self._policy_client.reset()

    def set_prompt(self, prompt: PolicyPrompt) -> None:
        """Update the language instruction and discard stale action chunks."""
        self._prompt = prompt
        self._next_policy_tick_time = None
        if self._policy_client is not None:
            self._policy_client.clear_action_buffer()

    def clear_action_buffer(self) -> None:
        """Discard queued open-loop policy actions without changing the prompt."""
        self._next_policy_tick_time = None
        if self._policy_client is not None:
            self._policy_client.clear_action_buffer()

    def close(self) -> None:
        if self._policy_client is not None:
            self._policy_client.close()
            self._policy_client = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __call__(self, t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._ensure_started(model)
        if not self._is_policy_tick_due(t):
            return
        policy_client = self._require_policy_client()
        actuator_ctrl = policy_client.next_actuator_ctrl(self._build_observation(model, data))
        data.ctrl[:] = actuator_ctrl
        self._next_policy_tick_time = t + DROID_CONTROL_PERIOD_SECONDS

    def _is_policy_tick_due(self, t: float) -> bool:
        return self._next_policy_tick_time is None or t >= self._next_policy_tick_time

    def _ensure_started(self, model: mujoco.MjModel) -> None:
        if self._renderer is None:
            self._renderer = NamedCameraRenderer(
                model,
                width=DROID_POLICY_CAMERA_RENDER_WIDTH,
                height=DROID_POLICY_CAMERA_RENDER_HEIGHT,
            )
        if self._policy_client is None:
            self._policy_client = HostedPolicyClient(
                endpoint=self._policy_endpoint,
                model=model,
                action_interpretation=PolicyActionInterpretation.DROID_JOINT_VELOCITY,
                max_buffered_actions=DROID_DEFAULT_OPEN_LOOP_HORIZON,
                droid_velocity_time_step_seconds=DROID_CONTROL_PERIOD_SECONDS,
            )

    def _build_observation(self, model: mujoco.MjModel, data: mujoco.MjData) -> DroidObservation:
        renderer = self._require_renderer()
        return build_droid_observation(
            model,
            data,
            lambda camera_name: resize_droid_policy_image(renderer.render(data, str(camera_name))),
            self._prompt,
        )

    def _require_renderer(self) -> NamedCameraRenderer:
        if self._renderer is None:
            raise RuntimeError("policy renderer has not been started")
        return self._renderer

    def _require_policy_client(self) -> HostedPolicyClient:
        if self._policy_client is None:
            raise RuntimeError("policy client has not been started")
        return self._policy_client


def make_step_free_play(
    *,
    policy_endpoint: PolicyEndpoint,
    prompt: PolicyPrompt,
) -> _FrankaDroidPolicyFreePlay:
    """Factory consumed by `mwb run --policy-host ...`."""
    return _FrankaDroidPolicyFreePlay(
        policy_endpoint=policy_endpoint,
        prompt=prompt,
    )
