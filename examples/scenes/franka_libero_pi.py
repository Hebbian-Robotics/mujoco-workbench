"""Franka Panda tabletop scene shaped for `pi05_libero` policy inference.

This is a native MuJoCo-workbench scene that mirrors the OpenPI LIBERO policy
interface as closely as the local simulator stack allows:

* Menagerie stock Franka Panda with Panda hand, matching LIBERO/robosuite's
  `robots="Panda"` default gripper more closely than the DROID Robotiq setup.
* Camera names and preprocessing follow OpenPI's LIBERO eval path:
  `agentview` and `robot0_eye_in_hand`, rendered at 256x256, rotated 180
  degrees, then resized/padded to 224x224.
* Hosted policy actions are 7-D robosuite-style `OSC_POSE` commands. The
  scene maps them through a scratch-data IK solve into this model's
  joint-position actuators.

This is still an approximation of LIBERO: the exact LIBERO task arenas,
object assets, robosuite Panda XML, and OSC torque controller live in
LIBERO/robosuite. For maximum fidelity, run OpenPI against LIBERO's
`OffScreenRenderEnv`; for workbench-native policy demos, this scene keeps the
right observation/action contract while staying in the existing runner.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from dm_control import mjcf

from examples.robots.franka_panda import load_franka_panda
from mujoco_workbench.arm_handles import (
    ArmHandles,
    ArmSide,
    ManipulatorSpec,
    RobotKind,
    get_arm_handles,
)
from mujoco_workbench.cameras import CameraRole
from mujoco_workbench.embodiments.libero import (
    LIBERO_ACTION_WIDTH,
    LIBERO_AGENTVIEW_CAMERA_NAME,
    LIBERO_CONTROL_PERIOD_SECONDS,
    LIBERO_DEFAULT_OPEN_LOOP_HORIZON,
    LIBERO_POLICY_CAMERA_RENDER_HEIGHT,
    LIBERO_POLICY_CAMERA_RENDER_WIDTH,
    LIBERO_WRIST_CAMERA_NAME,
    LiberoObservation,
    build_libero_observation,
    libero_action_to_actuator_ctrl,
    resize_libero_policy_image,
)
from mujoco_workbench.headless_renderer import NamedCameraRenderer
from mujoco_workbench.placement import camera_xyaxes_for_look_at
from mujoco_workbench.policy_client import HostedPolicyActionChunkClient
from mujoco_workbench.policy_types import PolicyEndpoint, PolicyPrompt


@dataclass(frozen=True)
class _CubeTargetSpec:
    body_name: str
    grippable_name: str
    x: float
    y: float
    half_size: float
    mass: float
    rgba: tuple[float, float, float, float]


_RED_CUBE_TARGET = _CubeTargetSpec(
    body_name="red_cube",
    grippable_name="red_cube",
    x=0.18,
    y=0.0,
    half_size=0.025,
    mass=0.06,
    rgba=(0.85, 0.05, 0.03, 1.0),
)

# ---------------------------------------------------------------------------
# Scene-module public attributes (introspected by `runtime.load_scene`).
# ---------------------------------------------------------------------------
NAME = "Franka LIBERO (pi05)"
ROBOT_KIND = RobotKind.FRANKA_PANDA
MANIPULATORS: tuple[ManipulatorSpec, ...] = (
    ManipulatorSpec(side=ArmSide.LEFT, robot_kind=RobotKind.FRANKA_PANDA),
)
N_CUBES = 1
GRIPPABLES: tuple[str, ...] = (_RED_CUBE_TARGET.grippable_name,)
CAMERAS: tuple[tuple[str, CameraRole], ...] = (
    (str(LIBERO_AGENTVIEW_CAMERA_NAME), CameraRole.TOP),
    (str(LIBERO_WRIST_CAMERA_NAME), CameraRole.WRIST),
)
CAMERA_FEED_RENDER_HEIGHT = LIBERO_POLICY_CAMERA_RENDER_HEIGHT
CAMERA_FEED_RENDER_WIDTH = LIBERO_POLICY_CAMERA_RENDER_WIDTH

ALLOWED_STATIC_OVERLAPS: tuple[tuple[str, str], ...] = (
    ("table_top", "left//unnamed_geom_7"),
    ("table_top", "left//unnamed_geom_11"),
)

# ---------------------------------------------------------------------------
# Layout constants.
# ---------------------------------------------------------------------------
_TABLE_HALF_X: float = 0.55
_TABLE_HALF_Y: float = 0.35
_TABLE_TOP_Z: float = 0.40

_ARM_MOUNT_X: float = -_TABLE_HALF_X + 0.14
_ARM_MOUNT_Y: float = 0.0
_ARM_MOUNT_Z: float = _TABLE_TOP_Z

_FRANKA_HOME_Q: tuple[float, ...] = (0.0, -0.8, 0.0, -2.0, 0.0, 1.5, 0.8)
_PANDA_OPEN_FINGER_QPOS: float = 0.04

_WRIST_CAMERA_POSITION_IN_HAND_FRAME: tuple[float, float, float] = (0.045, 0.0, 0.035)
_WRIST_CAMERA_TARGET_IN_HAND_FRAME: tuple[float, float, float] = (0.0, 0.0, 0.16)
_WRIST_CAMERA_UP_HINT_IN_HAND_FRAME: tuple[float, float, float] = (1.0, 0.0, 0.0)

_AGENTVIEW_CAMERA_EYE: tuple[float, float, float] = (0.42, -0.92, _TABLE_TOP_Z + 0.68)
_AGENTVIEW_CAMERA_TARGET: tuple[float, float, float] = (0.16, -0.02, _TABLE_TOP_Z + 0.07)


# ---------------------------------------------------------------------------
# Scene construction.
# ---------------------------------------------------------------------------
def _add_cube_target(parent: mjcf.Element, target: _CubeTargetSpec) -> None:
    target_body = parent.add(
        "body",
        name=target.body_name,
        pos=[target.x, target.y, _TABLE_TOP_Z + target.half_size + 0.001],
    )
    target_body.add("freejoint", name=f"{target.body_name}_freejoint")
    target_body.add(
        "geom",
        name=f"{target.body_name}_geom",
        type="box",
        size=[target.half_size, target.half_size, target.half_size],
        rgba=list(target.rgba),
        mass=target.mass,
        friction=[1.2, 0.01, 0.0001],
        contype=1,
        conaffinity=1,
    )


def _build_root() -> mjcf.RootElement:
    """Compose the scene MJCF: world, table, Panda, target, cameras, welds."""
    root = mjcf.RootElement(model="franka_libero_pi")

    root.option.timestep = 0.002
    root.option.gravity = [0.0, 0.0, -9.81]
    root.option.integrator = "implicitfast"
    root.visual.headlight.diffuse = [0.34, 0.34, 0.34]
    root.visual.headlight.ambient = [0.40, 0.40, 0.40]
    root.visual.headlight.specular = [0.0, 0.0, 0.0]
    root.visual.rgba.haze = [0.72, 0.76, 0.80, 1.0]
    root.visual.__getattr__("global").azimuth = 145
    root.visual.__getattr__("global").elevation = -22

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
        name="floor_grid",
        type="2d",
        builtin="checker",
        rgb1=[0.50, 0.51, 0.50],
        rgb2=[0.66, 0.67, 0.65],
        width=300,
        height=300,
        mark="edge",
        markrgb=[0.43, 0.43, 0.42],
    )
    root.asset.add(
        "material",
        name="floor_grid",
        texture="floor_grid",
        texrepeat=[5, 5],
        reflectance=0.0,
    )
    root.asset.add(
        "material",
        name="robosuite_gray_table",
        rgba=[0.43, 0.45, 0.44, 1.0],
        specular=0.02,
        shininess=0.08,
        reflectance=0.0,
    )

    root.worldbody.add(
        "geom",
        name="floor",
        type="plane",
        size=[5.0, 5.0, 0.05],
        material="floor_grid",
        contype=1,
        conaffinity=1,
    )
    root.worldbody.add(
        "light",
        name="overhead_key_light",
        pos=[0.2, -0.1, 2.4],
        dir=[0.0, 0.0, -1.0],
        directional="true",
        diffuse=[0.36, 0.36, 0.36],
        specular=[0.04, 0.04, 0.04],
    )
    root.worldbody.add(
        "light",
        name="agentview_fill_light",
        pos=[-0.3, -0.8, 1.2],
        dir=[0.3, 0.7, -0.8],
        diffuse=[0.20, 0.20, 0.20],
        specular=[0.0, 0.0, 0.0],
        castshadow="false",
    )

    table_body = root.worldbody.add("body", name="table", pos=[0.0, 0.0, _TABLE_TOP_Z * 0.5])
    table_body.add(
        "geom",
        name="table_top",
        type="box",
        size=[_TABLE_HALF_X, _TABLE_HALF_Y, _TABLE_TOP_Z * 0.5],
        material="robosuite_gray_table",
        contype=1,
        conaffinity=1,
    )

    mount_body = root.worldbody.add(
        "body",
        name="arm_mount",
        pos=[_ARM_MOUNT_X, _ARM_MOUNT_Y, _ARM_MOUNT_Z],
    )
    mount_site = mount_body.add("site", name="arm_mount_site", size=[0.005, 0.005, 0.005])

    franka_root = load_franka_panda(ArmSide.LEFT)
    hand_body = franka_root.find("body", "hand")
    if hand_body is None:
        raise RuntimeError("Franka loader returned a Panda without a hand body.")
    hand_body.add(
        "camera",
        name="robot0_eye_in_hand",
        pos=list(_WRIST_CAMERA_POSITION_IN_HAND_FRAME),
        xyaxes=list(
            camera_xyaxes_for_look_at(
                np.asarray(_WRIST_CAMERA_POSITION_IN_HAND_FRAME, dtype=float),
                np.asarray(_WRIST_CAMERA_TARGET_IN_HAND_FRAME, dtype=float),
                up_hint=np.asarray(_WRIST_CAMERA_UP_HINT_IN_HAND_FRAME, dtype=float),
            )
        ),
        mode="fixed",
        fovy=45.0,
    )
    mount_site.attach(franka_root)

    root.worldbody.add(
        "camera",
        name="agentview",
        pos=list(_AGENTVIEW_CAMERA_EYE),
        xyaxes=list(
            camera_xyaxes_for_look_at(
                np.asarray(_AGENTVIEW_CAMERA_EYE, dtype=float),
                np.asarray(_AGENTVIEW_CAMERA_TARGET, dtype=float),
            )
        ),
        fovy=45.0,
    )

    _add_cube_target(root.worldbody, _RED_CUBE_TARGET)

    for i, grippable_name in enumerate(GRIPPABLES):
        root.equality.add(
            "weld",
            name=f"left_grasp_cube{i}",
            body1="left/hand",
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
    """Set Panda to its LIBERO-like home pose, gripper open, cube on table."""
    del grippable_body_ids  # target freejoint starts at its MJCF body pose

    arm = arms[ArmSide.LEFT]
    home_q = np.asarray(_FRANKA_HOME_Q, dtype=float)
    data.qpos[arm.arm_qpos_idx] = home_q
    data.qvel[arm.arm_dof_idx] = 0.0
    data.ctrl[arm.act_arm_ids] = home_q
    data.ctrl[arm.act_gripper_id] = arm.gripper_open
    _set_panda_fingers_open(model, data, arm)

    mujoco.mj_forward(model, data)


def _set_panda_fingers_open(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmHandles) -> None:
    for joint_suffix in ("finger_joint1", "finger_joint2"):
        joint_name = f"{arm.side}{joint_suffix}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"Panda finger joint {joint_name!r} not found.")
        data.qpos[model.jnt_qposadr[joint_id]] = _PANDA_OPEN_FINGER_QPOS
        data.qvel[model.jnt_dofadr[joint_id]] = 0.0


# ---------------------------------------------------------------------------
# Free-play hook and hosted policy controller.
# ---------------------------------------------------------------------------
def step_free_play(t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """No-op fallback for plain `mwb run` without a hosted policy."""
    del t, model, data


def preprocess_camera_feed(_camera_name: str, rendered_image: np.ndarray) -> np.ndarray:
    """Show the same rotated/resized LIBERO image that policy mode sends."""
    return resize_libero_policy_image(rendered_image)


class _FrankaLiberoPolicyFreePlay:
    """Callable free-play controller backed by hosted LIBERO policy inference."""

    def __init__(self, *, policy_endpoint: PolicyEndpoint, prompt: PolicyPrompt) -> None:
        self._policy_endpoint = policy_endpoint
        self._prompt = prompt
        self._renderer: NamedCameraRenderer | None = None
        self._policy_client: HostedPolicyActionChunkClient | None = None
        self._scratch_data: mujoco.MjData | None = None
        self._arm: ArmHandles | None = None
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

    def close(self) -> None:
        if self._policy_client is not None:
            self._policy_client.close()
            self._policy_client = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._scratch_data = None
        self._arm = None

    def __call__(self, t: float, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._ensure_started(model)
        if not self._is_policy_tick_due(t):
            return
        observation = self._build_observation(model, data)
        raw_action = self._require_policy_client().next_action(observation)
        data.ctrl[:] = libero_action_to_actuator_ctrl(
            model,
            data,
            self._require_scratch_data(),
            self._require_arm(),
            raw_action,
        )
        self._next_policy_tick_time = t + LIBERO_CONTROL_PERIOD_SECONDS

    def _is_policy_tick_due(self, t: float) -> bool:
        return self._next_policy_tick_time is None or t >= self._next_policy_tick_time

    def _ensure_started(self, model: mujoco.MjModel) -> None:
        if self._arm is None:
            self._arm = get_arm_handles(model, MANIPULATORS[0], N_CUBES)
        if self._scratch_data is None:
            self._scratch_data = mujoco.MjData(model)
        if self._renderer is None:
            self._renderer = NamedCameraRenderer(
                model,
                width=LIBERO_POLICY_CAMERA_RENDER_WIDTH,
                height=LIBERO_POLICY_CAMERA_RENDER_HEIGHT,
            )
        if self._policy_client is None:
            self._policy_client = HostedPolicyActionChunkClient(
                endpoint=self._policy_endpoint,
                expected_action_width=LIBERO_ACTION_WIDTH,
                max_buffered_actions=LIBERO_DEFAULT_OPEN_LOOP_HORIZON,
            )

    def _build_observation(self, model: mujoco.MjModel, data: mujoco.MjData) -> LiberoObservation:
        renderer = self._require_renderer()
        return build_libero_observation(
            model,
            data,
            self._require_arm(),
            lambda camera_name: resize_libero_policy_image(renderer.render(data, str(camera_name))),
            self._prompt,
        )

    def _require_renderer(self) -> NamedCameraRenderer:
        if self._renderer is None:
            raise RuntimeError("policy renderer has not been started")
        return self._renderer

    def _require_policy_client(self) -> HostedPolicyActionChunkClient:
        if self._policy_client is None:
            raise RuntimeError("policy client has not been started")
        return self._policy_client

    def _require_scratch_data(self) -> mujoco.MjData:
        if self._scratch_data is None:
            raise RuntimeError("policy scratch data has not been started")
        return self._scratch_data

    def _require_arm(self) -> ArmHandles:
        if self._arm is None:
            raise RuntimeError("policy arm handles have not been resolved")
        return self._arm


def make_step_free_play(
    *,
    policy_endpoint: PolicyEndpoint,
    prompt: PolicyPrompt,
) -> _FrankaLiberoPolicyFreePlay:
    """Factory consumed by `mwb run --policy-host ...`."""
    return _FrankaLiberoPolicyFreePlay(
        policy_endpoint=policy_endpoint,
        prompt=prompt,
    )
