# Architecture

`mujoco-workbench` is a MuJoCo robotics workspace for scripted, inspectable
simulation scenes. The reusable package is intentionally separate from the
included robot examples so other projects can bring their own scene modules.

## Codemap

`mujoco_workbench.cli` is the public command surface. The installed commands are
`mujoco-workbench` and `mwb`.

`mujoco_workbench.runner` is the interactive Viser runner. It imports a fully
qualified scene module, compiles the MuJoCo model, starts Viser, advances the
scripted timeline or free-play callback, updates Viser geometry, and optionally
streams Rerun events.

`mujoco_workbench.debug_cli` implements the `mwb debug ...` subcommands:
snapshots, camera grids, videos, timelines, phase checks, phase replays, image
diffs, IK sweeps, timeline clearance checks, and review packets.

`mujoco_workbench.runtime` is the non-interactive counterpart to the runner. It
loads a scene, applies initial state, advances the scripted timeline, and
renders frames without a browser. Agent-facing tools should use this module
instead of duplicating runner behavior.

`mujoco_workbench.scene_base` defines shared domain types: `Step`, `TaskPhase`,
`PhaseState`, `PhaseContract`, and geometry/value aliases. Most scene/tool
boundaries are expressed through these types.

`mujoco_workbench.observability` owns phase-contract checking and run artifacts.
It writes JSONL events, summaries, serialized contracts, state snapshots, and
before/after renders.

`mujoco_workbench.arm_handles`, `ik`, `welds`, `cameras`, and `viser_render` are
shared runtime infrastructure. They resolve MuJoCo ids, solve differential IK,
toggle equality-based grasp/attachment helpers, maintain camera widgets, and
mirror MuJoCo geometry into Viser.

`mujoco_workbench.teleop` contains browser TCP-drag authoring and replay support.
It is currently shaped by the Mobile ALOHA server-swap example, so treat it as
less generic than the core runtime.

`examples.scenes` contains the bundled robot scenes. Each scene owns its MJCF
assembly, initial state, task plan, optional phase contracts, and optional
layout dataclasses.

`examples.robots`, `examples.paths`, and `examples.assets` contain the bundled
robot loaders and assets. Menagerie assets are resolved lazily through
`examples.paths`; Mobile ALOHA mesh assets are vendored under `examples/assets/`.

`examples.video_export` implements the `mwb video-export` command for the
indicator-check example. It is intentionally scene-specific because it encodes
camera passes and directorial cuts.

## Scene Boundary

A scene is a fully qualified Python module, such as
`examples.scenes.mobile_aloha_ur10e_server_swap`. It must export the runner
contract:

```python
import mujoco

from mujoco_workbench.arm_handles import ArmHandles, ArmSide, ManipulatorSpec, RobotKind
from mujoco_workbench.cameras import CameraRole
from mujoco_workbench.scene_base import Step

NAME = "my_scene"
ARM_PREFIXES: tuple[ArmSide, ...] = (ArmSide.LEFT, ArmSide.RIGHT)
MANIPULATORS = (
    ManipulatorSpec(side=ArmSide.LEFT, robot_kind=RobotKind.PIPER),
    ManipulatorSpec(side=ArmSide.RIGHT, robot_kind=RobotKind.PIPER),
)
N_CUBES = 0

GRIPPABLES: tuple[str, ...] = ()
CAMERAS: tuple[tuple[str, CameraRole], ...] = ()
AUX_ACTUATOR_NAMES: tuple[str, ...] = ()
BASE_ACTUATOR_NAMES: tuple[str, ...] = ()
LIFT_ACTUATOR_NAME: str | None = None


def build_spec() -> tuple[mujoco.MjModel, mujoco.MjData]: ...


def apply_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    cube_body_ids: list[int],
) -> None: ...


def make_task_plan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arms: dict[ArmSide, ArmHandles],
    cube_body_ids: list[int],
) -> dict[ArmSide, list[Step]]: ...
```

Alternatively, export `step_free_play(t, model, data)` instead of
`make_task_plan` for a non-scripted scene.

`ARM_PREFIXES` plus scene-global `ROBOT_KIND` is still accepted for bundled
legacy scenes. New scenes should prefer `MANIPULATORS`, which is parsed into
per-arm robot adapters at the runtime boundary. The supported topology remains
deliberately narrow: one or two manipulators, with optional mobile-base
actuators declared via `BASE_ACTUATOR_NAMES` and an optional vertical lift or
torso actuator declared via `LIFT_ACTUATOR_NAME`.

`Step` is the timeline boundary. It carries target arm joints, gripper state,
planar base targets, lift targets, remaining auxiliary actuator targets, visual
state changes, grasp weld transitions, attachment weld transitions, and optional
`TaskPhase` labels.
`Step.arm_q` must be a 1-D joint vector; runtime validates its length against
the parsed robot adapter for that manipulator.

Scene-owned motion is split into three component slots. `Step.base_target`
drives the `(x, y, yaw)` actuator tuple declared by `BASE_ACTUATOR_NAMES`;
`Step.lift_target` drives the actuator declared by `LIFT_ACTUATOR_NAME`; and
`Step.aux_ctrl` is reserved for remaining scene-specific actuators that are not
part of the arm, planar base, or lift.

## Architectural Invariants

Scene modules are the only place that should encode task-specific choreography.
Reusable workbench modules should operate through the scene contract and
`Step`/phase types rather than importing scene-local constants.

`mujoco_workbench.runner` and `mujoco_workbench.runtime` should agree on
timeline semantics. When a new `Step` field affects sim state, update both the
live runner path and the non-interactive runtime path so screenshots, contracts,
videos, and browser behavior match.

Asset lookup is lazy. Importing a module should not require every optional robot
asset to exist unless that scene actually touches the asset.

Headless rendering must set `MUJOCO_GL` before importing `mujoco`. The tooling
defaults to `glfw` on macOS and `egl` on Linux.

Generated run artifacts belong under ignored output directories such as
`results/` or `/tmp`, not in source directories.

Remote usage is just a deployment mode. The runner binds to `127.0.0.1` by
default for safe SSH tunneling, but the code should not assume a specific cloud
provider, username, key path, or host.
