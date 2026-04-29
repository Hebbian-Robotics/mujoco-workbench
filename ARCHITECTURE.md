# Architecture

`mujoco-workbench` is a small MuJoCo robotics workspace for scripted, inspectable
simulation scenes. The core problem is not only running a scene, but making
visual simulation changes reviewable: the code turns browser-only behavior into
deterministic renders, phase-contract artifacts, and replayable state snapshots.

## Codemap

`runner.py` is the interactive entrypoint. It loads a scene module by name,
compiles the MuJoCo model, starts a Viser server, advances the scripted timeline
or free-play callback, updates Viser geometry, and optionally streams Rerun
events.

`scene_base.py` defines shared domain types: `Step`, `TaskPhase`, `PhaseState`,
`PhaseContract`, and small geometry/value aliases. Most scene/tool boundaries
are expressed through these types.

`scenes/` contains scene modules. Each scene owns its MJCF assembly, initial
state, task plan, optional phase contracts, and optional layout dataclasses.
Layout modules keep durable world-frame geometry constants separate from the
scene's simulation logic.

`robots/` contains robot and sensor loaders. These modules adapt Menagerie or
vendored assets into namespaced MJCF subtrees and expose stable mount/joint/body
names to scenes.

`arm_handles.py`, `ik.py`, `welds.py`, `cameras.py`, and `viser_render.py` are
shared runtime infrastructure. They resolve MuJoCo ids, solve differential IK,
toggle equality-based grasp/attachment helpers, maintain camera widgets, and
mirror MuJoCo geometry into Viser.

`tools/_runtime.py` is the non-interactive counterpart to `runner.py`. It loads
a scene, applies initial state, advances the scripted timeline, and renders
frames without a browser. Agent-facing tools should prefer this module over
duplicating runner behavior.

`tools/mj.py` is the main debug CLI. It provides snapshots, camera grids,
timelines, phase checks, phase replays, image diffs, IK sweeps, and review
packets.

`tools/observability.py` owns phase-contract checking and run artifacts. It
writes JSONL events, summaries, serialized contracts, state snapshots, and
before/after renders.

`tools/render_pov_videos.py` is a scene-specific video renderer for the
indicator-check scene. It is intentionally less generic than `tools/mj.py`
because it encodes camera passes and directorial cuts.

`paths.py` is the asset boundary. It resolves external MuJoCo Menagerie assets
through `MENAGERIE_PATH` or `~/mujoco_menagerie`, and resolves vendored Mobile
ALOHA meshes under `assets/`.

`tests/` covers structural helpers and observability behavior. Tests that need
Menagerie-backed assets skip when those assets are not present.

## Scene Boundary

A scene module under `scenes/` is loaded as `scenes.<name>`. It must export the
runner contract:

```python
import mujoco

from arm_handles import ArmHandles, ArmSide
from cameras import CameraRole
from scene_base import Step

NAME = "my_scene"
ARM_PREFIXES: tuple[ArmSide, ...] = (ArmSide.LEFT, ArmSide.RIGHT)
N_CUBES = 0

GRIPPABLES: tuple[str, ...] = ()
CAMERAS: tuple[tuple[str, CameraRole], ...] = ()
AUX_ACTUATOR_NAMES: tuple[str, ...] = ()


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

Alternatively, a scene can export `step_free_play(t, model, data)` instead of a
scripted task plan.

`Step` is the timeline boundary. It carries target arm joints, gripper state,
auxiliary actuator targets, visual state changes, grasp weld transitions,
attachment weld transitions, and optional `TaskPhase` labels.

## Architectural Invariants

Scene modules are the only place that should encode task-specific choreography.
Shared tools should operate through the scene contract and `Step`/phase types
rather than importing scene-local constants, except for intentionally
scene-specific tools like `render_pov_videos.py`.

`runner.py` and `tools/_runtime.py` should agree on timeline semantics. When a
new `Step` field affects sim state, update both the live runner path and the
non-interactive runtime path so screenshots, contracts, and videos reproduce
browser behavior.

Asset lookup is lazy. Importing a module should not require every optional robot
asset to exist unless that scene actually touches the asset.

The project uses top-level modules rather than a packaged `src/` layout. Scripts
that are executed by path add the repository root to `sys.path` before importing
sibling modules.

Headless rendering must set `MUJOCO_GL` before importing `mujoco`. The tooling
defaults to `glfw` on macOS and `egl` on Linux.

Generated run artifacts belong under ignored output directories such as
`results/` or `/tmp`, not in source directories.

## Cross-Cutting Concerns

Phase contracts are executable documentation. When a scene has meaningful phase
boundaries, update `PHASE_CONTRACTS` with the choreography instead of relying on
comments or screenshots.

Rerun integration is optional. The base sim should keep working with MuJoCo,
Viser, and ImageIO installed; Rerun is for live/offline observability.

Remote usage is just a deployment mode. The runner binds to `127.0.0.1` by
default for safe SSH tunneling, but the code should not assume a specific cloud
provider, username, key path, or host.

MuJoCo Menagerie is not vendored. Vendored assets should be limited to files
that are specific to these scenes and small enough to keep repository operations
reasonable.
