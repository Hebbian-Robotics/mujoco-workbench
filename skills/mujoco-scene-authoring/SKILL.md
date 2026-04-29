---
name: mujoco-scene-authoring
description: Use this when adding or editing mujoco-workbench scene modules, example robot loaders, layout dataclasses, assets, cameras, task plans, or the scene module contract.
---

# Mujoco Scene Authoring

Read `ARCHITECTURE.md` before changing scene boundaries or reusable package
modules.

Scenes are fully qualified Python modules. Bundled examples live in:

```text
examples/scenes/
```

Reusable workbench code lives in:

```text
mujoco_workbench/
```

A scene module must export:

```python
NAME: str
ARM_PREFIXES: tuple[ArmSide, ...]
N_CUBES: int
GRIPPABLES: tuple[str, ...]
CAMERAS: tuple[tuple[str, CameraRole], ...]
AUX_ACTUATOR_NAMES: tuple[str, ...]

def build_spec() -> tuple[mujoco.MjModel, mujoco.MjData]: ...
def apply_initial_state(model, data, arms, cube_body_ids) -> None: ...
def make_task_plan(model, data, arms, cube_body_ids) -> dict[ArmSide, list[Step]]: ...
```

Alternatively, a non-scripted scene may export `step_free_play(t, model, data)`.

Keep task-specific choreography inside the scene module or its layout module.
Keep generic stepping/rendering/contract behavior in `mujoco_workbench`.

Prefer layout dataclasses for durable world-frame geometry constants. Keep
robot/sensor loading details in `examples/robots/` unless they are truly generic.

Use fully qualified scene modules in commands:

```bash
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check --inspect
```

After scene edits, run:

```bash
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check --inspect
```

If the edit touches task timing, phase labels, geometry, attachments, or IK
targets, also run contract checks.

