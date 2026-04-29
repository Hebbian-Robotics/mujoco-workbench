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

## Placement Pattern

For cameras and contact/touch poses, calculate first and teleop second. The
indicator-check scene is the reference pattern:

- Express target geometry in the layout module: object centers, front faces,
  alert/contact points, row normals, chassis click poses, and clearance margins.
- Use `mwb debug surface-point` to pick a point just outside an object AABB.
  The normal points outward from the object toward the robot/camera.
- Use `mwb debug standoff` to back a camera, TCP, or wrist mount away from
  that point by a chosen depth.
- Use `mwb debug camera-look-at` to compute MuJoCo `xyaxes`; MuJoCo cameras
  look along local `-z`, so do not guess quaternions by eye.
- Bake the resulting values into named layout constants or scene tunables, then
  verify with rendered snapshots/grids and only use teleop for final joint poses.

Examples:

```bash
uv run mwb debug surface-point \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --geom light_left_r3_s10 \
  --normal 0,-1,0 \
  --clearance 0.02

uv run mwb debug standoff \
  --target 5.10,1.04,1.00 \
  --direction 0,1,0 \
  --distance 0.25

uv run mwb debug camera-look-at \
  --position 5.10,0.79,1.08 \
  --target 5.10,1.04,1.00 \
  --up 0,0,1
```

For wrist cameras, define the tool axis in the link-local frame, place the mesh
and camera outside the wrist housing with a small standoff, and aim the camera
at the TCP/contact point. The camera should frame the interaction target, not
the robot's own wrist geometry. Declare `CAMERA_INVARIANTS` so parent-body or
mode regressions fail during `--inspect`.

For server/rack contact, derive the click pose from the rack face plus the
robot's nose/swept-corner extents. Keep yaw-in-place and final approach as
separate base-only steps when rotation would otherwise clip into nearby static
geometry.

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
