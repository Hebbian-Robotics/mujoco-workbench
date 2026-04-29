---
name: mujoco-phase-contracts
description: Use this when working on mujoco-workbench phase contracts, TaskPhase labels, PhaseState expectations, invariants, observability artifacts, or debugging mwb debug contracts and phase failures.
---

# Mujoco Phase Contracts

Phase contracts are executable documentation for scripted scenes. They should
answer what must be true at phase starts, phase ends, and invariant samples.

Relevant modules:

- `mujoco_workbench.scene_base`: `TaskPhase`, `PhaseState`, `PhaseContract`, invariants.
- `mujoco_workbench.observability`: contract evaluation and artifacts.
- `mujoco_workbench.phase_monitor`: live runner enforcement.
- `mujoco_workbench.runtime`: batch timeline replay used by debug commands.

## Breaking Down Large Motions

Use phases to make placement explicit. A good phase should change one kind of
state at a time:

- Base-only movement: drive, yaw, translate, lift, or align the chassis while
  arm joints stay static.
- Arm-only movement: approach, touch/grip, pull/insert, release, or retract
  while base/lift joints stay static.
- Attachment state: activate/deactivate one semantic grasp, weld, connect, or
  visual state change at an authored boundary.

For long or fragile motions, split into minute waypoints:

- `approach`: standoff point in free space, before contact.
- `at_target`: TCP/contact point on the surface.
- `hold_or_grip`: no pose change; only gripper/attachment state changes.
- `clearance`: back out along the same surface normal or lift direction.
- `transfer`: move base or lift while the object is held.
- `seat_or_release`: final surface/slot point, then deactivate the hold.

Derive these waypoints from layout geometry, not hand literals. Use:

```bash
uv run mwb debug surface-point ...
uv run mwb debug standoff ...
uv run mwb debug camera-look-at ...
```

Contract the boundary of each phase with outcome facts: base aux pose,
expected object pose, active/inactive attachments, static joint sets, held-object
levelness, and qacc sentinels. The indicator-check pattern is the model:
drive straight, yaw where the swept radius clears, translate into final click
pose, then move arms while the base is frozen.

Run all contracts:

```bash
uv run mwb debug contracts \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs
```

Replay one phase:

```bash
uv run mwb debug phase wait_at_server \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs
```

Render the phase graph:

```bash
uv run mwb debug phase-graph \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out /tmp/phases.dot
```

Artifacts are plain files:

- `events.jsonl`
- `summary.json`
- `phase_contracts.json`
- `snapshots/*.npz`
- `renders/*.png`

When adding a new `Step` field that affects sim state, update both
`mujoco_workbench.runner` and `mujoco_workbench.runtime` so live behavior and
debug artifacts stay identical.

Do not make contracts assert implementation details. Assert outcomes: base pose,
attachment state, grippable pose, held-object invariants, static joints, gripper
state, or MuJoCo warning counts.
