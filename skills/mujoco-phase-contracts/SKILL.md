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

