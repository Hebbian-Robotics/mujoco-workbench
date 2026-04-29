---
name: mujoco-workbench
description: Use this as a lightweight router when working in the mujoco-workbench repository; it explains the public CLI surface, primary example scene, and which focused mujoco-workbench skill to use for running/debugging, scene authoring, or phase contracts.
---

# Mujoco Workbench

Use this repo through the package CLI:

```bash
uv run mwb --help
```

Do not use old path-executed scripts. The intended command surface is:

- `uv run mwb run ...`
- `uv run mwb debug ...`
- `uv run mwb video-export ...`

The primary example scene is:

```bash
examples.scenes.mobile_aloha_piper_indicator_check
```

Read `ARCHITECTURE.md` before changing package boundaries, scene contracts, or
the examples/package split.

Use the focused skills when relevant:

- `mujoco-run-debug`: running scenes, rendering snapshots/grids, video export, Rerun, and artifact-producing debug commands.
- `mujoco-scene-authoring`: adding or editing scene modules, robot loaders, and example assets.
- `mujoco-phase-contracts`: updating phase contracts, invariants, observability checks, and contract failure debugging.

Always run the repository checks after code changes:

```bash
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
```

