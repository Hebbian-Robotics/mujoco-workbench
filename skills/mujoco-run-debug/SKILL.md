---
name: mujoco-run-debug
description: Use this when an agent needs to run mujoco-workbench scenes, inspect Viser/Rerun behavior, render snapshots or camera grids, export videos, run mwb debug commands, or produce file-based artifacts for simulation debugging.
---

# Mujoco Run Debug

Use `uv run mwb` as the only public CLI.

Primary scene:

```bash
examples.scenes.mobile_aloha_piper_indicator_check
```

Run the interactive Viser scene:

```bash
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check
```

Inspect without starting Viser:

```bash
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check --inspect
```

Render a snapshot:

```bash
uv run mwb debug snapshot \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --t 22 \
  --camera top_d435i_cam \
  --out /tmp/camera.png
```

Render a camera grid:

```bash
uv run mwb debug grid \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --t 22 \
  --out /tmp/grid.png
```

Print the task plan:

```bash
uv run mwb debug plan --scene examples.scenes.mobile_aloha_piper_indicator_check
```

Run phase contracts and write artifacts:

```bash
uv run mwb debug contracts \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs
```

Export the indicator-check multi-camera videos:

```bash
uv run mwb video-export --out-dir /tmp/pov
```

Use `results/` for repo-local artifacts and `/tmp` for scratch renders. Avoid
committing generated videos, logs, caches, or run artifacts.

Headless rendering sets `MUJOCO_GL` before importing MuJoCo: `glfw` on macOS,
`egl` on Linux. If rendering fails on Linux, suspect EGL/driver availability
before changing scene code.

