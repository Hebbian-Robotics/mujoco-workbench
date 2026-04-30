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

Check timeline clearance after changing scene geometry or motion:

```bash
uv run mwb debug clearance \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --sample-dt 0.50 \
  --max-distance 0.005 \
  --exact-geom \
  --top 12
```

`clearance` reports the worst arm-arm, arm-static, arm-grippable, and
grippable-static pairs with time and active step labels. It exits 0 by default
because some tasks intentionally touch objects; add `--fail-below 0` when any
penetration should fail the run.

Calculate placement values before hand-tuning camera/contact poses:

```bash
# Aim a MuJoCo fixed camera at a world target; output is ready for xyaxes.
uv run mwb debug camera-look-at \
  --position 5.10,0.79,1.08 \
  --target 5.10,1.04,1.00

# Pick a clearance point just outside a scene object.
uv run mwb debug surface-point \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --geom light_left_r3_s10 \
  --normal 0,-1,0 \
  --clearance 0.02

# Back a TCP/camera away from the target along an approach ray.
uv run mwb debug standoff \
  --target 5.10,1.04,1.00 \
  --direction 0,1,0 \
  --distance 0.25
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
