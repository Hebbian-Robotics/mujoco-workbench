# mujoco-workbench

MuJoCo robotics simulation workspace with Viser playback, Rerun observability,
headless multi-camera video export, scripted bimanual tasks, phase contracts,
and file-based debug tooling designed for agent-assisted development.

The repo is intentionally runnable as a standalone project. Scene code lives in
`scenes/`, robot loaders live in `robots/`, and the `tools/` commands make sim
state inspectable through screenshots, camera grids, videos, Rerun recordings,
phase artifacts, and visual diffs without manually watching a browser.

For the code map, module boundaries, and scene contract, read
`ARCHITECTURE.md`.

## What Is Included

- `mobile_aloha_ur10e_server_swap`: Mobile ALOHA-style base with two UR10e arms
  and Robotiq 2F-85 grippers performing a server swap.
- `mobile_aloha_piper_indicator_check`: Mobile ALOHA-style base with two Piper
  arms inspecting a rack indicator, including top and wrist camera views for
  headless multi-camera video export.
- `tiago_piper_server_cable_swap`: older TIAGo/Piper server-swap variant kept
  as a reference scene.

Scenes use the naming pattern `<base>_<arms>_<task>`. The runner and debug tools
load scenes dynamically from `scenes.<name>`.

## Quick Start

Install `uv` if needed:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Clone MuJoCo Menagerie. The default lookup path is `~/mujoco_menagerie`; set
`MENAGERIE_PATH` if you keep it elsewhere.

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

Install Python dependencies and start the default scene:

```bash
uv sync
./serve.sh start
open http://localhost:8080
```

Run the runner directly when you want explicit options:

```bash
uv run python runner.py --scene mobile_aloha_ur10e_server_swap
```

Useful runner flags:

- `--host`, `--port`: Viser bind address. Defaults to `127.0.0.1:8080`.
- `--speed`: multiplier on scripted step durations.
- `--render-hz`: Viser and physics update cap. Defaults to `45`.
- `--max-rate`: run as fast as MuJoCo can step.
- `--inspect`: compile the scene, run structural checks, print a schematic, and exit.
- `--strict`: stop on the first phase-contract failure.
- `--teleop`: replace the scripted task plan with browser drag handles.
- `--start-phase PHASE`: boot from a scene-defined phase home.
- `--rerun-port`, `--rerun-connect`, `--rerun-rrd`: stream or save Rerun data.

## Agent Tooling

The most important design choice in this repo is that visual sim work can be
debugged through deterministic commands and artifacts. That makes it practical
for humans and coding agents to inspect failures, compare images, and verify
phase contracts without relying on ad hoc screen observation.

`tools/mj.py` is the main entrypoint:

```bash
# Render one frame from a free camera.
uv run python tools/mj.py snapshot --az 45 --el -20 --out /tmp/home.png

# Render from a named MuJoCo camera at a specific sim time.
uv run python tools/mj.py snapshot --t 22 --camera top_d435i_cam --out /tmp/camera.png

# Render all scene cameras plus a free camera into a labeled grid.
uv run python tools/mj.py grid --t 22 --out /tmp/grid.png

# Print the task plan timeline.
uv run python tools/mj.py plan

# Check phase contracts and write machine-readable artifacts.
uv run python tools/mj.py contracts --out-root results/runs

# Replay one phase and save before/after state snapshots and renders.
uv run python tools/mj.py phase remove_old_server --out-root results/runs

# Emit GraphViz DOT for the phase predecessor graph.
uv run python tools/mj.py phase-graph --out /tmp/phases.dot

# Compare two renders.
uv run python tools/mj.py diff --a /tmp/before.png --b /tmp/after.png --out /tmp/diff.png

# Sweep IK feasibility for planned arm waypoints.
uv run python tools/mj.py ik

# Produce a compact review packet.
uv run python tools/mj.py review --out-dir results/review
```

`tools/_runtime.py` is the shared non-interactive runtime. It imports a scene,
builds the MuJoCo model, applies initial state, advances the same scripted
timeline used by the live runner, and renders frames. Most agent-facing tooling
is intentionally thin around this module.

`tools/observability.py` writes plain artifacts:

- `events.jsonl`: append-only phase-boundary events.
- `summary.json`: CI-friendly pass/fail summary.
- `phase_contracts.json`: serialized expectations for the run.
- `snapshots/*.npz`: `qpos`, `qvel`, `ctrl`, equality state, and sim time.
- `renders/*.png`: before/after visuals for checked boundaries.

## Phase Contracts

Scripted scenes can label steps with `TaskPhase` values and define
`PHASE_CONTRACTS`. A contract describes what must be true at the start and end
of a phase: active attachments, inactive attachments, base pose expectations,
grippable object poses, held-object invariants, static joint sets, gripper
state, and MuJoCo warning counts.

This turns vague failures like "the arm looks wrong" into narrower questions:

- Did the previous phase end in the expected state?
- Did an equality weld activate or deactivate at the wrong boundary?
- Did a held object drift while the phase was running?
- Did a planned waypoint become IK-fragile?

Run contract checks after changing geometry, attachment names, IK targets, step
timing, or phase labels:

```bash
uv run python tools/mj.py contracts --out-root results/runs
```

## Teleop Authoring

Teleop mode is for shaping task motion directly in Viser when a scripted IK path
is valid but awkward.

```bash
uv run python runner.py --scene mobile_aloha_ur10e_server_swap --teleop
```

The browser exposes TCP drag handles, per-joint sliders, base controls,
phase selection, weld/grasp controls, and capture/save buttons. Captures are
written to `/tmp/teleop_recordings/<scene>/teleop_<UTC>.json` and can be
replayed:

```bash
uv run python runner.py \
  --scene mobile_aloha_ur10e_server_swap \
  --play-recording /tmp/teleop_recordings/mobile_aloha_ur10e_server_swap/teleop_YYYYMMDDTHHMMSSZ.json
```

Use `--start-phase PHASE` with teleop or replay when authoring from a specific
phase home.

## Headless Video Export

`tools/render_pov_videos.py` renders the indicator-check scene from four views:

- `forward.mp4`: chassis-mounted top D435i.
- `left_wrist.mp4`: left wrist D405.
- `right_wrist.mp4`: right wrist D405.
- `directorial.mp4`: free-camera hard cuts from authored keyframes.

Stop the live runner before rendering videos so the renderer owns the GL context:

```bash
./serve.sh stop
uv run python tools/render_pov_videos.py --out-dir /tmp/pov
```

The renderer handles timeline-accurate frame stepping, runtime RGBA changes,
per-pass RGBA reset, and hiding debug-only geometry. Camera constants for this
workflow live near the top of `scenes/mobile_aloha_piper_indicator_check.py`.
Directorial keyframes live in `tools/render_pov_videos.py`.

## Remote Linux Usage

The project is not tied to a cloud provider. It runs locally or on any machine
with Python, MuJoCo-compatible rendering, and the required assets.

For a remote Linux GPU host, keep the runner bound to `127.0.0.1` and tunnel the
Viser port:

```bash
rsync -av --delete \
  --exclude='.venv' --exclude='__pycache__' \
  --exclude='*.log' --exclude='*.pid' --exclude='.ruff_cache' \
  -e "ssh -i ~/.ssh/YOUR_KEY.pem" \
  ./ \
  user@your.host:/path/on/remote/

ssh -i ~/.ssh/YOUR_KEY.pem user@your.host /path/on/remote/serve.sh start
ssh -i ~/.ssh/YOUR_KEY.pem -L 8080:localhost:8080 user@your.host
open http://localhost:8080
```

Headless render tools set `MUJOCO_GL` before importing MuJoCo:

- macOS: `glfw`
- Linux: `egl`

If your Linux machine does not expose an EGL-capable driver or software renderer,
headless rendering commands will fail even though non-rendering checks may still
work.

## External Assets

MuJoCo Menagerie is intentionally not vendored. `paths.py` resolves it lazily so
tests and tools that do not touch Menagerie-backed robots can still run.

Default:

```bash
~/mujoco_menagerie
```

Override:

```bash
export MENAGERIE_PATH=/path/to/mujoco_menagerie
```

The Mobile ALOHA chassis mesh used by these scenes is vendored in
`assets/mobile_aloha/`.

## Adding A Scene

See `ARCHITECTURE.md` for the scene module contract and the boundaries between
scene code, robot loaders, runtime helpers, and debug tooling.

Start a new scene with:

```bash
./serve.sh start my_scene
```

## Code Quality

Run checks before committing:

```bash
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
```

For scene changes, also run at least:

```bash
uv run python runner.py --scene mobile_aloha_ur10e_server_swap --inspect
uv run python tools/mj.py contracts --out-root results/runs
```
