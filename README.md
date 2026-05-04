# mujoco-workbench

https://github.com/user-attachments/assets/af5a213e-7783-4ee4-82b9-29833cae0406

Prototype custom scenes in MuJoCo with coding agents like Codex and Claude Code, for generating training data needed for diversity, or for quick, directionally correct evaluation checks.

**How it works:** install the bundled Agent skills, describe what you want, and the agent scaffolds a working sim for you. No MuJoCo experience required to get started. The skills teach the agent the `mwb` CLI, the scene layout conventions, and the debug tools so it can iterate on behavior without you needing to know the plumbing.

The reusable runtime lives in `mujoco_workbench/`. Prebuilt robot demos live in `examples/`. For the code map and module boundaries, read `ARCHITECTURE.md`.

## The `mwb` CLI

Everything in this repo is driven through a single command-line entry point: `mwb`, short for **m**ujoco-**w**ork**b**ench. It's the one tool you (and your agent) use to run scenes, render snapshots, check phase contracts, export videos, and inspect timelines.

```bash
uv run mwb run <scene>          # launch a scene in the browser
uv run mwb debug snapshot ...   # render a single frame
uv run mwb debug contracts ...  # verify phase contracts
uv run mwb video-export ...     # headless multi-camera video
```

The agent skills are built around this CLI, so when an agent scaffolds or edits a scene it iterates by calling `mwb` subcommands rather than poking at MuJoCo internals directly.

## Requirements

MuJoCo simulation runs on CPU alone — you can prototype scenes, inspect timelines, and check phase contracts on any modern laptop. A GPU is only required if you want to render and save videos (e.g. `mwb video-export` or headless multi-camera output), since offscreen rendering uses EGL/OpenGL.

## Install

Install dependencies:

```bash
uv sync
```

Clone MuJoCo Menagerie for the included robot examples. Menagerie is Google
DeepMind's open-source library of high-quality MuJoCo models for real-world
robots (arms, grippers, humanoids, mobile bases) — the workbench loads its
robots and scene assets straight from there. The default lookup path is
`~/mujoco_menagerie`; set `MENAGERIE_PATH` if you keep it elsewhere.

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

Recommended: install the agent skills from this repository so coding agents
learn the `mwb` workflow and avoid stale script paths.

```bash
npx skills add Hebbian-Robotics/mujoco-workbench --list
npx skills add Hebbian-Robotics/mujoco-workbench --skill mujoco-workbench
npx skills add Hebbian-Robotics/mujoco-workbench --skill mujoco-run-debug
```

Install all bundled skills:

```bash
npx skills add Hebbian-Robotics/mujoco-workbench --skill '*'
```

## Run An Example Prebuilt Scene

Use the single package CLI:

```bash
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check
open http://localhost:8080
```

`serve.sh` is only a small lifecycle wrapper around the same CLI:

```bash
./serve.sh start
./serve.sh status
./serve.sh logs 80
./serve.sh stop
```

Useful `mwb run` flags:

- `--host`, `--port`: Viser bind address. Defaults to `127.0.0.1:8080`.
- `--speed`: multiplier on scripted step durations.
- `--render-hz`: Viser and physics update cap. Defaults to `45`.
- `--max-rate`: run as fast as MuJoCo can step.
- `--inspect`: compile the scene, run structural checks, print a schematic, and exit.
- `--strict`: stop on the first phase-contract failure.
- `--teleop`: replace the scripted task plan with browser drag handles.
- `--start-phase PHASE`: boot from a scene-defined phase home.
- `--rerun-port`, `--rerun-connect`, `--rerun-rrd`: stream or save Rerun data.

## Included Examples

- `examples.scenes.mobile_aloha_piper_indicator_check`: Mobile ALOHA-style base
  with two Piper arms inspecting a rack indicator, including top and wrist
  camera views for headless multi-camera video export. This is the primary
  example used throughout the README.
- `examples.scenes.mobile_aloha_ur10e_server_swap`: Mobile ALOHA-style base with
  two UR10e arms and Robotiq 2F-85 grippers performing a server swap.
- `examples.scenes.tiago_piper_server_cable_swap`: older TIAGo/Piper
  server-swap variant kept as a reference scene.

## Agent Tooling

The main workflow is intentionally CLI-shaped so humans and agents can inspect
visual sim behavior through deterministic files instead of screen watching.

```bash
# Render one frame from a free camera.
uv run mwb debug snapshot \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --az 45 \
  --el -20 \
  --out /tmp/home.png

# Render from a named MuJoCo camera at a specific sim time.
uv run mwb debug snapshot \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --t 22 \
  --camera top_d435i_cam \
  --out /tmp/camera.png

# Render all scene cameras plus a free camera into a labeled grid.
uv run mwb debug grid \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --t 22 \
  --out /tmp/grid.png

# Print the task plan timeline.
uv run mwb debug plan --scene examples.scenes.mobile_aloha_piper_indicator_check

# Check phase contracts and write machine-readable artifacts.
uv run mwb debug contracts \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs

# Replay one phase and save before/after state snapshots and renders.
uv run mwb debug phase wait_at_server \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs

# Emit GraphViz DOT for the phase predecessor graph.
uv run mwb debug phase-graph \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out /tmp/phases.dot

# Compare two renders.
uv run mwb debug diff --a /tmp/before.png --b /tmp/after.png --out /tmp/diff.png

# Sweep IK feasibility for planned arm waypoints.
uv run mwb debug ik --scene examples.scenes.mobile_aloha_piper_indicator_check

# Replay the timeline and report close/clipping geom pairs.
uv run mwb debug clearance \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --sample-dt 0.50 \
  --max-distance 0.005 \
  --exact-geom \
  --top 12

# Produce a compact review packet.
uv run mwb debug review \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-dir results/review
```

`mujoco_workbench.runtime` is the shared non-interactive runtime behind these
commands. It imports a scene, builds the MuJoCo model, applies initial state,
advances the same scripted timeline used by the live runner, and renders frames.

`mujoco_workbench.observability` writes plain artifacts:

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

Run contract checks after changing geometry, attachment names, IK targets, step
timing, or phase labels:

```bash
uv run mwb debug contracts --out-root results/runs
```

Run clearance checks after changing robot/object placement or carry paths:

```bash
uv run mwb debug clearance \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --sample-dt 0.50 \
  --max-distance 0.005 \
  --exact-geom
```

`clearance` is diagnostic by default and exits 0 so intentional contact scenes,
such as the indicator check, still pass. Use `--fail-below 0` or a positive
clearance threshold when a phase must be collision-free.

## Teleop Authoring

Teleop mode is for shaping task motion directly in Viser when a scripted IK path
is valid but awkward.

```bash
uv run mwb run examples.scenes.mobile_aloha_ur10e_server_swap --teleop
```

The browser exposes TCP drag handles, per-joint sliders, base controls,
phase selection, weld/grasp controls, and capture/save buttons. Captures are
written to `/tmp/teleop_recordings/<scene>/teleop_<UTC>.json` and can be
replayed:

```bash
uv run mwb run examples.scenes.mobile_aloha_ur10e_server_swap \
  --play-recording /tmp/teleop_recordings/mobile_aloha_ur10e_server_swap/teleop_YYYYMMDDTHHMMSSZ.json
```

Use `--start-phase PHASE` with teleop or replay when authoring from a specific
phase home.

## Headless Video Export

The included indicator-check example has a dedicated multi-camera export command:

```bash
uv run mwb video-export --out-dir /tmp/pov
```

It renders:

- `forward.mp4`: chassis-mounted top D435i.
- `left_wrist.mp4`: left wrist D405.
- `right_wrist.mp4`: right wrist D405.
- `directorial.mp4`: free-camera hard cuts from authored keyframes.

The exporter handles timeline-accurate frame stepping, runtime RGBA changes,
per-pass RGBA reset, and hiding debug-only geometry. Camera constants for this
workflow live in `examples.scenes.mobile_aloha_piper_indicator_check`.
Directorial keyframes live in `examples.video_export`.

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

MuJoCo Menagerie is intentionally not vendored. `examples.paths` resolves it
lazily so tests and tools that do not touch Menagerie-backed robots can still
run.

Default:

```bash
~/mujoco_menagerie
```

Override:

```bash
export MENAGERIE_PATH=/path/to/mujoco_menagerie
```

The Mobile ALOHA chassis mesh used by the examples is vendored in
`examples/assets/mobile_aloha/`.

## Code Quality

Run checks before committing:

```bash
uv run ruff check --fix
uv run ruff format
uv run ty check
uv run pytest
lychee -v .
```

For scene changes, also run at least:

```bash
uv run mwb run examples.scenes.mobile_aloha_piper_indicator_check --inspect
uv run mwb debug contracts \
  --scene examples.scenes.mobile_aloha_piper_indicator_check \
  --out-root results/runs
```
