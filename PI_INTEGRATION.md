# Hosted pi0.5 policy integration

Plan and progress for hooking `mujoco-sim` up to a remote `pi0.5` policy
server (running the `openpi-flash` stack at `../hosting`). Living
document — update the **Status** and **Remaining work** sections as
tasks complete.

## Goal

Drive `mujoco-sim` scenes with hosted `pi0.5` policy inference. Server
runs on EC2; the sim sends observations and applies the action chunks
returned. Final state: a polished demo with editable prompt, live
camera views in viser, and end-to-end success on at least one task.

## Why now / context

* Hosting layer (`/Users/kkuan/openpi/hosting`) ships a Python client
  (`hosting.flash_transport_policy.FlashTransportPolicy`) that speaks
  QUIC via a local Rust subprocess (`openpi-flash-transport`) to the
  remote server.
* Sim today has zero policy integration — scenes are scripted via
  `make_task_plan()` or free-play via `step_free_play(t, model, data)`
  (`mujoco_workbench/scene_base.py:32`).
* Each `pi0.5` checkpoint is **embodiment-specific**. Cheapest
  integration: build sim scenes whose joints + cameras match the
  model's schema 1:1, eliminating embodiment mapping.

## Approach

Single-arm DROID first: vendor a Franka Panda from `mujoco_menagerie`
(7 DoF + 1 gripper matches `pi05_droid` natively) and drive it via
`FlashTransportPolicy`. Bilateral ALOHA Phase 2 is purely additive
later (new scene + new adapter, shared policy client unchanged).

### Design principle: additive, non-breaking

Every existing scripted-demo capability must keep working unchanged.
Policy mode is layered on top. Concretely:

| Capability | Scripted demos | Policy demos |
|---|---|---|
| `mwb run <scene>` | unchanged | extended (3 new flags) |
| `mwb debug *`, `mwb video-export` | unchanged | reusable as-is |
| `make_task_plan` dispatch | unchanged | n/a |
| `step_free_play` dispatch | unchanged | extended via factory form |
| `PHASE_CONTRACTS` enforcement | unchanged | reused in eval mode (`--policy-eval`) |
| `scene_check.py` invariants | unchanged | reused |
| Welds for grasping (Step boundary triggers) | unchanged | n/a — replaced by state-based grasp detector |
| `--strict`, `--inspect`, `--teleop`, `--play-recording` | unchanged | reusable |
| viser, rerun, mink IK | unchanged | reused |

**Net rule:** if a code change would alter behaviour of a scripted
scene that doesn't opt in to policy mode, it's wrong.

### Why DROID first (and why Franka)

* 1 arm vs 2 → half the joints, half the cameras, half the chance of
  misalignment
* 2 cameras (`observation/exterior_image_1_left`,
  `observation/wrist_image_left`) vs 4 for ALOHA
* 7-DoF + 1-D gripper matches DROID's Franka + Robotiq control shape — no
  padding/remapping
* `franka_emika_panda` and `robotiq_2f85` MJCFs in `mujoco_menagerie` are
  well-validated; `mink` (already a sim dep) supports the Franka arm

## Status

Last updated: 2026-05-05. Tasks tracked in the agent task list (use
`TaskList` to inspect).

### Done

* **Franka Panda registered** in `examples/paths.py` — lazy menagerie
  path lookup. No vendoring needed since `~/mujoco_menagerie/franka_emika_panda/`
  is already cloned.
* **Franka loaders** `examples/robots/franka_panda.py` — keeps the stock
  Panda-hand loader and adds `load_franka_panda_with_robotiq_2f85`, which
  removes the upstream Panda hand and attaches Menagerie's Robotiq 2F-85 at
  the Franka flange.
* **`RobotKind.FRANKA_PANDA_ROBOTIQ_2F85` adapter** in
  `mujoco_workbench/arm_handles.py` — 7 arm joints (`joint1`..`joint7`) +
  actuators (`actuator1`..`actuator7`) + Robotiq gripper
  (`gripper/fingers_actuator`, ctrlrange 0..255), wrist body =
  `gripper/base`, TCP site = `gripper/pinch`.
* **Scene `examples/scenes/franka_droid_pi.py`** — Franka mounted on a
  tabletop with 3 graspable cubes. Cameras `cam_exterior` +
  `left/gripper/cam_wrist` feed the DROID schema. Exposes
  both a no-op `step_free_play` fallback and a hosted-policy
  `make_step_free_play(policy_endpoint, prompt)` factory. Policy execution
  ticks at DROID's 15 Hz control rate and uses an 8-step open-loop window.
* **Headless policy rendering** in `mujoco_workbench/headless_renderer.py` —
  cached `mujoco.Renderer` instances keyed by named camera. The DROID scene
  renders policy/feed cameras at 320x180, then applies OpenPI-style
  resize-with-pad to the final 224x224 observation images.
* **Hosted policy client wrapper** in `mujoco_workbench/policy_client.py` —
  lazy import of `hosting.flash_transport_policy.FlashTransportPolicy`,
  action-chunk buffering, timing metadata capture, reset/close hooks, and
  defensive clipping to `model.actuator_ctrlrange`. DROID velocity actions
  are integrated into position-actuator targets at 15 Hz, with the gripper
  thresholded as an open/closed command.
* **Policy boundary domain types** in `mujoco_workbench/policy_types.py` —
  `PolicyEndpoint` and `PolicyPrompt` parse raw CLI values once, then the
  runner and scene factory consume refined domain objects. Action semantics
  are explicitly modeled as `PolicyActionInterpretation` variants.
* **DROID embodiment observation builder** in
  `mujoco_workbench/embodiments/droid.py` — single-arm action schema:
  `observation/exterior_image_1_left`, `observation/wrist_image_left`,
  `observation/joint_position`, `observation/gripper_position`, `prompt`,
  and `mode="action_only"`.
* **Policy CLI + runner wiring** — `mwb run` now accepts `--policy-host`,
  `--policy-port`, `--prompt`, and `--policy-eval`; the runner detects
  `make_step_free_play`, skips scripted task-plan solving in policy mode,
  calls policy prewarm before the loop, and closes policy resources on exit.
* **Phase monitor evaluation flag** — `PhaseRuntimeMonitor` accepts
  `evaluation_mode` so evaluation runs cannot raise through strict mode.
* **Policy contract tests** in `tests/test_policy_client_contract.py` —
  DROID obs shape/schema checks and action chunk buffering/clamping with a
  fake hosted policy.
* **Runner policy helper tests** in `tests/test_policy_runner_helpers.py` —
  fake scene factory dispatch plus prewarm/reset/close lifecycle coverage
  without touching the real transport.

### Deferred / blocked

* **Dependency declaration is intentionally not added yet.** The local hosting
  package (`/Users/kkuan/openpi/hosting`) declares `requires-python =
  ">=3.11,<3.12"` while this sim project is locked to Python 3.12, and
  `openpi-client` declares `numpy<2.0` while this sim uses `numpy>=2.4.4`.
  The simulator keeps the transport import lazy so all non-policy workflows
  keep working. Live policy mode still requires running from an environment
  where `hosting.flash_transport_policy` and its transport dependencies are
  importable, or relaxing/updating those upstream package constraints.

### Verified

* `mwb run examples.scenes.franka_droid_pi --inspect` → `check_scene: OK`
  (18 bodies, 12 joints, 8 actuators, 4 equalities)
* `pytest tests/test_phase_contracts_structural.py` → 8/8 pass
* `mwb run examples.scenes.mobile_aloha_ur10e_server_swap --inspect` →
  still compiles (regression check on the largest scripted scene)
* `ruff check`, `ruff format`, `ty check` clean on all changed files
* `pytest tests/test_policy_client_contract.py tests/test_policy_runner_helpers.py tests/test_runtime_scene_loader.py`
  → 16/16 pass

## Remaining work

In rough implementation order. Each task is independently testable.

### Phase A: deps + foundational modules

1. **Resolve hosted dependency packaging.** Either update the hosting stack
   for Python 3.12 + NumPy 2.x compatibility or provide a separate policy
   extras/env workflow. Then add the dependency path/source and run `uv sync`.

### Phase B: runner + CLI wiring

Completed.

### Phase C: opt-in helpers (only used by policy scenes)

2. **`mujoco_workbench/grasp_detector.py`** — state machine watching
   gripper closure + EE proximity to graspable bodies; activates
   existing `welds.activate_grasp_weld` API. Existing scripted scenes
   don't reference it. The underlying `welds.py` primitives are
   untouched.
3. **`mujoco_workbench/safety_monitor.py`** — watches `data.contact`
   for self-collisions and EE entering forbidden volumes. Triggers a
   clean rollout end. Opt-in per scene.
4. **Finish policy evaluation scoring** — `PhaseRuntimeMonitor` now has
   `evaluation_mode: bool` and `--policy-eval` sets it, but contract
   scoring/logging to rerun for policy rollouts still needs a scene-level
   phase signal and rerun event schema.

### Phase D: polish

5. **Interactive viser GUI** (only when `--policy-host` set): mirror
    the `teleop.py:451-727` pattern.
    * `server.gui.add_image()` for `cam_exterior` + `cam_wrist` (live
      view of policy-input images, throttled to ~10–15 Hz)
    * `gui.add_text("Prompt", ...)` editable prompt
    * `gui.add_dropdown(...)` of preset prompts
    * `gui.add_number(disabled=True)` showing `server_timing.infer_ms`
    * Pause/resume policy button
    * Clear action buffer button
6. **`scripts/install_flash_transport.sh`** — fetch the
    `openpi-flash-transport` Rust binary from the GitHub release
    artifact for the host platform (darwin-arm64, linux-x86_64).
7. **`examples/scenes/franka_action_replay_debug.py`** — replays a
    canned action chunk against ground-truth qpos. Use to verify
    position/velocity/delta interpretation BEFORE going live.
8. **README + docs** — "Running with a hosted policy" section
    (env vars, MUJOCO_GL=egl on Linux, connection test, end-to-end
    command).
9. **End-to-end verification on real EC2 server** — deploy
    `pi05_droid` with `config.example.json`, install flash-transport
    binary on sim host, run the scene with a real prompt. Iterate on
    action scaling. Run lint/typecheck.

## Phase 2 (ALOHA — later, purely additive)

* Vendor `trossen_vx300s` from mujoco_menagerie under `examples/robots/viperx/`
* Add `RobotKind.VIPERX_300S` adapter
* New `examples/scenes/aloha_pi.py` bilateral scene with `cam_high`,
  `cam_low`, `cam_left_wrist`, `cam_right_wrist`
* New `mujoco_workbench/embodiments/aloha.py` obs builder
* Existing `policy_client.py`, runner CLI, headless renderer, grasp
  detector, safety monitor — all unchanged

## Critical risks

### Action-space calibration

`pi05_droid` emits 8-D actions: 7 joint deltas/positions/velocities + 1
gripper. The exact interpretation depends on the train config. If we
get this wrong the arm will fly off on first inference. The replay-debug
scene (task 13) exists specifically to de-risk this before connecting
the live policy. Budget half a day.

### Franka gripper convention

The adapter currently sets `gripper_open=0, gripper_closed=255` (the
Robotiq-default branch). The actual `mujoco_menagerie` Franka
convention may be inverted. Doesn't affect policy mode (policy emits
raw ctrl values), but scripted Franka scenes (if any are added) would
need this flipped. Verify during calibration.

### Headless rendering on EC2 vs laptop

macOS uses CGL automatically; EC2 needs `MUJOCO_GL=egl` + EGL drivers.
`runtime.py:24` already handles the env var. The driver dependency is
a runtime concern documented in the README task.

## Reference: file map

### Existing infrastructure to reuse (no edits)

* `mujoco_workbench/scene_base.py:32` — `step_free_play` hook
* `mujoco_workbench/scene_check.py` — static invariants
* `mujoco_workbench/welds.py` — grasp weld primitives (called by the new grasp detector)
* `mujoco_workbench/ik.py` + `mink` — Franka EE poses
* `mujoco_workbench/cameras.py` — viser/rerun camera metadata
* `hosting.flash_transport_policy.FlashTransportPolicy` — QUIC client
* `hosting/src/hosting/warmup.py:61-77` — canonical DROID obs schema
* `hosting/examples/galaxea/galaxea_client.py` — client lifecycle reference
* `mujoco_workbench/teleop.py:451-727` — viser GUI pattern reference

### Files modified so far

* `examples/paths.py` — register `FRANKA_PANDA_XML`
* `mujoco_workbench/arm_handles.py` — `RobotKind.FRANKA_PANDA` adapter
* `examples/scenes/franka_droid_pi.py` — hosted-policy free-play factory
* `mujoco_workbench/cli.py` — policy flags
* `mujoco_workbench/runner.py` — policy factory dispatch + prewarm
* `mujoco_workbench/runtime.py` — parse `make_step_free_play`
* `mujoco_workbench/phase_monitor.py` — evaluation-mode flag
* `tests/test_runtime_scene_loader.py` — policy factory loader coverage

### Files created so far

* `examples/robots/franka_panda.py`
* `examples/scenes/franka_droid_pi.py`
* `mujoco_workbench/headless_renderer.py`
* `mujoco_workbench/policy_client.py`
* `mujoco_workbench/policy_types.py`
* `mujoco_workbench/embodiments/__init__.py`
* `mujoco_workbench/embodiments/droid.py`
* `tests/test_policy_client_contract.py`
* `tests/test_policy_runner_helpers.py`

### Files to be modified

* `pyproject.toml` (dependency packaging once upstream constraints are compatible)

### Files to be created

* `mujoco_workbench/grasp_detector.py` (Phase C.8)
* `mujoco_workbench/safety_monitor.py` (Phase C.9)
* `examples/scenes/franka_action_replay_debug.py` (Phase D.13)
* `scripts/install_flash_transport.sh` (Phase D.12)

## Verification commands

```bash
# Existing scripted demos still pass
uv run pytest tests/test_phase_contracts_structural.py
uv run mwb run examples.scenes.mobile_aloha_ur10e_server_swap --inspect

# New Franka scene compiles + obeys invariants
uv run mwb run examples.scenes.franka_droid_pi --inspect

# Lint / format / typecheck
uv run ruff check --fix && uv run ruff format && uv run ty check

# Once policy wiring lands:
./scripts/install_flash_transport.sh
uv run mwb run examples.scenes.franka_droid_pi \
    --policy-host <ec2-ip> --prompt "pick up the red cube" --policy-eval
```
