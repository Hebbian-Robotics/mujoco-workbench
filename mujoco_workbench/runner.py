"""Generic scene runner.

Imports a scene module by name, compiles its spec, and plays it through Viser.
Two modes, picked by what the scene module exposes:

  * Task-planned (`make_task_plan`): per-arm state machines advance in
    parallel, each interpolating ctrl through a list of Steps. Weld
    activate/deactivate transitions fire on entry to the relevant step.
  * Free-play (`step_free_play`): the scene's callback is invoked every
    render tick to set ctrl directly.

`--render-hz` caps the viser+physics update rate; the browser doesn't see
60 Hz vs 125 Hz, but the websocket CPU cost scales linearly. `--max-rate`
drops the realtime throttle so the sim runs as fast as MuJoCo can step.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import signal
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import viser

from mujoco_workbench.arm_handles import (
    ArmHandles,
    ArmSide,
    arm_joint_labels,
    get_arm_handles,
    write_gripper_target,
)
from mujoco_workbench.cameras import CameraRole, add_frustum_widgets, update_frustum_widgets
from mujoco_workbench.headless_renderer import NamedCameraRenderer
from mujoco_workbench.phase_monitor import PhaseContractViolation, PhaseRuntimeMonitor
from mujoco_workbench.policy_types import (
    PolicyEndpoint,
    PolicyPrompt,
    make_policy_endpoint,
    make_policy_prompt,
)
from mujoco_workbench.rerun_stream import RerunStreamer
from mujoco_workbench.runtime import (
    LoadedScene,
    StepFreePlay,
    load_scene,
    resolve_timeline_actuator_maps,
    validate_task_plan_targets,
)
from mujoco_workbench.scene_base import PhaseContract, Step, TaskPhase, ViserCameraPose
from mujoco_workbench.scene_check import (
    CameraInvariant,
    check_scene,
    print_schematic,
)
from mujoco_workbench.teleop import TeleopController
from mujoco_workbench.viser_render import build_viser_scene, update_geom_rgba, update_viser
from mujoco_workbench.welds import (
    activate_attachment_weld,
    activate_grasp_weld,
    deactivate_grasp_weld,
    deactivate_weld,
)


@dataclass
class ArmTimelineState:
    """Per-arm progress through its Step list. Each arm advances independently."""

    start_q: np.ndarray
    start_g: float
    start_base: dict[str, float] = field(default_factory=dict)
    start_lift: float | None = None
    # Most-recently committed target per scene-owned aux actuator, so
    # interpolation continues smoothly across steps. Missing key ⇒ use current
    # data.ctrl at tick entry.
    start_aux: dict[str, float] = field(default_factory=dict)
    step: int = 0
    t: float = 0.0


def _collect_grippable_body_ids(
    model: mujoco.MjModel,
    grippable_names: tuple[str, ...],
) -> list[int]:
    return [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, grippable_name)
        for grippable_name in grippable_names
    ]


@contextlib.contextmanager
def shutdown_signal_as_keyboard_interrupt() -> Iterator[None]:
    """Let normal runner cleanup run when the process receives SIGTERM."""
    original_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def _raise_keyboard_interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, original_sigterm_handler)


def build_policy_step_free_play(
    scene: LoadedScene,
    *,
    scene_module_name: str,
    policy_endpoint: PolicyEndpoint,
    policy_prompt: PolicyPrompt,
) -> StepFreePlay:
    """Instantiate a scene's hosted-policy free-play controller."""
    if scene.make_step_free_play is None:
        raise ValueError(
            f"--policy-host was set, but scene {scene_module_name!r} does not expose "
            "make_step_free_play(policy_endpoint=..., prompt=...)"
        )
    return scene.make_step_free_play(
        policy_endpoint=policy_endpoint,
        prompt=policy_prompt,
    )


def prewarm_step_free_play(
    step_free_play: StepFreePlay | None,
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> bool:
    """Run a free-play prewarm hook when the controller exposes one."""
    free_play_prewarm = getattr(step_free_play, "prewarm", None)
    if not callable(free_play_prewarm):
        return False
    free_play_prewarm(model, data)
    return True


def set_client_viser_camera_pose(client: viser.ClientHandle, pose: ViserCameraPose) -> None:
    """Move one browser client's orbit camera to a scene-declared startup pose."""
    client.camera.position = pose.position
    client.camera.look_at = pose.lookat


def reset_step_free_play(step_free_play: StepFreePlay | None) -> bool:
    """Run a free-play reset hook when the controller exposes one."""
    free_play_reset = getattr(step_free_play, "reset", None)
    if not callable(free_play_reset):
        return False
    free_play_reset()
    return True


def close_step_free_play(step_free_play: StepFreePlay | None) -> bool:
    """Run a free-play close hook when the controller exposes one."""
    free_play_close = getattr(step_free_play, "close", None)
    if not callable(free_play_close):
        return False
    free_play_close()
    return True


def set_step_free_play_prompt(
    step_free_play: StepFreePlay | None,
    policy_prompt: PolicyPrompt,
) -> bool:
    """Update a policy free-play prompt when the controller exposes a setter."""
    free_play_set_prompt = getattr(step_free_play, "set_prompt", None)
    if not callable(free_play_set_prompt):
        return False
    free_play_set_prompt(policy_prompt)
    return True


def clear_step_free_play_action_buffer(step_free_play: StepFreePlay | None) -> bool:
    """Clear a policy free-play action buffer when the controller exposes one."""
    free_play_clear_action_buffer = getattr(step_free_play, "clear_action_buffer", None)
    if not callable(free_play_clear_action_buffer):
        return False
    free_play_clear_action_buffer()
    return True


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        required=True,
        help="Fully qualified scene module, e.g. examples.scenes.mobile_aloha_ur10e_server_swap.",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--render-hz",
        type=float,
        default=45.0,
        help=(
            "viser + physics update rate; physics timestep is still the scene's"
            " mj timestep. Default 45 Hz trades 60→45 of server-side updates for"
            " ~25%% less websocket / message-buffer CPU; the browser renders"
            " between updates via WebGL so the visual result stays smooth."
        ),
    )
    parser.add_argument(
        "--max-rate",
        action="store_true",
        help="skip the realtime sleep throttle; run physics as fast as CPU allows",
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help=(
            "build + compile the scene, run scene_check, print a body/geom "
            "schematic to stdout, and exit before physics/viser. Pair with a "
            "layout edit to confirm the scene still passes invariants."
        ),
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "if the scene exposes PHASE_CONTRACTS, raise the moment any "
            "contract (boundary expectation or in-phase invariant) fails. "
            "Without --strict, contract failures are collected and printed "
            "at exit but the demo keeps running."
        ),
    )
    parser.add_argument(
        "--rerun-port",
        type=int,
        default=None,
        help=(
            "open a rerun gRPC server on localhost:PORT and stream qpos / "
            "body transforms / phase events live. For EC2→laptop streaming, "
            "SSH-tunnel with `-L PORT:localhost:PORT` then connect a local "
            "rerun viewer to `rerun+http://localhost:PORT`."
        ),
    )
    parser.add_argument(
        "--rerun-connect",
        type=str,
        default=None,
        help=(
            "push events to an already-running rerun viewer at URL "
            "(e.g. `rerun+http://laptop.local:9876`). Mutually exclusive "
            "with --rerun-port."
        ),
    )
    parser.add_argument(
        "--rerun-rrd",
        type=Path,
        default=None,
        help="write the same rerun stream to a `.rrd` file for offline replay.",
    )
    parser.add_argument(
        "--rerun-camera-every",
        type=int,
        default=0,
        help=(
            "if >0, log named-camera frames every N render ticks (e.g. 5 → "
            "9 Hz at the default 45 Hz render rate). 0 disables camera "
            "logging — recommended over slow SSH tunnels."
        ),
    )
    parser.add_argument(
        "--camera-feed-every",
        type=int,
        default=3,
        help=(
            "refresh in-browser named-camera feed images every N render ticks; "
            "0 disables camera feeds for smoother live playback"
        ),
    )
    parser.add_argument(
        "--teleop",
        action="store_true",
        help=(
            "skip the scripted task plan; instead expose viser drag handles "
            "for each arm's TCP and run live IK. Use the in-browser GUI to "
            "capture keyframes per phase and save them to JSON for later "
            "replay via --play-recording."
        ),
    )
    parser.add_argument(
        "--play-recording",
        type=Path,
        default=None,
        help=(
            "load a teleop JSON recording (saved by --teleop) and replay "
            "it as the task plan. Mutually exclusive with --teleop."
        ),
    )
    parser.add_argument(
        "--start-phase",
        type=str,
        default=None,
        help=(
            "boot the scene at a hand-authored phase START state instead of "
            "scene home. Pass a TaskPhase value (case-insensitive, e.g. "
            "'remove_old_server' or 'REMOVE_OLD_SERVER'). The pose comes "
            "from the scene layout's PHASE_HOMES map (populated via teleop's "
            "'Print phase homes for layout' button). Compatible with both "
            "--teleop and --play-recording — boot mid-demo and either "
            "author further from there or replay onward."
        ),
    )
    parser.add_argument(
        "--policy-host",
        type=str,
        default=None,
        help=(
            "connect to a hosted OpenPI policy server and use the scene's "
            "make_step_free_play factory instead of a scripted task plan"
        ),
    )
    parser.add_argument(
        "--policy-port",
        type=int,
        default=5555,
        help="hosted OpenPI policy server QUIC port",
    )
    parser.add_argument(
        "--policy-local-port",
        type=int,
        default=5556,
        help="local UDP port used by the OpenPI flash transport client sidecar",
    )
    parser.add_argument(
        "--prompt",
        type=str,
        default="do something",
        help="language instruction sent to the hosted policy",
    )
    parser.add_argument(
        "--policy-eval",
        action="store_true",
        help=(
            "evaluate phase contracts without raising; intended for policy "
            "rollouts where failures should be logged and scored"
        ),
    )
    args = parser.parse_args(argv)
    if args.teleop and args.play_recording is not None:
        parser.error("--teleop and --play-recording are mutually exclusive")
    if args.policy_host is not None and args.teleop:
        parser.error("--policy-host and --teleop are mutually exclusive")
    if args.policy_host is not None and args.play_recording is not None:
        parser.error("--policy-host and --play-recording are mutually exclusive")
    if args.policy_eval and args.strict:
        parser.error("--policy-eval and --strict are mutually exclusive")
    if args.policy_eval and args.policy_host is None:
        parser.error("--policy-eval requires --policy-host")
    policy_endpoint: PolicyEndpoint | None = None
    policy_prompt: PolicyPrompt | None = None
    if args.policy_host is not None:
        try:
            policy_endpoint = make_policy_endpoint(
                host=args.policy_host,
                port=args.policy_port,
                local_port=args.policy_local_port,
            )
            policy_prompt = make_policy_prompt(args.prompt)
        except ValueError as err:
            parser.error(str(err))
    start_phase: TaskPhase | None = None
    if args.start_phase is not None:
        # Case-insensitive — accept 'REMOVE_OLD_SERVER' or 'remove_old_server'.
        try:
            start_phase = TaskPhase(args.start_phase.lower())
        except ValueError:
            valid = ", ".join(p.value for p in TaskPhase)
            parser.error(f"--start-phase {args.start_phase!r} not in TaskPhase. Valid: {valid}")
    if args.rerun_port is not None and args.rerun_connect is not None:
        parser.error("--rerun-port and --rerun-connect are mutually exclusive")
    policy_mode = policy_endpoint is not None

    print(f"Loading {args.scene} ...")
    scene = load_scene(args.scene)
    print(f"Building scene: {scene.display_name}")

    model, data = scene.build_spec()
    print(
        f"compiled: nbody={model.nbody} njnt={model.njnt} nu={model.nu} "
        f"neq={model.neq} ngeom={model.ngeom}"
    )

    cube_body_ids = _collect_grippable_body_ids(model, scene.grippable_names)
    arms: dict[ArmSide, ArmHandles] = {
        manipulator.side: get_arm_handles(model, manipulator, scene.n_cubes)
        for manipulator in scene.manipulators
    }
    arm_sides = scene.arm_sides

    # Scene-owned actuators (e.g. a lift prismatic). qpos/qvel addresses are
    # resolved alongside ids so puppet-mode can do direct writes.
    timeline_actuators = resolve_timeline_actuator_maps(model, scene)
    for name, aid in timeline_actuators.aux_name_to_id.items():
        if aid < 0:
            raise ValueError(
                f"scene declared aux actuator {name!r} but no such actuator in compiled model"
            )
    aux_qposadr: dict[str, int] = {}
    aux_dofadr: dict[str, int] = {}
    for name, aid in timeline_actuators.aux_name_to_id.items():
        # Aux actuators must be JOINT-transmission position actuators.
        jnt_id = int(model.actuator_trnid[aid][0])
        aux_qposadr[name] = int(model.jnt_qposadr[jnt_id])
        aux_dofadr[name] = int(model.jnt_dofadr[jnt_id])

    scene.apply_initial_state(model, data, arms, cube_body_ids, start_phase=start_phase)

    # check_scene runs before make_task_plan so geometry bugs surface as
    # scene errors instead of IK-residual panics.
    grippable_names = scene.grippable_names
    allowed_overlaps = scene.allowed_static_overlaps
    attachment_constraints = scene.attachment_constraints
    camera_invariants: tuple[CameraInvariant, ...] = scene.camera_invariants

    if args.inspect:
        # Print first so the user sees the body/geom tree even if check_scene
        # is about to raise.
        print_schematic(
            model,
            data,
            arms=arms,
            grippable_names=grippable_names,
            attachment_constraints=attachment_constraints,
        )
        check_scene(
            model,
            data,
            arms=arms,
            grippable_names=grippable_names,
            allowed_static_overlaps=allowed_overlaps,
            attachment_constraints=attachment_constraints,
            camera_invariants=camera_invariants,
        )
        print("\ncheck_scene: OK")
        return

    check_scene(
        model,
        data,
        arms=arms,
        grippable_names=grippable_names,
        allowed_static_overlaps=allowed_overlaps,
        attachment_constraints=attachment_constraints,
        camera_invariants=camera_invariants,
    )

    active_step_free_play = scene.step_free_play
    if policy_mode:
        assert policy_endpoint is not None
        assert policy_prompt is not None
        try:
            active_step_free_play = build_policy_step_free_play(
                scene,
                scene_module_name=args.scene,
                policy_endpoint=policy_endpoint,
                policy_prompt=policy_prompt,
            )
        except ValueError as err:
            raise SystemExit(str(err)) from err
        print(
            f"Policy mode: hosted OpenPI server {policy_endpoint.host}:{policy_endpoint.port}; "
            f"local_port={policy_endpoint.local_port}; "
            f"prompt={policy_prompt!r}"
        )

    task_plan: dict[ArmSide, list[Step]] | None = None
    if args.teleop:
        print("Teleop mode: dragging TCP handles drives live IK.")
    elif args.play_recording is not None:
        from mujoco_workbench.teleop import load_recording

        loaded = load_recording(args.play_recording)
        task_plan = {side: list(steps) for side, steps in loaded.items()}
        validate_task_plan_targets(scene, arms, task_plan)
        scene.apply_initial_state(model, data, arms, cube_body_ids, start_phase=start_phase)
        print(f"Replaying recording: {args.play_recording}")
        for side in scene.arm_sides:
            print(f"  [{side}] {len(task_plan[side])} steps loaded")
    elif not policy_mode and scene.make_task_plan is not None:
        print("Solving IK waypoints...")
        task_plan = scene.make_task_plan(model, data, arms, cube_body_ids)
        validate_task_plan_targets(scene, arms, task_plan)
        scene.apply_initial_state(model, data, arms, cube_body_ids, start_phase=start_phase)

    phase_contracts: tuple[PhaseContract, ...] = scene.phase_contracts
    phase_monitor = PhaseRuntimeMonitor(
        phase_contracts,
        strict=args.strict,
        evaluation_mode=args.policy_eval,
    )
    if phase_monitor.enabled:
        print(
            f"PhaseRuntimeMonitor active ({len(phase_contracts)} contracts, "
            f"strict={phase_monitor.strict}, evaluation={phase_monitor.evaluation_mode})"
        )

    # One source feeds at most one rerun sink (gRPC serve, gRPC connect, .rrd).
    rerun_streamer: RerunStreamer | None = None
    if args.rerun_port is not None:
        rerun_streamer = RerunStreamer.serve_grpc(scene_name=args.scene, grpc_port=args.rerun_port)
    elif args.rerun_connect is not None:
        rerun_streamer = RerunStreamer.connect_grpc(scene_name=args.scene, url=args.rerun_connect)
    elif args.rerun_rrd is not None:
        rerun_streamer = RerunStreamer.save_rrd(scene_name=args.scene, rrd_path=args.rerun_rrd)

    # Cache body ids + joint suffixes once so per-tick rerun logging skips
    # name resolution.
    rerun_body_ids: dict[str, int] = {}
    rerun_joint_names: dict[ArmSide, tuple[str, ...]] = {}
    if rerun_streamer is not None:
        for grippable_name in grippable_names:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, grippable_name)
            if bid >= 0:
                rerun_body_ids[grippable_name] = bid
        for side, arm in arms.items():
            wrist_id = arm.grasp_body_id
            if wrist_id >= 0:
                rerun_body_ids[f"{side.rstrip('/')}/wrist"] = wrist_id
        for side, arm in arms.items():
            rerun_joint_names[side] = arm_joint_labels(arm)

    rerun_tick_counter = {"n": 0}
    camera_feed_tick_counter = {"n": 0}

    has_free_play = active_step_free_play is not None
    if task_plan is None and not has_free_play and not args.teleop:
        print(
            "warning: scene provides neither make_task_plan nor step_free_play; "
            "arms will hold their initial pose"
        )

    server = viser.ViserServer(host=args.host, port=args.port)
    default_viser_camera_pose = scene.default_viser_camera_pose
    if default_viser_camera_pose is not None:

        @server.on_client_connect
        def _set_default_client_camera(client: viser.ClientHandle) -> None:
            set_client_viser_camera_pose(client, default_viser_camera_pose)

    # `build_viser_scene` reads `data` to bake initial static-geom poses, so
    # `apply_initial_state` must already have run.
    handles, handle_by_geom_id = build_viser_scene(server, model, data)

    # Snapshot the initial geom RGBA + track which geom_ids `set_geom_rgba`
    # mutates during the run, so `restart()` can restore them. Without this,
    # an indicator-light flip persists across scene resets — `model.geom_rgba`
    # is on the model (not data), so `mj_resetData` doesn't touch it.
    initial_geom_rgba: np.ndarray = np.asarray(model.geom_rgba, dtype=float).copy()
    rgba_dirty_geom_ids: set[int] = set()

    cameras: tuple[tuple[str, CameraRole], ...] = scene.cameras
    frustum_handles = add_frustum_widgets(server, model, data, cameras) if cameras else []

    gui_state = server.gui.add_text("state", initial_value="ready", disabled=True)
    gui_speed = server.gui.add_slider(
        "speed", min=0.1, max=3.0, step=0.1, initial_value=float(args.speed)
    )
    gui_play = server.gui.add_button("▶ play / ⏸ pause")
    gui_reset = server.gui.add_button("↺ reset")
    gui_focus_robot = server.gui.add_button("📷 focus on robot")
    gui_log_cam = server.gui.add_button("📸 log cam pose")

    # Resolve chassis qpos addresses once so the focus button can read the
    # live chassis world position. Empty tuple if the scene has no planar
    # base joints (free-play scenes); the button just no-ops in that case.
    base_focus_qposadr: list[int] = []
    for jname in scene.ik_locked_joint_names[:2]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            base_focus_qposadr.append(int(model.jnt_qposadr[jid]))

    per_arm_gui: dict[ArmSide, viser.GuiTextHandle] = {}
    for side in arm_sides:
        per_arm_gui[side] = server.gui.add_text(
            side.rstrip("_") or "arm", initial_value="-", disabled=True
        )

    teleop_controller: TeleopController | None = None
    if args.teleop:
        # Teleop drives the chassis from a base handle, so it requires the
        # scene to expose a (base_x, base_y, base_yaw) triple.
        teleop_locked = scene.ik_locked_joint_names
        if len(teleop_locked) != 3:
            raise SystemExit(
                f"--teleop requires the scene to expose IK_LOCKED_JOINT_NAMES as a "
                f"3-tuple (base_x, base_y, base_yaw); got {teleop_locked!r}"
            )
        a, b, c = teleop_locked
        teleop_base_actuator_names: tuple[str, str, str] = (str(a), str(b), str(c))
        teleop_controller = TeleopController.attach(
            server,
            model=model,
            data=data,
            arms=arms,
            locked_joint_names=teleop_locked,
            attachments=attachment_constraints,
            phase_contracts=phase_contracts,
            grippable_names=grippable_names,
            cube_body_ids=tuple(cube_body_ids),
            base_actuator_names=teleop_base_actuator_names,
            scene_name=args.scene,
        )

    control = {"playing": False, "reset_requested": False}

    if policy_mode:
        assert policy_endpoint is not None
        assert policy_prompt is not None
        with server.gui.add_folder("Policy"):
            server.gui.add_text(
                "endpoint",
                initial_value=f"{policy_endpoint.host}:{policy_endpoint.port}",
                disabled=True,
            )
            gui_policy_prompt = server.gui.add_text(
                "prompt",
                initial_value=str(policy_prompt),
                hint="language instruction sent with each hosted policy observation",
            )
            gui_policy_status = server.gui.add_text(
                "policy",
                initial_value="set prompt, apply, then play",
                disabled=True,
            )
            gui_apply_policy_prompt = server.gui.add_button("apply prompt")
            gui_clear_policy_actions = server.gui.add_button("clear action buffer")

        @gui_apply_policy_prompt.on_click
        def _on_apply_policy_prompt(_event: Any) -> None:
            nonlocal policy_prompt
            try:
                updated_policy_prompt = make_policy_prompt(str(gui_policy_prompt.value))
            except ValueError as err:
                gui_policy_status.value = str(err)
                return
            policy_prompt = updated_policy_prompt
            set_step_free_play_prompt(active_step_free_play, updated_policy_prompt)
            gui_policy_status.value = f"applied: {updated_policy_prompt}; press play"

        @gui_clear_policy_actions.on_click
        def _on_clear_policy_actions(_event: Any) -> None:
            if clear_step_free_play_action_buffer(active_step_free_play):
                gui_policy_status.value = "cleared buffered actions"
            else:
                gui_policy_status.value = "policy controller has no action buffer"

    @gui_play.on_click
    def _on_play(_event: Any) -> None:
        control["playing"] = not control["playing"]
        gui_state.value = "running" if control["playing"] else "paused"

    @gui_reset.on_click
    def _on_reset(_event: Any) -> None:
        control["reset_requested"] = True

    @gui_focus_robot.on_click
    def _on_focus_robot(_event: Any) -> None:
        """Re-anchor every connected client's orbit pivot at the chassis
        position. viser orbits / zooms around `look_at`, which defaults to
        the scene origin — when the chassis has driven 5 m down the aisle,
        zooming in toward (0,0,0) feels like a hard limit because you're
        not actually approaching the robot. This snaps the pivot to the
        chassis so subsequent zoom/orbit happens around the action."""
        if len(base_focus_qposadr) < 2:
            return
        chassis_x = float(data.qpos[base_focus_qposadr[0]])
        chassis_y = float(data.qpos[base_focus_qposadr[1]])
        target = (chassis_x, chassis_y, 1.0)
        for client in server.get_clients().values():
            client.camera.look_at = target

    @gui_log_cam.on_click
    def _on_log_cam(event: Any) -> None:
        """Print the clicking client's orbit-cam pose + sim time to stdout.
        Format mirrors what `examples.video_export` consumes as a
        directorial keyframe: `pos` and `lookat` are world-frame xyz tuples,
        `t` is the current sim-clock second. Pause first to freeze the
        moment, orbit to the framing you want, click — paste the line into
        the directorial keyframe list."""
        client = event.client
        if client is None:
            return
        pos = client.camera.position
        look = client.camera.look_at
        print(
            f"cam_pose t={float(data.time):6.2f}  "
            f"pos=({float(pos[0]):.3f}, {float(pos[1]):.3f}, {float(pos[2]):.3f})  "
            f"lookat=({float(look[0]):.3f}, {float(look[1]):.3f}, {float(look[2]):.3f})",
            flush=True,
        )

    camera_feed_renderer: NamedCameraRenderer | None = None
    camera_feed_handles: dict[str, viser.GuiImageHandle] = {}
    camera_feed_every = int(args.camera_feed_every)
    if cameras and camera_feed_every > 0:
        camera_feed_renderer = NamedCameraRenderer(
            model,
            width=scene.camera_feed.render_width,
            height=scene.camera_feed.render_height,
        )
        initial_camera_feed_image = np.zeros((224, 224, 3), dtype=np.uint8)
        if scene.camera_feed.preprocess is None:
            initial_camera_feed_image = np.zeros(
                (scene.camera_feed.render_height, scene.camera_feed.render_width, 3),
                dtype=np.uint8,
            )
        with server.gui.add_folder("Camera feeds"):
            for camera_name, _camera_role in cameras:
                camera_feed_handles[camera_name] = server.gui.add_image(
                    initial_camera_feed_image,
                    label=camera_name,
                    format="jpeg",
                    jpeg_quality=75,
                )

    sim_dt = float(model.opt.timestep)
    # Decouple render rate from physics timestep: each frame we step physics
    # `phys_steps_per_frame` times so wall-clock advance = render_dt.
    render_dt = 1.0 / max(args.render_hz, 1e-3)
    phys_steps_per_frame = max(1, round(render_dt / sim_dt))
    # Re-derive render_dt from the rounded step count so the physics clock and
    # the sleep throttle agree exactly.
    render_dt = sim_dt * phys_steps_per_frame
    print(
        f"render: {1.0 / render_dt:.1f} Hz "
        f"({phys_steps_per_frame} physics steps x {sim_dt * 1000:.1f} ms)"
        + (" [max-rate: throttle off]" if args.max_rate else "")
    )

    def fresh_state() -> dict[ArmSide, ArmTimelineState]:
        return {
            side: ArmTimelineState(
                start_q=np.array([data.qpos[i] for i in arms[side].arm_qpos_idx]),
                start_g=float(data.ctrl[arms[side].act_gripper_id]),
            )
            for side in arm_sides
        }

    per_arm: dict[ArmSide, ArmTimelineState] = fresh_state()

    def restart() -> None:
        nonlocal per_arm
        scene.apply_initial_state(model, data, arms, cube_body_ids, start_phase=start_phase)
        per_arm = fresh_state()
        phase_monitor.reset()
        reset_step_free_play(active_step_free_play)
        # Restore any geom RGBAs that were mutated by `set_geom_rgba` during
        # the run. Iterate over a copy because `update_geom_rgba` may end up
        # mutating the registry; clear the dirty set after.
        for geom_id in list(rgba_dirty_geom_ids):
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if not geom_name:
                continue
            initial = initial_geom_rgba[geom_id]
            update_geom_rgba(
                server,
                model,
                data,
                handles,
                handle_by_geom_id,
                geom_name,
                (float(initial[0]), float(initial[1]), float(initial[2]), float(initial[3])),
            )
        rgba_dirty_geom_ids.clear()

    def advance_arm(side: ArmSide, dt: float) -> str:
        assert task_plan is not None
        script = task_plan[side]
        st = per_arm[side]
        arm = arms[side]

        if st.step >= len(script):
            return "done"

        step = script[st.step]
        duration = step.duration / max(gui_speed.value, 0.05)

        first_tick = st.t == 0.0
        st.t += dt

        if first_tick:
            if step.weld_activate is not None:
                activate_grasp_weld(
                    model,
                    data,
                    int(arm.weld_ids[step.weld_activate]),
                    arm.grasp_body_id,
                    cube_body_ids[step.weld_activate],
                    arm.tcp_site_id,
                )
            if step.weld_deactivate is not None:
                deactivate_grasp_weld(data, int(arm.weld_ids[step.weld_deactivate]))
            # WELD freezes the current relpose (no teleport); CONNECT is a
            # simple flag flip — the anchor is already in eq_data.
            for weld_name in step.attach_activate:
                eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
                if eq_id < 0:
                    raise ValueError(f"step '{step.label}': unknown attach weld {weld_name!r}")
                if int(model.eq_type[eq_id]) == int(mujoco.mjtEq.mjEQ_CONNECT):
                    data.eq_active[eq_id] = 1
                else:
                    body_a = int(model.eq_obj1id[eq_id])
                    body_b = int(model.eq_obj2id[eq_id])
                    activate_attachment_weld(model, data, eq_id, body_a, body_b)
            for weld_name, target_xyz, target_quat in step.attach_activate_at or ():
                eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
                if eq_id < 0:
                    raise ValueError(f"step '{step.label}': unknown attach_at weld {weld_name!r}")
                if int(model.eq_type[eq_id]) == int(mujoco.mjtEq.mjEQ_CONNECT):
                    raise ValueError(
                        f"step '{step.label}': attach_activate_at requires a WELD "
                        f"equality, got CONNECT for {weld_name!r}"
                    )
                body_a = int(model.eq_obj1id[eq_id])
                body_b = int(model.eq_obj2id[eq_id])
                activate_attachment_weld(
                    model,
                    data,
                    eq_id,
                    body_a,
                    body_b,
                    target_world_pose=(target_xyz, target_quat),
                )
            for weld_name in step.attach_deactivate:
                eq_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, weld_name))
                if eq_id < 0:
                    raise ValueError(f"step '{step.label}': unknown attach weld {weld_name!r}")
                deactivate_weld(data, eq_id)
            # Visual-only state changes — flip indicator-light colours and the
            # like. Writes both `model.geom_rgba` and the viser handle so the
            # browser actually sees the new colour (viser bakes RGBA at handle
            # creation, so this is a remove-and-re-add under the hood).
            # Track touched geoms so `restart()` can restore them — without
            # this, a flipped indicator stays flipped forever after the first
            # demo run, since `model.geom_rgba` survives `mj_resetData`.
            for geom_name, rgba in step.set_geom_rgba:
                geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name))
                if geom_id >= 0:
                    rgba_dirty_geom_ids.add(geom_id)
                update_geom_rgba(
                    server,
                    model,
                    data,
                    handles,
                    handle_by_geom_id,
                    geom_name,
                    rgba,
                )

        alpha = min(1.0, st.t / max(duration, 1e-3))
        alpha_s = 0.5 - 0.5 * math.cos(math.pi * alpha)

        # Puppet mode: direct qpos write. Zero qvel so mj_step doesn't drift
        # the integrator off the puppet target; mirror ctrl so position
        # actuators don't fight back with stale PD force.
        curr_q = (1.0 - alpha_s) * st.start_q + alpha_s * step.arm_q
        data.qpos[arm.arm_qpos_idx] = curr_q
        data.qvel[arm.arm_dof_idx] = 0.0
        data.ctrl[arm.act_arm_ids] = curr_q

        tgt_g = arm.gripper_open if step.gripper == "open" else arm.gripper_closed
        curr_g = (1.0 - alpha_s) * st.start_g + alpha_s * tgt_g
        write_gripper_target(data, arm, curr_g)

        # Multiple arms may write the same aux on overlapping steps; last
        # write wins — scenes are expected to keep their targets consistent.
        if step.base_target is not None:
            for base_name, base_target in zip(
                timeline_actuators.base_name_to_id,
                step.base_target.as_tuple(),
                strict=True,
            ):
                start = st.start_base.get(
                    base_name,
                    float(data.qpos[timeline_actuators.base_qposadr[base_name]]),
                )
                current_base_value = (1.0 - alpha_s) * start + alpha_s * base_target
                data.qpos[timeline_actuators.base_qposadr[base_name]] = current_base_value
                data.qvel[timeline_actuators.base_dofadr[base_name]] = 0.0
                data.ctrl[timeline_actuators.base_name_to_id[base_name]] = current_base_value

        if step.lift_target is not None:
            for lift_name, actuator_id in timeline_actuators.lift_name_to_id.items():
                start = st.start_lift
                if start is None:
                    start = float(data.qpos[timeline_actuators.lift_qposadr[lift_name]])
                current_lift_value = (1.0 - alpha_s) * start + alpha_s * step.lift_target.position
                data.qpos[timeline_actuators.lift_qposadr[lift_name]] = current_lift_value
                data.qvel[timeline_actuators.lift_dofadr[lift_name]] = 0.0
                data.ctrl[actuator_id] = current_lift_value

        if step.aux_ctrl:
            for aux_name, aux_target in step.aux_ctrl.items():
                aux_key = str(aux_name)
                aid = timeline_actuators.aux_name_to_id[aux_key]
                start = st.start_aux.get(aux_key, float(data.qpos[aux_qposadr[aux_key]]))
                curr_aux = (1.0 - alpha_s) * start + alpha_s * aux_target
                data.qpos[aux_qposadr[aux_key]] = curr_aux
                data.qvel[aux_dofadr[aux_key]] = 0.0
                data.ctrl[aid] = curr_aux

        label = step.label
        if alpha >= 1.0:
            st.start_q = step.arm_q.copy()
            st.start_g = tgt_g
            if step.base_target is not None:
                for base_name, base_target in zip(
                    timeline_actuators.base_name_to_id,
                    step.base_target.as_tuple(),
                    strict=True,
                ):
                    st.start_base[base_name] = base_target
            if step.lift_target is not None:
                st.start_lift = step.lift_target.position
            if step.aux_ctrl:
                for aux_name, aux_target in step.aux_ctrl.items():
                    st.start_aux[str(aux_name)] = aux_target
            st.step += 1
            st.t = 0.0
            return f"{st.step}/{len(script)} {label} ✓"
        return f"{st.step + 1}/{len(script)} {label}"

    def all_done() -> bool:
        if task_plan is None:
            return False
        return all(per_arm[side].step >= len(task_plan[side]) for side in arm_sides)

    def observed_scene_phase() -> TaskPhase | None:
        """Return the scene-wide phase only when every active arm agrees.

        Per-arm teleop recordings can be staggered. A phase contract is a
        global scene assertion, so do not evaluate a phase boundary while one
        arm is still finishing the previous phase.
        """
        if task_plan is None:
            return None
        active_phases: set[TaskPhase] = set()
        for side in arm_sides:
            script = task_plan[side]
            step_index = per_arm[side].step
            if step_index >= len(script):
                continue
            phase = script[step_index].phase
            if phase is not TaskPhase.UNPHASED:
                active_phases.add(phase)
        if len(active_phases) != 1:
            return None
        return next(iter(active_phases))

    print(f"Viser on {args.host}:{args.port} ({len(handles)} geoms)")
    print(
        f"If remote: `ssh -L {args.port}:localhost:{args.port} user@host` "
        f"then open http://localhost:{args.port}"
    )
    free_play_can_prewarm = callable(getattr(active_step_free_play, "prewarm", None))
    if free_play_can_prewarm and policy_mode:
        print("Pre-warming hosted policy with the initial observation...")
        prewarm_step_free_play(active_step_free_play, model, data)
        print("Policy pre-warm complete")
        print("Policy mode starts paused. Set/apply the prompt in the Viser UI, then press play.")
    elif free_play_can_prewarm:
        print("Pre-warming free-play controller with the initial scene state...")
        prewarm_step_free_play(active_step_free_play, model, data)
        print("Free-play pre-warm complete")
    if task_plan is not None:
        parts = ", ".join(f"{side}={len(task_plan[side])}" for side in arm_sides)
        print(f"Timeline: {parts} steps (run in parallel)")
        print("Scripted scenes start paused. Press play in the Viser UI to run.")

    next_tick = time.perf_counter()
    sim_t = 0.0

    plan_finished_announced = False

    try:
        with shutdown_signal_as_keyboard_interrupt():
            while True:
                if control["reset_requested"]:
                    restart()
                    sim_t = 0.0
                    plan_finished_announced = False
                    control["reset_requested"] = False

                # Teleop tick is unconditional: dragging a handle must move the
                # arm even when the sim is paused, since teleop is for authoring.
                if teleop_controller is not None:
                    teleop_controller.tick(render_dt)
                    gui_state.value = "teleop"
                elif control["playing"]:
                    if task_plan is not None:
                        phase_monitor.on_phase_observed(observed_scene_phase(), model, data)
                        for side in arm_sides:
                            per_arm_gui[side].value = advance_arm(side, render_dt)
                        if all_done():
                            if not plan_finished_announced:
                                phase_monitor.on_plan_finished(model, data)
                                plan_finished_announced = True
                            gui_state.value = "done — press reset"
                            control["playing"] = False
                        else:
                            gui_state.value = "running"
                    elif active_step_free_play is not None:
                        gui_state.value = "policy infer..."
                        try:
                            active_step_free_play(sim_t, model, data)
                        except Exception as err:
                            control["playing"] = False
                            gui_state.value = f"policy error: {type(err).__name__}"
                            print(f"\n[policy] {type(err).__name__}: {err}", flush=True)
                        else:
                            gui_state.value = "running"
                    else:
                        gui_state.value = "idle"
                else:
                    gui_state.value = "paused" if not all_done() else "done — press reset"

                for _ in range(phys_steps_per_frame):
                    mujoco.mj_step(model, data)
                    phase_monitor.on_tick(model, data)
                sim_t += render_dt

                update_viser(server, model, data, handles)
                if frustum_handles:
                    update_frustum_widgets(server, data, frustum_handles)
                if (
                    camera_feed_renderer is not None
                    and camera_feed_handles
                    and camera_feed_tick_counter["n"] % camera_feed_every == 0
                ):
                    for camera_name, image_handle in camera_feed_handles.items():
                        rendered_camera_feed_image = camera_feed_renderer.render(data, camera_name)
                        if scene.camera_feed.preprocess is not None:
                            rendered_camera_feed_image = scene.camera_feed.preprocess(
                                camera_name,
                                rendered_camera_feed_image,
                            )
                        image_handle.image = rendered_camera_feed_image
                camera_feed_tick_counter["n"] += 1

                # Log once per render tick: rerun is for user-visible debugging,
                # so per-physics-tick is wasteful.
                if rerun_streamer is not None:
                    rerun_streamer.set_sim_time(sim_t)
                    for side in arm_sides:
                        arm = arms[side]
                        qpos = np.asarray([data.qpos[i] for i in arm.arm_qpos_idx], dtype=float)
                        rerun_streamer.log_joint_scalars(
                            side_prefix=str(side),
                            joint_names=rerun_joint_names[side],
                            qpos=qpos,
                        )
                    for body_name, body_id in rerun_body_ids.items():
                        rerun_streamer.log_body_transform(
                            name=body_name,
                            xpos=np.asarray(data.xpos[body_id], dtype=float),
                            xquat=np.asarray(data.xquat[body_id], dtype=float),
                        )
                    rerun_tick_counter["n"] += 1

                if not args.max_rate:
                    next_tick += render_dt
                    sleep = next_tick - time.perf_counter()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("stopped")
    except PhaseContractViolation as exc:
        print(f"\n[strict] phase contract violation:\n  {exc}")
        sys.exit(2)
    finally:
        close_step_free_play(active_step_free_play)
        if camera_feed_renderer is not None:
            camera_feed_renderer.close()
        if phase_monitor.enabled and phase_monitor.failures:
            print(f"\nPhaseRuntimeMonitor: {len(phase_monitor.failures)} contract failure(s):")
            for failure in phase_monitor.failures:
                print(f"  [{failure.kind}] {failure.name}: {failure.message}")


if __name__ == "__main__":
    main()
