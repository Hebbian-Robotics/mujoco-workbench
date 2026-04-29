"""Single CLI entrypoint for mujoco-workbench."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

import typer

from examples import video_export as example_video_export
from mujoco_workbench import debug_cli

app = typer.Typer(
    help="Run MuJoCo workbench scenes and debug tools.",
    no_args_is_help=True,
    rich_markup_mode="rich",
    context_settings={"help_option_names": ["-h", "--help"]},
)
app.add_typer(debug_cli.app, name="debug", help="Render, inspect, diff, and check scene artifacts.")


@app.command()
def run(
    scene: Annotated[
        str,
        typer.Argument(help="Fully qualified scene module."),
    ],
    host: Annotated[str, typer.Option(help="Viser bind host.")] = "127.0.0.1",
    port: Annotated[int, typer.Option(help="Viser port.")] = 8080,
    speed: Annotated[float, typer.Option(help="Multiplier on scripted step durations.")] = 1.0,
    render_hz: Annotated[float, typer.Option(help="Viser and physics update cap.")] = 45.0,
    max_rate: Annotated[bool, typer.Option(help="Run as fast as MuJoCo can step.")] = False,
    inspect: Annotated[
        bool, typer.Option(help="Compile, check, print schematic, and exit.")
    ] = False,
    strict: Annotated[bool, typer.Option(help="Stop on first phase-contract failure.")] = False,
    teleop: Annotated[
        bool, typer.Option(help="Use browser drag handles instead of task plan.")
    ] = False,
    play_recording: Annotated[
        Path | None,
        typer.Option(help="Replay a teleop JSON recording."),
    ] = None,
    start_phase: Annotated[
        str | None,
        typer.Option(help="Boot from a scene-defined phase home."),
    ] = None,
    rerun_port: Annotated[
        int | None,
        typer.Option(help="Serve a local Rerun gRPC stream on this port."),
    ] = None,
    rerun_connect: Annotated[
        str | None,
        typer.Option(help="Push to an already-running Rerun viewer URL."),
    ] = None,
    rerun_rrd: Annotated[
        Path | None,
        typer.Option(help="Write an offline Rerun .rrd recording."),
    ] = None,
    rerun_camera_every: Annotated[
        int,
        typer.Option(help="Include named-camera frames every N render ticks; 0 disables."),
    ] = 0,
) -> None:
    """Run an interactive Viser scene."""
    from mujoco_workbench import runner

    argv = [
        "--scene",
        scene,
        "--host",
        host,
        "--port",
        str(port),
        "--speed",
        str(speed),
        "--render-hz",
        str(render_hz),
        "--rerun-camera-every",
        str(rerun_camera_every),
    ]
    if max_rate:
        argv.append("--max-rate")
    if inspect:
        argv.append("--inspect")
    if strict:
        argv.append("--strict")
    if teleop:
        argv.append("--teleop")
    if play_recording is not None:
        argv.extend(["--play-recording", str(play_recording)])
    if start_phase is not None:
        argv.extend(["--start-phase", start_phase])
    if rerun_port is not None:
        argv.extend(["--rerun-port", str(rerun_port)])
    if rerun_connect is not None:
        argv.extend(["--rerun-connect", rerun_connect])
    if rerun_rrd is not None:
        argv.extend(["--rerun-rrd", str(rerun_rrd)])
    runner.main(argv)


@app.command("video-export")
def video_export(
    out_dir: Annotated[Path, typer.Option(help="Directory for the four .mp4 files")] = Path(
        "/tmp/pov"
    ),
    fps: Annotated[int, typer.Option(min=1, help="Playback fps")] = 30,
    width: Annotated[int, typer.Option(min=1, help="Render width px")] = 1920,
    height: Annotated[int, typer.Option(min=1, help="Render height px")] = 1080,
    duration_s: Annotated[
        float,
        typer.Option(min=0.0, help="Sim seconds to render; 0 means full task plan length"),
    ] = 0.0,
    crf: Annotated[
        int,
        typer.Option(min=0, max=51, help="libx264 CRF; lower is better"),
    ] = 18,
    preset: Annotated[
        example_video_export.FfmpegPreset,
        typer.Option(help="libx264 preset"),
    ] = example_video_export.FfmpegPreset.SLOWER,
) -> None:
    """Export the example indicator-check scene as multi-camera videos."""
    example_video_export.main(
        out_dir=out_dir,
        fps=fps,
        width=width,
        height=height,
        duration_s=duration_s,
        crf=crf,
        preset=preset,
    )


def main() -> None:
    app()
