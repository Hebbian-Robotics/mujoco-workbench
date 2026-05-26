"""Small cached wrapper around MuJoCo's offscreen renderer."""

from __future__ import annotations

import contextlib

import mujoco
import numpy as np


class NamedCameraRenderer:
    """Render named MuJoCo cameras at a fixed resolution.

    `mujoco.Renderer` owns GL resources, so constructing one every policy tick
    is expensive. This wrapper keeps one renderer per camera id and exposes a
    simple `render(camera_name)` callable for observation builders.
    """

    def __init__(self, model: mujoco.MjModel, *, width: int = 224, height: int = 224) -> None:
        self._model = model
        self._width = width
        self._height = height
        self._camera_id_by_name: dict[str, int] = {}
        self._renderer_by_camera_id: dict[int, mujoco.Renderer] = {}

    def render(self, data: mujoco.MjData, camera_name: str) -> np.ndarray:
        camera_id = self._camera_id(camera_name)
        renderer = self._renderer_by_camera_id.get(camera_id)
        if renderer is None:
            renderer = mujoco.Renderer(self._model, height=self._height, width=self._width)
            self._renderer_by_camera_id[camera_id] = renderer
        renderer.update_scene(data, camera=camera_id)
        return np.asarray(renderer.render(), dtype=np.uint8)

    def close(self) -> None:
        for renderer in self._renderer_by_camera_id.values():
            with contextlib.suppress(AttributeError):
                renderer.close()
        self._renderer_by_camera_id.clear()

    def _camera_id(self, camera_name: str) -> int:
        cached_camera_id = self._camera_id_by_name.get(camera_name)
        if cached_camera_id is not None:
            return cached_camera_id
        camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            available_camera_names = [
                mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_CAMERA, i) or f"camera_{i}"
                for i in range(self._model.ncam)
            ]
            raise ValueError(f"unknown camera {camera_name!r}; available: {available_camera_names}")
        self._camera_id_by_name[camera_name] = camera_id
        return camera_id
