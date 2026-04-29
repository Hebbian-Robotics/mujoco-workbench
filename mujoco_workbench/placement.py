"""Reusable camera and interaction-placement geometry.

The helpers here capture the math that otherwise tends to get hand-tuned in
scene modules: aim a MuJoCo camera at a target, pick a point just outside an
object surface, and back a mount/TCP away from that target by a fixed standoff.
They are intentionally model-light so agents can use them before reaching for
teleop.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_workbench.scene_base import Position3


@dataclass(frozen=True)
class WorldAabb:
    """World-space axis-aligned bound for one object."""

    name: str
    min_xyz: Position3
    max_xyz: Position3

    @property
    def center(self) -> Position3:
        return (self.min_xyz + self.max_xyz) / 2.0

    @property
    def half_extent(self) -> Position3:
        return (self.max_xyz - self.min_xyz) / 2.0


def normalized_vector(raw_vector: np.ndarray, *, label: str) -> Position3:
    """Return `raw_vector / ||raw_vector||`, rejecting near-zero vectors."""
    vector = np.asarray(raw_vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError(f"{label} vector is too small to normalize")
    return vector / norm


def camera_xyaxes_for_look_at(
    camera_position: Position3,
    target_position: Position3,
    *,
    up_hint: Position3 | None = None,
) -> tuple[float, float, float, float, float, float]:
    """Return MuJoCo `<camera xyaxes=...>` values that aim at `target_position`.

    MuJoCo cameras look along local `-z`; `xyaxes` stores local camera `+x`
    then `+y` in the parent frame. `up_hint` chooses the visual horizon but is
    automatically replaced when it is almost parallel to the look direction.
    """
    camera_pos = np.asarray(camera_position, dtype=float)
    target_pos = np.asarray(target_position, dtype=float)
    look_direction = normalized_vector(target_pos - camera_pos, label="look")
    camera_z_axis = -look_direction
    up = normalized_vector(
        np.asarray((0.0, 0.0, 1.0), dtype=float) if up_hint is None else np.asarray(up_hint),
        label="up_hint",
    )
    if abs(float(np.dot(up, camera_z_axis))) > 0.98:
        up = np.asarray((0.0, 1.0, 0.0), dtype=float)
        if abs(float(np.dot(up, camera_z_axis))) > 0.98:
            up = np.asarray((1.0, 0.0, 0.0), dtype=float)

    camera_x_axis = normalized_vector(np.cross(up, camera_z_axis), label="camera_x")
    camera_y_axis = normalized_vector(np.cross(camera_z_axis, camera_x_axis), label="camera_y")
    return (
        float(camera_x_axis[0]),
        float(camera_x_axis[1]),
        float(camera_x_axis[2]),
        float(camera_y_axis[0]),
        float(camera_y_axis[1]),
        float(camera_y_axis[2]),
    )


def standoff_position(
    target_position: Position3,
    approach_direction: Position3,
    *,
    distance_m: float,
) -> Position3:
    """Place a camera/TCP `distance_m` before a target along an approach ray.

    `approach_direction` points from the camera/TCP toward the target. The
    returned point is therefore `target - direction * distance`.
    """
    if distance_m < 0.0:
        raise ValueError("distance_m must be non-negative")
    direction = normalized_vector(np.asarray(approach_direction, dtype=float), label="approach")
    return np.asarray(target_position, dtype=float) - direction * distance_m


def surface_point_from_aabb(
    aabb: WorldAabb,
    outward_normal: Position3,
    *,
    clearance_m: float = 0.0,
) -> Position3:
    """Return a point just outside an AABB face along `outward_normal`.

    This is the generic version of "touch the server bezel without clipping":
    choose the object surface normal facing the robot, then add a small positive
    clearance for TCP/camera standoff.
    """
    if clearance_m < 0.0:
        raise ValueError("clearance_m must be non-negative")
    normal = normalized_vector(np.asarray(outward_normal, dtype=float), label="outward_normal")
    half_distance_along_normal = float(np.dot(np.abs(normal), aabb.half_extent))
    return aabb.center + normal * (half_distance_along_normal + clearance_m)


def geom_world_aabb(model: mujoco.MjModel, data: mujoco.MjData, geom_name: str) -> WorldAabb:
    """Compute a conservative world-space AABB for one compiled MuJoCo geom."""
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        raise ValueError(f"geom {geom_name!r} not found")
    xpos = np.asarray(data.geom_xpos[geom_id], dtype=float)
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    geom_type = int(model.geom_type[geom_id])
    geom_size = np.asarray(model.geom_size[geom_id], dtype=float)

    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        radius = float(model.geom_rbound[geom_id])
        half_world = np.array([radius, radius, radius], dtype=float)
    elif geom_type == mujoco.mjtGeom.mjGEOM_BOX or geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        half_world = np.abs(xmat) @ geom_size
    elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        radius = float(geom_size[0])
        half_world = np.array([radius, radius, radius], dtype=float)
    elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER or geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        radius = float(geom_size[0])
        half_length = float(geom_size[1])
        local_half = np.array([radius, radius, half_length + radius], dtype=float)
        half_world = np.abs(xmat) @ local_half
    else:
        half_world = np.zeros(3, dtype=float)

    return WorldAabb(name=geom_name, min_xyz=xpos - half_world, max_xyz=xpos + half_world)


def body_world_aabb(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> WorldAabb:
    """Compute a conservative world-space AABB over geoms directly on a body."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"body {body_name!r} not found")
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != body_id:
            continue
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"g{geom_id}"
        geom_aabb = geom_world_aabb(model, data, geom_name)
        mins = np.minimum(mins, geom_aabb.min_xyz)
        maxs = np.maximum(maxs, geom_aabb.max_xyz)
    if not np.all(np.isfinite(mins)):
        raise ValueError(f"body {body_name!r} has no direct geoms")
    return WorldAabb(name=body_name, min_xyz=mins, max_xyz=maxs)
