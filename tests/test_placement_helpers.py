from __future__ import annotations

import numpy as np
import pytest

from mujoco_workbench.placement import (
    WorldAabb,
    camera_xyaxes_for_look_at,
    standoff_position,
    surface_point_from_aabb,
)


def test_camera_xyaxes_aims_mujoco_negative_z_at_target() -> None:
    xyaxes = camera_xyaxes_for_look_at(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        up_hint=np.array([0.0, 0.0, 1.0]),
    )

    camera_x = np.array(xyaxes[:3])
    camera_y = np.array(xyaxes[3:])
    camera_z = np.cross(camera_x, camera_y)

    np.testing.assert_allclose(-camera_z, np.array([1.0, 0.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(camera_x, np.array([0.0, -1.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(camera_y, np.array([0.0, 0.0, 1.0]), atol=1e-9)


def test_camera_xyaxes_recovers_when_up_hint_is_parallel_to_look_direction() -> None:
    xyaxes = camera_xyaxes_for_look_at(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        up_hint=np.array([0.0, 0.0, 1.0]),
    )

    camera_x = np.array(xyaxes[:3])
    camera_y = np.array(xyaxes[3:])
    camera_z = np.cross(camera_x, camera_y)

    np.testing.assert_allclose(-camera_z, np.array([0.0, 0.0, 1.0]), atol=1e-9)
    assert np.isclose(np.linalg.norm(camera_x), 1.0)
    assert np.isclose(np.linalg.norm(camera_y), 1.0)


def test_standoff_position_backs_away_from_target() -> None:
    position = standoff_position(
        np.array([5.0, 1.0, 0.9]),
        np.array([0.0, 1.0, 0.0]),
        distance_m=0.25,
    )

    np.testing.assert_allclose(position, np.array([5.0, 0.75, 0.9]))


def test_surface_point_from_aabb_uses_outward_normal_and_clearance() -> None:
    aabb = WorldAabb(
        name="server",
        min_xyz=np.array([4.8, 0.7, 0.8]),
        max_xyz=np.array([5.2, 1.1, 1.0]),
    )

    point = surface_point_from_aabb(
        aabb,
        np.array([0.0, -1.0, 0.0]),
        clearance_m=0.02,
    )

    np.testing.assert_allclose(point, np.array([5.0, 0.68, 0.9]))


def test_standoff_rejects_negative_distance() -> None:
    with pytest.raises(ValueError, match="distance_m"):
        standoff_position(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            distance_m=-0.1,
        )
