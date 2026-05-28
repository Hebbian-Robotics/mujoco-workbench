"""Tests for the runtime keyframe / geom toggle helpers.

These cover the small, robot-agnostic utilities that scenes (and the new
`mwb view` command) use to load named keyframes without leaving stale
position-actuator targets, and to disable named geoms at load time without
authoring scene variants.
"""

from __future__ import annotations

import mujoco
import pytest

from mujoco_workbench.arm_handles import (
    ArmSide,
    GripperControlSpec,
    GripperPuppetJoint,
    ManipulatorSpec,
    get_arm_handles,
    write_gripper_target,
)
from mujoco_workbench.runtime import (
    apply_keyframe,
    disable_geom_collision,
    hide_geom,
    sync_position_actuators_to_qpos,
)

# Minimal MJCF with two hinge joints, two position actuators, and one named
# keyframe. `joint1` lives in body `link1`; `joint2` in body `link2`.
_KEYFRAME_XML = """
<mujoco model="kf_test">
  <worldbody>
    <body name="link1">
      <joint name="joint1" type="hinge" axis="0 0 1" range="-1 1"/>
      <geom name="g1" type="box" size="0.05 0.05 0.05" contype="1" conaffinity="1" rgba="1 0 0 1"/>
      <body name="link2" pos="0.1 0 0">
        <joint name="joint2" type="hinge" axis="0 1 0" range="-1 1"/>
        <geom name="g2" type="box" size="0.05 0.05 0.05" contype="2" conaffinity="3" rgba="0 1 0 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="a1" joint="joint1" kp="10"/>
    <position name="a2" joint="joint2" kp="10"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.3 -0.4"/>
  </keyframe>
</mujoco>
"""

_EXPLICIT_MANIPULATOR_XML = """
<mujoco model="explicit_test">
  <worldbody>
    <body name="robot_base">
      <joint name="arm_joint1" type="hinge" range="-1 1"/>
      <geom type="sphere" size="0.01"/>
      <body name="robot_wrist">
        <joint name="arm_joint2" type="hinge" range="-1 1"/>
        <joint name="finger_left" type="slide" range="-1 1"/>
        <joint name="finger_right" type="slide" range="-1 1"/>
        <geom type="sphere" size="0.01"/>
        <site name="robot_tcp" size="0.01"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="arm_ctrl1" joint="arm_joint1"/>
    <position name="arm_ctrl2" joint="arm_joint2"/>
    <position name="gripper_ctrl" joint="finger_left"/>
  </actuator>
</mujoco>
"""


def _load_keyframe_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(_KEYFRAME_XML)
    data = mujoco.MjData(model)
    return model, data


def _explicit_manipulator_spec() -> ManipulatorSpec:
    return ManipulatorSpec(
        side=ArmSide.LEFT,
        name="explicit_test_arm",
        joint_names=("arm_joint1", "arm_joint2"),
        arm_actuator_names=("arm_ctrl1", "arm_ctrl2"),
        gripper=GripperControlSpec(
            actuator_name="gripper_ctrl",
            open_ctrl=0.5,
            closed_ctrl=0.0,
            puppet_joints=(
                GripperPuppetJoint("finger_left", scale=1.0),
                GripperPuppetJoint("finger_right", scale=-1.0),
            ),
        ),
        wrist_body_name="robot_wrist",
        tcp_site_name="robot_tcp",
        base_body_name="robot_base",
        joint_labels=("shoulder", "elbow"),
    )


def test_sync_position_actuators_to_qpos_copies_each_actuated_joint() -> None:
    model, data = _load_keyframe_model()
    data.qpos[:] = [0.25, -0.5]
    data.ctrl[:] = 0.0
    sync_position_actuators_to_qpos(model, data)
    assert data.ctrl[0] == pytest.approx(0.25)
    assert data.ctrl[1] == pytest.approx(-0.5)


def test_explicit_manipulator_names_resolve_without_side_prefix() -> None:
    model = mujoco.MjModel.from_xml_string(_EXPLICIT_MANIPULATOR_XML)
    arm = get_arm_handles(model, _explicit_manipulator_spec(), n_cubes=0)
    assert arm.joint_names == ("arm_joint1", "arm_joint2")
    assert arm.joint_labels == ("shoulder", "elbow")
    assert arm.tcp_site_name == "robot_tcp"


def test_write_gripper_target_applies_declared_puppet_joints() -> None:
    model = mujoco.MjModel.from_xml_string(_EXPLICIT_MANIPULATOR_XML)
    data = mujoco.MjData(model)
    arm = get_arm_handles(model, _explicit_manipulator_spec(), n_cubes=0)
    write_gripper_target(data, arm, 0.4)
    assert data.ctrl[arm.act_gripper_id] == pytest.approx(0.4)
    assert data.qpos[arm.gripper_puppet_joints[0].qpos_idx] == pytest.approx(0.4)
    assert data.qpos[arm.gripper_puppet_joints[1].qpos_idx] == pytest.approx(-0.4)


def test_apply_keyframe_resets_qpos_and_syncs_ctrl() -> None:
    model, data = _load_keyframe_model()
    data.ctrl[:] = 99.0
    apply_keyframe(model, data, "home")
    assert data.qpos[0] == pytest.approx(0.3)
    assert data.qpos[1] == pytest.approx(-0.4)
    # Without ctrl-sync the position actuators would still hold the stale 99.0
    # targets and immediately drag the model back. Verifying the sync ran.
    assert data.ctrl[0] == pytest.approx(0.3)
    assert data.ctrl[1] == pytest.approx(-0.4)


def test_apply_keyframe_by_index_works() -> None:
    model, data = _load_keyframe_model()
    apply_keyframe(model, data, 0)
    assert data.qpos[0] == pytest.approx(0.3)


def test_apply_keyframe_rejects_unknown_name() -> None:
    model, data = _load_keyframe_model()
    with pytest.raises(ValueError, match="unknown keyframe 'nope'"):
        apply_keyframe(model, data, "nope")


def test_apply_keyframe_rejects_out_of_range_index() -> None:
    model, data = _load_keyframe_model()
    with pytest.raises(ValueError, match="out of range"):
        apply_keyframe(model, data, 99)


def test_disable_geom_collision_zeros_masks() -> None:
    model, _ = _load_keyframe_model()
    disable_geom_collision(model, "g2")
    g2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "g2")
    assert model.geom_contype[g2_id] == 0
    assert model.geom_conaffinity[g2_id] == 0
    # g1 is untouched.
    g1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "g1")
    assert model.geom_contype[g1_id] == 1
    assert model.geom_conaffinity[g1_id] == 1


def test_disable_geom_collision_rejects_unknown_geom() -> None:
    model, _ = _load_keyframe_model()
    with pytest.raises(ValueError, match="unknown geom 'ghost'"):
        disable_geom_collision(model, "ghost")


def test_hide_geom_zeros_alpha() -> None:
    model, _ = _load_keyframe_model()
    g1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "g1")
    hide_geom(model, "g1")
    assert model.geom_rgba[g1_id, 3] == pytest.approx(0.0)


def test_hide_geom_rejects_unknown_geom() -> None:
    model, _ = _load_keyframe_model()
    with pytest.raises(ValueError, match="unknown geom 'ghost'"):
        hide_geom(model, "ghost")
