"""Contract tests for optional OpenPI embodiment adapters."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

pytest.importorskip("openpi_client")

from examples.scenes import franka_libero_pi
from mujoco_workbench.arm_handles import (
    ArmHandles,
    ArmSide,
    RobotKind,
    get_arm_handles,
)
from mujoco_workbench.embodiments.droid import (
    DROID_OBSERVATION_KEYS,
    DroidCameraName,
    build_droid_observation,
    resize_droid_policy_image,
)
from mujoco_workbench.embodiments.libero import (
    LIBERO_OBSERVATION_KEYS,
    LiberoCameraName,
    build_libero_observation,
    libero_action_to_actuator_ctrl,
    resize_libero_policy_image,
)


def _make_droid_observation_model() -> mujoco.MjModel:
    chain_xml = """
              <body name="left/link1">
                <joint name="left/joint1" type="hinge"/>
                <geom type="sphere" size="0.01"/>
                <body name="left/link2">
                  <joint name="left/joint2" type="hinge"/>
                  <geom type="sphere" size="0.01"/>
                  <body name="left/link3">
                    <joint name="left/joint3" type="hinge"/>
                    <geom type="sphere" size="0.01"/>
                    <body name="left/link4">
                      <joint name="left/joint4" type="hinge"/>
                      <geom type="sphere" size="0.01"/>
                      <body name="left/link5">
                        <joint name="left/joint5" type="hinge"/>
                        <geom type="sphere" size="0.01"/>
                        <body name="left/link6">
                          <joint name="left/joint6" type="hinge"/>
                          <geom type="sphere" size="0.01"/>
                          <body name="left/link7">
                            <joint name="left/joint7" type="hinge"/>
                            <joint name="left/finger_joint" type="slide"/>
                            <geom type="sphere" size="0.01"/>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
    """
    return mujoco.MjModel.from_xml_string(
        f"""
        <mujoco>
          <worldbody>
            <camera name="cam_exterior" pos="0 -1 1"/>
            <camera name="left/gripper/cam_wrist" pos="0 0 1"/>
            {chain_xml}
          </worldbody>
          <actuator>
            <position
              name="left/gripper/fingers_actuator"
              joint="left/finger_joint"
              ctrlrange="0 255"
            />
          </actuator>
        </mujoco>
        """
    )


def test_droid_observation_matches_hosted_warmup_schema() -> None:
    model = _make_droid_observation_model()
    data = mujoco.MjData(model)
    expected_joint_positions = np.arange(7, dtype=float)
    for joint_index, joint_position in enumerate(expected_joint_positions, start=1):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"left/joint{joint_index}")
        data.qpos[int(model.jnt_qposadr[joint_id])] = joint_position
    data.ctrl[0] = 127.5

    def render_camera(camera_name: DroidCameraName) -> np.ndarray:
        if camera_name is DroidCameraName.EXTERIOR:
            return np.zeros((224, 224, 3), dtype=np.uint8)
        if camera_name is DroidCameraName.LEFT_WRIST:
            return np.ones((224, 224, 3), dtype=np.uint8)
        raise AssertionError(f"unexpected camera {camera_name}")

    observation = build_droid_observation(
        model,
        data,
        render_camera,
        "pick up the red cube",
    )

    assert tuple(observation) == DROID_OBSERVATION_KEYS
    assert observation["observation/exterior_image_1_left"].shape == (224, 224, 3)
    assert observation["observation/exterior_image_1_left"].dtype == np.uint8
    assert observation["observation/wrist_image_left"].shape == (224, 224, 3)
    assert observation["observation/wrist_image_left"].dtype == np.uint8
    np.testing.assert_allclose(
        observation["observation/joint_position"],
        expected_joint_positions,
    )
    np.testing.assert_allclose(observation["observation/gripper_position"], np.array([0.5]))
    assert observation["prompt"] == "pick up the red cube"
    assert observation["mode"] == "action_only"


def test_droid_policy_image_resize_pads_sixteen_by_nine_frames() -> None:
    rendered_image = np.full((180, 320, 3), 127, dtype=np.uint8)

    policy_image = resize_droid_policy_image(rendered_image)

    assert policy_image.shape == (224, 224, 3)
    assert policy_image.dtype == np.uint8
    np.testing.assert_array_equal(policy_image[:49], 0)
    np.testing.assert_array_equal(policy_image[49:175], 127)
    np.testing.assert_array_equal(policy_image[175:], 0)


def test_droid_observation_rejects_wrong_image_shape() -> None:
    model = _make_droid_observation_model()
    data = mujoco.MjData(model)

    with pytest.raises(ValueError, match="observation/exterior_image_1_left"):
        build_droid_observation(
            model,
            data,
            lambda _camera_name: np.zeros((64, 64, 3), dtype=np.uint8),
            "prompt",
        )


def _make_libero_observation_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="left/hand" pos="0.1 0.2 0.3">
              <joint name="left/finger_joint1" type="slide" range="0 0.04"/>
              <joint name="left/finger_joint2" type="slide" range="0 0.04"/>
              <site name="left/tcp" pos="0 0 0.1"/>
              <geom type="sphere" size="0.01"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def _fake_libero_arm(model: mujoco.MjModel) -> ArmHandles:
    tcp_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left/tcp")
    return ArmHandles(
        side=ArmSide.LEFT,
        robot_kind=RobotKind.FRANKA_PANDA,
        qpos_idx=np.zeros(7, dtype=np.int64),
        dof_idx=np.zeros(7, dtype=np.int64),
        jnt_ids=np.zeros(7, dtype=np.int64),
        arm_dof_idx=np.zeros(7, dtype=np.int64),
        act_arm_ids=np.zeros(7, dtype=np.int64),
        act_gripper_id=0,
        link6_id=0,
        tcp_site_id=tcp_site_id,
        gripper_open=255.0,
        gripper_closed=0.0,
        weld_ids=np.zeros(0, dtype=np.int64),
    )


def test_libero_observation_matches_openpi_schema() -> None:
    model = _make_libero_observation_model()
    data = mujoco.MjData(model)
    first_finger_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left/finger_joint1")
    second_finger_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left/finger_joint2")
    data.qpos[int(model.jnt_qposadr[first_finger_id])] = 0.04
    data.qpos[int(model.jnt_qposadr[second_finger_id])] = 0.02
    mujoco.mj_forward(model, data)

    def render_camera(camera_name: LiberoCameraName) -> np.ndarray:
        if camera_name is LiberoCameraName.AGENTVIEW:
            return np.zeros((224, 224, 3), dtype=np.uint8)
        if camera_name is LiberoCameraName.WRIST:
            return np.ones((224, 224, 3), dtype=np.uint8)
        raise AssertionError(f"unexpected camera {camera_name}")

    observation = build_libero_observation(
        model,
        data,
        _fake_libero_arm(model),
        render_camera,
        "pick up the red cube",
    )

    assert tuple(observation) == LIBERO_OBSERVATION_KEYS
    assert observation["observation/image"].shape == (224, 224, 3)
    assert observation["observation/wrist_image"].shape == (224, 224, 3)
    assert observation["observation/state"].shape == (8,)
    expected_tcp_position = np.asarray(data.site_xpos[_fake_libero_arm(model).tcp_site_id])
    np.testing.assert_allclose(observation["observation/state"][:3], expected_tcp_position)
    np.testing.assert_allclose(observation["observation/state"][3:6], np.zeros(3))
    np.testing.assert_allclose(observation["observation/state"][6:], np.array([0.04, 0.02]))
    assert observation["prompt"] == "pick up the red cube"


def test_libero_policy_image_resize_rotates_training_frames() -> None:
    rendered_image = np.zeros((224, 224, 3), dtype=np.uint8)
    rendered_image[0, 0] = np.array([255, 0, 0], dtype=np.uint8)
    rendered_image[-1, -1] = np.array([0, 255, 0], dtype=np.uint8)

    policy_image = resize_libero_policy_image(rendered_image)

    assert policy_image.shape == (224, 224, 3)
    assert policy_image.dtype == np.uint8
    np.testing.assert_array_equal(policy_image[0, 0], np.array([0, 255, 0], dtype=np.uint8))
    np.testing.assert_array_equal(policy_image[-1, -1], np.array([255, 0, 0], dtype=np.uint8))


def test_libero_action_mapping_applies_incremental_robosuite_gripper_commands() -> None:
    model, data = franka_libero_pi.build_spec()
    arm = get_arm_handles(model, franka_libero_pi.MANIPULATORS[0], franka_libero_pi.N_CUBES)
    franka_libero_pi.apply_initial_state(model, data, {ArmSide.LEFT: arm}, [])
    scratch_data = mujoco.MjData(model)

    hold_action = np.zeros(7, dtype=float)
    hold_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, hold_action)
    np.testing.assert_allclose(hold_ctrl[arm.act_gripper_id], arm.gripper_open)

    close_action = np.zeros(7, dtype=float)
    close_action[-1] = 1.0
    close_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, close_action)
    expected_close_step = arm.gripper_open + 0.1 * (arm.gripper_closed - arm.gripper_open)
    np.testing.assert_allclose(close_ctrl[arm.act_gripper_id], expected_close_step)

    data.ctrl[arm.act_gripper_id] = arm.gripper_closed
    open_action = np.zeros(7, dtype=float)
    open_action[-1] = -1.0
    open_ctrl = libero_action_to_actuator_ctrl(model, data, scratch_data, arm, open_action)
    expected_open_step = arm.gripper_open + 0.9 * (arm.gripper_closed - arm.gripper_open)
    np.testing.assert_allclose(open_ctrl[arm.act_gripper_id], expected_open_step)
