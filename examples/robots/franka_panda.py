"""Franka Emika Panda loaders.

Loads `franka_emika_panda/panda.xml` from mujoco_menagerie. The upstream
XML includes the stock Panda hand. DROID, however, uses a Franka arm with a
Robotiq 2F-85 gripper, so this module exposes both:

* `load_franka_panda` keeps the upstream Panda hand.
* `load_franka_panda_with_robotiq_2f85` removes the stock hand and attaches
  Menagerie's Robotiq 2F-85 at the Franka flange.

Conventions:

* 7 named revolute arm joints (`joint1`..`joint7`) and one `<general>`
  actuator per joint (`actuator1`..`actuator7`). Note the joint↔actuator
  suffix mismatch — Menagerie names actuators `actuatorN` rather than
  reusing the joint name as Piper does.
* Hand has two slide joints (`finger_joint1`, `finger_joint2`) coupled
  via the `split` tendon, driven by a single `actuator8`. ctrlrange is
  `0..255`, following the same convention as the Robotiq 2F-85 the
  upstream model borrowed from: 0 = fully open, 255 = fully closed.
* The upstream MJCF has no TCP site. We add one between the fingers
  inside the `hand` body so the runner's IK + grasp-detector code has
  a stable site to read the jaw-center pose from.
* After attaching with `model="left"` (or any namespace), every
  panda joint, body, actuator, etc. gets the prefixed name (e.g.
  `left/joint1`, `left/actuator8`, `left/hand`, `left/tcp`).
* The Robotiq variant compiles its gripper under `left/gripper/...`, with
  `left/gripper/fingers_actuator` and `left/gripper/pinch`.

Single-arm DROID is the only embodiment using this today; bilateral
ALOHA Phase 2 uses ViperX (`trossen_vx300s`) instead of Franka.
"""

from __future__ import annotations

from dm_control import mjcf

from examples.paths import FRANKA_PANDA_XML, ROBOTIQ_2F85_XML
from mujoco_workbench.arm_handles import ArmSide

FRANKA_ARM_JOINT_NAMES: tuple[str, ...] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
)
"""7 arm joints in canonical chain order."""

FRANKA_ARM_ACTUATOR_NAMES: tuple[str, ...] = (
    "actuator1",
    "actuator2",
    "actuator3",
    "actuator4",
    "actuator5",
    "actuator6",
    "actuator7",
)
"""Position-style `<general>` actuators driving the 7 arm joints."""

GRIPPER_ACTUATOR_NAME = "actuator8"
"""Single tendon-driven actuator on the hand. ctrlrange 0..255 with
0 = fully open, 255 = fully closed."""

GRIPPER_FINGER_JOINT_NAMES: tuple[str, str] = ("finger_joint1", "finger_joint2")
"""Two slide joints, equality-coupled and tendon-coupled to actuator8."""

ROBOTIQ_GRIPPER_MODEL_NAME = "gripper"
ROBOTIQ_GRIPPER_ACTUATOR_NAME = "gripper/fingers_actuator"
ROBOTIQ_GRIPPER_BASE_BODY_NAME = "gripper/base"
ROBOTIQ_GRIPPER_PINCH_SITE_NAME = "gripper/pinch"

_ROBOTIQ_MOUNT_SITE_NAME = "robotiq_mount"
_ROBOTIQ_MOUNT_POSITION_IN_LINK7_FRAME: tuple[float, float, float] = (0.0, 0.0, 0.107)
_ROBOTIQ_MOUNT_QUATERNION_IN_LINK7_FRAME: tuple[float, float, float, float] = (
    0.9238795,
    0.0,
    0.0,
    -0.3826834,
)

_FRANKA_REQUIRED_BODIES: tuple[str, ...] = (
    "link0",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "link7",
    "hand",
    "left_finger",
    "right_finger",
)

_FRANKA_ARM_REQUIRED_BODIES: tuple[str, ...] = tuple(
    body_name for body_name in _FRANKA_REQUIRED_BODIES if body_name != "hand"
)


def _assert_menagerie_shape(panda: mjcf.RootElement) -> None:
    """Fail at load time if upstream renamed something we depend on."""
    for name in _FRANKA_REQUIRED_BODIES:
        if panda.find("body", name) is None:
            raise RuntimeError(
                f"Franka upstream XML missing expected body {name!r}. "
                "Menagerie's franka_emika_panda/panda.xml may have changed shape — "
                "update examples/robots/franka_panda.py if so."
            )
    for jname in FRANKA_ARM_JOINT_NAMES:
        if panda.find("joint", jname) is None:
            raise RuntimeError(f"Franka upstream XML missing expected joint {jname!r}.")
    for fname in GRIPPER_FINGER_JOINT_NAMES:
        if panda.find("joint", fname) is None:
            raise RuntimeError(f"Franka upstream XML missing expected finger joint {fname!r}.")
    for aname in (*FRANKA_ARM_ACTUATOR_NAMES, GRIPPER_ACTUATOR_NAME):
        if panda.find("actuator", aname) is None:
            raise RuntimeError(f"Franka upstream XML missing expected actuator {aname!r}.")


def _assert_franka_arm_shape(panda: mjcf.RootElement) -> None:
    """Fail at load time if upstream renamed the arm pieces we attach to."""
    for name in _FRANKA_ARM_REQUIRED_BODIES:
        if panda.find("body", name) is None:
            raise RuntimeError(
                f"Franka upstream XML missing expected body {name!r}. "
                "Menagerie's franka_emika_panda/panda.xml may have changed shape — "
                "update examples/robots/franka_panda.py if so."
            )
    for joint_name in FRANKA_ARM_JOINT_NAMES:
        if panda.find("joint", joint_name) is None:
            raise RuntimeError(f"Franka upstream XML missing expected joint {joint_name!r}.")
    for actuator_name in FRANKA_ARM_ACTUATOR_NAMES:
        if panda.find("actuator", actuator_name) is None:
            raise RuntimeError(f"Franka upstream XML missing expected actuator {actuator_name!r}.")


def _add_tcp_site(panda: mjcf.RootElement) -> None:
    """Add a `tcp` site between the two fingers, anchored to `hand`.

    `hand` sits 107 mm below `link7`'s frame; the fingers extend a
    further 58.4 mm along +z and slide outward up to 40 mm. The
    grasp-jaw centroid lives ~40 mm beyond the finger pads. We anchor
    the TCP site at z = 0.103 m relative to `hand` so it tracks the
    kinematic point a policy would care about ("where the gripper
    will close").
    """
    hand = panda.find("body", "hand")
    if hand is None:
        raise RuntimeError(
            "hand body missing — cannot add TCP site (assert pass should have caught)."
        )
    hand.add(
        "site",
        name="tcp",
        pos=(0.0, 0.0, 0.103),
        size=(0.005, 0.005, 0.005),
        rgba=(1.0, 0.0, 1.0, 0.0),  # invisible by default; flip alpha for debugging
        group=4,
    )


def _remove_stock_panda_hand(panda: mjcf.RootElement) -> None:
    """Remove the upstream Panda hand and the elements that reference it."""
    for actuator in list(panda.find_all("actuator")):
        if getattr(actuator, "name", None) == GRIPPER_ACTUATOR_NAME:
            actuator.remove()
    for tendon in list(panda.find_all("tendon")):
        if getattr(tendon, "name", None) == "split":
            tendon.remove()
    for equality in list(panda.find_all("equality")):
        if any(
            finger_joint_name in str(equality) for finger_joint_name in GRIPPER_FINGER_JOINT_NAMES
        ):
            equality.remove()

    hand = panda.find("body", "hand")
    if hand is None:
        raise RuntimeError("Franka upstream XML missing stock hand body; cannot remove it.")
    hand.remove()


def _attach_robotiq_2f85(panda: mjcf.RootElement) -> None:
    """Attach Menagerie's Robotiq 2F-85 to the same flange pose as Panda hand."""
    link7 = panda.find("body", "link7")
    if link7 is None:
        raise RuntimeError("Franka upstream XML missing link7; cannot attach Robotiq gripper.")
    robotiq_mount_site = link7.add(
        "site",
        name=_ROBOTIQ_MOUNT_SITE_NAME,
        pos=_ROBOTIQ_MOUNT_POSITION_IN_LINK7_FRAME,
        quat=_ROBOTIQ_MOUNT_QUATERNION_IN_LINK7_FRAME,
        size=(0.005, 0.005, 0.005),
    )
    robotiq_gripper = mjcf.from_path(str(ROBOTIQ_2F85_XML))
    robotiq_gripper.model = ROBOTIQ_GRIPPER_MODEL_NAME
    robotiq_mount_site.attach(robotiq_gripper)

    if panda.find("body", ROBOTIQ_GRIPPER_BASE_BODY_NAME) is None:
        raise RuntimeError("Robotiq gripper attach did not expose expected base body.")
    if panda.find("site", ROBOTIQ_GRIPPER_PINCH_SITE_NAME) is None:
        raise RuntimeError("Robotiq gripper attach did not expose expected pinch site.")
    if panda.find("actuator", ROBOTIQ_GRIPPER_ACTUATOR_NAME) is None:
        raise RuntimeError("Robotiq gripper attach did not expose expected actuator.")


def load_franka_panda(side: ArmSide) -> mjcf.RootElement:
    """Load Menagerie's Franka Panda + hand and namespace under `side`.

    Returns the `mjcf.RootElement` *without* attaching it to a parent.
    Caller is expected to mutate the subtree (add a wrist camera, etc.)
    BEFORE calling `parent_site.attach(root)`. Anything added inside
    the subtree before attach inherits the namespace prefix.
    """
    panda = mjcf.from_path(str(FRANKA_PANDA_XML))
    panda.model = side.rstrip("/")
    _assert_menagerie_shape(panda)
    _add_tcp_site(panda)
    return panda


def load_franka_panda_with_robotiq_2f85(side: ArmSide) -> mjcf.RootElement:
    """Load Franka Panda arm with a Robotiq 2F-85 gripper for DROID scenes."""
    panda = mjcf.from_path(str(FRANKA_PANDA_XML))
    panda.model = side.rstrip("/")
    _assert_franka_arm_shape(panda)
    _remove_stock_panda_hand(panda)
    _attach_robotiq_2f85(panda)
    return panda
