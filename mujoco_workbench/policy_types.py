"""Domain types for hosted policy integration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import NewType

PolicyHost = NewType("PolicyHost", str)
PolicyPort = NewType("PolicyPort", int)
PolicyLocalPort = NewType("PolicyLocalPort", int)
PolicyPrompt = NewType("PolicyPrompt", str)


class PolicyActionInterpretation(StrEnum):
    """How the simulator interprets rows returned under `response["actions"]`."""

    DIRECT_ACTUATOR_CTRL = "direct_actuator_ctrl"
    DROID_ABSOLUTE_JOINT_POSITION = "droid_absolute_joint_position"
    DROID_JOINT_DELTA = "droid_joint_delta"
    DROID_JOINT_VELOCITY = "droid_joint_velocity"


@dataclass(frozen=True)
class PolicyEndpoint:
    """Parsed hosted-policy network endpoint."""

    host: PolicyHost
    port: PolicyPort
    local_port: PolicyLocalPort


def make_policy_endpoint(*, host: str, port: int, local_port: int = 5556) -> PolicyEndpoint:
    """Parse CLI endpoint primitives into a refined policy endpoint."""
    trimmed_host = host.strip()
    if not trimmed_host:
        raise ValueError("policy host must not be empty")
    if not 1 <= port <= 65535:
        raise ValueError(f"policy port must be in 1..65535, got {port}")
    if not 1 <= local_port <= 65535:
        raise ValueError(f"policy local port must be in 1..65535, got {local_port}")
    return PolicyEndpoint(
        host=PolicyHost(trimmed_host),
        port=PolicyPort(port),
        local_port=PolicyLocalPort(local_port),
    )


def make_policy_prompt(raw_prompt: str) -> PolicyPrompt:
    """Parse a language instruction for policy inference."""
    trimmed_prompt = raw_prompt.strip()
    if not trimmed_prompt:
        raise ValueError("policy prompt must not be empty")
    return PolicyPrompt(trimmed_prompt)


def parse_policy_action_interpretation(raw_value: str) -> PolicyActionInterpretation:
    """Parse an action interpretation name from external config or CLI input."""
    try:
        return PolicyActionInterpretation(raw_value)
    except ValueError as err:
        valid_values = ", ".join(value.value for value in PolicyActionInterpretation)
        raise ValueError(
            f"unsupported policy action interpretation {raw_value!r}; expected one of: "
            f"{valid_values}"
        ) from err
