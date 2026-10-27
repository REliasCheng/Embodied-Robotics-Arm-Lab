"""Read data from the existing SO-100 YAML without constructing hardware objects."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml  # type: ignore[import-untyped]

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
LEADER_TARGET = "lerobot.common.robot_devices.motors.gbot.GBotMotorsBus"
FOLLOWER_TARGET = "lerobot.common.robot_devices.motors.feetech.FeetechMotorsBus"
DEFAULT_CONFIG = Path("third_party/genkiarm/lerobot/configs/robot/so100.yaml")


class ConfigError(ValueError):
    """Invalid configuration data; no hardware has been accessed."""


@dataclass(frozen=True)
class Arm:
    order: tuple[str, ...]
    ids: tuple[int, ...]


@dataclass(frozen=True)
class Contract:
    leader: Arm
    follower: Arm


def _arm(root: dict[str, Any], key: str, target: str) -> Arm:
    group = root.get(key)
    if not isinstance(group, dict) or set(group) != {"main"}:
        raise ConfigError(f"{key}.main must be the sole arm")
    arm = group["main"]
    if not isinstance(arm, dict) or arm.get("_target_") != target:
        raise ConfigError(f"{key}.main backend mismatch")
    motors = arm.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(JOINTS):
        raise ConfigError(f"{key}.main motors missing or unknown channels")
    ids = []
    for name, definition in motors.items():
        if (not isinstance(definition, list) or len(definition) != 2
                or type(definition[0]) is not int or not 1 <= definition[0] <= 253
                or not isinstance(definition[1], str) or not definition[1]):
            raise ConfigError(f"invalid motor definition: {name}")
        ids.append(definition[0])
    if len(ids) != len(set(ids)):
        raise ConfigError(f"duplicate motor ID in {key}")
    return Arm(tuple(motors), tuple(ids))


def load_contract(path: Path) -> Contract:
    if path.stat().st_size > 64 * 1024:
        raise ConfigError("configuration too large")
    # BaseLoader avoids YAML object construction and also preserves duplicate-key detection.
    class UniqueLoader(yaml.SafeLoader):
        pass

    def unique_mapping(loader: yaml.SafeLoader, node: yaml.MappingNode) -> dict:
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node)
            if key in result:
                raise ConfigError(f"duplicate YAML key: {key}")
            result[key] = loader.construct_object(value_node)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    try:
        root = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
    except yaml.YAMLError as exc:
        raise ConfigError("invalid YAML") from exc
    if not isinstance(root, dict) or root.get("max_relative_target", object()) is not None:
        raise ConfigError("expected max_relative_target: null; no physical limit is inferred")
    leader = _arm(root, "leader_arms", LEADER_TARGET)
    follower = _arm(root, "follower_arms", FOLLOWER_TARGET)
    if set(leader.order) != set(follower.order):
        raise ConfigError("leader/follower channel mismatch")
    return Contract(leader, follower)
