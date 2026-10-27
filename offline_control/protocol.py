"""Synthetic input and command types; values are not physical angles."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Sample:
    tick: int
    sequence: int
    source: str
    channels: dict[str, float]


@dataclass(frozen=True)
class Command:
    tick: int
    sequence: int
    channels: tuple[tuple[str, float], ...]
