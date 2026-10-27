"""Fail-closed state machine for synthetic leader commands."""

import math
from enum import Enum

from offline_control.config import Contract
from offline_control.fake_bus import FakeMotorBus
from offline_control.protocol import Command, Sample


class State(str, Enum):
    INITIALIZED = "INITIALIZED"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    FAULT_LATCHED = "FAULT_LATCHED"
    STOPPED = "STOPPED"


class ControlError(RuntimeError):
    """Rejected command or invalid transition."""


class Controller:
    # Synthetic test units only. These are not mechanical limits or a physical E-stop.
    ABS_LIMIT = 100.0
    STEP_LIMIT = 20.0

    def __init__(self, contract: Contract, bus: FakeMotorBus) -> None:
        self.contract = contract
        self.bus = bus
        self.state = State.INITIALIZED
        self.last: Sample | None = None
        self.events: list[str] = []

    def initialize(self) -> None:
        if self.state != State.INITIALIZED:
            raise ControlError("already initialized")
        try:
            self.bus.initialize()
            self.bus.connect()
        except Exception as exc:
            self._fault(f"connect error: {exc}")
            raise ControlError("connect failed") from exc
        self.state = State.READY
        self.events.append("READY")

    def start(self) -> None:
        if self.state != State.READY:
            raise ControlError(f"cannot start from {self.state.value}")
        self.state = State.RUNNING
        self.events.append("RUNNING")

    def pause(self) -> None:
        if self.state != State.RUNNING:
            raise ControlError("pause requires RUNNING")
        self.state = State.PAUSED
        self.events.append("PAUSED")

    def resume(self) -> None:
        if self.state != State.PAUSED:
            raise ControlError("resume requires PAUSED")
        self.state = State.RUNNING
        self.events.append("RUNNING")

    def stop(self) -> None:
        self.bus.close()
        self.state = State.STOPPED
        self.events.append("STOPPED")

    def _fault(self, reason: str) -> None:
        self.state = State.FAULT_LATCHED
        self.events.append(f"FAULT: {reason}")
        self.bus.close()

    def reset(self, replacement: FakeMotorBus) -> None:
        if self.state != State.FAULT_LATCHED:
            raise ControlError("reset requires FAULT_LATCHED")
        if replacement is self.bus:
            raise ControlError("reset requires a fresh bus")
        try:
            replacement.initialize()
            replacement.connect()
        except Exception as exc:
            replacement.close()
            self.events.append(f"RECOVERY_FAILED: {exc}")
            raise ControlError("reconnect failed; fault remains latched") from exc
        self.bus = replacement
        self.last = None
        self.state = State.READY
        self.events.append("RESET_READY")

    def process(self, sample: Sample) -> Command:
        if self.state != State.RUNNING:
            self._reject(f"output blocked in {self.state.value}")
        if type(sample.tick) is not int or type(sample.sequence) is not int:
            self._reject("tick and sequence must be integers")
        if (sample.tick < 0 or sample.sequence < 0
                or not isinstance(sample.source, str) or not sample.source):
            self._reject("invalid sample metadata")
        if self.last is not None and (sample.tick <= self.last.tick
                                      or sample.sequence != self.last.sequence + 1):
            self._reject("duplicate, skipped or regressing tick/sequence")
        if (not isinstance(sample.channels, dict)
                or set(sample.channels) != set(self.contract.leader.order)):
            self._reject("missing or unknown channel")
        values: dict[str, float] = {}
        for name, value in sample.channels.items():
            if type(value) not in (int, float) or not math.isfinite(value):
                self._reject(f"invalid numeric value: {name}")
            if abs(value) > self.ABS_LIMIT:
                self._reject(f"synthetic absolute limit exceeded: {name}")
            if self.last is not None and abs(value - self.last.channels[name]) > self.STEP_LIMIT:
                self._reject(f"synthetic step limit exceeded: {name}")
            values[name] = float(value)
        command = Command(sample.tick, sample.sequence,
                          tuple((name, values[name]) for name in self.contract.follower.order))
        try:
            outcome = self.bus.send(command)
        except Exception as exc:
            self._fault(f"write exception, outcome unknown: {exc}")
            raise ControlError("write outcome unknown") from exc
        if outcome.kind != "committed":
            self._fault(f"write {outcome.kind}; committed={outcome.committed_channels}")
            raise ControlError(f"write {outcome.kind}")
        self.last = sample
        self.events.append(f"COMMITTED: {sample.sequence}")
        return command

    def _reject(self, reason: str) -> None:
        self.events.append(f"REJECTED: {reason}")
        raise ControlError(reason)
