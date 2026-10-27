"""In-memory motor bus with explicit uncertain partial-write outcomes."""

from dataclasses import dataclass

from offline_control.protocol import Command


@dataclass(frozen=True)
class WriteOutcome:
    kind: str  # committed, failed, partial, unknown
    committed_channels: tuple[str, ...] = ()


class FakeMotorBus:
    def __init__(self, fault_at: int | None = None, fault_kind: str = "failed",
                 connect_fail: bool = False) -> None:
        self.initialized = False
        self.connected = False
        self.closed = False
        self.fault_at = fault_at
        self.fault_kind = fault_kind
        self.connect_fail = connect_fail
        self.attempts = 0
        self.captured: list[Command] = []
        self.last_outcome: WriteOutcome | None = None

    def initialize(self) -> None:
        if self.closed:
            raise RuntimeError("bus closed")
        self.initialized = True

    def connect(self) -> None:
        if not self.initialized or self.closed:
            raise RuntimeError("bus not initialized or closed")
        if self.connect_fail:
            raise RuntimeError("injected virtual connection failure")
        self.connected = True  # memory endpoint only

    def send(self, command: Command) -> WriteOutcome:
        if not self.connected or self.closed:
            raise RuntimeError("bus disconnected or closed")
        self.attempts += 1
        if self.attempts == self.fault_at:
            if self.fault_kind == "partial":
                outcome = WriteOutcome("partial", (command.channels[0][0],))
            elif self.fault_kind == "unknown":
                outcome = WriteOutcome("unknown")
            else:
                outcome = WriteOutcome("failed")
            self.last_outcome = outcome
            return outcome
        self.captured.append(command)
        self.last_outcome = WriteOutcome("committed", tuple(name for name, _ in command.channels))
        return self.last_outcome

    def read_recorded_state(self) -> tuple[Command, ...]:
        return tuple(self.captured)

    def close(self) -> None:
        self.connected = False
        self.closed = True
