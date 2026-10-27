"""Five real offline scenarios. No option selects a hardware backend."""

import argparse
import json
import tempfile
from pathlib import Path

from offline_control.config import DEFAULT_CONFIG, load_contract
from offline_control.controller import ControlError, Controller, State
from offline_control.fake_bus import FakeMotorBus
from offline_control.protocol import Sample
from offline_control.trace import deterministic_samples, load_trace, save_trace


def run(scenario: str, config: Path) -> dict:
    contract = load_contract(config)
    if scenario in ("transport-fault", "recovery"):
        bus = FakeMotorBus(fault_at=2, fault_kind="partial")
    else:
        bus = FakeMotorBus()
    controller = Controller(contract, bus)
    samples = deterministic_samples(contract.leader.order)
    if scenario == "replay":
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.json"
            save_trace(trace_path, samples)
            samples = load_trace(trace_path)
    controller.initialize()
    controller.start()
    try:
        if scenario == "invalid-input":
            controller.process(samples[0])
            invalid = Sample(2, 2, "synthetic-v1", {"unknown": 1.0})
            try:
                controller.process(invalid)
            except ControlError:
                pass
            else:
                raise AssertionError("invalid input accepted")
            assert len(bus.captured) == 1 and controller.last == samples[0]
        elif scenario in ("transport-fault", "recovery"):
            controller.process(samples[0])
            try:
                controller.process(samples[1])
            except ControlError:
                pass
            else:
                raise AssertionError("fault was not detected")
            assert controller.state == State.FAULT_LATCHED
            assert bus.last_outcome is not None and bus.last_outcome.kind == "partial"
            try:
                controller.process(samples[2])
            except ControlError:
                pass
            else:
                raise AssertionError("fault latch was bypassed")
            assert len(bus.captured) == 1
            if scenario == "recovery":
                fresh_bus = FakeMotorBus()
                controller.reset(fresh_bus)
                controller.start()
                controller.process(samples[2])
                assert len(fresh_bus.captured) == 1 and len(bus.captured) == 1
        else:
            for sample in samples:
                controller.process(sample)
            assert [c.sequence for c in bus.captured] == [1, 2, 3]
            assert all(tuple(name for name, _ in c.channels) == contract.follower.order
                       for c in bus.captured)
    finally:
        state_before_close = controller.state.value
        controller.stop()
    total_committed = len(bus.captured)
    if scenario == "recovery":
        total_committed += len(fresh_bus.captured)
    return {"scenario": scenario, "result": "PASS", "state_before_close": state_before_close,
            "committed": total_committed, "events": controller.events}


def main() -> int:
    parser = argparse.ArgumentParser(description="Synthetic offline control only")
    scenarios = ("normal", "invalid-input", "transport-fault", "recovery", "replay")
    parser.add_argument("--scenario", required=True, choices=scenarios)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    try:
        print(json.dumps(run(args.scenario, args.config)))
    except Exception as exc:
        print(json.dumps({"scenario": args.scenario, "result": "FAIL", "error": str(exc)}))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
