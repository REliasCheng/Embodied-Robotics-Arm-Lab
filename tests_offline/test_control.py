import pytest

from offline_control.cli import run
from offline_control.config import DEFAULT_CONFIG, ConfigError, load_contract
from offline_control.controller import ControlError, Controller, State
from offline_control.fake_bus import FakeMotorBus
from offline_control.protocol import Sample
from offline_control.trace import TraceError, deterministic_samples, load_trace, save_trace


def real_contract():
    return load_contract(DEFAULT_CONFIG)


def test_actual_so100_contract_and_mapping():
    contract = real_contract()
    assert contract.leader.order == (
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
    )
    assert contract.leader.ids == contract.follower.ids == (1, 2, 3, 4, 5, 6)
    bus = FakeMotorBus()
    controller = Controller(contract, bus)
    controller.initialize()
    controller.start()
    sample = deterministic_samples(contract.leader.order)[0]
    command = controller.process(sample)
    expected = tuple((name, sample.channels[name]) for name in contract.follower.order)
    assert command.channels == expected
    assert bus.read_recorded_state() == (command,)
    controller.stop()


@pytest.mark.parametrize("scenario,committed", [
    ("normal", 3), ("invalid-input", 1), ("transport-fault", 1),
    ("recovery", 2), ("replay", 3),
])
def test_actual_scenarios(scenario, committed):
    result = run(scenario, DEFAULT_CONFIG)
    assert result["result"] == "PASS"
    assert result["committed"] == committed
    assert result["state_before_close"] == (
        "FAULT_LATCHED" if scenario == "transport-fault" else "RUNNING"
    )


@pytest.mark.parametrize("mutation", [
    lambda text: text.replace("max_relative_target: null", "max_relative_target: 10"),
    lambda text: text.replace("shoulder_pan: [1, \"genki_M1\"]", "shoulder_pan: [2, \"genki_M1\"]"),
    lambda text: text.replace("  gripper: [6, \"sts3215\"]", "  extra: [6, \"sts3215\"]"),
    lambda text: text.replace(
        '  gripper: [6, "sts3215"]',
        '  gripper: [6, "sts3215"]\n  gripper: [7, "sts3215"]',
    ),
    lambda text: text.replace("FeetechMotorsBus", "UnknownBus"),
])
def test_config_rejects_invalid_contract(tmp_path, mutation):
    source = DEFAULT_CONFIG.read_text(encoding="utf-8")
    changed = mutation(source)
    assert changed != source
    path = tmp_path / "config.yaml"
    path.write_text(changed, encoding="utf-8")
    with pytest.raises(ConfigError):
        load_contract(path)


def running_controller(fault_at=None, fault_kind="failed"):
    bus = FakeMotorBus(fault_at=fault_at, fault_kind=fault_kind)
    controller = Controller(real_contract(), bus)
    controller.initialize()
    controller.start()
    return controller, bus, deterministic_samples(controller.contract.leader.order)


def test_state_gates_and_closed_resource():
    bus = FakeMotorBus()
    controller = Controller(real_contract(), bus)
    sample = deterministic_samples(controller.contract.leader.order)[0]
    with pytest.raises(ControlError):
        controller.process(sample)
    controller.initialize()
    with pytest.raises(ControlError):
        controller.initialize()
    controller.start()
    controller.pause()
    with pytest.raises(ControlError):
        controller.process(sample)
    assert not bus.captured
    controller.resume()
    controller.process(sample)
    controller.stop()
    controller.stop()  # idempotent close
    with pytest.raises(ControlError):
        controller.process(sample)
    with pytest.raises(RuntimeError):
        bus.send(bus.captured[0])


@pytest.mark.parametrize("bad", [
    lambda s: Sample(s.tick, s.sequence, s.source, {"unknown": 1.0}),
    lambda s: Sample(s.tick, s.sequence, s.source, {**s.channels, "unknown": 1.0}),
    lambda s: Sample(s.tick, s.sequence, s.source, {**s.channels, "gripper": float("nan")}),
    lambda s: Sample(s.tick, s.sequence, s.source, {**s.channels, "gripper": float("inf")}),
    lambda s: Sample(s.tick, s.sequence, s.source, {**s.channels, "gripper": True}),
    lambda s: Sample(s.tick, s.sequence, s.source, {**s.channels, "gripper": 101.0}),
    lambda s: Sample(s.tick, s.sequence, "", s.channels),
])
def test_invalid_sample_is_atomic(bad):
    controller, bus, samples = running_controller()
    with pytest.raises(ControlError):
        controller.process(bad(samples[0]))
    assert controller.last is None and not bus.captured
    assert controller.events[-1].startswith("REJECTED:")
    controller.process(samples[0])
    controller.stop()


def test_sequence_tick_and_slew_rejection():
    controller, bus, samples = running_controller()
    controller.process(samples[0])
    for sample in (
        samples[0], Sample(0, 2, "synthetic-v1", samples[1].channels),
        Sample(2, 0, "synthetic-v1", samples[1].channels),
        Sample(4, 4, "synthetic-v1", samples[1].channels),
        Sample(2, 2, "synthetic-v1", {**samples[1].channels, "gripper": 50.0}),
    ):
        with pytest.raises(ControlError):
            controller.process(sample)
    assert len(bus.captured) == 1 and controller.last == samples[0]
    controller.process(samples[1])
    controller.stop()


@pytest.mark.parametrize("kind", ["failed", "partial", "unknown"])
def test_fault_latch_and_explicit_recovery(kind):
    controller, bus, samples = running_controller(fault_at=2, fault_kind=kind)
    controller.process(samples[0])
    with pytest.raises(ControlError):
        controller.process(samples[1])
    assert controller.state == State.FAULT_LATCHED
    assert controller.last == samples[0]
    assert bus.last_outcome.kind == kind
    assert len(bus.captured) == 1
    with pytest.raises(ControlError):
        controller.start()
    with pytest.raises(ControlError):
        controller.process(samples[2])
    with pytest.raises(ControlError):
        controller.reset(bus)
    fresh = FakeMotorBus()
    controller.reset(fresh)
    controller.start()
    controller.process(samples[2])
    assert [c.sequence for c in fresh.captured] == [3]
    controller.stop()


def test_replay_rejects_oversize_duplicate_and_wrong_version(tmp_path):
    path = tmp_path / "trace.json"
    save_trace(path, deterministic_samples(real_contract().leader.order))
    assert len(load_trace(path)) == 3
    path.write_bytes(b" " * (64 * 1024 + 1))
    with pytest.raises(TraceError):
        load_trace(path)
    path.write_text('{"version":1,"version":1,"kind":"synthetic-offline","samples":[]}',
                    encoding="utf-8")
    with pytest.raises(TraceError):
        load_trace(path)
    path.write_text('{"version":2,"kind":"synthetic-offline","samples":[]}', encoding="utf-8")
    with pytest.raises(TraceError):
        load_trace(path)


def test_connect_and_reconnect_failure_are_latched():
    initial = Controller(real_contract(), FakeMotorBus(connect_fail=True))
    with pytest.raises(ControlError):
        initial.initialize()
    assert initial.state == State.FAULT_LATCHED
    assert initial.bus.closed
    controller, _, samples = running_controller(fault_at=1)
    with pytest.raises(ControlError):
        controller.process(samples[0])
    broken = FakeMotorBus(connect_fail=True)
    with pytest.raises(ControlError):
        controller.reset(broken)
    assert controller.state == State.FAULT_LATCHED and broken.closed
    assert controller.events[-1].startswith("RECOVERY_FAILED:")
    controller.reset(FakeMotorBus())
    controller.start()
    controller.process(samples[1])
    controller.stop()
