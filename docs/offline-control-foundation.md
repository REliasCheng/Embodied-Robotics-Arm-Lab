# Offline Leader–Follower Control and Safety Validation Lab

This repository now contains an original, executable **software-only** control model in
`offline_control/`. The existing GenkiArm / LeRobot subtree is read as configuration data only.
No third-party robot class, serial driver, camera, motor, or calibration routine is imported or run.

## Data path and configuration

`so100.yaml` is loaded through PyYAML `SafeLoader` with duplicate-key rejection. The loader
checks one `main` leader with the GBot target string and one `main` follower with the Feetech
target string. It validates the six joint names, unique integer IDs and an explicit follower order.
`_target_` strings are compared as data; no Hydra object is instantiated. The `port` and
`cameras` fields are never read by the offline controller. `max_relative_target: null` is
checked and **does not supply a motion limit**. The synthetic ±100-unit and 20-unit step limits
are test contracts only; they are not motor angles, encoder conversions or physical safety limits.

```text
actual SO-100 YAML -> contract -> synthetic Sample -> Controller
                                                 -> FakeMotorBus -> captured Command
```

Each sample carries `tick`, `sequence`, `source`, and six named numeric channels. The controller
rejects unknown or missing channels, booleans, NaN/Infinity, out-of-range values, step jumps,
duplicate or skipped sequence values, and non-increasing ticks. It maps by joint name into the
follower order, then commits only after Fake Bus reports a complete write.

## State and fault contract

`INITIALIZED -> READY -> RUNNING` follows explicit bus initialization and memory connection.
`RUNNING <-> PAUSED` gates output. `STOPPED` is terminal. A write failure, partial write or
unknown write outcome enters `FAULT_LATCHED` and closes the old bus. A subsequent good sample
cannot resume output. `reset` requires a **fresh** in-memory bus, successful reconnect and
another explicit `start`. A reconnect failure leaves the fault latched. Prior partial work is
never marked complete or replayed automatically. Rejected inputs leave the last valid sample
and captured commands unchanged. Event records distinguish `COMMITTED`, `REJECTED`, `FAULT`,
`RECOVERY_FAILED`, and state changes. Fake Bus `partial` reports only explicitly committed
channels; `unknown` reports no confirmed channels. Resource closure is idempotent.

This software latch is **not a physical emergency stop**. It has no torque cutoff, mechanical
limit, collision model or hardware validation.

## Versioned replay

The generated JSON trace declares `version: 1` and `kind: synthetic-offline`. A trace is limited
to 64 KiB and 100 samples; duplicate JSON keys, nonstandard NaN/Infinity constants, unknown
envelopes and unsupported versions are rejected. Samples are validated again by the controller
when replayed. The trace is not a LeRobot dataset, camera recording or hardware teleoperation log.

## Reproduce

From the repository root with Python 3.10 or newer:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e . pytest==8.3.5 build==1.2.2.post1 ruff==0.11.13 mypy==1.15.0
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m ruff check offline_control tests_offline
.\.venv\Scripts\python.exe -m mypy offline_control
.\.venv\Scripts\python.exe -m build
.\.venv\Scripts\robotics-offline.exe --scenario normal --config third_party/genkiarm/lerobot/configs/robot/so100.yaml
```

Run `invalid-input`, `transport-fault`, `recovery`, and `replay` with the same CLI arguments.
The CLI has no parameter for real GBot, Feetech, serial or camera backends. It returns JSON and
exit code 1 on a failed scenario. CI runs tests, static checks, build and all five scenarios on
Ubuntu and Windows with Python 3.10 and 3.12, including CLI execution from the built wheel.

## Source and license

Original `offline_control/`, tests and documentation are covered by the repository MIT license.
The preserved GenkiArm / LeRobot subtree remains Apache-2.0 under its own LICENSE and NOTICE;
its upstream commit is recorded in `THIRD_PARTY_NOTICES.md`. The only runtime package dependency
is [PyYAML 6.0.2](https://pypi.org/project/PyYAML/6.0.2/), which declares MIT license.
CI tools are development-only dependencies. No third-party source is copied into the offline module.

## Evidence boundary

This lab verifies only synthetic Python execution, actual SO-100 YAML parsing, name mapping,
Fake Bus behavior, fault handling and replay. Third-party `ManipulatorRobot.teleop_step()` and
its motor bus runtime were not executed. Real motor bus, physical E-stop, calibration, camera,
dataset recording, model training and autonomous policy execution remain unverified.
