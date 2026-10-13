"""Exercise the installed source tree without replacing import-time core modules."""

import os
import subprocess
import sys
import unittest
from pathlib import Path


def _probe_original_package() -> None:
    from unittest.mock import Mock, patch

    import torch
    from lerobot.common.robot_devices.motors.gbot import (
        GBotCommunicationError,
        GBotMotorsBus,
    )
    from lerobot.common.robot_devices.robots.manipulator import (
        ManipulatorRobot,
        ManipulatorRobotConfig,
    )

    checks = unittest.TestCase()

    class Reply:
        def __init__(self, value=0, success=True):
            self.value = value
            self.success = success

        def is_success(self):
            return self.success

        def get_data(self, address):
            return self.value

        def get_error_code(self):
            return 27

    def leader_bus(replies):
        with patch("lerobot.common.robot_devices.motors.gbot.PortHandler") as port_type, patch(
            "lerobot.common.robot_devices.motors.gbot.SyncConnector"
        ) as connector_type:
            bus = GBotMotorsBus("NO_DEVICE", {"m1": (1, "genki_M1"), "m2": (2, "genki_M1")})
        port_type.return_value.open.assert_not_called()
        bus.is_connected = True
        bus._GBotMotorsBus__port_handler.is_open.return_value = False
        bus.set_calibration({
            "motor_names": ["m1", "m2"],
            "calib_mode": ["DEGREE", "DEGREE"],
            "drive_mode": [0, 0],
            "homing_offset": [0, 0],
        })
        connector_type.return_value.read.side_effect = replies
        return bus, connector_type.return_value

    bus, connector = leader_bus([Reply(0), Reply(0)])
    follower = Mock()
    follower.motor_names = ["m1", "m2"]
    robot = ManipulatorRobot(ManipulatorRobotConfig(
        robot_type="so100", leader_arms={"main": bus}, follower_arms={"main": follower}
    ))
    robot.is_connected = True
    robot.teleop_step()
    checks.assertEqual(follower.write.call_args.args[1].tolist(), [0, 0])
    checks.assertEqual(connector.read.call_count, 2)

    failing_bus, failing_connector = leader_bus([Reply(0), Reply(success=False)])
    blocked_follower = Mock()
    blocked_follower.motor_names = ["m1", "m2"]
    blocked_robot = ManipulatorRobot(ManipulatorRobotConfig(
        robot_type="so100", leader_arms={"main": failing_bus},
        follower_arms={"main": blocked_follower}
    ))
    blocked_robot.is_connected = True
    try:
        blocked_robot.teleop_step()
    except GBotCommunicationError as exc:
        checks.assertEqual(exc.motor_id, 2)
        checks.assertIs(blocked_robot.control_fault, exc)
    else:
        raise AssertionError("failed motor read did not propagate")
    blocked_follower.write.assert_not_called()
    checks.assertEqual(failing_connector.read.call_count, 2)

    first, second = Mock(), Mock()
    first.motor_names = second.motor_names = ["m1"]
    mixed_robot = ManipulatorRobot(ManipulatorRobotConfig(
        robot_type="so100", follower_arms={"first": first, "second": second}
    ))
    mixed_robot.is_connected = True
    try:
        mixed_robot.send_action(torch.tensor([0.0, float("nan")]))
    except ValueError:
        pass
    else:
        raise AssertionError("non-finite second target was accepted")
    first.write.assert_not_called()
    second.write.assert_not_called()
    checks.assertIsNotNone(mixed_robot.control_fault)


class RealPackageIntegrationTests(unittest.TestCase):
    def test_clean_import_and_control_chain(self):
        repository = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(repository / "third_party" / "genkiarm")
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--probe"],
            cwd=repository,
            check=False,
            env=environment,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    if "--probe" in sys.argv:
        _probe_original_package()
    else:
        unittest.main()
