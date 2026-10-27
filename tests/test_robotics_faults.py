"""Hardware-free regression tests of the repository's original robot classes."""

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1] / "third_party" / "genkiarm"
sys.path.insert(0, str(ROOT))

# Only the hardware transport boundary is replaced. The package's utility,
# motor and robot modules are imported from the original source tree.
hardware_stub = types.ModuleType("lerobot.common.robot_devices.motors.GBot")
hardware_stub.PortHandler = Mock
hardware_stub.SyncConnector = Mock
hardware_stub.Address = types.SimpleNamespace(
    CURRENT_POSITION="position", TORQUE_ENABLE="torque", MOTOR_TYPE="model", ID="id"
)
hardware_stub.Result = Mock
sys.modules[hardware_stub.__name__] = hardware_stub

from lerobot.common.robot_devices.motors.gbot import (
    GBotCommunicationError,
    GBotMotorsBus,
    JointOutOfRangeError,
)
from lerobot.common.robot_devices.robots.manipulator import (
    ManipulatorRobot,
    ManipulatorRobotConfig,
    RobotControlFaultError,
)
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError


class Reply:
    def __init__(self, value=12, success=True, error=7):
        self.value, self.success, self.error = value, success, error

    def is_success(self):
        return self.success

    def get_data(self, address):
        return self.value

    def get_error_code(self):
        return self.error


def bus(count=3):
    motors = {f"m{i}": (i, "genki_M1") for i in range(1, count + 1)}
    result = GBotMotorsBus("MOCK_ONLY", motors)
    result.is_connected = True
    # The mock transport has no serial handle, so a successful close observes
    # a closed port rather than an unconstrained Mock truth value.
    result._GBotMotorsBus__port_handler.is_open.return_value = False
    connector = Mock()
    connector.read.return_value = Reply()
    connector.write.return_value = Reply()
    result._GBotMotorsBus__sync_connector = connector
    result.set_calibration({
        "motor_names": list(motors),
        "calib_mode": ["DEGREE"] * count,
        "drive_mode": [0] * count,
        "homing_offset": [0] * count,
    })
    return result, connector


class BusReadTests(unittest.TestCase):
    def test_single_success_and_real_zero(self):
        device, io = bus(1)
        self.assertEqual(device.read("ID", "m1").tolist(), [12])
        io.read.return_value = Reply(0)
        self.assertEqual(device.read("Present_Position", "m1").tolist(), [0.0])
        self.assertEqual(device.read_with_motor_ids(["genki_M1"], 1, "ID"), 0)

    def test_failure_and_timeout_never_return_position(self):
        device, io = bus(1)
        io.read.return_value = Reply(success=False, error=21)
        for read in (lambda: device.read("ID", "m1"),
                     lambda: device.read_with_motor_ids(["genki_M1"], 1, "ID")):
            with self.assertRaises(GBotCommunicationError) as cm:
                read()
            self.assertEqual((cm.exception.operation, cm.exception.motor_id, cm.exception.error_code),
                             ("read", 1, 21))
        io.read.side_effect = TimeoutError("mock timeout")
        with self.assertRaises(GBotCommunicationError) as cm:
            device.read("ID", "m1")
        self.assertIsInstance(cm.exception.__cause__, TimeoutError)

    def test_missing_data_and_register(self):
        device, io = bus(1)
        io.read.return_value = Reply(None)
        with self.assertRaisesRegex(GBotCommunicationError, "no register data"):
            device.read("ID", "m1")
        with self.assertRaises(GBotCommunicationError):
            device.read_with_motor_ids(["genki_M1"], 1, "Unknown")
        io.read.assert_called_once()

    def test_multi_success_and_each_failure_position(self):
        for method in ("read", "read_with_motor_ids"):
            for failed_index in range(3):
                device, io = bus()
                replies = [Reply(i) for i in range(3)]
                replies[failed_index] = Reply(success=False, error=30 + failed_index)
                io.read.side_effect = replies
                with self.subTest(method=method, failed_index=failed_index):
                    with self.assertRaises(GBotCommunicationError) as cm:
                        if method == "read":
                            device.read("ID")
                        else:
                            device.read_with_motor_ids(["genki_M1"] * 3, [1, 2, 3], "ID")
                    self.assertEqual(cm.exception.motor_id, failed_index + 1)
                    self.assertEqual(io.read.call_count, failed_index + 1)
            device, io = bus()
            io.read.side_effect = [Reply(0), Reply(2), Reply(3)]
            if method == "read":
                self.assertEqual(device.read("ID").tolist(), [0, 2, 3])
            else:
                self.assertEqual(device.read_with_motor_ids(["genki_M1"] * 3, [1, 2, 3], "ID"),
                                 [0, 2, 3])


class BusWriteTests(unittest.TestCase):
    def test_single_success_and_failure(self):
        device, io = bus(1)
        self.assertIsNone(device.write("Torque_Enable", 1, "m1"))
        self.assertIsNone(device.write_with_motor_ids(["genki_M1"], [1], "Torque_Enable", [1]))
        io.write.return_value = Reply(success=False, error=44)
        for write in (lambda: device.write("Torque_Enable", 1, "m1"),
                      lambda: device.write_with_motor_ids(["genki_M1"], [1], "Torque_Enable", [1])):
            with self.assertRaises(GBotCommunicationError) as cm:
                write()
            self.assertEqual((cm.exception.operation, cm.exception.register, cm.exception.motor_id,
                              cm.exception.error_code, cm.exception.completed_motor_ids),
                             ("write", "Torque_Enable", 1, 44, ()))
            self.assertTrue(cm.exception.state_uncertain)

    def test_multi_success_and_stop_at_each_failure(self):
        for method in ("write", "write_with_motor_ids"):
            for failed_index in range(3):
                device, io = bus()
                replies = [Reply() for _ in range(3)]
                replies[failed_index] = Reply(success=False, error=50)
                io.write.side_effect = replies
                with self.subTest(method=method, failed_index=failed_index):
                    with self.assertRaises(GBotCommunicationError) as cm:
                        if method == "write":
                            device.write("Torque_Enable", [1, 2, 3])
                        else:
                            device.write_with_motor_ids(["genki_M1"] * 3, [1, 2, 3],
                                                        "Torque_Enable", [1, 2, 3])
                    self.assertEqual(cm.exception.completed_motor_ids, tuple(range(1, failed_index + 1)))
                    self.assertEqual(cm.exception.partial_execution, failed_index > 0)
                    self.assertTrue(cm.exception.state_uncertain)
                    self.assertEqual(io.write.call_count, failed_index + 1)
                    self.assertEqual([call.args[0] for call in io.write.call_args_list],
                                     list(range(1, failed_index + 2)))
            device, io = bus()
            if method == "write":
                device.write("Torque_Enable", [1, 2, 3])
            else:
                device.write_with_motor_ids(["genki_M1"] * 3, [1, 2, 3],
                                            "Torque_Enable", [1, 2, 3])
            self.assertEqual(io.write.call_count, 3)

    def test_invalid_address_and_transport_exception_preserve_completed(self):
        device, io = bus()
        device.model_ctrl_table["genki_M1"]["Torque_Enable"] = None
        with self.assertRaises(GBotCommunicationError):
            device.write("Torque_Enable", [1, 2, 3])
        io.write.assert_not_called()
        device.model_ctrl_table["genki_M1"]["Torque_Enable"] = "torque"
        io.write.side_effect = [Reply(), TimeoutError("mock timeout")]
        with self.assertRaises(GBotCommunicationError) as cm:
            device.write("Torque_Enable", [1, 2, 3])
        self.assertEqual(cm.exception.completed_motor_ids, (1,))
        self.assertIsInstance(cm.exception.__cause__, TimeoutError)
        self.assertEqual(io.write.call_count, 2)


class CalibrationTests(unittest.TestCase):
    def test_degree_list_numpy_zero_and_boundary(self):
        device, _ = bus(2)
        for value in ([0, 2048], np.array([0, 2048])):
            np.testing.assert_array_equal(device.apply_calibration(value, ["m1", "m2"]), [0, 180])
        np.testing.assert_array_equal(device.revert_calibration([0, 180], ["m1", "m2"]), [0, 2048])
        np.testing.assert_array_equal(device.revert_calibration(0, ["m1"]), [0])
        np.testing.assert_array_equal(device.apply_calibration([1024], ["m1"]), [90])

    def test_linear_list_and_numpy(self):
        device, _ = bus(1)
        device.set_calibration({"motor_names": ["m1"], "calib_mode": ["LINEAR"],
                                "start_pos": [10], "end_pos": [110]})
        for value in ([10], np.array([10])):
            np.testing.assert_array_equal(device.apply_calibration(value, ["m1"]), [0])
        np.testing.assert_array_equal(device.apply_calibration([110], ["m1"]), [100])
        np.testing.assert_array_equal(device.revert_calibration([100], ["m1"]), [110])
        device.autocorrect_calibration([50], ["m1"])

    def test_validation_and_autocorrect_list(self):
        device, _ = bus(2)
        for fn in (device.apply_calibration, device.autocorrect_calibration, device.revert_calibration):
            with self.assertRaises(ValueError):
                fn([[1, 2]], ["m1", "m2"])
            with self.assertRaises(ValueError):
                fn([1], ["m1", "m2"])
            with self.assertRaises(ValueError):
                fn([1], ["unknown"])
            device.calibration["motor_names"] = ["m2"]
            with self.assertRaisesRegex(ValueError, "Missing calibration"):
                fn([1], ["m1"])
            device.calibration["motor_names"] = ["m1", "m2"]
        device.autocorrect_calibration([4096], ["m1"])
        self.assertEqual(device.calibration["homing_offset"][0], -4096)
        np.testing.assert_array_equal(device.apply_calibration(4096, ["m1"]), [0])


def robot(two_followers=False, max_relative_target=None):
    leader = Mock()
    leader.motor_names = ["joint"]
    leader.read.return_value = np.array([10], dtype=np.float32)
    followers = {"main": Mock()}
    if two_followers:
        followers["other"] = Mock()
    for follower in followers.values():
        follower.motor_names = ["joint"]
        follower.read.return_value = np.array([10], dtype=np.float32)
    # For two followers teleop's matching leader names are required.
    leaders = {name: leader if name == "main" else Mock(read=Mock(return_value=np.array([10], dtype=np.float32)))
               for name in followers}
    for arm in leaders.values():
        arm.motor_names = ["joint"]
    result = ManipulatorRobot(ManipulatorRobotConfig(
        robot_type="so100", leader_arms=leaders, follower_arms=followers,
        max_relative_target=max_relative_target,
    ))
    result.is_connected = True  # Mocked devices only; no connect()/serial access.
    return result, leaders, followers


class ControlTests(unittest.TestCase):
    def test_leader_read_failure_blocks_writes_and_ema(self):
        unit, leaders, followers = robot()
        error = GBotCommunicationError("read", "Present_Position", 1, 21)
        leaders["main"].read.side_effect = error
        with self.assertRaises(GBotCommunicationError) as cm:
            unit.teleop_step()
        self.assertIs(cm.exception, error)
        self.assertEqual(followers["main"].write.call_count, 0)
        self.assertEqual(unit.smoothed_leader_pos, {})
        leaders["main"].read.side_effect = None
        with self.assertRaises(RobotControlFaultError):
            unit.teleop_step()
        with self.assertRaises(RobotControlFaultError):
            unit.send_action(torch.tensor([12.0]))
        self.assertEqual(followers["main"].write.call_count, 0)

    def test_calibration_error_blocks_writes(self):
        unit, leaders, followers = robot()
        leaders["main"].read.side_effect = JointOutOfRangeError("bad calibration")
        with self.assertRaises(JointOutOfRangeError):
            unit.teleop_step()
        followers["main"].write.assert_not_called()

    def test_second_leader_failure_discards_first_ema(self):
        unit, leaders, followers = robot(two_followers=True)
        leaders["other"].read.side_effect = GBotCommunicationError("read", "Present_Position", 2, 21)
        with self.assertRaises(GBotCommunicationError):
            unit.teleop_step()
        self.assertEqual(unit.smoothed_leader_pos, {})
        for follower in followers.values():
            follower.write.assert_not_called()

    def test_successful_ema_and_action(self):
        unit, leaders, followers = robot(max_relative_target=5)
        unit.teleop_step()
        self.assertEqual(unit.smoothed_leader_pos["main"].tolist(), [10.0])
        leaders["main"].read.return_value = np.array([14], dtype=np.float32)
        unit.teleop_step()
        self.assertEqual(unit.smoothed_leader_pos["main"].tolist(), [11.0])
        result = unit.send_action(torch.tensor([20.0]))
        self.assertEqual(result.tolist(), [15.0])
        self.assertEqual(int(followers["main"].write.call_args.args[1][0]), 15)

    def test_record_data_success_and_disconnected_call_does_not_latch(self):
        unit, _, followers = robot()
        observation, action = unit.teleop_step(record_data=True)
        self.assertEqual(observation["observation.state"].tolist(), [10.0])
        self.assertEqual(action["action"].tolist(), [10.0])
        unit.is_connected = False
        with self.assertRaises(RobotDeviceNotConnectedError):
            unit.teleop_step()
        self.assertIsNone(unit.control_fault)
        followers["main"].write.assert_called_once()

    def test_teleop_partial_followers_stop_and_latch(self):
        unit, _leaders, followers = robot(two_followers=True)
        error = GBotCommunicationError("write", "Goal_Position", 2, 50, [1])
        followers["main"].write.side_effect = error
        with self.assertRaises(GBotCommunicationError):
            unit.teleop_step()
        followers["other"].write.assert_not_called()
        self.assertEqual(unit.smoothed_leader_pos, {})
        self.assertEqual(unit.completed_follower_arms, [])
        self.assertEqual(unit.control_fault.completed_motor_ids, (1,))
        with self.assertRaises(RobotControlFaultError):
            unit.teleop_step()

    def test_send_action_partial_followers_stop_and_latch(self):
        unit, _, followers = robot(two_followers=True)
        error = GBotCommunicationError("write", "Goal_Position", 2, 50)
        followers["other"].write.side_effect = error
        with self.assertRaises(GBotCommunicationError):
            unit.send_action(torch.tensor([1.0, 2.0]))
        self.assertEqual(unit.completed_follower_arms, ["main"])
        self.assertEqual(followers["main"].write.call_count, 1)
        with self.assertRaises(RobotControlFaultError):
            unit.send_action(torch.tensor([1.0, 2.0]))
        self.assertEqual(followers["main"].write.call_count, 1)

    def test_send_action_read_failure_and_manual_recovery(self):
        unit, _, followers = robot(max_relative_target=5)
        followers["main"].read.side_effect = TimeoutError("mock read timeout")
        with self.assertRaises(TimeoutError):
            unit.send_action(torch.tensor([12.0]))
        followers["main"].write.assert_not_called()
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        unit.disconnect()
        with self.assertRaises(RobotControlFaultError):
            unit.connect()
        unit.acknowledge_control_fault()
        self.assertTrue(unit.fault_acknowledged)
        followers["main"].read.side_effect = None
        feetech_stub = types.ModuleType("lerobot.common.robot_devices.motors.feetech")
        feetech_stub.TorqueMode = types.SimpleNamespace(DISABLED=types.SimpleNamespace(value=0))
        with patch.dict(sys.modules, {feetech_stub.__name__: feetech_stub}), \
             patch.object(unit, "activate_calibration"), patch.object(unit, "set_so100_robot_preset"):
            unit.connect()  # Reconnect only Mock buses, never a real port.
        self.assertIsNone(unit.control_fault)
        unit.send_action(torch.tensor([12.0]))
        self.assertEqual(followers["main"].write.call_args.args[0], "Goal_Position")


if __name__ == "__main__":
    unittest.main()
