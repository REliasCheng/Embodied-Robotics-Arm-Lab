"""Hardware-free lifecycle tests against the checkout's real bus and robot classes."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from test_robotics_faults import bus, robot
from lerobot.common.robot_devices.motors.gbot import GBotConnectionError, GBotMotorsBus
from lerobot.common.robot_devices.robots.manipulator import (
    RobotControlFaultError, RobotDisconnectError,
)
from lerobot.common.robot_devices.utils import (
    RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError,
)
from test_robotics_rework import mock_connect_dependencies


def real_port_handler(serial_factory):
    """Load the original PortHandler with only pyserial's device boundary stubbed."""
    path = (Path(__file__).resolve().parents[1] / "third_party" / "genkiarm" /
            "lerobot/common/robot_devices/motors/GBot/port_handler.py")
    serial_module = types.ModuleType("serial")
    serial_module.Serial = serial_factory
    tools_module = types.ModuleType("serial.tools")
    ports_module = types.ModuleType("serial.tools.list_ports")
    ports_module.comports = lambda: []
    tools_module.list_ports = ports_module
    with patch.dict(sys.modules, {"serial": serial_module, "serial.tools": tools_module,
                                  "serial.tools.list_ports": ports_module}):
        spec = importlib.util.spec_from_file_location("original_gbot_port_handler", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module.PortHandler


class OriginalPortHandlerTests(unittest.TestCase):
    def test_open_close_and_repeated_close(self):
        transport = Mock()
        transport.is_open = True
        transport.close.side_effect = lambda: setattr(transport, "is_open", False)
        factory = Mock(return_value=transport)
        handler = real_port_handler(factory)()
        self.assertTrue(handler.open("MOCK_ONLY"))
        self.assertTrue(handler.is_open())
        handler.close()
        self.assertFalse(handler.is_open())
        handler.close()
        transport.close.assert_called_once()

    def test_close_exception_keeps_handle_and_retry_succeeds(self):
        transport = Mock()
        transport.is_open = True
        failure = OSError("mock close failed")
        transport.close.side_effect = failure
        handler = real_port_handler(Mock(return_value=transport))()
        handler.open("MOCK_ONLY")
        with self.assertRaises(OSError) as cm:
            handler.close()
        self.assertIs(cm.exception, failure)
        self.assertTrue(handler.is_open())
        self.assertIs(handler._PortHandler__serial, transport)
        transport.close.side_effect = lambda: setattr(transport, "is_open", False)
        handler.close()
        self.assertFalse(handler.is_open())
        self.assertIsNone(handler._PortHandler__serial)

    def test_close_returning_without_closing_is_rejected_and_retryable(self):
        transport = Mock()
        transport.is_open = True
        handler = real_port_handler(Mock(return_value=transport))()
        handler.open("MOCK_ONLY")
        with self.assertRaisesRegex(OSError, "remains open"):
            handler.close()
        self.assertTrue(handler.is_open())
        self.assertIs(handler._PortHandler__serial, transport)
        transport.close.side_effect = lambda: setattr(transport, "is_open", False)
        handler.close()
        self.assertFalse(handler.is_open())

    def test_open_error_and_reopen_close_error_preserve_causes(self):
        failure = OSError("mock open failure")
        handler = real_port_handler(Mock(side_effect=failure))()
        with self.assertRaises(OSError) as cm:
            handler.open("MOCK_ONLY")
        self.assertIs(cm.exception, failure)
        self.assertFalse(handler.is_open())
        transport = Mock()
        transport.is_open = True
        transport.close.side_effect = OSError("mock previous close failure")
        factory = Mock(return_value=transport)
        handler = real_port_handler(factory)()
        handler.open("MOCK_ONLY")
        with self.assertRaisesRegex(OSError, "previous close failure"):
            handler.open("SECOND_MOCK")
        factory.assert_called_once()
        self.assertTrue(handler.is_open())


class OriginalBusLifecycleTests(unittest.TestCase):
    def test_open_failure_and_false_open_state_keep_disconnected(self):
        for failure in (OSError("serial unavailable"), None):
            device = GBotMotorsBus("MOCK_ONLY", {"m1": (1, "genki_M1")})
            port = Mock()
            port.open.side_effect = failure
            port.open.return_value = True
            port.is_open.return_value = failure is not None
            device._GBotMotorsBus__port_handler = port
            with self.subTest(failure=failure), self.assertRaises(RobotDeviceNotConnectedError) as cm:
                device.connect()
            self.assertFalse(device.is_connected)
            self.assertIsInstance(cm.exception.__cause__, OSError)
            self.assertTrue(device._cleanup_required)
            with self.assertRaises(GBotConnectionError):
                device.connect()
            port.is_open.return_value = False
            device.disconnect()
            self.assertFalse(device._cleanup_required)

    def test_close_failure_keeps_connected_and_blocks_reconnect(self):
        device, _ = bus(1)
        port = Mock()
        failure = OSError("close failed")
        port.close.side_effect = failure
        device._GBotMotorsBus__port_handler = port
        with self.assertRaises(GBotConnectionError) as cm:
            device.disconnect()
        self.assertEqual((cm.exception.operation, cm.exception.port), ("disconnect", "MOCK_ONLY"))
        self.assertIs(cm.exception.__cause__, failure)
        self.assertTrue(cm.exception.state_uncertain)
        self.assertTrue(device.is_connected)
        with self.assertRaises(GBotConnectionError):
            device.reconnect()
        port.open.assert_not_called()
        port.close.side_effect = None
        port.is_open.return_value = False
        device.disconnect()
        self.assertFalse(device.is_connected)
        with self.assertRaises(RobotDeviceNotConnectedError):
            device.disconnect()

    def test_close_that_returns_open_is_rejected(self):
        device, _ = bus(1)
        port = Mock()
        port.is_open.return_value = True
        device._GBotMotorsBus__port_handler = port
        with self.assertRaises(GBotConnectionError) as cm:
            device.disconnect()
        self.assertIsInstance(cm.exception.__cause__, OSError)
        self.assertTrue(device.is_connected)
        port.is_open.return_value = False
        device.disconnect()

    def test_reconnect_failure_does_not_restore_connected_state(self):
        device, _ = bus(1)
        port = Mock()
        port.is_open.return_value = False
        port.open.side_effect = OSError("reopen failed")
        device._GBotMotorsBus__port_handler = port
        with self.assertRaises(RobotDeviceNotConnectedError):
            device.reconnect()
        self.assertFalse(device.is_connected)
        self.assertEqual(port.close.call_count, 1)
        self.assertTrue(device._cleanup_required)
        device.disconnect()
        self.assertFalse(device._cleanup_required)

    def test_failed_open_then_failed_cleanup_blocks_retry_until_confirmed(self):
        device = GBotMotorsBus("MOCK_ONLY", {"m1": (1, "genki_M1")})
        port = Mock()
        port.open.side_effect = OSError("open failed")
        port.close.side_effect = OSError("close unconfirmed")
        device._GBotMotorsBus__port_handler = port
        with self.assertRaises(RobotDeviceNotConnectedError) as cm:
            device.connect()
        self.assertIsInstance(cm.exception.__cause__, OSError)
        with self.assertRaises(GBotConnectionError):
            device.disconnect()
        self.assertTrue(device._cleanup_required)
        with self.assertRaises(GBotConnectionError):
            device.connect()
        self.assertEqual(port.open.call_count, 1)
        port.close.side_effect = None
        port.is_open.return_value = False
        device.disconnect()
        self.assertFalse(device._cleanup_required)


class OriginalRobotLifecycleTests(unittest.TestCase):
    def test_failed_gbot_open_and_cleanup_remain_tracked_by_robot(self):
        unit, _, followers = robot()
        unit.is_connected = False
        gbot = GBotMotorsBus("MOCK_ONLY", {"m1": (1, "genki_M1")})
        port = Mock()
        port.open.side_effect = OSError("open failed")
        port.close.side_effect = OSError("close failed")
        gbot._GBotMotorsBus__port_handler = port
        unit.leader_arms["main"] = gbot
        with mock_connect_dependencies(unit), self.assertRaises(RobotDeviceNotConnectedError) as cm:
            unit.connect()
        self.assertIs(unit.control_fault, cm.exception)
        self.assertIn("disconnect:leader:main", unit.connection_cleanup_failures)
        self.assertEqual(list(unit._resources_needing_disconnect), ["leader:main"])
        self.assertTrue(unit.is_connected)
        followers["main"].disconnect.assert_called_once()
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        port.close.side_effect = None
        port.is_open.return_value = False
        unit.disconnect()
        unit.acknowledge_control_fault()
        self.assertFalse(unit.is_connected)

    def test_gbot_close_failure_retained_other_devices_cleaned_and_ack_blocked(self):
        unit, leaders, followers = robot(two_followers=True)
        gbot, _ = bus(1)
        port = Mock()
        port.close.side_effect = OSError("mock GBot serial close failure")
        gbot._GBotMotorsBus__port_handler = port
        unit.leader_arms["main"] = gbot
        camera = Mock()
        unit.cameras["camera"] = camera
        prior_fault = RuntimeError("original control failure")
        unit.control_fault = prior_fault
        with self.assertRaises(RobotDisconnectError) as cm:
            unit.disconnect()
        self.assertIn("leader:main", cm.exception.failures)
        self.assertIs(unit.control_fault, prior_fault)
        self.assertTrue(gbot.is_connected)
        self.assertEqual(list(unit._resources_needing_disconnect), ["leader:main"])
        self.assertTrue(unit.is_connected)
        for follower in followers.values():
            follower.disconnect.assert_called_once()
        camera.disconnect.assert_called_once()
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        with self.assertRaises(RobotControlFaultError):
            unit.send_action(__import__("torch").tensor([0.0, 0.0]))
        with self.assertRaises(RobotControlFaultError):
            unit.teleop_step()
        port.close.side_effect = None
        port.is_open.return_value = False
        unit.disconnect()
        self.assertFalse(gbot.is_connected)
        self.assertFalse(unit.is_connected)
        self.assertEqual(unit._resources_needing_disconnect, {})
        camera.disconnect.assert_called_once()
        unit.acknowledge_control_fault()
        self.assertTrue(unit.fault_acknowledged)

    def test_setup_entrypoints_reject_fault_and_disconnected_state(self):
        unit, leaders, followers = robot()
        methods = (unit.activate_calibration, unit.set_so100_robot_preset,
                   unit.set_koch_robot_preset, unit.set_aloha_robot_preset)
        unit.is_connected = False
        for operation in methods:
            with self.subTest(operation=operation.__name__), self.assertRaises(RobotDeviceNotConnectedError):
                operation()
        for arm in (*leaders.values(), *followers.values()):
            arm.write.assert_not_called()
        unit.control_fault = RuntimeError("latched")
        unit.is_connected = True
        for operation in methods:
            with self.subTest(operation=operation.__name__), self.assertRaises(RobotControlFaultError):
                operation()
        for arm in (*leaders.values(), *followers.values()):
            arm.write.assert_not_called()

    def test_direct_preset_failure_latches_and_blocks_other_setup(self):
        unit, _, followers = robot()
        failure = RuntimeError("preset write failed")
        followers["main"].write.side_effect = failure
        with self.assertRaises(RuntimeError) as cm:
            unit.set_so100_robot_preset()
        self.assertIs(cm.exception, failure)
        self.assertIs(unit.control_fault, failure)
        with self.assertRaises(RobotControlFaultError):
            unit.set_so100_robot_preset()
        self.assertEqual(followers["main"].write.call_count, 1)

    def test_recovery_connect_discards_ema_and_uses_new_position(self):
        unit, leaders, followers = robot()
        unit.smoothed_leader_pos["main"] = __import__("torch").tensor([999.0])
        unit.control_fault = RuntimeError("prior fault")
        unit.disconnect()
        unit.acknowledge_control_fault()
        leaders["main"].read.return_value = np.array([0], dtype=np.int32)
        followers["main"].read.return_value = np.array([0], dtype=np.int32)
        with mock_connect_dependencies(unit):
            unit.connect()
        self.assertIsNone(unit.control_fault)
        self.assertEqual(unit.smoothed_leader_pos, {})
        followers["main"].write.reset_mock()
        unit.teleop_step()
        self.assertEqual(followers["main"].write.call_args.args[1].tolist(), [0])


if __name__ == "__main__":
    unittest.main()
