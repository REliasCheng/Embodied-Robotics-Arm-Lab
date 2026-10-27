"""Adversarial, hardware-free tests of the original Robotics control classes."""

import sys
import types
import unittest
from contextlib import contextmanager
from unittest.mock import Mock, patch

import numpy as np
import torch

from test_robotics_faults import Reply, bus, robot
from lerobot.common.robot_devices.motors.gbot import GBotCommunicationError
from lerobot.common.robot_devices.robots.manipulator import (
    ManipulatorRobot,
    ManipulatorRobotConfig,
    RobotControlFaultError,
    RobotDisconnectError,
)


@contextmanager
def mock_connect_dependencies(unit):
    feetech = types.ModuleType("lerobot.common.robot_devices.motors.feetech")
    feetech.TorqueMode = types.SimpleNamespace(DISABLED=types.SimpleNamespace(value=0))
    with patch.dict(sys.modules, {feetech.__name__: feetech}), \
         patch.object(unit, "activate_calibration"), patch.object(unit, "set_so100_robot_preset"):
        yield


class NumericPreflightTests(unittest.TestCase):
    def test_send_action_rejects_nonfinite_and_int32_overflow_before_write(self):
        invalid = [float("nan"), float("inf"), -float("inf"), 2147483648.0, -2147483649.0]
        for value in invalid:
            unit, _, followers = robot()
            with self.subTest(value=value), self.assertRaises(ValueError):
                unit.send_action(torch.tensor([value], dtype=torch.float64))
            followers["main"].write.assert_not_called()
            self.assertIsNotNone(unit.control_fault)
            self.assertEqual(unit.completed_follower_arms, [])
        unit, _, followers = robot()
        with self.assertRaises(ValueError):
            unit.send_action(torch.tensor([2147483647.0], dtype=torch.float32))
        followers["main"].write.assert_not_called()

    def test_send_action_shape_length_and_mixed_follower_input(self):
        for invalid in (torch.tensor(1.0), torch.tensor([[1.0]]), torch.tensor([1.0, 2.0])):
            unit, _, followers = robot()
            with self.subTest(shape=tuple(invalid.shape)), self.assertRaises(ValueError):
                unit.send_action(invalid)
            followers["main"].write.assert_not_called()
            self.assertIsNotNone(unit.control_fault)
        unit, _, followers = robot(two_followers=True)
        with self.assertRaises(ValueError):
            unit.send_action(torch.tensor([1.0, float("nan")]))
        for follower in followers.values():
            follower.write.assert_not_called()

    def test_send_action_prevalidates_all_follower_reads(self):
        unit, _, followers = robot(two_followers=True, max_relative_target=5)
        followers["other"].read.return_value = np.array([float("inf")], dtype=np.float32)
        with self.assertRaises(ValueError):
            unit.send_action(torch.tensor([1.0, 2.0]))
        for follower in followers.values():
            follower.write.assert_not_called()

    def test_teleop_rejects_raw_ema_final_and_second_leader_faults(self):
        for raw in (float("nan"), float("inf"), -float("inf"), 2147483648.0):
            unit, leaders, followers = robot()
            leaders["main"].read.return_value = np.array([raw], dtype=np.float64)
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                unit.teleop_step()
            followers["main"].write.assert_not_called()
            self.assertEqual(unit.smoothed_leader_pos, {})
        unit, _, followers = robot()
        unit.smoothing_factor = float("nan")
        with self.assertRaises(ValueError):
            unit.teleop_step()
        followers["main"].write.assert_not_called()
        unit, _, followers = robot(max_relative_target=float("nan"))
        with self.assertRaises(ValueError):
            unit.teleop_step()
        followers["main"].write.assert_not_called()
        unit, leaders, followers = robot(two_followers=True)
        leaders["other"].read.return_value = np.array([float("nan")], dtype=np.float32)
        with self.assertRaises(ValueError):
            unit.teleop_step()
        for follower in followers.values():
            follower.write.assert_not_called()

    def test_zero_and_int32_boundary_remain_valid(self):
        unit, _, followers = robot()
        unit.send_action(torch.tensor([0.0]))
        self.assertEqual(int(followers["main"].write.call_args.args[1][0]), 0)
        unit.send_action(torch.tensor([2147483647.0], dtype=torch.float64))
        self.assertEqual(int(followers["main"].write.call_args.args[1][0]), 2147483647)

    def test_teleop_second_follower_invalid_present_position_blocks_whole_batch(self):
        unit, _, followers = robot(two_followers=True, max_relative_target=5)
        followers["other"].read.return_value = np.array([float("nan")], dtype=np.float32)
        with self.assertRaises(ValueError):
            unit.teleop_step()
        for follower in followers.values():
            follower.write.assert_not_called()
        self.assertEqual(unit.completed_follower_arms, [])
        self.assertIsNotNone(unit.control_fault)


class OriginalClassIntegrationTests(unittest.TestCase):
    def test_gbot_read_flows_through_actual_teleop_to_mock_follower(self):
        leader, connector = bus(2)
        connector.read.side_effect = [Reply(0), Reply(10)]
        follower = Mock()
        follower.motor_names = ["m1", "m2"]
        unit = ManipulatorRobot(ManipulatorRobotConfig(
            robot_type="so100", leader_arms={"main": leader}, follower_arms={"main": follower}))
        unit.is_connected = True
        unit.teleop_step()
        target = follower.write.call_args.args[1]
        self.assertEqual(target.tolist(), [0, 0])  # 10 raw steps truncates after degree calibration.
        self.assertEqual(connector.read.call_count, 2)

    def test_multi_motor_gbot_failure_never_writes_follower(self):
        leader, connector = bus(3)
        connector.read.side_effect = [Reply(0), Reply(success=False, error=27)]
        follower = Mock()
        follower.motor_names = ["m1", "m2", "m3"]
        unit = ManipulatorRobot(ManipulatorRobotConfig(
            robot_type="so100", leader_arms={"main": leader}, follower_arms={"main": follower}))
        unit.is_connected = True
        with self.assertRaises(GBotCommunicationError) as cm:
            unit.teleop_step()
        self.assertEqual(cm.exception.motor_id, 2)
        follower.write.assert_not_called()
        self.assertIs(unit.control_fault, cm.exception)


class ReconnectAndDisconnectTests(unittest.TestCase):
    def test_acknowledged_reconnect_health_checks_precede_enable(self):
        unit, leaders, followers = robot()
        leaders["main"].read.side_effect = RuntimeError("initial read fault")
        with self.assertRaises(RuntimeError):
            unit.teleop_step()
        unit.disconnect()
        unit.acknowledge_control_fault()
        events = []
        leaders["main"].read.side_effect = lambda _: events.append("leader_read") or np.array([10], dtype=np.float32)
        followers["main"].read.side_effect = lambda _: events.append("follower_read") or np.array([10], dtype=np.float32)
        followers["main"].write.side_effect = lambda register, value: events.append((register, value))
        camera = Mock()
        camera.connect.side_effect = lambda: events.append("camera_connect")
        unit.cameras["mock_camera"] = camera
        with mock_connect_dependencies(unit):
            unit.connect()
        enable_at = events.index(("Torque_Enable", 1))
        self.assertIn("follower_read", events[:enable_at])
        self.assertIn("leader_read", events[:enable_at])
        self.assertIn("camera_connect", events[:enable_at])
        self.assertIn("follower_read", events[enable_at + 1:])
        self.assertIsNone(unit.control_fault)
        unit.disconnect()

    def test_pre_enable_health_failure_keeps_fault_latched(self):
        unit, _, followers = robot()
        unit.is_connected = False
        unit.control_fault = RuntimeError("prior fault")
        unit.fault_acknowledged = True
        failure = TimeoutError("health read failed")
        followers["main"].read.side_effect = failure
        with mock_connect_dependencies(unit), self.assertRaises(TimeoutError) as cm:
            unit.connect()
        self.assertIs(cm.exception, failure)
        self.assertIs(unit.control_fault, failure)
        self.assertFalse(unit.fault_acknowledged)
        self.assertFalse(unit.is_connected)
        self.assertFalse(any(call.args[:2] == ("Torque_Enable", 1)
                             for call in followers["main"].write.call_args_list))
        with self.assertRaises(RobotControlFaultError):
            unit.connect()

    def test_enable_failure_attempts_cleanup_and_latches(self):
        unit, _, followers = robot(two_followers=True)
        unit.is_connected = False
        failure = RuntimeError("second enable failed")
        def second_write(register, value):
            if register == "Torque_Enable" and value == 1:
                raise failure
        followers["other"].write.side_effect = second_write
        with mock_connect_dependencies(unit), self.assertRaises(RuntimeError) as cm:
            unit.connect()
        self.assertIs(cm.exception, failure)
        self.assertTrue(unit.torque_state_uncertain)
        self.assertIs(unit.control_fault, failure)
        self.assertFalse(unit.is_connected)
        for follower in followers.values():
            self.assertIn(("Torque_Enable", 0), [call.args for call in follower.write.call_args_list])
            follower.disconnect.assert_called_once()

    def test_post_enable_failure_reports_cleanup_failures_without_hiding_root(self):
        unit, _, followers = robot()
        unit.is_connected = False
        failure = TimeoutError("post-enable read failed")
        followers["main"].read.side_effect = [np.array([10], dtype=np.float32), failure]
        seen_enable = []
        def write(register, value):
            if register == "Torque_Enable" and value == 1:
                seen_enable.append(True)
            elif register == "Torque_Enable" and value == 0 and seen_enable:
                raise RuntimeError("cleanup torque disable failed")
        followers["main"].write.side_effect = write
        followers["main"].disconnect.side_effect = RuntimeError("disconnect failed")
        with mock_connect_dependencies(unit), self.assertRaises(TimeoutError) as cm:
            unit.connect()
        self.assertIs(cm.exception, failure)
        self.assertIs(unit.control_fault, failure)
        self.assertTrue(unit.torque_state_uncertain)
        self.assertIn("torque_disable:follower:main", failure.cleanup_failures)
        self.assertIn("disconnect:follower:main", failure.cleanup_failures)
        self.assertIn("follower:main", unit._resources_needing_disconnect)
        self.assertTrue(unit.is_connected)
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        followers["main"].disconnect.side_effect = None
        unit.disconnect()
        self.assertFalse(unit.is_connected)

    def test_partial_disconnect_retries_only_unconfirmed_devices(self):
        unit, leaders, followers = robot(two_followers=True)
        camera = Mock()
        unit.cameras["mock_camera"] = camera
        original_fault = RuntimeError("control fault")
        unit.control_fault = original_fault
        followers["main"].disconnect.side_effect = OSError("port did not close")
        with self.assertRaises(RobotDisconnectError) as cm:
            unit.disconnect()
        self.assertIn("follower:main", cm.exception.failures)
        self.assertIs(unit.control_fault, original_fault)
        self.assertTrue(unit.is_connected)
        self.assertEqual(list(unit._resources_needing_disconnect), ["follower:main"])
        followers["other"].disconnect.assert_called_once()
        camera.disconnect.assert_called_once()
        for arm in leaders.values():
            arm.disconnect.assert_called_once()
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        followers["main"].disconnect.side_effect = None
        unit.disconnect()
        self.assertFalse(unit.is_connected)
        self.assertEqual(unit._resources_needing_disconnect, {})
        followers["other"].disconnect.assert_called_once()
        camera.disconnect.assert_called_once()
        unit.acknowledge_control_fault()
        self.assertTrue(unit.fault_acknowledged)


if __name__ == "__main__":
    unittest.main()
