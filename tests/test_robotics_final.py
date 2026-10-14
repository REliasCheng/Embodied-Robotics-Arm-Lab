"""Final hardware-free control-boundary tests using the checkout's robot class."""

import sys
import types
import unittest
from contextlib import contextmanager
from unittest.mock import Mock, patch

import numpy as np
import torch

from test_robotics_faults import bus, robot
from lerobot.common.robot_devices.robots.manipulator import (
    ManipulatorRobot, ManipulatorRobotConfig, RobotControlFaultError,
)


@contextmanager
def koch_dependencies(unit):
    module = types.ModuleType("lerobot.common.robot_devices.motors.dynamixel")
    module.TorqueMode = types.SimpleNamespace(DISABLED=types.SimpleNamespace(value=0))
    with patch.dict(sys.modules, {module.__name__: module}), patch.object(unit, "activate_calibration"):
        yield


def koch(*, bimanual=False, target=0, camera=False):
    names = ("left", "right") if bimanual else ("main",)
    events = []
    leaders, followers = {}, {}
    for role, mapping in (("leader", leaders), ("follower", followers)):
        for name in names:
            arm = Mock()
            arm.motor_names = ["joint", "gripper"]
            arm.read.side_effect = lambda register, role=role, name=name: (
                events.append((role, name, "read", register)) or
                np.array([0, 0], dtype=np.int32)
            )
            arm.write.side_effect = lambda register, value, *args, role=role, name=name: (
                events.append((role, name, "write", register, value, *args))
            )
            arm.connect.side_effect = lambda role=role, name=name: events.append((role, name, "connect"))
            arm.disconnect.side_effect = lambda role=role, name=name: events.append((role, name, "disconnect"))
            mapping[name] = arm
    cameras = {}
    if camera:
        cam = Mock()
        cam.connect.side_effect = lambda: events.append(("camera", "main", "connect"))
        cam.disconnect.side_effect = lambda: events.append(("camera", "main", "disconnect"))
        cameras["main"] = cam
    unit = ManipulatorRobot(ManipulatorRobotConfig(
        robot_type="koch_bimanual" if bimanual else "koch",
        leader_arms=leaders, follower_arms=followers, cameras=cameras,
        gripper_open_degree=target,
    ))
    return unit, leaders, followers, cameras, events


def is_enable(event):
    return len(event) > 4 and event[2:4] == ("write", "Torque_Enable") and event[4] == 1


def is_goal(event):
    return len(event) > 4 and event[2:4] == ("write", "Goal_Position")


class KochConnectionTests(unittest.TestCase):
    def test_normal_koch_and_bimanual_have_one_post_health_gripper_goal(self):
        for bimanual, target in ((False, 0), (True, 25.5), (True, None)):
            unit, leaders, followers, cameras, events = koch(
                bimanual=bimanual, target=target, camera=True
            )
            with self.subTest(bimanual=bimanual, target=target), koch_dependencies(unit):
                unit.connect()
            health = [i for i, e in enumerate(events) if e[2:4] == ("read", "Present_Position")]
            camera_at = events.index(("camera", "main", "connect"))
            enables = [i for i, e in enumerate(events) if is_enable(e)]
            self.assertTrue(enables)
            self.assertLess(max(health[:2 * len(leaders)]), min(enables))
            self.assertLess(camera_at, min(enables))
            for name in leaders:
                goals = [e for e in events if e[:4] == ("leader", name, "write", "Goal_Position")]
                leader_enables = [e for e in events if e[:4] == ("leader", name, "write", "Torque_Enable")
                                  and e[4] == 1]
                self.assertEqual(len(goals), int(target is not None))
                self.assertEqual(len(leader_enables), int(target is not None))
                if goals:
                    self.assertEqual(goals[0][4], target)
            for name in followers:
                self.assertEqual(sum(e[:4] == ("follower", name, "write", "Torque_Enable")
                                     and e[4] == 1 for e in events), 1)
            unit.disconnect()

    def test_invalid_gripper_values_fail_before_connect_or_write(self):
        invalid = (float("nan"), float("inf"), -float("inf"), 2147483648,
                   -2147483649, "12", [12], (12,), torch.tensor([12]), True)
        for target in invalid:
            unit, leaders, followers, _, events = koch(target=target)
            with self.subTest(target=repr(target)), koch_dependencies(unit), self.assertRaises(ValueError):
                unit.connect()
            self.assertEqual(events, [])
            self.assertIsNotNone(unit.control_fault)
            self.assertFalse(unit.is_connected)
            with self.assertRaises(RobotControlFaultError):
                unit.connect()

    def test_each_pre_enable_failure_blocks_motion_and_latches(self):
        stages = ("follower_connect", "leader_connect", "calibration", "preset",
                  "follower_health", "leader_health", "camera_connect")
        for stage in stages:
            unit, leaders, followers, cameras, events = koch(target=30, camera=True)
            failure = RuntimeError(stage)
            if stage == "follower_connect":
                followers["main"].connect.side_effect = failure
            elif stage == "leader_connect":
                leaders["main"].connect.side_effect = failure
            elif stage == "calibration":
                calibration = patch.object(unit, "activate_calibration", side_effect=failure)
            elif stage == "preset":
                followers["main"].write.side_effect = lambda reg, val, *rest: (
                    (_ for _ in ()).throw(failure) if reg == "Operating_Mode" else
                    events.append(("follower", "main", "write", reg, val, *rest))
                )
            elif stage == "follower_health":
                followers["main"].read.side_effect = lambda reg: (
                    (_ for _ in ()).throw(failure) if reg == "Present_Position" else
                    np.array([0, 0], dtype=np.int32)
                )
            elif stage == "leader_health":
                leaders["main"].read.side_effect = lambda reg: (
                    (_ for _ in ()).throw(failure) if reg == "Present_Position" else
                    np.array([0, 0], dtype=np.int32)
                )
            elif stage == "camera_connect":
                cameras["main"].connect.side_effect = failure
            with self.subTest(stage=stage), koch_dependencies(unit), self.assertRaises(RuntimeError) as cm:
                if stage == "calibration":
                    with calibration:
                        unit.connect()
                else:
                    unit.connect()
            self.assertIs(cm.exception, failure)
            self.assertIs(unit.control_fault, failure)
            self.assertFalse(any(is_goal(e) or is_enable(e) for e in events))
            self.assertFalse(unit.torque_state_uncertain)
            self.assertFalse(unit.is_connected)
            self.assertTrue(all(a.disconnect.called for a in (*leaders.values(), *followers.values())
                                if a.connect.called))

    def test_gripper_enable_and_goal_failures_are_tracked_before_cleanup(self):
        for failing_register in ("Torque_Enable", "Goal_Position"):
            unit, leaders, followers, _, events = koch(target=20)
            failure = RuntimeError(failing_register)
            leader = leaders["main"]
            original_write = leader.write.side_effect
            leader.write.side_effect = lambda reg, val, *rest: (
                (_ for _ in ()).throw(failure)
                if reg == failing_register and (reg != "Torque_Enable" or val == 1)
                else original_write(reg, val, *rest)
            )
            with self.subTest(register=failing_register), koch_dependencies(unit), self.assertRaises(RuntimeError) as cm:
                unit.connect()
            self.assertIs(cm.exception, failure)
            self.assertIs(unit.control_fault, failure)
            self.assertTrue(unit.torque_state_uncertain)
            self.assertIn(("leader", "main", "write", "Torque_Enable", 0, "gripper"), events)
            self.assertFalse(any(is_enable(e) for e in events if e[:2] == ("follower", "main")))
            self.assertFalse(unit.is_connected)

    def test_cleanup_failure_does_not_hide_original_and_blocks_acknowledgement(self):
        unit, leaders, followers, _, events = koch(target=20)
        failure = RuntimeError("enable failed")
        def bad_write(reg, value, *rest):
            if reg == "Torque_Enable" and value == 1:
                raise failure
            if reg == "Torque_Enable" and value == 0 and rest:
                raise OSError("disable unconfirmed")
            events.append(("leader", "main", "write", reg, value, *rest))
        leaders["main"].write.side_effect = bad_write
        leaders["main"].disconnect.side_effect = OSError("port remains open")
        with koch_dependencies(unit), self.assertRaises(RuntimeError) as cm:
            unit.connect()
        self.assertIs(cm.exception, failure)
        self.assertIn("torque_disable:leader:main", unit.connection_cleanup_failures)
        self.assertIn("disconnect:leader:main", unit.connection_cleanup_failures)
        self.assertTrue(unit.torque_state_uncertain)
        self.assertTrue(unit.is_connected)
        with self.assertRaises(RobotControlFaultError):
            unit.acknowledge_control_fault()
        leaders["main"].disconnect.side_effect = None
        unit.disconnect()
        unit.acknowledge_control_fault()


class ObservationAndNumericTests(unittest.TestCase):
    def test_calibration_unknown_mode_fails_explicitly_without_io(self):
        device, connector = bus(1)
        device.calibration["calib_mode"] = ["UNSUPPORTED"]
        for operation in (device.apply_calibration, device.autocorrect_calibration,
                          device.revert_calibration):
            with self.assertRaisesRegex(ValueError, "Invalid calibration mode"):
                operation([0], ["m1"])
        connector.write.assert_not_called()

    def test_float32_rounded_int32_edge_is_rejected(self):
        unit, _, followers = robot()
        with self.assertRaises(ValueError):
            unit.send_action(torch.tensor([2147483647.0], dtype=torch.float32))
        followers["main"].write.assert_not_called()

    def test_capture_observation_fault_latches_and_blocks_repeated_capture(self):
        unit, _, followers = robot()
        failure = TimeoutError("observation read")
        followers["main"].read.side_effect = failure
        with self.assertRaises(TimeoutError) as cm:
            unit.capture_observation()
        self.assertIs(cm.exception, failure)
        self.assertIs(unit.control_fault, failure)
        followers["main"].read.side_effect = None
        with self.assertRaises(RobotControlFaultError):
            unit.capture_observation()
        self.assertEqual(followers["main"].read.call_count, 1)

    def test_record_data_failure_retains_prior_write_fact_and_latches(self):
        unit, _, followers = robot()
        followers["main"].read.side_effect = TimeoutError("post-write observation")
        with self.assertRaises(TimeoutError):
            unit.teleop_step(record_data=True)
        self.assertEqual(unit.completed_follower_arms, ["main"])
        self.assertEqual(followers["main"].write.call_count, 1)
        self.assertEqual(unit.smoothed_leader_pos, {})
        with self.assertRaises(RobotControlFaultError):
            unit.teleop_step(record_data=True)

    def test_empty_wrong_shape_negative_float_and_multi_follower_lengths(self):
        for action in (torch.empty(0), torch.tensor([[1.0]]), torch.tensor([1.0, 2.0, 3.0])):
            unit, _, followers = robot(two_followers=True)
            with self.assertRaises(ValueError):
                unit.send_action(action)
            for follower in followers.values():
                follower.write.assert_not_called()
        unit, _, followers = robot(two_followers=True)
        unit.send_action(torch.tensor([-5.5, 0.0]))
        self.assertEqual([int(f.write.call_args.args[1][0]) for f in followers.values()], [-5, 0])

    def test_existing_ema_history_is_discarded_after_bad_read(self):
        unit, leaders, followers = robot()
        unit.teleop_step()
        self.assertIn("main", unit.smoothed_leader_pos)
        leaders["main"].read.return_value = np.array([float("nan")], dtype=np.float32)
        with self.assertRaises(ValueError):
            unit.teleop_step()
        self.assertEqual(unit.smoothed_leader_pos, {})
        self.assertEqual(followers["main"].write.call_count, 1)


if __name__ == "__main__":
    unittest.main()
