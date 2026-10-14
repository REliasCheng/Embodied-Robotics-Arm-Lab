"""Contains logic to instantiate a robot, read information from its motors and cameras,
and send orders to its motors.
"""
# TODO(rcadene, aliberts): reorganize the codebase into one file per robot, with the associated
# calibration procedure, to make it easy for people to add their own robot.

import json
import logging
import time
import warnings
from dataclasses import dataclass, field, replace
from functools import wraps
from pathlib import Path
from typing import Sequence

import numpy as np
import torch

from lerobot.common.robot_devices.cameras.utils import Camera
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.robots.utils import get_arm_id
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError


class RobotControlFaultError(RuntimeError):
    """Control commands are blocked until a fault is acknowledged and reconnected."""


class RobotDisconnectError(RuntimeError):
    """One or more devices could not be confirmed disconnected."""

    def __init__(self, failures: dict[str, Exception]):
        self.failures = failures.copy()
        super().__init__("Disconnect failed for: " +
                         ", ".join(f"{name} ({error!r})" for name, error in self.failures.items()))


def validate_control_positions(values: torch.Tensor, expected: int, label: str) -> torch.Tensor:
    """Reject malformed or unsafe-to-convert positions before any command is sent."""
    if not isinstance(values, torch.Tensor) or values.ndim != 1 or len(values) != expected:
        raise ValueError(f"{label} must be a one-dimensional tensor with {expected} positions")
    if values.device.type != "cpu" or values.requires_grad or values.dtype == torch.bool or values.is_complex():
        raise ValueError(f"{label} must be a real CPU numeric tensor without gradients")
    numeric = values.to(torch.float64)
    if not torch.isfinite(numeric).all().item():
        raise ValueError(f"{label} contains NaN or infinity")
    bounds = np.iinfo(np.int32)
    if ((numeric < bounds.min) | (numeric > bounds.max)).any().item():
        raise ValueError(f"{label} cannot be safely converted to int32")
    return values


def checked_motor_read(raw: np.ndarray, expected: int, label: str) -> torch.Tensor:
    if not isinstance(raw, np.ndarray):
        raise ValueError(f"{label} must be a NumPy array")
    return validate_control_positions(torch.from_numpy(raw), expected, label)


def validate_gripper_target(value) -> int | float | np.integer | np.floating:
    """Validate a scalar target before connecting or writing to any device."""
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError("gripper_open_degree must be a real numeric scalar")
    try:
        numeric = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError("gripper_open_degree cannot be represented as a finite target") from exc
    bounds = np.iinfo(np.int32)
    if not np.isfinite(numeric) or numeric < bounds.min or numeric > bounds.max:
        raise ValueError("gripper_open_degree cannot be safely converted to int32")
    return value


def ensure_safe_goal_position(
    goal_pos: torch.Tensor, present_pos: torch.Tensor, max_relative_target: float | list[float]
):
    # Cap relative action target magnitude for safety.
    diff = goal_pos - present_pos
    relative_limit = torch.tensor(max_relative_target)
    safe_diff = torch.minimum(diff, relative_limit)
    safe_diff = torch.maximum(safe_diff, -relative_limit)
    safe_goal_pos = present_pos + safe_diff

    if not torch.allclose(goal_pos, safe_goal_pos):
        logging.warning(
            "Relative goal position magnitude had to be clamped to be safe.\n"
            f"  requested relative goal position target: {diff}\n"
            f"    clamped relative goal position target: {safe_diff}"
        )

    return safe_goal_pos


def guarded_setup(operation):
    """Apply the same fault gate to public setup methods and control methods."""
    @wraps(operation)
    def guarded(self, *args, **kwargs):
        if not self._connection_setup_active:
            self._check_control_fault()
            if not self.is_connected:
                raise RobotDeviceNotConnectedError(
                    "ManipulatorRobot must be connected before changing calibration or presets."
                )
        try:
            return operation(self, *args, **kwargs)
        except Exception as exc:
            if not self._connection_setup_active:
                self._latch_control_fault(exc)
            raise
    return guarded


@dataclass
class ManipulatorRobotConfig:
    """
    Example of usage:
    ```python
    ManipulatorRobotConfig()
    ```
    """

    # Define all components of the robot
    robot_type: str = "koch"
    leader_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    follower_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    cameras: dict[str, Camera] = field(default_factory=lambda: {})

    # Optionally limit the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length
    # as the number of motors in your follower arms (assumes all follower arms have the same number of
    # motors).
    max_relative_target: list[float] | float | None = None

    # Optionally set the leader arm in torque mode with the gripper motor set to this angle. This makes it
    # possible to squeeze the gripper and have it spring back to an open position on its own. If None, the
    # gripper is not put in torque mode.
    gripper_open_degree: float | None = None

    def __setattr__(self, prop: str, val):
        if prop == "max_relative_target" and val is not None and isinstance(val, Sequence):
            for name in self.follower_arms:
                if len(self.follower_arms[name].motors) != len(val):
                    raise ValueError(
                        f"len(max_relative_target)={len(val)} but the follower arm with name {name} has "
                        f"{len(self.follower_arms[name].motors)} motors. Please make sure that the "
                        f"`max_relative_target` list has as many parameters as there are motors per arm. "
                        "Note: This feature does not yet work with robots where different follower arms have "
                        "different numbers of motors."
                    )
        super().__setattr__(prop, val)

    def __post_init__(self):
        if self.robot_type not in ["koch", "koch_bimanual", "aloha", "so100", "moss"]:
            raise ValueError(f"Provided robot type ({self.robot_type}) is not supported.")


class ManipulatorRobot:
    # TODO(rcadene): Implement force feedback
    """This class allows to control any manipulator robot of various number of motors.

    Non exaustive list of robots:
    - [Koch v1.0](https://github.com/AlexanderKoch-Koch/low_cost_robot), with and without the wrist-to-elbow expansion, developed
    by Alexander Koch from [Tau Robotics](https://tau-robotics.com)
    - [Koch v1.1](https://github.com/jess-moss/koch-v1-1) developed by Jess Moss
    - [Aloha](https://www.trossenrobotics.com/aloha-kits) developed by Trossen Robotics

    Example of highest frequency teleoperation without camera:
    ```python
    # Defines how to communicate with the motors of the leader and follower arms
    leader_arms = {
        "main": DynamixelMotorsBus(
            port="/dev/tty.usbmodem575E0031751",
            motors={
                # name: (index, model)
                "shoulder_pan": (1, "xl330-m077"),
                "shoulder_lift": (2, "xl330-m077"),
                "elbow_flex": (3, "xl330-m077"),
                "wrist_flex": (4, "xl330-m077"),
                "wrist_roll": (5, "xl330-m077"),
                "gripper": (6, "xl330-m077"),
            },
        ),
    }
    follower_arms = {
        "main": DynamixelMotorsBus(
            port="/dev/tty.usbmodem575E0032081",
            motors={
                # name: (index, model)
                "shoulder_pan": (1, "xl430-w250"),
                "shoulder_lift": (2, "xl430-w250"),
                "elbow_flex": (3, "xl330-m288"),
                "wrist_flex": (4, "xl330-m288"),
                "wrist_roll": (5, "xl330-m288"),
                "gripper": (6, "xl330-m288"),
            },
        ),
    }
    robot = ManipulatorRobot(
        robot_type="koch",
        calibration_dir=".cache/calibration/koch",
        leader_arms=leader_arms,
        follower_arms=follower_arms,
    )

    # Connect motors buses and cameras if any (Required)
    robot.connect()

    while True:
        robot.teleop_step()
    ```

    Example of highest frequency data collection without camera:
    ```python
    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = ManipulatorRobot(
        robot_type="koch",
        calibration_dir=".cache/calibration/koch",
        leader_arms=leader_arms,
        follower_arms=follower_arms,
    )
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of highest frequency data collection with cameras:
    ```python
    # Defines how to communicate with 2 cameras connected to the computer.
    # Here, the webcam of the laptop and the phone (connected in USB to the laptop)
    # can be reached respectively using the camera indices 0 and 1. These indices can be
    # arbitrary. See the documentation of `OpenCVCamera` to find your own camera indices.
    cameras = {
        "laptop": OpenCVCamera(camera_index=0, fps=30, width=640, height=480),
        "phone": OpenCVCamera(camera_index=1, fps=30, width=640, height=480),
    }

    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = ManipulatorRobot(
        robot_type="koch",
        calibration_dir=".cache/calibration/koch",
        leader_arms=leader_arms,
        follower_arms=follower_arms,
        cameras=cameras,
    )
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of controlling the robot with a policy (without running multiple policies in parallel to ensure highest frequency):
    ```python
    # Assumes leader and follower arms + cameras have been instantiated already (see previous example)
    robot = ManipulatorRobot(
        robot_type="koch",
        calibration_dir=".cache/calibration/koch",
        leader_arms=leader_arms,
        follower_arms=follower_arms,
        cameras=cameras,
    )
    robot.connect()
    while True:
        # Uses the follower arms and cameras to capture an observation
        observation = robot.capture_observation()

        # Assumes a policy has been instantiated
        with torch.inference_mode():
            action = policy.select_action(observation)

        # Orders the robot to move
        robot.send_action(action)
    ```

    Example of disconnecting which is not mandatory since we disconnect when the object is deleted:
    ```python
    robot.disconnect()
    ```
    """

    def __init__(
        self,
        config: ManipulatorRobotConfig | None = None,
        calibration_dir: Path | str = ".cache/calibration/koch",
        **kwargs,
    ):
        if config is None:
            config = ManipulatorRobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.calibration_dir = Path(calibration_dir)

        self.robot_type = self.config.robot_type
        self.leader_arms = self.config.leader_arms
        self.follower_arms = self.config.follower_arms
        self.cameras = self.config.cameras
        self.is_connected = False
        self.logs: dict[str, float] = {}

        # EMA filter for smoothing leader positions to reduce follower jitter
        self.smoothing_factor = 0.25  # Higher value = less smoothing, faster response
        self.smoothed_leader_pos: dict[str, torch.Tensor] = {}
        self.control_fault: Exception | None = None
        self.fault_acknowledged = False
        self.completed_follower_arms: list[str] = []
        self._resources_needing_disconnect: dict[str, object] = {}
        self.disconnect_failures: dict[str, Exception] = {}
        self.connection_cleanup_failures: dict[str, Exception] = {}
        self.torque_state_uncertain = False
        self.fault_history: list[Exception] = []
        self._connection_setup_active = False

    def _devices(self):
        for role, devices in (("follower", self.follower_arms), ("leader", self.leader_arms),
                              ("camera", self.cameras)):
            for name, device in devices.items():
                yield f"{role}:{name}", device

    def _disconnect_pending_devices(self):
        if self.is_connected and not self._resources_needing_disconnect:
            self._resources_needing_disconnect = dict(self._devices())
        failures = {}
        for key, device in list(self._resources_needing_disconnect.items()):
            try:
                device.disconnect()
            except Exception as exc:
                failures[key] = exc
            else:
                del self._resources_needing_disconnect[key]
        self.disconnect_failures = failures
        self.is_connected = bool(self._resources_needing_disconnect)
        return failures

    def acknowledge_control_fault(self):
        """Manual fault acknowledgement, only after disconnecting the robot."""
        if self.is_connected or self._resources_needing_disconnect:
            raise RobotControlFaultError("Disconnect before acknowledging the control fault.")
        if self.control_fault is None:
            raise RobotControlFaultError("There is no control fault to acknowledge.")
        self.fault_acknowledged = True

    def _check_control_fault(self):
        if self.control_fault is not None:
            raise RobotControlFaultError(
                "Control fault is latched. Disconnect, acknowledge_control_fault(), then reconnect. "
                f"Original error: {self.control_fault}"
            ) from self.control_fault

    def _latch_control_fault(self, error):
        if self.control_fault is not None and self.control_fault is not error:
            self.fault_history.append(self.control_fault)
        self.control_fault = error
        self.fault_acknowledged = False
        self.smoothed_leader_pos.clear()

    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)

    @property
    def available_arms(self):
        available_arms = []
        for name in self.follower_arms:
            arm_id = get_arm_id(name, "follower")
            available_arms.append(arm_id)
        for name in self.leader_arms:
            arm_id = get_arm_id(name, "leader")
            available_arms.append(arm_id)
        return available_arms

    def connect(self):
        if self.control_fault is not None and not self.fault_acknowledged:
            self._check_control_fault()
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "ManipulatorRobot is already connected. Do not run `robot.connect()` twice."
            )
        if self._resources_needing_disconnect:
            raise RobotControlFaultError("Unconfirmed connected devices remain; retry disconnect before reconnecting.")
        if not self.leader_arms and not self.follower_arms and not self.cameras:
            raise ValueError(
                "ManipulatorRobot doesn't have any device to connect. See example of usage in docstring of the class."
            )

        torque_enable_attempted = []
        self._connection_setup_active = True
        try:
            gripper_target = None
            if self.config.gripper_open_degree is not None:
                if self.robot_type not in ["koch", "koch_bimanual"]:
                    raise NotImplementedError(f"{self.robot_type} does not support gripper_open_degree")
                gripper_target = validate_gripper_target(self.config.gripper_open_degree)

            for key, device in self._devices():
                if key.startswith("camera:"):
                    continue
                # A failed connect() may leave a port open: track the attempt first.
                self._resources_needing_disconnect[key] = device
                device.connect()

            if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
                from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
            elif self.robot_type in ["so100", "moss"]:
                from lerobot.common.robot_devices.motors.feetech import TorqueMode
            elif self.robot_type == "gbot":
                from lerobot.common.robot_devices.motors.gbot import TorqueMode

            # Preserve the torque-off calibration and preset sequence.
            for arm in (*self.follower_arms.values(), *self.leader_arms.values()):
                arm.write("Torque_Enable", TorqueMode.DISABLED.value)
            self.activate_calibration()
            if self.robot_type in ["koch", "koch_bimanual"]:
                self.set_koch_robot_preset()
            elif self.robot_type == "aloha":
                self.set_aloha_robot_preset()
            elif self.robot_type in ["so100", "moss"]:
                self.set_so100_robot_preset()

            # All required health checks precede the normal torque-enable stage.
            for role, arms in (("follower", self.follower_arms), ("leader", self.leader_arms)):
                for name, arm in arms.items():
                    checked_motor_read(arm.read("Present_Position"), len(arm.motor_names),
                                       f"connect {role} {name} position")
            for name, camera in self.cameras.items():
                self._resources_needing_disconnect[f"camera:{name}"] = camera
                camera.connect()

            if gripper_target is not None:
                for name, arm in self.leader_arms.items():
                    torque_enable_attempted.append((f"leader:{name}", arm, "gripper"))
                    arm.write("Torque_Enable", 1, "gripper")
                    arm.write("Goal_Position", gripper_target, "gripper")

            for name, arm in self.follower_arms.items():
                torque_enable_attempted.append((f"follower:{name}", arm, None))
                arm.write("Torque_Enable", 1)

            # A post-enable read may still fail; do not clear a fault until it passes.
            for name, arm in self.follower_arms.items():
                checked_motor_read(arm.read("Present_Position"), len(arm.motor_names),
                                   f"post-enable follower {name} position")
        except Exception as exc:
            cleanup_errors = {}
            if torque_enable_attempted:
                self.torque_state_uncertain = True
            for key, arm, motor_name in reversed(torque_enable_attempted):
                try:
                    if motor_name is None:
                        arm.write("Torque_Enable", 0)
                    else:
                        arm.write("Torque_Enable", 0, motor_name)
                except Exception as cleanup_exc:
                    cleanup_errors[f"torque_disable:{key}"] = cleanup_exc
            cleanup_errors.update(
                {f"disconnect:{key}": failure
                 for key, failure in self._disconnect_pending_devices().items()}
            )
            self.connection_cleanup_failures = cleanup_errors
            for key, failure in cleanup_errors.items():
                logging.error("Connection cleanup failed for %s: %r", key, failure)
            try:
                exc.cleanup_failures = cleanup_errors
            except (AttributeError, TypeError):
                # The same details remain available on the robot instance.
                logging.error("Could not attach cleanup failures to the original exception")
            self._latch_control_fault(exc)
            raise
        finally:
            self._connection_setup_active = False

        self.is_connected = True
        self.disconnect_failures = {}
        if self.fault_acknowledged:
            self.fault_history.append(self.control_fault)
            self.control_fault = None
            self.fault_acknowledged = False
            self.smoothed_leader_pos.clear()

    @guarded_setup
    def activate_calibration(self):
        """After calibration all motors function in human interpretable ranges.
        Rotations are expressed in degrees in nominal range of [-180, 180],
        and linear motions (like gripper of Aloha) in nominal range of [0, 100].
        """

        def load_or_run_calibration_(name, arm, arm_type):
            arm_id = get_arm_id(name, arm_type)
            arm_calib_path = self.calibration_dir / f"{arm_id}.json"

            if arm_calib_path.exists():
                with open(arm_calib_path) as f:
                    calibration = json.load(f)
            else:
                # TODO(rcadene): display a warning in __init__ if calibration file not available
                print(f"Missing calibration file '{arm_calib_path}'")

                if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
                    from lerobot.common.robot_devices.robots.dynamixel_calibration import run_arm_calibration

                    calibration = run_arm_calibration(arm, self.robot_type, name, arm_type)

                elif self.robot_type in ["so100", "moss"]:
                    from lerobot.common.robot_devices.robots.feetech_calibration import (
                        run_arm_auto_calibration,
                        run_arm_manual_calibration,
                    )

                    # TODO(rcadene): better way to handle mocking + test run_arm_auto_calibration
                    if arm_type == "leader" or arm.mock:
                        calibration = run_arm_manual_calibration(arm, self.robot_type, name, arm_type)
                    elif arm_type == "follower":
                        calibration = run_arm_auto_calibration(arm, self.robot_type, name, arm_type)
                    else:
                        raise ValueError(arm_type)
                elif self.robot_type == "gbot":
                    from lerobot.common.robot_devices.robots.gbot_calibration import run_arm_manual_calibration

                    calibration = run_arm_manual_calibration(arm, self.robot_type, name, arm_type)

                print(f"Calibration is done! Saving calibration file '{arm_calib_path}'")
                arm_calib_path.parent.mkdir(parents=True, exist_ok=True)
                with open(arm_calib_path, "w") as f:
                    json.dump(calibration, f)

            return calibration

        for name, arm in self.follower_arms.items():
            calibration = load_or_run_calibration_(name, arm, "follower")
            arm.set_calibration(calibration)
        for name, arm in self.leader_arms.items():
            calibration = load_or_run_calibration_(name, arm, "leader")
            arm.set_calibration(calibration)

    @guarded_setup
    def set_koch_robot_preset(self):
        def set_operating_mode_(arm):
            from lerobot.common.robot_devices.motors.dynamixel import TorqueMode

            if (arm.read("Torque_Enable") != TorqueMode.DISABLED.value).any():
                raise ValueError("To run set robot preset, the torque must be disabled on all motors.")

            # Use 'extended position mode' for all motors except gripper, because in joint mode the servos can't
            # rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while assembling the arm,
            # you could end up with a servo with a position 0 or 4095 at a crucial point See [
            # https://emanual.robotis.com/docs/en/dxl/x/x_series/#operating-mode11]
            all_motors_except_gripper = [name for name in arm.motor_names if name != "gripper"]
            if len(all_motors_except_gripper) > 0:
                # 4 corresponds to Extended Position on Koch motors
                arm.write("Operating_Mode", 4, all_motors_except_gripper)

            # Use 'position control current based' for gripper to be limited by the limit of the current.
            # For the follower gripper, it means it can grasp an object without forcing too much even tho,
            # it's goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
            # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
            # to make it move, and it will move back to its original target position when we release the force.
            # 5 corresponds to Current Controlled Position on Koch gripper motors "xl330-m077, xl330-m288"
            arm.write("Operating_Mode", 5, "gripper")

        for name in self.follower_arms:
            set_operating_mode_(self.follower_arms[name])

            # Set better PID values to close the gap between recorded states and actions
            # TODO(rcadene): Implement an automatic procedure to set optimial PID values for each motor
            self.follower_arms[name].write("Position_P_Gain", 1500, "elbow_flex")
            self.follower_arms[name].write("Position_I_Gain", 0, "elbow_flex")
            self.follower_arms[name].write("Position_D_Gain", 600, "elbow_flex")

        if self.config.gripper_open_degree is not None:
            for name in self.leader_arms:
                set_operating_mode_(self.leader_arms[name])

    @guarded_setup
    def set_aloha_robot_preset(self):
        def set_shadow_(arm):
            # Set secondary/shadow ID for shoulder and elbow. These joints have two motors.
            # As a result, if only one of them is required to move to a certain position,
            # the other will follow. This is to avoid breaking the motors.
            if "shoulder_shadow" in arm.motor_names:
                shoulder_idx = arm.read("ID", "shoulder")
                arm.write("Secondary_ID", shoulder_idx, "shoulder_shadow")

            if "elbow_shadow" in arm.motor_names:
                elbow_idx = arm.read("ID", "elbow")
                arm.write("Secondary_ID", elbow_idx, "elbow_shadow")

        for name in self.follower_arms:
            set_shadow_(self.follower_arms[name])

        for name in self.leader_arms:
            set_shadow_(self.leader_arms[name])

        for name in self.follower_arms:
            # Set a velocity limit of 131 as advised by Trossen Robotics
            self.follower_arms[name].write("Velocity_Limit", 131)

            # Use 'extended position mode' for all motors except gripper, because in joint mode the servos can't
            # rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while assembling the arm,
            # you could end up with a servo with a position 0 or 4095 at a crucial point See [
            # https://emanual.robotis.com/docs/en/dxl/x/x_series/#operating-mode11]
            all_motors_except_gripper = [
                name for name in self.follower_arms[name].motor_names if name != "gripper"
            ]
            if len(all_motors_except_gripper) > 0:
                # 4 corresponds to Extended Position on Aloha motors
                self.follower_arms[name].write("Operating_Mode", 4, all_motors_except_gripper)

            # Use 'position control current based' for follower gripper to be limited by the limit of the current.
            # It can grasp an object without forcing too much even tho,
            # it's goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
            # 5 corresponds to Current Controlled Position on Aloha gripper follower "xm430-w350"
            self.follower_arms[name].write("Operating_Mode", 5, "gripper")

            # Note: We can't enable torque on the leader gripper since "xc430-w150" doesn't have
            # a Current Controlled Position mode.

        if self.config.gripper_open_degree is not None:
            warnings.warn(
                f"`gripper_open_degree` is set to {self.config.gripper_open_degree}, but None is expected for Aloha instead",
                stacklevel=1,
            )

    @guarded_setup
    def set_so100_robot_preset(self):
        for name in self.follower_arms:
            # Mode=0 for Position Control
            self.follower_arms[name].write("Mode", 0)
            # Restore a stable baseline for PID and acceleration.
            # The main stabilization will be handled by the EMA filter on the leader's position.
            # P_Coefficient is increased for faster response.
            # D_Coefficient is kept low to prevent jitter.
            self.follower_arms[name].write("P_Coefficient", 10)
            self.follower_arms[name].write("I_Coefficient", 0)
            self.follower_arms[name].write("D_Coefficient", 5)
            self.follower_arms[name].write("Maximum_Acceleration", 100)
            self.follower_arms[name].write("Acceleration", 50)

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        self._check_control_fault()
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )
        self.completed_follower_arms = []
        try:
            return self._teleop_step_unchecked(record_data)
        except Exception as exc:
            self._latch_control_fault(exc)
            raise

    def _teleop_step_unchecked(self, record_data=False):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        # Prepare to assign the position of the leader to the follower
        leader_pos = {}
        for name in self.leader_arms:
            before_lread_t = time.perf_counter()
            expected = len(self.leader_arms[name].motor_names)
            pos = checked_motor_read(self.leader_arms[name].read("Present_Position"), expected,
                                     f"leader {name} position")

            # Initialize or update smoothed position with EMA filter.
            if name not in self.smoothed_leader_pos:
                self.smoothed_leader_pos[name] = pos.clone()

            self.smoothed_leader_pos[name] = (
                self.smoothing_factor * pos
                + (1 - self.smoothing_factor) * self.smoothed_leader_pos[name]
            )
            validate_control_positions(self.smoothed_leader_pos[name], expected, f"leader {name} EMA")
            leader_pos[name] = self.smoothed_leader_pos[name]
            self.logs[f"read_leader_{name}_pos_dt_s"] = time.perf_counter() - before_lread_t

        # Prepare and validate the entire batch before writing to any follower.
        follower_goal_pos = {}
        prepared_writes = []
        for name in self.follower_arms:
            goal_pos = leader_pos[name]
            expected = len(self.follower_arms[name].motor_names)
            validate_control_positions(goal_pos, expected, f"follower {name} target")

            # Cap goal position when too far away from present position.
            # Slower fps expected due to reading from the follower.
            if self.config.max_relative_target is not None:
                present_pos = checked_motor_read(self.follower_arms[name].read("Present_Position"), expected,
                                                 f"follower {name} present position")
                goal_pos = ensure_safe_goal_position(goal_pos, present_pos, self.config.max_relative_target)

            validate_control_positions(goal_pos, expected, f"follower {name} final target")
            follower_goal_pos[name] = goal_pos
            prepared_writes.append((name, goal_pos.numpy().astype(np.int32)))

        for name, target in prepared_writes:
            before_fwrite_t = time.perf_counter()
            self.follower_arms[name].write("Goal_Position", target)
            self.completed_follower_arms.append(name)
            self.logs[f"write_follower_{name}_goal_pos_dt_s"] = time.perf_counter() - before_fwrite_t

        # Early exit when recording data is not requested
        if not record_data:
            return

        # TODO(rcadene): Add velocity and other info
        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            before_fread_t = time.perf_counter()
            follower_pos[name] = checked_motor_read(
                self.follower_arms[name].read("Present_Position"),
                len(self.follower_arms[name].motor_names), f"follower {name} recorded position"
            )
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - before_fread_t

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = torch.cat(state)

        # Create action by concatenating follower goal position
        action = []
        for name in self.follower_arms:
            if name in follower_goal_pos:
                action.append(follower_goal_pos[name])
        action = torch.cat(action)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state
        action_dict["action"] = action
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict

    def capture_observation(self):
        """The returned observations do not have a batch dimension."""
        self._check_control_fault()
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        try:
            return self._capture_observation_unchecked()
        except Exception as exc:
            self._latch_control_fault(exc)
            raise

    def _capture_observation_unchecked(self):

        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            before_fread_t = time.perf_counter()
            follower_pos[name] = checked_motor_read(
                self.follower_arms[name].read("Present_Position"),
                len(self.follower_arms[name].motor_names), f"follower {name} observed position"
            )
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - before_fread_t

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = torch.cat(state)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = state
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]
        return obs_dict

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        """Command the follower arms to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Args:
            action: tensor containing the concatenated goal positions for the follower arms.
        """
        self._check_control_fault()
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )
        self.completed_follower_arms = []
        try:
            return self._send_action_unchecked(action)
        except Exception as exc:
            self._latch_control_fault(exc)
            raise

    def _send_action_unchecked(self, action: torch.Tensor) -> torch.Tensor:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        expected_total = sum(len(arm.motor_names) for arm in self.follower_arms.values())
        validate_control_positions(action, expected_total, "action")
        from_idx = 0
        to_idx = 0
        action_sent = []
        prepared_writes = []
        for name in self.follower_arms:
            # Get goal position of each follower arm by splitting the action vector
            to_idx += len(self.follower_arms[name].motor_names)
            goal_pos = action[from_idx:to_idx]
            from_idx = to_idx
            expected = len(self.follower_arms[name].motor_names)

            # Cap goal position when too far away from present position.
            # Slower fps expected due to reading from the follower.
            if self.config.max_relative_target is not None:
                present_pos = checked_motor_read(self.follower_arms[name].read("Present_Position"), expected,
                                                 f"follower {name} present position")
                goal_pos = ensure_safe_goal_position(goal_pos, present_pos, self.config.max_relative_target)

            validate_control_positions(goal_pos, expected, f"follower {name} final target")
            action_sent.append(goal_pos)
            prepared_writes.append((name, goal_pos.numpy().astype(np.int32)))

        for name, target in prepared_writes:
            self.follower_arms[name].write("Goal_Position", target)
            self.completed_follower_arms.append(name)

        return torch.cat(action_sent)

    def print_logs(self):
        pass
        # TODO(aliberts): move robot-specific logs logic here

    def disconnect(self):
        if not self.is_connected and not self._resources_needing_disconnect:
            raise RobotDeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()` before disconnecting."
            )

        failures = self._disconnect_pending_devices()
        if failures:
            error = RobotDisconnectError(failures)
            if self.control_fault is None:
                self._latch_control_fault(error)
            raise error

    def __del__(self):
        if getattr(self, "is_connected", False) or getattr(self, "_resources_needing_disconnect", {}):
            try:
                self.disconnect()
            except Exception:
                logging.exception("ManipulatorRobot destructor could not confirm all devices disconnected")
