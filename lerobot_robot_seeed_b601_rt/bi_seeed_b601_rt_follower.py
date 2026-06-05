import logging
import time
from concurrent.futures import ThreadPoolExecutor
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_bi_seeed_b601_rt_follower import BiSeeedB601RTFollowerConfig
from .config_seeed_b601_rt_follower import SeeedB601RTFollowerConfig
from .seeed_b601_rt_follower import SeeedB601RTFollower


logger = logging.getLogger(__name__)


class BiSeeedB601RTFollower(Robot):
    """Dual Seeed B601 follower composed from two RT B601 follower instances."""

    config_class = BiSeeedB601RTFollowerConfig
    name = "bi_seeed_b601_rt_follower"

    def __init__(self, config: BiSeeedB601RTFollowerConfig):
        super().__init__(config)
        self.config = config
        self.left = SeeedB601RTFollower(self._single_config("left"))
        self.right = SeeedB601RTFollower(self._single_config("right"))
        self.cameras = make_cameras_from_configs(config.cameras)

    def _single_config(self, side: str) -> SeeedB601RTFollowerConfig:
        if side not in {"left", "right"}:
            raise ValueError(f"Unknown side {side!r}.")

        return SeeedB601RTFollowerConfig(
            id=self.config.left_id if side == "left" else self.config.right_id,
            calibration_dir=self.calibration_dir,
            port=self.config.left_port if side == "left" else self.config.right_port,
            can_adapter=self.config.can_adapter,
            arm_cfg_path=(
                self.config.left_arm_cfg_path if side == "left" else self.config.right_arm_cfg_path
            ),
            disable_torque_on_disconnect=self.config.disable_torque_on_disconnect,
            max_relative_target=self.config.max_relative_target,
            action_mode=self.config.action_mode,
            control_mode=self.config.control_mode,
            control_gripper=self.config.control_gripper,
            enabled_gripper_force=self.config.enabled_gripper_force,
            gripper_force_pos_torque_ratio=self.config.gripper_force_pos_torque_ratio,
            enable_observation_joint_pos=self.config.enable_observation_joint_pos,
            enable_observation_joint_vel=self.config.enable_observation_joint_vel,
            enable_observation_joint_torque=self.config.enable_observation_joint_torque,
            enable_observation_gripper_vel=self.config.enable_observation_gripper_vel,
            enable_observation_gripper_torque=self.config.enable_observation_gripper_torque,
            rt_rate=self.config.rt_rate,
            rt_command_gap_us=self.config.rt_command_gap_us,
            rt_priority=(
                self.config.left_rt_priority if side == "left" else self.config.right_rt_priority
            ),
            rt_cpu=self.config.left_rt_cpu if side == "left" else self.config.right_rt_cpu,
            damiao_tx_debug=self.config.damiao_tx_debug,
            debug_motion=self.config.debug_motion,
            debug_motion_interval_s=self.config.debug_motion_interval_s,
            tool_tcp_offset_xyz=self.config.tool_tcp_offset_xyz,
            tool_tcp_offset_rpy_deg=self.config.tool_tcp_offset_rpy_deg,
            motor_can_ids=self.config.motor_can_ids,
            motor_models=self.config.motor_models,
            motor_vendor=self.config.motor_vendor,
            motor_vendors=self.config.motor_vendors,
            pos_vel_velocity=self.config.pos_vel_velocity,
            return_to_initial_vlim_deg_s=self.config.return_to_initial_vlim_deg_s,
            joint_limits=self.config.joint_limits,
            cameras={},
        )

    @staticmethod
    def _prefix_key(side: str, key: str) -> str:
        return f"{side}_{key}"

    @staticmethod
    def _strip_side_prefix(key: str, side: str) -> str | None:
        prefix = f"{side}_"
        if key.startswith(prefix):
            return key[len(prefix):]
        return None

    @staticmethod
    def _prefix_dict(side: str, data: dict[str, Any]) -> dict[str, Any]:
        return {BiSeeedB601RTFollower._prefix_key(side, key): value for key, value in data.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        camera_features = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }
        return {
            **self._prefix_dict("left", self.left.observation_features),
            **self._prefix_dict("right", self.right.observation_features),
            **camera_features,
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **self._prefix_dict("left", self.left.action_features),
            **self._prefix_dict("right", self.right.action_features),
        }

    @property
    def is_connected(self) -> bool:
        return (
            self.left.is_connected
            and self.right.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        return self.left.is_calibrated and self.right.is_calibrated

    def calibrate(self) -> None:
        self.left.calibrate()
        self.right.calibrate()

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(
            "Connecting dual B601 arms: left=%s right=%s...",
            self.config.left_port,
            self.config.right_port,
        )
        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = [
                    executor.submit(self.left.connect, calibrate),
                    executor.submit(self.right.connect, calibrate),
                ]
                for future in futures:
                    future.result()

            if not self.cameras:
                logger.info("No cameras configured; skipping camera connect.")
            else:
                logger.info("Connecting %d camera(s): %s...", len(self.cameras), ", ".join(self.cameras))
                for cam in self.cameras.values():
                    cam.connect()
                logger.info("All cameras connected.")
        except Exception:
            self._disconnect_connected_children()
            raise

        logger.info("%s connected.", self)

    def configure(self) -> None:
        self.left.configure()
        self.right.configure()

    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.get_observation)
            right_future = executor.submit(self.right.get_observation)
            left_obs = left_future.result()
            right_obs = right_future.result()

        obs: RobotObservation = {
            **self._prefix_dict("left", left_obs),
            **self._prefix_dict("right", right_obs),
        }
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()

        logger.debug("%s get_observation took: %.1fms", self, (time.perf_counter() - start) * 1e3)
        return obs

    def _split_action(self, action: RobotAction) -> tuple[RobotAction, RobotAction]:
        left_action: RobotAction = {}
        right_action: RobotAction = {}
        for key, value in action.items():
            left_key = self._strip_side_prefix(key, "left")
            if left_key is not None:
                left_action[left_key] = value
                continue

            right_key = self._strip_side_prefix(key, "right")
            if right_key is not None:
                right_action[right_key] = value

        return left_action, right_action

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_action, right_action = self._split_action(action)
        if not left_action and not right_action:
            raise ValueError("Dual B601 action must contain left_* or right_* keys.")

        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.send_action, left_action) if left_action else None
            right_future = executor.submit(self.right.send_action, right_action) if right_action else None
            sent_left = left_future.result() if left_future is not None else {}
            sent_right = right_future.result() if right_future is not None else {}

        return {
            **self._prefix_dict("left", sent_left),
            **self._prefix_dict("right", sent_right),
        }

    def get_current_tcp_pose_quat(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.get_current_tcp_pose_quat)
            right_future = executor.submit(self.right.get_current_tcp_pose_quat)
            return left_future.result(), right_future.result()

    def reset_to_initial_position(self) -> None:
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(self.left.reset_to_initial_position),
                executor.submit(self.right.reset_to_initial_position),
            ]
            for future in futures:
                future.result()

    def _disconnect_connected_children(self) -> None:
        for arm in (self.left, self.right):
            try:
                if arm.arm is not None:
                    arm.disconnect()
            except Exception:
                logger.debug("Failed to disconnect %s during cleanup.", arm, exc_info=True)
        for cam in self.cameras.values():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception:
                logger.debug("Failed to disconnect camera during cleanup.", exc_info=True)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = [
                    executor.submit(self.left.disconnect),
                    executor.submit(self.right.disconnect),
                ]
                for future in futures:
                    future.result()
        finally:
            for cam in self.cameras.values():
                cam.disconnect()

        logger.info("%s disconnected.", self)
