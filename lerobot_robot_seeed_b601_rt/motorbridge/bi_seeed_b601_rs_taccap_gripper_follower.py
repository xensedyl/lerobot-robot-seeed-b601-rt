import logging
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_bi_seeed_b601_rs_taccap_gripper_follower import (
    BiSeeedB601RSTacCapGripperFollowerConfig,
)
from .config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig
from .seeed_b601_rs_follower import SeeedB601RSFollower


logger = logging.getLogger(__name__)


class BiSeeedB601RSTacCapGripperFollower(Robot):
    """Dual RS B601 + TacCap follower composed from two single-arm robots."""

    config_class = BiSeeedB601RSTacCapGripperFollowerConfig
    name = "bi_seeed_b601_rs_taccap_gripper_follower"

    def __init__(self, config: BiSeeedB601RSTacCapGripperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.left = SeeedB601RSFollower(self._single_config("left"))
        self.right = SeeedB601RSFollower(self._single_config("right"))
        self.cameras = make_cameras_from_configs(config.cameras)

    def _single_config(self, side: str) -> SeeedB601RSFollowerConfig:
        if side not in {"left", "right"}:
            raise ValueError(f"Unknown side {side!r}.")

        values: dict[str, Any] = {}
        for field in fields(SeeedB601RSFollowerConfig):
            if not field.init or field.name == "type":
                continue
            if hasattr(self.config, field.name):
                values[field.name] = getattr(self.config, field.name)

        if side == "left":
            values.update(
                id=self.config.left_id,
                port=self.config.left_port,
                can_adapter=self.config.left_can_adapter,
                taccap_side=self.config.left_taccap_side,
                gripper_device=self.config.left_gripper_device,
            )
            if self.config.left_gripper_feedforward_torque is not None:
                values["gripper_feedforward_torque"] = (
                    self.config.left_gripper_feedforward_torque
                )
        else:
            values.update(
                id=self.config.right_id,
                port=self.config.right_port,
                can_adapter=self.config.right_can_adapter,
                taccap_side=self.config.right_taccap_side,
                gripper_device=self.config.right_gripper_device,
            )
            if self.config.right_gripper_feedforward_torque is not None:
                values["gripper_feedforward_torque"] = (
                    self.config.right_gripper_feedforward_torque
                )

        if "calibration_dir" in values:
            values["calibration_dir"] = self.calibration_dir

        # Extra dual-level cameras remain owned by the dual wrapper. Each child
        # robot starts with an empty camera map, then may auto-discover its own
        # side-specific TacCap tactile/wrist cameras.
        values["cameras"] = {}
        values.update(action_mode="joint", gripper_type="taccap")
        return SeeedB601RSFollowerConfig(**values)

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
        return {
            BiSeeedB601RSTacCapGripperFollower._prefix_key(side, key): value
            for key, value in data.items()
        }

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

    def configure(self) -> None:
        self.left.configure()
        self.right.configure()

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(
            "Connecting dual RS B601 TacCap arms: left=%s right=%s...",
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
                logger.info("No dual-level cameras configured; skipping camera connect.")
            else:
                logger.info(
                    "Connecting %d dual-level camera(s): %s...",
                    len(self.cameras),
                    ", ".join(self.cameras),
                )
                for cam in self.cameras.values():
                    cam.connect()
                logger.info("All dual-level cameras connected.")
        except Exception:
            self._disconnect_connected_children()
            raise

        logger.info("%s connected.", self)

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
            raise ValueError(
                "Dual RS B601 TacCap action must contain left_* or right_* keys."
            )

        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.send_action, left_action) if left_action else None
            right_future = executor.submit(self.right.send_action, right_action) if right_action else None
            sent_left = left_future.result() if left_future is not None else {}
            sent_right = right_future.result() if right_future is not None else {}

        return {
            **self._prefix_dict("left", sent_left),
            **self._prefix_dict("right", sent_right),
        }

    def reset_to_initial_position(self) -> None:
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(self.left._return_to_initial_position, "manual"),
                executor.submit(self.right._return_to_initial_position, "manual"),
            ]
            for future in futures:
                future.result()

    def disable_torque(self) -> None:
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(self.left.disable_torque),
                executor.submit(self.right.disable_torque),
            ]
            for future in futures:
                future.result()

    @staticmethod
    def _child_has_connection(child: SeeedB601RSFollower) -> bool:
        taccap_controller = getattr(child, "taccap_gripper", None)
        return getattr(child, "bus", None) is not None or (
            taccap_controller is not None and taccap_controller.is_connected
        )

    def _disconnect_connected_children(self) -> None:
        for arm in (self.left, self.right):
            try:
                if self._child_has_connection(arm):
                    arm.disconnect()
            except Exception:
                logger.debug("Failed to disconnect %s during cleanup.", arm, exc_info=True)
        for cam in self.cameras.values():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception:
                logger.debug("Failed to disconnect dual-level camera during cleanup.", exc_info=True)

    def disconnect(self) -> None:
        any_connected = (
            self._child_has_connection(self.left)
            or self._child_has_connection(self.right)
            or any(cam.is_connected for cam in self.cameras.values())
        )
        if not any_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = []
                if self._child_has_connection(self.left):
                    futures.append(executor.submit(self.left.disconnect))
                if self._child_has_connection(self.right):
                    futures.append(executor.submit(self.right.disconnect))
                for future in futures:
                    future.result()
        finally:
            for cam in self.cameras.values():
                if cam.is_connected:
                    cam.disconnect()

        logger.info("%s disconnected.", self)
