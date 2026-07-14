import ctypes
import logging
import sys
import time
from pathlib import Path
from typing import Any

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from . import taccap_follower_discovery as taccap_discovery
from .config_taccap_gripper import TacCapGripperConfig


logger = logging.getLogger(__name__)


def _preload_conda_libjpeg_for_taccap() -> None:
    """Make conda's libjpeg symbols visible before importing the TacCap extension."""

    for library_name in ("libjpeg.so.8", "libjpeg.so"):
        library_path = Path(sys.prefix) / "lib" / library_name
        if not library_path.exists():
            continue
        try:
            ctypes.CDLL(str(library_path), mode=ctypes.RTLD_GLOBAL)
            return
        except OSError:
            logger.debug("Failed to preload %s for TacCap", library_path, exc_info=True)


def _load_follower_gripper_class():
    _preload_conda_libjpeg_for_taccap()
    try:
        from xense.taccap import FollowerGripper
    except ImportError as e:
        raise ImportError(
            "xense.taccap is required for TacCap gripper control. "
            "Build/install TacCap-Gripper in the active Python environment."
        ) from e
    return FollowerGripper


class TacCapGripper:
    """TacCap follower gripper controlled through raw MIT impedance commands."""

    def __init__(self, config: TacCapGripperConfig):
        self.config = config
        self.gripper: Any | None = None
        self.last_position = 0.0
        self.last_velocity = 0.0
        self.last_torque = 0.0
        self.last_target_torque = 0.0
        self.last_control_mode = -1
        self.last_target = 0.0
        self.last_command_mode = "idle"
        self._last_torque_print_s = 0.0

    @property
    def is_connected(self) -> bool:
        return self.gripper is not None

    def _resolve_device(self) -> str:
        if self.config.device:
            return self.config.device
        return taccap_discovery.discover_gripper_device(
            self.config.role,
            self.config.side,
        )

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError("TacCap gripper is already connected.")

        follower_gripper_class = _load_follower_gripper_class()
        device = self._resolve_device()
        gripper = follower_gripper_class(
            mcu_device=device,
            wrist_video=self.config.wrist_video,
            baudrate=int(self.config.baudrate),
            ack_timeout_ms=int(self.config.ack_timeout_ms),
            max_retries=int(self.config.max_retries),
            open_cameras=bool(self.config.open_cameras),
        )
        self.gripper = gripper
        try:
            if self.config.reload_config_on_connect:
                gripper.reload_config()
            if self.config.clear_fault_on_connect:
                gripper.motor.clear_fault()
            if self.config.enable_on_connect:
                gripper.motor.enable()
            self.read_observation()
            self.last_target = self.last_position
        except Exception:
            try:
                self._release_gripper(disable=True)
            except Exception:
                logger.debug("Failed to release TacCap after connect error.", exc_info=True)
            raise

        logger.info("TacCap gripper connected on %s.", device)

    def normalize_action_position(self, position: float) -> float:
        position = float(position)
        if self.config.normalize_action:
            position = (position - self.config.action_min) / (
                self.config.action_max - self.config.action_min
            )
        else:
            # RT gripper convention: 0=open, 1=closed.
            # TacCap SDK convention: 0=closed, 1=open.
            position = 1.0 - position
        return max(0.0, min(1.0, position))

    def action_position_from_normalized(self, position: float) -> float:
        position = max(0.0, min(1.0, float(position)))
        if not self.config.normalize_action:
            return 1.0 - position
        return self.config.action_min + position * (
            self.config.action_max - self.config.action_min
        )

    @property
    def observation_position(self) -> float:
        """TacCap position in the RT robot convention: 0=open, 1=closed."""

        return 1.0 - self.last_position

    def read_observation(self) -> dict[str, float | int]:
        if self.gripper is None:
            raise DeviceNotConnectedError("TacCap gripper is not connected.")

        status = self.gripper.motor.read_status(int(self.config.ack_timeout_ms))
        self.last_position = float(self.gripper.rad_to_pos(status.actual_pos))
        self.last_velocity = float(status.actual_vel)
        self.last_torque = float(status.actual_torque)
        self.last_target_torque = float(status.target_torque)
        self.last_control_mode = int(status.control_mode)
        self._maybe_print_torque()
        return {
            "gripper.pos": self.observation_position,
            "gripper.vel": self.last_velocity,
            "gripper.torque": self.last_torque,
            "gripper.target_torque": self.last_target_torque,
            "gripper.control_mode": self.last_control_mode,
            "gripper.target": 1.0 - self.last_target,
        }

    def _maybe_print_torque(self) -> None:
        if not self.config.print_torque or self.config.torque_print_hz <= 0:
            return
        now_s = time.perf_counter()
        if now_s - self._last_torque_print_s < 1.0 / self.config.torque_print_hz:
            return
        self._last_torque_print_s = now_s
        print(
            "[TacCapGripper] "
            f"pos={self.last_position:.3f} "
            f"target={self.last_target:.3f} "
            f"torque={self.last_torque:+.3f}Nm "
            f"target_torque={self.last_target_torque:+.3f}Nm "
            f"ff_cmd={self.config.feedforward_torque:+.3f}Nm "
            f"vel={self.last_velocity:+.3f}rad/s "
            f"mode={self.last_control_mode} "
            f"cmd={self.last_command_mode}",
            flush=True,
        )

    def send_position(self, position: float) -> float:
        if self.gripper is None:
            raise DeviceNotConnectedError("TacCap gripper is not connected.")

        target = self.normalize_action_position(position)
        if self.config.torque_grasp_enabled and target < self.last_position:
            target = self.last_position
            kp = 0.0
            kd = 0.0
            self.last_command_mode = "torque_grasp"
        else:
            kp = float(self.config.kp)
            kd = float(self.config.kd)
            self.last_command_mode = "position"

        self.gripper.motor.submit_impedance(
            self.gripper.pos_to_rad(target),
            kp,
            kd,
            float(self.config.feedforward_torque),
        )
        self.last_target = target
        return self.action_position_from_normalized(target)

    def _release_gripper(self, *, disable: bool) -> None:
        gripper = self.gripper
        if gripper is None:
            return
        try:
            if disable and self.config.disable_on_disconnect:
                gripper.motor.disable()
        finally:
            try:
                if getattr(gripper, "is_streaming", False):
                    gripper.stop_streaming()
            finally:
                self.gripper = None

    def disconnect(self) -> None:
        if self.gripper is None:
            raise DeviceNotConnectedError("TacCap gripper is not connected.")
        try:
            self._release_gripper(disable=True)
        finally:
            self.last_position = 0.0
            self.last_velocity = 0.0
            self.last_torque = 0.0
            self.last_target_torque = 0.0
            self.last_control_mode = -1
            self.last_target = 0.0
            self.last_command_mode = "idle"
            self._last_torque_print_s = 0.0
        logger.info("TacCap gripper disconnected.")
