"""Shared TacCap discovery, control, and LeRobot wrapper."""

from __future__ import annotations

import ctypes
import glob
import logging
import os
import re
import sys
import time
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_taccap_gripper import (
    TacCapGripperConfig,
    TacCapGripperFollowerConfig,
)

logger = logging.getLogger(__name__)


SIDES = ("left", "right")

_BYID_DIR = "/dev/v4l/by-id"
_V4L_BYPATH_DIR = "/dev/v4l/by-path"
_SERIAL_BYPATH_DIR = "/dev/serial/by-path"

_USB_PORT_RE = re.compile(r"usb-(\d+):([\d.]+):")
_GRIPPER_RE = re.compile(r"^TCGU01[A-Z]\d{2}[ZA](\d{4})([ms])$")
_TACTILE_RE = re.compile(r"^GSPS01[A-Z]\d{2}[ZA](\d{4})$")
_CAMERA_RE = re.compile(r"^XC[A-Z]\d{2}[ZA](\d{4})([ms])$")
_TACTILE_BYID_RE = re.compile(r"(GSPS01[A-Z]\d{2}[ZA]\d{4})")
_CAMERA_BYID_RE = re.compile(r"(XC[A-Z]\d{2}[ZA]\d{4}[ms])")

_PATCH_ROLE = {"m": "leader", "s": "follower"}
_ROLE_ALIASES = {
    "leader": "leader",
    "master": "leader",
    "m": "leader",
    "follower": "follower",
    "slave": "follower",
    "s": "follower",
}


def normalize_role(role: str) -> str:
    key = str(role).strip().lower()
    if key not in _ROLE_ALIASES:
        raise ValueError("role must be leader/master/m or follower/slave/s")
    return _ROLE_ALIASES[key]


def side_of_sequence(sequence: str) -> str:
    return "left" if int(sequence[-1]) % 2 == 1 else "right"


def _hub_of_bypath(link: str) -> str | None:
    match = _USB_PORT_RE.search(link)
    if not match:
        return None
    bus, ports = match.group(1), match.group(2)
    parent = ports.rsplit(".", 1)[0] if "." in ports else ports
    return f"{bus}:{parent}"


def _device_hub(node_or_symlink: str, bypath_dir: str) -> str | None:
    real = os.path.realpath(node_or_symlink)
    for link in glob.glob(f"{bypath_dir}/*"):
        if "usbv2" in os.path.basename(link):
            continue
        if os.path.realpath(link) == real:
            return _hub_of_bypath(os.path.basename(link))
    return None


def _scan_grippers():
    try:
        from xense.taccap import scan_grippers
    except ImportError as e:
        raise ImportError("xense.taccap SDK is required for TacCap gripper discovery") from e
    return scan_grippers()


def parse_camera_serial(sn: str) -> tuple[str, str]:
    match = _CAMERA_RE.match(sn)
    if not match:
        raise ValueError(f"Invalid TacCap wrist camera serial: {sn!r}")
    return side_of_sequence(match.group(1)), _PATCH_ROLE[match.group(2)]


def _byid_serials(extract_re: re.Pattern) -> list[str]:
    found: set[str] = set()
    for path in glob.glob(f"{_BYID_DIR}/*"):
        match = extract_re.search(path)
        if match:
            found.add(match.group(1))
    return sorted(found)


def discover_grippers(role: str = "follower") -> dict[str, Any]:
    role = normalize_role(role)
    grouped: dict[str, list[Any]] = {"left": [], "right": []}
    for ep in _scan_grippers():
        if ep.firmware_sn and not _GRIPPER_RE.match(ep.firmware_sn):
            raise ValueError(f"Invalid TacCap gripper firmware serial: {ep.firmware_sn!r}")
        if ep.role.name.lower() != role:
            continue
        grouped[ep.side.name.lower()].append(ep)

    result: dict[str, Any] = {}
    for side in SIDES:
        if len(grouped[side]) > 1:
            raise ValueError(f"Found multiple {role} TacCap grippers for {side}: {grouped[side]}")
        if grouped[side]:
            result[side] = grouped[side][0]
    return result


def resolve_gripper_side(role: str = "follower", side: str | None = None) -> str:
    if side:
        normalized_side = side.strip().lower()
        if normalized_side not in SIDES:
            raise ValueError("TacCap side must be 'left' or 'right'.")
        return normalized_side

    grippers = discover_grippers(role)
    if len(grippers) == 1:
        return next(iter(grippers))
    if not grippers:
        raise RuntimeError(
            f"No {role} TacCap gripper discovered; set taccap_side or check hardware."
        )
    raise RuntimeError(
        f"Both TacCap sides are present ({sorted(grippers)}); set taccap_side."
    )


def discover_gripper_device(role: str = "follower", side: str | None = None) -> str:
    resolved_side = resolve_gripper_side(role, side)
    endpoint = discover_grippers(role).get(resolved_side)
    if endpoint is None:
        raise RuntimeError(f"No {role} TacCap gripper found for side {resolved_side!r}.")
    return endpoint.mcu_device


def _gripper_hub_sides(role: str = "follower") -> dict[str, tuple[str, str]]:
    hub_side: dict[str, tuple[str, str]] = {}
    for ep in discover_grippers(role).values():
        hub = _device_hub(ep.mcu_device, _SERIAL_BYPATH_DIR)
        if hub is None:
            raise ValueError(f"Could not resolve USB hub for TacCap gripper {ep.firmware_sn!r}")
        hub_side[hub] = (ep.side.name.lower(), ep.firmware_sn)
    return hub_side


def discover_tactiles_by_hub(role: str = "follower") -> dict[str, dict[str, str]]:
    hub_side = _gripper_hub_sides(role)
    hub_fingers: dict[str, dict[str, str]] = {}
    seen: set[str] = set()
    for path in glob.glob(f"{_BYID_DIR}/*"):
        match = _TACTILE_BYID_RE.search(os.path.basename(path))
        if not match:
            continue
        sn = match.group(1)
        if sn in seen:
            continue
        seen.add(sn)
        if not _TACTILE_RE.match(sn):
            raise ValueError(f"Invalid TacCap tactile serial: {sn!r}")
        hub = _device_hub(path, _V4L_BYPATH_DIR)
        if hub is None:
            raise ValueError(f"Could not resolve USB hub for TacCap tactile {sn!r}")
        finger = side_of_sequence(sn[-4:])
        fingers = hub_fingers.setdefault(hub, {})
        if finger in fingers:
            raise ValueError(f"Two TacCap tactile sensors on hub {hub!r} map to {finger}")
        fingers[finger] = sn

    result: dict[str, dict[str, str]] = {"left": {}, "right": {}}
    for hub, fingers in hub_fingers.items():
        if hub not in hub_side:
            raise ValueError(f"TacCap tactile sensors on hub {hub!r} have no matching gripper")
        result[hub_side[hub][0]] = fingers
    return result


def discover_wrist_cameras(role: str = "follower") -> dict[str, str]:
    role = normalize_role(role)
    result: dict[str, str] = {}
    grouped: dict[str, list[str]] = {"left": [], "right": []}
    for sn in _byid_serials(_CAMERA_BYID_RE):
        side, sn_role = parse_camera_serial(sn)
        if sn_role == role:
            grouped[side].append(sn)
    for side in SIDES:
        if len(grouped[side]) > 1:
            raise ValueError(f"Found multiple {role} TacCap wrist cameras for {side}: {grouped[side]}")
        if grouped[side]:
            result[side] = grouped[side][0]
    return result


def resolve_wrist_camera_path(serial: str) -> str:
    matches = sorted(glob.glob(f"/dev/v4l/by-id/*{serial}*-video-index0"))
    if not matches:
        raise RuntimeError(f"No TacCap wrist camera matching serial {serial!r}")
    if len(matches) > 1:
        raise RuntimeError(f"Multiple TacCap wrist cameras match serial {serial!r}: {matches}")
    return matches[0]


_XENSE_CONFIG_CACHE_PSWD = "Wz8mmWz2ALJ6X5Ic"


def _wait_nodes_settle(serials: list[str], logger, timeout_s: float = 15.0) -> None:
    """Wait for V4L2 nodes to return after xensesdk flash reads reset sensors."""
    deadline = time.perf_counter() + timeout_s
    for serial_number in serials:
        settled = False
        while time.perf_counter() < deadline:
            matches = glob.glob(f"/dev/v4l/by-id/*{serial_number}*-video-index0")
            if matches:
                try:
                    fd = os.open(os.path.realpath(matches[0]), os.O_RDWR)
                    os.close(fd)
                    settled = True
                    break
                except OSError:
                    pass
            time.sleep(0.2)
        if not settled:
            logger.warning(
                "Sensor %s V4L2 node did not settle within %.0fs after config pre-warm",
                serial_number,
                timeout_s,
            )


def prewarm_tactile_config_cache(camera_configs: dict[str, Any], logger) -> None:
    """Warm xensesdk's per-serial config cache before opening tactile cameras."""
    try:
        from lerobot_camera_xense import XenseTactileCameraConfig
    except ImportError:
        return

    serials = [
        cfg.serial_number
        for cfg in camera_configs.values()
        if isinstance(cfg, XenseTactileCameraConfig) and getattr(cfg, "serial_number", None)
    ]
    if not serials:
        return

    try:
        from xensesdk.core.ctx_builders import CONFIG_CACHE_DIR
        from xensesdk.flash import FlashClient
        from xensesdk.flash.sunplus_backend import is_sunplus
        from xensesdk.utils.encrypt import encrypt_config_file
    except Exception as e:
        logger.debug("Xense config pre-warm unavailable (%s); skipping", e)
        return

    uncached = [
        serial_number
        for serial_number in serials
        if is_sunplus(serial_number) and not (CONFIG_CACHE_DIR / serial_number).exists()
    ]
    if not uncached:
        return

    logger.info(
        "Pre-warming Xense config cache for %d Sunplus sensor(s): %s",
        len(uncached),
        uncached,
    )
    client = FlashClient()
    try:
        for serial_number in uncached:
            try:
                patch = client.read_patch(serial_number=serial_number)
                CONFIG_CACHE_DIR.mkdir(parents=True, exist_ok=True)
                encrypt_config_file(
                    patch,
                    CONFIG_CACHE_DIR / serial_number,
                    password=_XENSE_CONFIG_CACHE_PSWD,
                    format="xbin",
                )
            except Exception as e:
                logger.warning("Xense config pre-warm failed for %s: %s", serial_number, e)
    finally:
        client.cleanup()

    _wait_nodes_settle(uncached, logger)


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
        return discover_gripper_device(
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
        elif self.config.invert_position:
            position = 1.0 - position
        return max(0.0, min(1.0, position))

    def action_position_from_normalized(self, position: float) -> float:
        position = max(0.0, min(1.0, float(position)))
        if self.config.normalize_action:
            return self.config.action_min + position * (
                self.config.action_max - self.config.action_min
            )
        if self.config.invert_position:
            return 1.0 - position
        return position

    @property
    def observation_position(self) -> float:
        """TacCap position in the RT robot convention: 0=open, 1=closed."""

        if self.config.invert_position:
            return 1.0 - self.last_position
        return self.last_position

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
            "gripper.target": (
                1.0 - self.last_target
                if self.config.invert_position
                else self.last_target
            ),
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


class TacCapGripperFollower(Robot):
    """Standalone LeRobot device backed by the shared TacCapGripper controller."""

    config_class = TacCapGripperFollowerConfig
    name = "taccap_gripper_follower"
    gripper_action_name = "gripper.pos"

    def __init__(self, config: TacCapGripperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.controller = TacCapGripper(config._controller_config())

    @cached_property
    def observation_features(self) -> dict[str, type]:
        return {
            self.gripper_action_name: float,
            "gripper.vel": float,
            "gripper.torque": float,
            "gripper.target_torque": float,
            "gripper.control_mode": int,
            "gripper.target": float,
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {self.gripper_action_name: float}

    @property
    def is_connected(self) -> bool:
        return self.controller.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s has no LeRobot calibration step; use TacCap SDK calibration.", self)

    def configure(self) -> None:
        return

    def connect(self, calibrate: bool = True) -> None:
        self.controller.connect()
        logger.info("%s connected using the shared TacCap controller.", self)

    def get_observation(self) -> RobotObservation:
        return self.controller.read_observation()

    def send_action(self, action: RobotAction) -> RobotAction:
        if self.gripper_action_name not in action:
            return {}
        self.controller.send_position(action[self.gripper_action_name])
        return {self.gripper_action_name: self.controller.last_target}

    def disconnect(self) -> None:
        self.controller.disconnect()
        logger.info("%s disconnected.", self)
