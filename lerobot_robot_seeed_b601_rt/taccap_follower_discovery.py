"""TacCap follower gripper, tactile sensor, and wrist camera discovery helpers."""

from __future__ import annotations

import glob
import os
import re
import time
from typing import Any


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
        raise ImportError("xense.taccap SDK is required for TacCap discovery") from e
    return scan_grippers()


def parse_camera_serial(serial_number: str) -> tuple[str, str]:
    match = _CAMERA_RE.match(serial_number)
    if not match:
        raise ValueError(f"Invalid TacCap wrist camera serial: {serial_number!r}")
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
    for endpoint in _scan_grippers():
        if endpoint.firmware_sn and not _GRIPPER_RE.match(endpoint.firmware_sn):
            raise ValueError(
                f"Invalid TacCap gripper firmware serial: {endpoint.firmware_sn!r}"
            )
        if endpoint.role.name.lower() != role:
            continue
        grouped[endpoint.side.name.lower()].append(endpoint)

    result: dict[str, Any] = {}
    for side in SIDES:
        if len(grouped[side]) > 1:
            raise ValueError(
                f"Found multiple {role} TacCap grippers for {side}: {grouped[side]}"
            )
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
    for endpoint in discover_grippers(role).values():
        hub = _device_hub(endpoint.mcu_device, _SERIAL_BYPATH_DIR)
        if hub is None:
            raise ValueError(
                f"Could not resolve USB hub for TacCap gripper {endpoint.firmware_sn!r}"
            )
        hub_side[hub] = (endpoint.side.name.lower(), endpoint.firmware_sn)
    return hub_side


def discover_tactiles_by_hub(role: str = "follower") -> dict[str, dict[str, str]]:
    hub_side = _gripper_hub_sides(role)
    hub_fingers: dict[str, dict[str, str]] = {}
    seen: set[str] = set()
    for path in glob.glob(f"{_BYID_DIR}/*"):
        match = _TACTILE_BYID_RE.search(os.path.basename(path))
        if not match:
            continue
        serial_number = match.group(1)
        if serial_number in seen:
            continue
        seen.add(serial_number)
        if not _TACTILE_RE.match(serial_number):
            raise ValueError(f"Invalid TacCap tactile serial: {serial_number!r}")
        hub = _device_hub(path, _V4L_BYPATH_DIR)
        if hub is None:
            raise ValueError(
                f"Could not resolve USB hub for TacCap tactile {serial_number!r}"
            )
        finger = side_of_sequence(serial_number[-4:])
        fingers = hub_fingers.setdefault(hub, {})
        if finger in fingers:
            raise ValueError(f"Two TacCap tactile sensors on hub {hub!r} map to {finger}")
        fingers[finger] = serial_number

    result: dict[str, dict[str, str]] = {"left": {}, "right": {}}
    for hub, fingers in hub_fingers.items():
        if hub not in hub_side:
            raise ValueError(
                f"TacCap tactile sensors on hub {hub!r} have no matching gripper"
            )
        result[hub_side[hub][0]] = fingers
    return result


def discover_wrist_cameras(role: str = "follower") -> dict[str, str]:
    role = normalize_role(role)
    grouped: dict[str, list[str]] = {"left": [], "right": []}
    for serial_number in _byid_serials(_CAMERA_BYID_RE):
        side, serial_role = parse_camera_serial(serial_number)
        if serial_role == role:
            grouped[side].append(serial_number)

    result: dict[str, str] = {}
    for side in SIDES:
        if len(grouped[side]) > 1:
            raise ValueError(
                f"Found multiple {role} TacCap wrist cameras for {side}: {grouped[side]}"
            )
        if grouped[side]:
            result[side] = grouped[side][0]
    return result


def resolve_wrist_camera_path(serial_number: str) -> str:
    matches = sorted(glob.glob(f"/dev/v4l/by-id/*{serial_number}*-video-index0"))
    if not matches:
        raise RuntimeError(f"No TacCap wrist camera matching serial {serial_number!r}")
    if len(matches) > 1:
        raise RuntimeError(
            f"Multiple TacCap wrist cameras match serial {serial_number!r}: {matches}"
        )
    return matches[0]


_XENSE_CONFIG_CACHE_PASSWORD = "Wz8mmWz2ALJ6X5Ic"


def _wait_nodes_settle(serials: list[str], logger, timeout_s: float = 15.0) -> None:
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
    try:
        from lerobot_camera_xense import XenseTactileCameraConfig
    except ImportError:
        return

    serials = [
        config.serial_number
        for config in camera_configs.values()
        if isinstance(config, XenseTactileCameraConfig)
        and getattr(config, "serial_number", None)
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

    logger.info("Pre-warming Xense config cache for: %s", uncached)
    client = FlashClient()
    try:
        for serial_number in uncached:
            try:
                patch = client.read_patch(serial_number=serial_number)
                CONFIG_CACHE_DIR.mkdir(parents=True, exist_ok=True)
                encrypt_config_file(
                    patch,
                    CONFIG_CACHE_DIR / serial_number,
                    password=_XENSE_CONFIG_CACHE_PASSWORD,
                    format="xbin",
                )
            except Exception as e:
                logger.warning("Xense config pre-warm failed for %s: %s", serial_number, e)
    finally:
        client.cleanup()

    _wait_nodes_settle(uncached, logger)
