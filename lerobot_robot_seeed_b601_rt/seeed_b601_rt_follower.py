import logging
import math
import os
import time
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_seeed_b601_rt_follower import SeeedB601RTFollowerConfig


logger = logging.getLogger(__name__)

RETURN_TO_INITIAL_TIMEOUT_S = 3.0
RETURN_TO_INITIAL_TOLERANCE_DEG = 0.5


class SeeedB601RTFollower(Robot):
    """Seeed B601 follower backed by rebotarm_control_rt's Rust actuator loop."""

    config_class = SeeedB601RTFollowerConfig
    name = "seeed_b601_rt_follower"

    def __init__(self, config: SeeedB601RTFollowerConfig):
        super().__init__(config)
        self.config = config
        self.arm = None
        self.cameras = make_cameras_from_configs(config.cameras)
        self.motor_names = [
            name for name in config.motor_can_ids if config.control_gripper or name != "gripper"
        ]
        self._generated_cfg_path: Path | None = None
        self._initial_positions_deg: dict[str, float] = {}
        self._last_goal_deg: dict[str, float] = {}
        self._last_positions_deg: dict[str, float] = {}
        self._last_debug_motion_s = 0.0
        self._validate_config()

    def _validate_config(self) -> None:
        ids = set(self.config.motor_can_ids)
        required_maps = {
            "motor_models": set(self.config.motor_models),
            "joint_limits": set(self.config.joint_limits),
        }
        for field_name, keys in required_maps.items():
            missing = ids - keys
            if missing:
                raise ValueError(f"{field_name} missing keys: {sorted(missing)}")
        mode = self.config.control_mode.lower()
        if mode not in {"pos_vel", "mit"}:
            raise ValueError("control_mode must be 'pos_vel' or 'mit'.")
        if self.config.can_adapter not in {"damiao", "socketcan", "robstride"}:
            raise ValueError("can_adapter must be 'damiao', 'socketcan', or 'robstride'.")
        if isinstance(self.config.pos_vel_velocity, list) and len(self.config.pos_vel_velocity) != len(
            self.motor_names
        ):
            raise ValueError("pos_vel_velocity list length must match the controlled joint count.")
        if self.config.rt_command_gap_us < 0:
            raise ValueError("rt_command_gap_us must be >= 0.")
        if self.config.damiao_tx_debug < 0:
            raise ValueError("damiao_tx_debug must be >= 0.")
        if self.config.debug_motion_interval_s <= 0:
            raise ValueError("debug_motion_interval_s must be > 0.")

    @property
    def _action_motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motor_names}

    @property
    def _observation_motors_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor in self.motor_names:
            features[f"{motor}.pos"] = float
            features[f"{motor}.vel"] = float
            features[f"{motor}.torque"] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._observation_motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._action_motors_ft

    @property
    def is_connected(self) -> bool:
        return self.arm is not None and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s uses rebotarm_control_rt motor zero positions; LeRobot calibration is a no-op.", self)

    def _default_vendor(self, motor_name: str) -> str:
        if motor_name in self.config.motor_vendors:
            return self.config.motor_vendors[motor_name].lower()
        if self.config.motor_vendor:
            return self.config.motor_vendor.lower()
        if self.config.can_adapter == "robstride":
            return "robstride"
        return "damiao"

    def _velocity_limits_rad(self) -> list[float]:
        velocity = self.config.pos_vel_velocity
        if isinstance(velocity, list):
            values = velocity
        else:
            values = [velocity] * len(self.motor_names)
        return [math.radians(float(v)) for v in values]

    def _rt_cfg_path(self) -> Path:
        if self.config.arm_cfg_path is not None:
            return Path(self.config.arm_cfg_path).expanduser()

        path = self.calibration_dir / f"{self.id or 'default'}_rt_arm.yaml"
        path.write_text(self._render_rt_arm_yaml(), encoding="utf-8")
        self._generated_cfg_path = path
        return path

    def _render_rt_arm_yaml(self) -> str:
        lines = [
            "# Generated by lerobot_robot_seeed_b601_rt.",
            "name: reBotArmB601RT",
            f"channel: {self.config.port}",
            f"rate: {float(self.config.rt_rate)}",
            "",
            "joints:",
        ]
        vlim = self._velocity_limits_rad()
        for idx, motor_name in enumerate(self.motor_names):
            motor_id, feedback_id = self.config.motor_can_ids[motor_name]
            lines.extend(
                [
                    f"  - name: {motor_name}",
                    f"    motor_id: 0x{int(motor_id):02X}",
                    f"    feedback_id: 0x{int(feedback_id):02X}",
                    f"    model: \"{self.config.motor_models[motor_name]}\"",
                    f"    vendor: \"{self._default_vendor(motor_name)}\"",
                    "    POS_VEL:",
                    f"      vlim: {float(vlim[idx])}",
                    "",
                ]
            )
        return "\n".join(lines)

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        from rebotarm_control_rt.actuator import RobotArm

        cfg_path = self._rt_cfg_path()
        if self.config.damiao_tx_debug:
            tx_debug = int(self.config.damiao_tx_debug)
            os.environ["MOTORBRIDGE_DAMIAO_TX_DEBUG"] = str(tx_debug)
            logger.info("Configured MOTORBRIDGE_DAMIAO_TX_DEBUG=%s.", tx_debug)

        logger.info("Connecting arm on %s with rebotarm_control_rt config %s...", self.config.port, cfg_path)
        arm = RobotArm(str(cfg_path))
        try:
            arm.connect()
            self.arm = arm
            if not self.cameras:
                logger.info("No cameras configured; skipping camera connect.")
            else:
                logger.info("Connecting %d camera(s): %s...", len(self.cameras), ", ".join(self.cameras))
                for cam in self.cameras.values():
                    cam.connect()
                logger.info("All cameras connected.")
            self.configure()
        except Exception:
            try:
                arm.disconnect()
            except Exception:
                pass
            self.arm = None
            raise

        logger.info("%s connected.", self)

    def configure(self) -> None:
        if self.arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.arm.enable()
        mode = self.config.control_mode.lower()
        if mode == "pos_vel":
            self.arm.mode_pos_vel(vlim=self._velocity_limits_rad())
        else:
            self.arm.mode_mit()

        self._initial_positions_deg, _, _ = self._read_state_deg(request=True)
        self._apply_initial_gripper_pose(self._initial_positions_deg)
        self._last_goal_deg = dict(self._initial_positions_deg)
        self.arm.start_rt_loop(
            rate=float(self.config.rt_rate),
            rt_priority=int(self.config.rt_priority),
            cpu=self.config.rt_cpu,
            command_gap_us=int(self.config.rt_command_gap_us),
            request_feedback=False,
        )
        logger.info(
            "Started RT target loop at %.1f Hz (command gap %s us).",
            float(self.config.rt_rate),
            int(self.config.rt_command_gap_us),
        )

    def _read_state_deg(self, request: bool) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
        if self.arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        pos_rad, vel_rad, torque = self.arm.get_state(request=request)
        positions = np.asarray(pos_rad, dtype=float)
        velocities = np.asarray(vel_rad, dtype=float)
        torques = np.asarray(torque, dtype=float)

        pos_deg = {
            motor_name: math.degrees(float(positions[idx]))
            for idx, motor_name in enumerate(self.motor_names)
        }
        vel_deg = {
            motor_name: math.degrees(float(velocities[idx]))
            for idx, motor_name in enumerate(self.motor_names)
        }
        torq = {motor_name: float(torques[idx]) for idx, motor_name in enumerate(self.motor_names)}
        self._last_positions_deg = pos_deg
        return pos_deg, vel_deg, torq

    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        pos_deg, vel_deg, torque = self._read_state_deg(request=False)

        obs_dict: dict[str, Any] = {}
        for motor_name in self.motor_names:
            obs_dict[f"{motor_name}.pos"] = pos_deg[motor_name]
            obs_dict[f"{motor_name}.vel"] = vel_deg[motor_name]
            obs_dict[f"{motor_name}.torque"] = torque[motor_name]

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug("%s get_observation took: %.1fms", self, dt_ms)
        return obs_dict

    @staticmethod
    def _clip(value: float, limits: tuple[float, float]) -> float:
        return max(float(limits[0]), min(float(limits[1]), value))

    def _apply_initial_gripper_pose(self, positions_deg: dict[str, float]) -> None:
        if "gripper" not in positions_deg:
            return
        # B601 convention: gripper zero is the closed initial pose.
        positions_deg["gripper"] = self._clip(0.0, self.config.joint_limits["gripper"])

    def _complete_goal(self, action: RobotAction) -> dict[str, float]:
        goal_pos = dict(self._last_goal_deg)
        if not goal_pos and self._last_positions_deg:
            goal_pos = dict(self._last_positions_deg)

        for key, val in action.items():
            if not key.endswith(".pos"):
                continue
            motor_name = key.removesuffix(".pos")
            if motor_name in self.motor_names:
                goal_pos[motor_name] = float(val)

        if "wrist_yaw" in self.motor_names and "wrist_yaw" not in goal_pos:
            goal_pos["wrist_yaw"] = 0.0

        for motor_name in self.motor_names:
            goal_pos.setdefault(motor_name, self._last_positions_deg.get(motor_name, 0.0))
        return {motor_name: goal_pos[motor_name] for motor_name in self.motor_names}

    def send_action(self, action: RobotAction) -> RobotAction:
        if self.arm is None or not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = self._complete_goal(action)
        for motor_name, position in list(goal_pos.items()):
            goal_pos[motor_name] = self._clip(position, self.config.joint_limits[motor_name])

        if self.config.max_relative_target is not None:
            if not self._last_positions_deg:
                self._read_state_deg(request=True)
            goal_present_pos = {
                motor_name: (goal_pos[motor_name], self._last_positions_deg.get(motor_name, goal_pos[motor_name]))
                for motor_name in self.motor_names
            }
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        target_rad = [math.radians(goal_pos[motor_name]) for motor_name in self.motor_names]
        self.arm.set_targets(pos=target_rad, vlim=self._velocity_limits_rad())

        self._last_goal_deg = dict(goal_pos)
        self._maybe_log_motion_debug(goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def _maybe_log_motion_debug(self, goal_pos: dict[str, float]) -> None:
        if not self.config.debug_motion:
            return

        now = time.monotonic()
        if now - self._last_debug_motion_s < float(self.config.debug_motion_interval_s):
            return
        self._last_debug_motion_s = now

        if not self._last_positions_deg:
            try:
                self._read_state_deg(request=True)
            except Exception:
                logger.debug("Failed to refresh state for motion debug.", exc_info=True)

        deltas = {
            name: goal_pos[name] - self._last_positions_deg.get(name, goal_pos[name])
            for name in self.motor_names
        }
        max_delta_name = max(self.motor_names, key=lambda name: abs(deltas[name]))
        logger.info(
            "motion debug: max_delta=%s %.2f deg | target=%s | current=%s | rt_overruns(send/read)=%s/%s",
            max_delta_name,
            deltas[max_delta_name],
            {name: round(goal_pos[name], 2) for name in self.motor_names},
            {name: round(self._last_positions_deg.get(name, 0.0), 2) for name in self.motor_names},
            getattr(self.arm, "rt_send_overruns", getattr(self.arm, "rt_overruns", None)),
            getattr(self.arm, "rt_read_overruns", None),
        )

    def _return_to_initial_position(self) -> None:
        if self.arm is None or not self._initial_positions_deg:
            return

        target_rad = [math.radians(self._initial_positions_deg[name]) for name in self.motor_names]
        try:
            self.arm.set_targets(pos=target_rad, vlim=self._velocity_limits_rad())
            self._last_goal_deg = dict(self._initial_positions_deg)
        except Exception:
            logger.debug("Failed to command initial position before disconnect.", exc_info=True)
            return

        deadline = time.monotonic() + RETURN_TO_INITIAL_TIMEOUT_S
        tolerance = RETURN_TO_INITIAL_TOLERANCE_DEG
        while time.monotonic() < deadline:
            try:
                current, _, _ = self._read_state_deg(request=False)
            except Exception:
                logger.debug("Failed to read state while returning to initial position.", exc_info=True)
                break
            max_error = max(
                abs(current[name] - self._initial_positions_deg[name]) for name in self.motor_names
            )
            if max_error <= tolerance:
                logger.info("Returned to initial position before disconnect (max error %.2f deg).", max_error)
                return
            time.sleep(0.02)

        logger.info("Return-to-initial timeout before disconnect.")

    def disconnect(self) -> None:
        if self.arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            self._return_to_initial_position()
            self.arm.disconnect(disable=bool(self.config.disable_torque_on_disconnect))
        finally:
            self.arm = None
            for cam in self.cameras.values():
                cam.disconnect()

        logger.info("%s disconnected.", self)
