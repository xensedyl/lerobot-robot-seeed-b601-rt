"""Single motorbridge robot class for every B601 RS mode."""

import logging
import math
import time
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.motors import MotorCalibration
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceNotConnectedError

from . import taccap_gripper as taccap
from .config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig
from .config_taccap_gripper import TacCapGripperConfig
from .seeed_b601_follower import (
    FOLLOWER_GRIPPER_MOTOR,
    LONG_TIMEOUT_SEC,
    SeeedB601FollowerBase,
)

logger = logging.getLogger(__name__)

KINEMATIC_MOTORS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
TCP_POSE_KEYS = (
    "tcp.x",
    "tcp.y",
    "tcp.z",
    "tcp.r1",
    "tcp.r2",
    "tcp.r3",
    "tcp.r4",
    "tcp.r5",
    "tcp.r6",
)


class SeeedB601RSFollower(SeeedB601FollowerBase):
    """B601 RS arm selected by action_mode and gripper_type."""

    config_class = SeeedB601RSFollowerConfig
    name = "seeed_b601_rs_follower"
    motor_type = "rs"
    gripper_action_name = "gripper.pos"

    motor_model_mapping = {
        "shoulder_pan": "rs-06",
        "shoulder_lift": "rs-06",
        "elbow_flex": "rs-06",
        "wrist_flex": "rs-00",
        "wrist_yaw": "rs-00",
        "wrist_roll": "rs-00",
        "gripper": "rs-00",
    }

    def __init__(self, config: SeeedB601RSFollowerConfig):
        if self._config_uses_taccap(config) and config.auto_discover_taccap_cameras:
            self._add_discovered_taccap_camera_configs(config)
        super().__init__(config)
        self.config = config
        self._startup_sync_active = False
        self._startup_sync_command_deg: dict[str, float] | None = None
        self._startup_sync_last_time_s: float | None = None

        expected_motors = KINEMATIC_MOTORS + (
            ("gripper",) if config.gripper_type == "motor" else ()
        )
        if tuple(self.motor_names) != expected_motors:
            raise ValueError(
                f"Unexpected RS motor layout: {self.motor_names}; "
                f"expected {expected_motors}."
            )

        self._kinematic_model = None
        self._kinematic_frame_id: int | None = None
        self.taccap_gripper: taccap.TacCapGripper | None = (
            taccap.TacCapGripper(self._make_taccap_config())
            if self._config_uses_taccap(config)
            else None
        )

    @staticmethod
    def _config_uses_taccap(config: SeeedB601RSFollowerConfig) -> bool:
        return config.gripper_type == "taccap" and config.connect_taccap_gripper

    @property
    def _uses_taccap(self) -> bool:
        return self._config_uses_taccap(self.config)

    def _make_taccap_config(self) -> TacCapGripperConfig:
        return TacCapGripperConfig(
            device=self.config.gripper_device,
            role=self.config.taccap_role,
            side=self.config.taccap_side,
            wrist_video=self.config.gripper_wrist_video,
            baudrate=self.config.gripper_baudrate,
            ack_timeout_ms=self.config.gripper_ack_timeout_ms,
            max_retries=self.config.gripper_max_retries,
            open_cameras=self.config.gripper_open_cameras,
            reload_config_on_connect=self.config.gripper_reload_config_on_connect,
            enable_on_connect=self.config.enable_gripper_on_connect,
            disable_on_disconnect=self.config.disable_gripper_on_disconnect,
            clear_fault_on_connect=self.config.clear_gripper_fault_on_connect,
            kp=self.config.gripper_kp,
            kd=self.config.gripper_kd,
            feedforward_torque=self.config.gripper_feedforward_torque,
            torque_grasp_enabled=self.config.gripper_torque_grasp_enabled,
            print_torque=self.config.print_gripper_torque,
            torque_print_hz=self.config.gripper_torque_print_hz,
            normalize_action=self.config.normalize_gripper_action,
            action_min=self.config.gripper_action_min,
            action_max=self.config.gripper_action_max,
            invert_position=False,
        )

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = (
            dict.fromkeys(TCP_POSE_KEYS, float)
            if self.config.action_mode == "cartesian"
            else dict(self._motors_ft)
        )
        if self._uses_taccap:
            features[self.gripper_action_name] = float
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        if self.config.action_mode == "cartesian":
            features: dict[str, type | tuple] = {
                **dict.fromkeys(TCP_POSE_KEYS, float),
                **self._cameras_ft,
            }
        else:
            features = dict(super().observation_features)
        if self._uses_taccap:
            features.update(
                {
                    self.gripper_action_name: float,
                    "gripper.torque": float,
                    "gripper.target_torque": float,
                    "gripper.control_mode": int,
                    "gripper.vel": float,
                    "gripper.target": float,
                }
            )
        return features

    @property
    def is_connected(self) -> bool:
        taccap_ok = (
            not self._uses_taccap
            or (
                self.taccap_gripper is not None
                and self.taccap_gripper.is_connected
            )
        )
        return super().is_connected and taccap_ok

    def _add_motors_to_bus(self):
        for motor_name, (send_id, recv_id) in self.config.motor_can_ids.items():
            motor_type_str = self.motor_model_mapping[motor_name]
            self.motors[motor_name] = self.bus.add_robstride_motor(send_id, recv_id, motor_type_str)

    def mit_output_torque_limit(
        self,
        motor,
        pos_target: float,
    ) -> float | None:
        if motor is None:
            return None
        self.bus.poll_feedback_once()
        state = motor.get_state()
        if state is None:
            return None

        # Impedance control: tau = K*(x_r - x) + B*(x_dot_r - x_dot)
        control_dt_s = 0.02
        # Assumed control frequency.
        if control_dt_s <= 0.0:
            return None

        if not hasattr(self, "_gripper_prev_target_pos"):
            self._gripper_prev_target_pos = None
        if not hasattr(self, "_gripper_prev_filtered_target_vel"):
            self._gripper_prev_filtered_target_vel = None

        prev_target_pos = self._gripper_prev_target_pos
        if prev_target_pos is None:
            target_vel = 0.0
        else:
            target_vel = (pos_target - prev_target_pos) / control_dt_s
        self._gripper_prev_target_pos = pos_target

        lpf_alpha = 0.3
        target_vel_max = 3.0
        prev_filtered_vel = self._gripper_prev_filtered_target_vel
        if prev_filtered_vel is None:
            filtered_target_vel = target_vel
        else:
            filtered_target_vel = (
                lpf_alpha * target_vel + (1.0 - lpf_alpha) * prev_filtered_vel
            )
        target_vel = max(-target_vel_max, min(target_vel_max, filtered_target_vel))
        self._gripper_prev_filtered_target_vel = target_vel

        kp = float(self.config.gripper_mit_kp)
        kd = float(self.config.gripper_mit_kd)
        impedance_torque = (
            kp * (pos_target - state.pos)
            + kd * (target_vel - state.vel)
        )
        logger.debug(
            "Gripper MIT terms: pos_target=%.4f rad, state_pos=%.4f rad, "
            "target_vel=%.4f rad/s, state_vel=%.4f rad/s",
            pos_target,
            state.pos,
            target_vel,
            state.vel,
        )

        max_torque = max(0.0, float(self.config.gripper_mit_torque_limit))
        motor.request_feedback()
        return max(-max_torque, min(max_torque, impedance_torque))

    def calibrate(self) -> None:
        """Calibration procedure for B601."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using calibration file associated with the id {self.id}")
                return

        logger.info(f"\nRunning calibration for {self}")
        
        self.bus.disable_all()

        gripper_instruction = (
            ", and close its gripper"
            if FOLLOWER_GRIPPER_MOTOR in self.motors
            else ""
        )
        print(
            "\nCalibration: Set Zero Position\n"
            f"Please MANUALLY move the robot to its ZERO POSITION{gripper_instruction}.\n"
            "Reference the B601 manual for Zero Pose (generally the default sit-down position).\n"
        )
        input("Press ENTER when ready...")

        for motor in self.motors.values():
            motor.set_zero_position()
            time.sleep(LONG_TIMEOUT_SEC)
        
        logger.info("Arm zero position set.")

        logger.info("Setting range: -90° to +90° by default for all joints")
        self.calibration = {}
        for motor_name, (send_id, recv_id) in self.config.motor_can_ids.items():
            self.calibration[motor_name] = MotorCalibration(
                id=send_id,
                drive_mode=0,
                homing_offset=0,
                range_min=-90,
                range_max=90,
            )

        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def _send_joint_action(
        self,
        action: RobotAction
    ) -> RobotAction:
        """Send action command to robot."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Apply per-joint direction/scale mapping before clipping.
        for motor_name, position in goal_pos.items():
            direction = self.config.joint_directions.get(motor_name, 0.0)
            position = position * direction
            # print(f"motor_name: {motor_name}, position: {position}")
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                clipped_position = max(min_limit, min(max_limit, position))
                if clipped_position != position:
                    logger.debug(f"Clipped {motor_name} from {position:.2f} to {clipped_position:.2f}")
                position = clipped_position

            goal_pos[motor_name] = position

        # To tolerate 6-DOF leader arms that don't have a wrist_yaw joint, we can allow the follower to ignore missing wrist_yaw commands by treating them as 0.
        if 'wrist_yaw' not in goal_pos:
            goal_pos['wrist_yaw'] = 0.0

        goal_pos = self._apply_startup_action_sync(goal_pos)

        # Safety: Cap relative target
        if self.config.max_relative_target is not None:
            # We need current position in degrees to compare against relative limit safely
            present_pos = {}
            for motor_name, motor in self.motors.items():
                state = motor.get_state()
                if state is not None:
                    present_pos[motor_name] = math.degrees(state.pos)
                else:
                    present_pos[motor_name] = 0.0
            
            goal_present_pos = {key: (g_pos, present_pos.get(key, g_pos)) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Prepare and send commands
        for motor_name, position_degrees in goal_pos.items():
            try:
                idx = self.motor_names.index(motor_name)
            except ValueError:
                idx = 0 # Fallback

            # Convert target position from degrees to radians for motorbridge
            pos_rad = math.radians(position_degrees)
            vel_deg_s = (
                self.config.pos_vel_velocity[idx]
                if isinstance(self.config.pos_vel_velocity, list)
                else self.config.pos_vel_velocity
            )
            vel_rad = math.radians(vel_deg_s)

            motor = self.motors.get(motor_name)
            if motor is not None:
                if motor_name == FOLLOWER_GRIPPER_MOTOR:
                    if self.motor_type == "rs":
                        tau_ff = self.mit_output_torque_limit(motor, pos_rad)
                        if tau_ff is None:
                            tau_ff = 0.0
                        motor.send_mit(0, 0, 0, 1.5, tau_ff)
                        logger.debug(
                            f"Sent MIT command to {motor_name}: pos={position_degrees:.2f}°, "
                            f"tau_ff={tau_ff:.2f}"
                        )
                    else:
                        motor.send_force_pos(pos_rad, vel_rad, self.config.force_pos_torque_ration)
                        logger.debug(f"Sent FORCE_POS command to {motor_name}: pos={position_degrees:.2f}°, vel={vel_deg_s:.2f}°/s, ratio={0.1}")
                else:
                    if self.motor_type == "rs":
                        kp = getattr(self.config, "mit_kp", {}).get(motor_name, 0.0)
                        kd = getattr(self.config, "mit_kd", {}).get(motor_name, 0.0)
                        motor.send_mit(pos_rad, 0, kp, kd, 0)
                        logger.debug(
                            f"Sent MIT command to {motor_name}: "
                            f"pos={position_degrees:.2f}°, kp={kp}, kd={kd}"
                        )
                    else:
                        motor.send_pos_vel(pos_rad, 32)
                        logger.debug(f"Sent POS_VEL command to {motor_name}: target={pos_rad:.2f},pos={position_degrees:.2f}°, vel={vel_deg_s:.2f}°/s")

        # motorbridge sends packets mostly synchronously here over loop, 
        # so we don't need a bulk send command through ctypes.

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def _startup_sync_vlim_deg_s(self) -> dict[str, float]:
        vlim = self._joint_spec_to_deg_map(
            self.config.startup_sync_vlim_deg_s,
            field_name="startup_sync_vlim_deg_s",
        )
        if any(value < 0 for value in vlim.values()):
            raise ValueError("startup_sync_vlim_deg_s values must be >= 0.")
        return vlim

    def _apply_startup_action_sync(self, goal_pos: dict[str, float]) -> dict[str, float]:
        if not self._startup_sync_active or not goal_pos:
            return goal_pos

        now_s = time.perf_counter()
        if self._startup_sync_command_deg is None:
            current = self._read_motor_positions_rad(timeout_s=0.1)
            if current is None:
                logger.warning("Startup action sync skipped: current motor positions are unavailable.")
                self._startup_sync_active = False
                return goal_pos

            self._startup_sync_command_deg = {
                motor_name: math.degrees(current[motor_name])
                for motor_name in self.motor_names
                if motor_name in current
            }
            self._startup_sync_last_time_s = now_s
            logger.info(
                "Startup action sync active: limiting first leader tracking to configured velocity."
            )

        last_time_s = self._startup_sync_last_time_s or now_s
        dt_s = max(0.0, now_s - last_time_s)
        self._startup_sync_last_time_s = now_s

        vlim = self._startup_sync_vlim_deg_s()
        tolerance_deg = max(0.0, float(self.config.startup_sync_tolerance_deg))
        synced = True
        limited_goal_pos = dict(goal_pos)

        for motor_name, target_deg in goal_pos.items():
            if motor_name not in self.motor_names:
                continue

            command_deg = self._startup_sync_command_deg.get(motor_name, target_deg)
            max_step_deg = max(0.0, vlim[motor_name]) * dt_s
            delta_deg = target_deg - command_deg
            if max_step_deg <= 0.0:
                step_deg = 0.0
            else:
                step_deg = max(-max_step_deg, min(max_step_deg, delta_deg))
            command_deg += step_deg
            self._startup_sync_command_deg[motor_name] = command_deg
            limited_goal_pos[motor_name] = command_deg

            if abs(target_deg - command_deg) > tolerance_deg:
                synced = False

        if synced:
            self._startup_sync_active = False
            self._startup_sync_command_deg = None
            self._startup_sync_last_time_s = None
            logger.info("Startup action sync reached leader action; switching to direct teleop.")

        return limited_goal_pos

    def _read_motor_positions_rad(self, timeout_s: float = 0.5) -> dict[str, float] | None:
        if self.bus is None or not self.motors:
            return None

        deadline = time.perf_counter() + max(0.0, timeout_s)
        while True:
            for motor in self.motors.values():
                try:
                    motor.request_feedback()
                except Exception:
                    logger.debug("Failed to request motor feedback.", exc_info=True)
            try:
                self.bus.poll_feedback_once()
            except Exception:
                logger.debug("Failed to poll motor feedback.", exc_info=True)

            positions: dict[str, float] = {}
            for motor_name, motor in self.motors.items():
                state = motor.get_state()
                if state is None:
                    break
                positions[motor_name] = float(state.pos)
            if len(positions) == len(self.motors):
                return positions
            if time.perf_counter() >= deadline:
                return None
            time.sleep(0.02)

    def _return_to_initial_vlim_rad_s(self) -> dict[str, float]:
        spec = self.config.return_to_initial_vlim_deg_s
        return self._joint_spec_to_rad_map(spec, field_name="return_to_initial_vlim_deg_s")

    def _initial_joint_positions_rad(self) -> dict[str, float]:
        return self._joint_spec_to_rad_map(
            self.config.initial_joint_positions_deg,
            field_name="initial_joint_positions_deg",
        )

    def _joint_spec_to_rad_map(
        self,
        spec: float | list[float] | dict[str, float],
        *,
        field_name: str,
    ) -> dict[str, float]:
        values_deg = self._joint_spec_to_deg_map(spec, field_name=field_name)
        return {motor_name: math.radians(value) for motor_name, value in values_deg.items()}

    def _joint_spec_to_deg_map(
        self,
        spec: float | list[float] | dict[str, float],
        *,
        field_name: str,
    ) -> dict[str, float]:
        if isinstance(spec, dict):
            missing = [motor_name for motor_name in self.motor_names if motor_name not in spec]
            if missing:
                raise ValueError(f"{field_name} missing keys: {missing}")
            values_deg = {motor_name: float(spec[motor_name]) for motor_name in self.motor_names}
        elif isinstance(spec, list):
            if len(spec) != len(self.motor_names):
                raise ValueError(f"{field_name} list length must match the motor count.")
            values_deg = {
                motor_name: float(value)
                for motor_name, value in zip(self.motor_names, spec, strict=True)
            }
        else:
            values_deg = {motor_name: float(spec) for motor_name in self.motor_names}

        return values_deg

    def _send_motor_position_rad(self, motor_name: str, position_rad: float, vlim_rad_s: float) -> None:
        motor = self.motors.get(motor_name)
        if motor is None:
            return
        if self.motor_type == "rs":
            kp = getattr(self.config, "mit_kp", {}).get(motor_name, 0.0)
            kd = getattr(self.config, "mit_kd", {}).get(motor_name, 0.0)
            motor.send_mit(position_rad, 0.0, kp, kd, 0.0)
        else:
            motor.send_pos_vel(position_rad, vlim_rad_s)

    def _return_to_initial_position(
        self,
        reason: str = "disconnect",
        *,
        check_only: bool = False,
    ) -> None:
        current = self._read_motor_positions_rad(timeout_s=0.5)
        if current is None:
            logger.warning("Skip return-to-initial: current motor positions are unavailable.")
            return

        target = self._initial_joint_positions_rad()
        vlim = self._return_to_initial_vlim_rad_s()
        if any(value <= 0 for value in vlim.values()):
            raise ValueError("return_to_initial_vlim_deg_s values must be positive.")
        rate_hz = max(1.0, float(self.config.return_to_initial_rate_hz))
        dt_s = 1.0 / rate_hz
        tolerance_rad = math.radians(max(0.0, float(self.config.return_to_initial_tolerance_deg)))
        initial_error = max(abs(target[name] - current[name]) for name in self.motor_names)
        if initial_error <= tolerance_rad:
            logger.info(
                "Arm already at configured initial position before %s (max error %.2f deg).",
                reason,
                math.degrees(initial_error),
            )
            return
        if check_only:
            logger.info(
                "Arm is not at configured initial position before %s (max error %.2f deg); "
                "skipping return-to-initial.",
                reason,
                math.degrees(initial_error),
            )
            return

        command = dict(current)

        logger.info("Returning arm to configured initial position before %s.", reason)
        try:
            while True:
                for motor_name in self.motor_names:
                    delta = target[motor_name] - command[motor_name]
                    step = max(-vlim[motor_name] * dt_s, min(vlim[motor_name] * dt_s, delta))
                    command[motor_name] += step
                    self._send_motor_position_rad(motor_name, command[motor_name], vlim[motor_name])

                actual = self._read_motor_positions_rad(timeout_s=0.02)
                if actual is not None:
                    max_error = max(abs(target[name] - actual[name]) for name in self.motor_names)
                    if max_error <= tolerance_rad:
                        logger.info(
                            "Returned to initial position before %s (max error %.2f deg).",
                            reason,
                            math.degrees(max_error),
                        )
                        return
                time.sleep(dt_s)
        except KeyboardInterrupt:
            logger.warning("Return-to-initial interrupted; disconnecting now.")
            return
        except Exception:
            logger.warning("Return-to-initial failed; disconnecting now.", exc_info=True)
            return

    def _resolve_kinematic_urdf_path(self) -> Path:
        try:
            from rebotarm_control_rt.paths import (
                resolve_urdf_path,
                robstride_urdf_path,
            )
        except ImportError as error:
            raise ImportError(
                "rebotarm_control_rt is required for B601 RS Cartesian FK/IK. "
                "Install it in the same environment as this robot plugin."
            ) from error

        if self.config.kinematic_urdf_path is None:
            path = robstride_urdf_path()
        else:
            path = resolve_urdf_path(self.config.kinematic_urdf_path)
        path = Path(path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"B601 RS kinematic URDF not found: {path}")
        return path

    def _load_kinematic_model(self):
        if self._kinematic_model is None:
            try:
                from rebotarm_control_rt.kinematics import load_robot_model
            except ImportError as error:
                raise ImportError(
                    "rebotarm_control_rt is required for B601 RS Cartesian FK/IK. "
                    "Install it in the same environment as this robot plugin."
                ) from error

            urdf_path = self._resolve_kinematic_urdf_path()
            self._kinematic_model = load_robot_model(str(urdf_path))
            joint_names = tuple(self._kinematic_model.joint_names())
            if len(joint_names) != len(KINEMATIC_MOTORS):
                raise ValueError(
                    "B601 RS kinematic model must have exactly six joints; "
                    f"got {joint_names}."
                )
            self._kinematic_frame_id = self._kinematic_model.end_effector_frame_id()
        return self._kinematic_model

    @staticmethod
    def _rotation_6d_to_matrix(action: RobotAction) -> np.ndarray:
        a1 = np.array(
            [action["tcp.r1"], action["tcp.r2"], action["tcp.r3"]], dtype=float
        )
        a2 = np.array(
            [action["tcp.r4"], action["tcp.r5"], action["tcp.r6"]], dtype=float
        )
        a1_norm = float(np.linalg.norm(a1))
        if a1_norm < 1e-9:
            raise ValueError("Invalid tcp.r1-r3 rotation vector from Pico4Head.")
        b1 = a1 / a1_norm

        a2_orthogonal = a2 - float(np.dot(b1, a2)) * b1
        a2_norm = float(np.linalg.norm(a2_orthogonal))
        if a2_norm < 1e-9:
            raise ValueError("Invalid tcp.r4-r6 rotation vector from Pico4Head.")
        b2 = a2_orthogonal / a2_norm
        b3 = np.cross(b1, b2)
        return np.column_stack([b1, b2, b3])

    @staticmethod
    def _matrix_to_rotation_6d(rotation: np.ndarray) -> np.ndarray:
        return np.array(
            [
                rotation[0, 0],
                rotation[1, 0],
                rotation[2, 0],
                rotation[0, 1],
                rotation[1, 1],
                rotation[2, 1],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rotation[2, 1] - rotation[1, 2]) * s
            qy = (rotation[0, 2] - rotation[2, 0]) * s
            qz = (rotation[1, 0] - rotation[0, 1]) * s
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            s = 2.0 * math.sqrt(
                max(
                    1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
                    0.0,
                )
            )
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif rotation[1, 1] > rotation[2, 2]:
            s = 2.0 * math.sqrt(
                max(
                    1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
                    0.0,
                )
            )
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(
                max(
                    1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
                    0.0,
                )
            )
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s

        quaternion = np.array([qw, qx, qy, qz], dtype=np.float32)
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return quaternion / norm

    def _read_joint_positions_deg(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        positions = self._read_motor_positions_rad(timeout_s=0.1)
        if positions is None:
            raise TimeoutError("Timed out reading all six B601 RS joint positions.")
        return {
            motor_name: math.degrees(float(positions[motor_name]))
            for motor_name in KINEMATIC_MOTORS
        }

    @staticmethod
    def _kinematic_joint_rad(positions_deg: dict[str, float]) -> np.ndarray:
        return np.array(
            [math.radians(float(positions_deg[name])) for name in KINEMATIC_MOTORS],
            dtype=float,
        )

    def _tcp_pose_matrix_from_positions(
        self, positions_deg: dict[str, float]
    ) -> np.ndarray:
        model = self._load_kinematic_model()
        _, _, end_pose = model.fk(self._kinematic_joint_rad(positions_deg), "")
        return np.asarray(end_pose, dtype=float)

    def get_current_tcp_pose_quat(self) -> np.ndarray:
        """Return [x, y, z, qw, qx, qy, qz] for Pico4Head synchronization."""
        tcp_pose = self._tcp_pose_matrix_from_positions(
            self._read_joint_positions_deg()
        )
        quaternion = self._matrix_to_quaternion_wxyz(tcp_pose[:3, :3])
        return np.array(
            [
                float(tcp_pose[0, 3]),
                float(tcp_pose[1, 3]),
                float(tcp_pose[2, 3]),
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ],
            dtype=np.float32,
        )

    def _get_cartesian_observation(self) -> RobotObservation:
        tcp_pose = self._tcp_pose_matrix_from_positions(
            self._read_joint_positions_deg()
        )
        rotation_6d = self._matrix_to_rotation_6d(tcp_pose[:3, :3])
        observation: RobotObservation = {
            "tcp.x": float(tcp_pose[0, 3]),
            "tcp.y": float(tcp_pose[1, 3]),
            "tcp.z": float(tcp_pose[2, 3]),
        }
        for index, key in enumerate(TCP_POSE_KEYS[3:]):
            observation[key] = float(rotation_6d[index])
        for camera_name, camera in self.cameras.items():
            observation[camera_name] = camera.async_read()
        return observation

    def tcp_action_to_joint_action(self, action: RobotAction) -> RobotAction:
        missing = set(TCP_POSE_KEYS) - set(action)
        if missing:
            raise ValueError(f"Pico4Head TCP action missing keys: {sorted(missing)}")

        current_positions_deg = self._read_joint_positions_deg()
        target_tcp = np.eye(4, dtype=float)
        target_tcp[:3, :3] = self._rotation_6d_to_matrix(action)
        target_tcp[:3, 3] = [
            float(action["tcp.x"]),
            float(action["tcp.y"]),
            float(action["tcp.z"]),
        ]

        model = self._load_kinematic_model()
        q_seed = self._kinematic_joint_rad(current_positions_deg)
        result = model.solve_ik(target_tcp, q_seed, self._kinematic_frame_id)
        if not result.success and float(result.error) > float(self.config.ik_max_error):
            raise ValueError(
                "B601 RS IK failed: "
                f"error={float(result.error):.6f}, "
                f"iterations={getattr(result, 'iterations', None)}."
            )
        if not result.success:
            logger.warning(
                "B601 RS IK did not fully converge but is within tolerance: "
                "error=%.6f iterations=%s.",
                float(result.error),
                getattr(result, "iterations", None),
            )

        return {
            f"{motor_name}.pos": math.degrees(float(result.q[index]))
            for index, motor_name in enumerate(KINEMATIC_MOTORS)
        }

    def _send_cartesian_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        tcp_action: RobotAction = {key: float(action[key]) for key in TCP_POSE_KEYS}
        joint_action = self.tcp_action_to_joint_action(tcp_action)

        # IK output is already in the RS URDF/motor coordinate convention. The
        # base class direction mapping is for joint-space leader actions, so it
        # must not be applied a second time here. Pre-divide by that mapping so
        # the unchanged base sender produces the IK motor targets exactly.
        base_action: RobotAction = {}
        for motor_name in KINEMATIC_MOTORS:
            key = f"{motor_name}.pos"
            direction = float(self.config.joint_directions[motor_name])
            if direction == 0.0:
                raise ValueError(f"joint_directions[{motor_name!r}] must be non-zero.")
            base_action[key] = float(joint_action[key]) / direction
        # print(f"base_action: {base_action}")
        # print(f"joint_action: {joint_action}")
        # print(f"tcp_action: {tcp_action}")
        self._send_joint_action(base_action)

        return tcp_action

    def _add_discovered_taccap_camera_configs(
        self,
        config: SeeedB601RSFollowerConfig,
    ) -> None:
        side = taccap.resolve_gripper_side(config.taccap_role, config.taccap_side)
        camera_configs = dict(config.cameras)

        if config.enable_taccap_tactiles and config.expected_tactiles_per_side > 0:
            try:
                from lerobot_camera_xense import XenseTactileCameraConfig
            except ImportError as error:
                raise ImportError(
                    "lerobot_camera_xense is required when TacCap tactile cameras "
                    "are enabled."
                ) from error

            tactiles = taccap.discover_tactiles_by_hub(config.taccap_role)
            discovered = tactiles.get(side, {})
            if len(discovered) != config.expected_tactiles_per_side:
                raise ValueError(
                    f"Expected {config.expected_tactiles_per_side} {side} TacCap "
                    f"tactiles, found {len(discovered)}."
                )
            for finger, serial_number in sorted(discovered.items()):
                camera_configs[f"tactile_{finger}"] = XenseTactileCameraConfig(
                    serial_number=serial_number,
                    fps=config.tactile_fps,
                    output_types=list(config.tactile_output_types),
                )

        if config.enable_taccap_wrist_camera:
            serial_number = taccap.discover_wrist_cameras(config.taccap_role).get(side)
            if not serial_number:
                raise ValueError(f"No {side} follower TacCap wrist camera discovered.")
            camera_configs["wrist_cam"] = OpenCVCameraConfig(
                index_or_path=taccap.resolve_wrist_camera_path(serial_number),
                width=config.wrist_camera_width,
                height=config.wrist_camera_height,
                fps=config.wrist_camera_fps,
            )
        config.cameras = camera_configs

    def connect(self, calibrate: bool = True) -> None:
        if self._uses_taccap:
            taccap.prewarm_tactile_config_cache(self.config.cameras, logger)
        super().connect(calibrate=calibrate)
        if self.config.return_to_initial_on_connect:
            self._return_to_initial_position(reason="connect")
        self._startup_sync_active = bool(
            self.config.startup_sync_to_action_on_connect
        )
        self._startup_sync_command_deg = None
        self._startup_sync_last_time_s = None
        if self._uses_taccap:
            if self.taccap_gripper is None:
                raise RuntimeError("TacCap controller was not initialized.")
            self.taccap_gripper.connect()

    def get_observation(self) -> RobotObservation:
        observation = (
            self._get_cartesian_observation()
            if self.config.action_mode == "cartesian"
            else super().get_observation()
        )
        if self._uses_taccap:
            if self.taccap_gripper is None:
                raise DeviceNotConnectedError("TacCap gripper is not initialized.")
            observation.update(self.taccap_gripper.read_observation())
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        if self.config.action_mode == "cartesian":
            return self._send_cartesian_action(action)

        gripper_target = (
            action.get(self.gripper_action_name) if self._uses_taccap else None
        )
        arm_action = {
            key: value
            for key, value in action.items()
            if not (self._uses_taccap and key == self.gripper_action_name)
        }
        sent_action = self._send_joint_action(arm_action)
        if gripper_target is not None:
            if self.taccap_gripper is None:
                raise DeviceNotConnectedError("TacCap gripper is not initialized.")
            self.taccap_gripper.send_position(gripper_target)
            sent_action[self.gripper_action_name] = self.taccap_gripper.last_target
        return sent_action

    def disconnect(self) -> None:
        arm_connected = self.bus is not None
        controller = self.taccap_gripper
        if not arm_connected and (controller is None or not controller.is_connected):
            raise DeviceNotConnectedError(f"{self} is not connected.")

        sentinel_added = False
        try:
            if arm_connected:
                if self.config.return_to_initial_on_disconnect:
                    self._return_to_initial_position(
                        reason="disconnect",
                        check_only=bool(
                            self.config.return_to_initial_check_only_on_disconnect
                        ),
                    )
                if self._uses_taccap and controller is not None and not controller.is_connected:
                    controller.gripper = object()
                    sentinel_added = True
                super().disconnect()
        finally:
            if sentinel_added and controller is not None:
                controller.gripper = None
            if controller is not None and controller.is_connected:
                controller.disconnect()
