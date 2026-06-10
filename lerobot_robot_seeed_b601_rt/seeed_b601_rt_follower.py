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

KINEMATIC_MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]
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
        self._kinematic_model = None
        self._kinematic_frame_id: int | None = None
        self._validate_config()

    def _validate_config(self) -> None:
        ids = set(self.config.motor_can_ids)
        required_maps = {
            "motor_models": set(self.config.motor_models),
            "joint_limits": set(self.config.joint_limits),
            "pos_vel_gains": set(self.config.pos_vel_gains),
        }
        for field_name, keys in required_maps.items():
            missing = ids - keys
            if missing:
                raise ValueError(f"{field_name} missing keys: {sorted(missing)}")
        for motor_name in ids:
            gains = self.config.pos_vel_gains[motor_name]
            if len(gains) != 4:
                raise ValueError(
                    f"pos_vel_gains[{motor_name!r}] must contain "
                    "(vel_kp, vel_ki, pos_kp, pos_ki)."
                )
            if any(float(value) < 0.0 for value in gains):
                raise ValueError(f"pos_vel_gains[{motor_name!r}] values must be >= 0.")
        mode = self.config.control_mode.lower()
        if mode not in {"pos_vel", "mit"}:
            raise ValueError("control_mode must be 'pos_vel' or 'mit'.")
        action_mode = self.config.action_mode.lower()
        if action_mode not in {"joint", "cartesian"}:
            raise ValueError("action_mode must be 'joint' or 'cartesian'.")
        if self.config.can_adapter not in {"damiao", "socketcan", "robstride"}:
            raise ValueError("can_adapter must be 'damiao', 'socketcan', or 'robstride'.")
        if isinstance(self.config.pos_vel_velocity, list) and len(self.config.pos_vel_velocity) not in {
            len(self.motor_names),
            len(self.config.motor_can_ids),
        }:
            raise ValueError(
                "pos_vel_velocity list length must match the controlled joint count or the full motor count."
            )
        self._return_to_initial_vlim_rad()
        if self.config.rt_command_gap_us < 0:
            raise ValueError("rt_command_gap_us must be >= 0.")
        if self.config.damiao_tx_debug < 0:
            raise ValueError("damiao_tx_debug must be >= 0.")
        if self.config.debug_motion_interval_s <= 0:
            raise ValueError("debug_motion_interval_s must be > 0.")
        if not 0.0 <= self.config.gripper_force_pos_torque_ratio <= 1.0:
            raise ValueError("gripper_force_pos_torque_ratio must be in [0, 1].")
    @property
    def _action_motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motor_names}

    @property
    def _action_tcp_ft(self) -> dict[str, type]:
        features = dict.fromkeys(TCP_POSE_KEYS, float)
        if "gripper" in self.motor_names:
            features["gripper.pos"] = float
        return features

    def _observation_motor_key(self, motor_name: str) -> str:
        if motor_name not in KINEMATIC_MOTORS:
            return motor_name
        return f"joint_{KINEMATIC_MOTORS.index(motor_name) + 1}"

    @property
    def _observation_motors_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor in self.motor_names:
            obs_key = self._observation_motor_key(motor)
            if motor == "gripper":
                features[f"{obs_key}.pos"] = float
                if self.config.enable_observation_gripper_vel:
                    features[f"{obs_key}.vel"] = float
                if self.config.enable_observation_gripper_torque:
                    features[f"{obs_key}.torque"] = float
                continue

            if self.config.enable_observation_joint_pos:
                features[f"{obs_key}.pos"] = float
            if self.config.enable_observation_joint_vel:
                features[f"{obs_key}.vel"] = float
            if self.config.enable_observation_joint_torque:
                features[f"{obs_key}.torque"] = float
        return features

    @property
    def _observation_tcp_ft(self) -> dict[str, type]:
        if self.config.action_mode.lower() != "cartesian":
            return {}
        return dict.fromkeys(TCP_POSE_KEYS, float)

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._observation_motors_ft, **self._observation_tcp_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        if self.config.action_mode.lower() == "cartesian":
            return self._action_tcp_ft
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
            if len(velocity) == len(self.motor_names):
                values = velocity
            else:
                all_motor_names = list(self.config.motor_can_ids)
                values = [velocity[all_motor_names.index(motor_name)] for motor_name in self.motor_names]
        else:
            values = [velocity] * len(self.motor_names)
        return [math.radians(float(v)) for v in values]

    @staticmethod
    def _rotation_6d_to_matrix(action: RobotAction) -> np.ndarray:
        a1 = np.array([action["tcp.r1"], action["tcp.r2"], action["tcp.r3"]], dtype=float)
        a2 = np.array([action["tcp.r4"], action["tcp.r5"], action["tcp.r6"]], dtype=float)
        b1_norm = np.linalg.norm(a1)
        if b1_norm < 1e-9:
            raise ValueError("Invalid tcp.r1-r3 rotation vector from Pico4 action.")
        b1 = a1 / b1_norm

        a2_orth = a2 - np.dot(b1, a2) * b1
        b2_norm = np.linalg.norm(a2_orth)
        if b2_norm < 1e-9:
            raise ValueError("Invalid tcp.r4-r6 rotation vector from Pico4 action.")
        b2 = a2_orth / b2_norm
        b3 = np.cross(b1, b2)
        return np.column_stack([b1, b2, b3])

    @staticmethod
    def _matrix_to_rotation_6d(rot: np.ndarray) -> np.ndarray:
        return np.array(
            [
                rot[0, 0],
                rot[1, 0],
                rot[2, 0],
                rot[0, 1],
                rot[1, 1],
                rot[2, 1],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _matrix_to_quaternion_wxyz(rot: np.ndarray) -> np.ndarray:
        trace = float(np.trace(rot))
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rot[2, 1] - rot[1, 2]) * s
            qy = (rot[0, 2] - rot[2, 0]) * s
            qz = (rot[1, 0] - rot[0, 1]) * s
        elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s = 2.0 * math.sqrt(max(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2], 0.0))
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = 2.0 * math.sqrt(max(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2], 0.0))
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(max(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1], 0.0))
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s

        quat = np.array([qw, qx, qy, qz], dtype=np.float32)
        norm = np.linalg.norm(quat)
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return quat / norm

    def _kinematic_joint_rad(self, positions_deg: dict[str, float]) -> np.ndarray:
        return np.array([math.radians(float(positions_deg[name])) for name in KINEMATIC_MOTORS], dtype=float)

    def _load_kinematic_model(self):
        if self._kinematic_model is None:
            from rebotarm_control_rt.kinematics import load_robot_model

            self._kinematic_model = load_robot_model(self.config.kinematic_urdf_path)
            self._kinematic_frame_id = self._kinematic_model.end_effector_frame_id()
        return self._kinematic_model

    def _tcp_pose_matrix_from_positions(self, positions_deg: dict[str, float]) -> np.ndarray:
        model = self._load_kinematic_model()
        _, _, end_pose = model.fk(self._kinematic_joint_rad(positions_deg), "")
        return np.asarray(end_pose, dtype=float)

    def _gripper_pos_to_norm(self, gripper_pos_deg: float) -> float:
        open_deg, closed_deg = self.config.joint_limits["gripper"]
        span = closed_deg - open_deg
        if abs(span) < 1e-9:
            return 1.0
        gripper_norm = (float(gripper_pos_deg) - open_deg) / span
        return max(0.0, min(1.0, gripper_norm))

    def get_current_tcp_pose_quat(self) -> np.ndarray:
        """Return current TCP pose as [x, y, z, qw, qx, qy, qz, gripper_norm]."""
        if self.arm is None or not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self._last_positions_deg:
            positions_deg, _, _ = self._read_state_deg(request=False)
        else:
            positions_deg = dict(self._last_positions_deg)

        tcp_pose = self._tcp_pose_matrix_from_positions(positions_deg)
        quat = self._matrix_to_quaternion_wxyz(tcp_pose[:3, :3])

        gripper_norm = 1.0
        if "gripper" in positions_deg and "gripper" in self.config.joint_limits:
            gripper_norm = self._gripper_pos_to_norm(positions_deg["gripper"])

        return np.array(
            [
                float(tcp_pose[0, 3]),
                float(tcp_pose[1, 3]),
                float(tcp_pose[2, 3]),
                float(quat[0]),
                float(quat[1]),
                float(quat[2]),
                float(quat[3]),
                float(gripper_norm),
            ],
            dtype=np.float32,
        )

    def tcp_action_to_joint_action(self, action: RobotAction) -> RobotAction:
        """Convert a Pico4 Cartesian TCP action to B601 joint-space action."""
        required = set(TCP_POSE_KEYS)
        missing = required - set(action)
        if missing:
            raise ValueError(f"Pico4 TCP action missing keys: {sorted(missing)}")

        if not self._last_positions_deg:
            self._read_state_deg(request=False)

        model = self._load_kinematic_model()
        target_tcp = np.eye(4, dtype=float)
        target_tcp[:3, :3] = self._rotation_6d_to_matrix(action)
        target_tcp[:3, 3] = [float(action["tcp.x"]), float(action["tcp.y"]), float(action["tcp.z"])]

        q_seed = self._kinematic_joint_rad(self._last_positions_deg)
        result = model.solve_ik(target_tcp, q_seed, self._kinematic_frame_id)
        if not result.success:
            logger.warning(
                "Pico4 IK did not fully converge: error=%.5f iterations=%s; using best solution.",
                float(result.error),
                getattr(result, "iterations", None),
            )

        joint_action: RobotAction = {}
        for idx, motor_name in enumerate(KINEMATIC_MOTORS):
            joint_action[f"{motor_name}.pos"] = math.degrees(float(result.q[idx]))

        if "gripper" in self.motor_names and "gripper.pos" in action:
            gripper_norm = max(0.0, min(1.0, float(action["gripper.pos"])))
            open_deg, closed_deg = self.config.joint_limits["gripper"]
            joint_action["gripper.pos"] = open_deg + gripper_norm * (closed_deg - open_deg)

        return joint_action

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
            vel_kp, vel_ki, pos_kp, pos_ki = self.config.pos_vel_gains[motor_name]
            lines.extend(
                [
                    f"  - name: {motor_name}",
                    f"    motor_id: 0x{int(motor_id):02X}",
                    f"    feedback_id: 0x{int(feedback_id):02X}",
                    f"    model: \"{self.config.motor_models[motor_name]}\"",
                    f"    vendor: \"{self._default_vendor(motor_name)}\"",
                    "    POS_VEL:",
                    f"      vel_kp: {float(vel_kp)}",
                    f"      vel_ki: {float(vel_ki)}",
                    f"      pos_kp: {float(pos_kp)}",
                    f"      pos_ki: {float(pos_ki)}",
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
        current_positions_deg = self._read_positions_deg_blocking()
        self._clear_possible_mit_torque(current_positions_deg)

        mode = self.config.control_mode.lower()
        if mode == "pos_vel":
            self.arm.mode_pos_vel(vlim=self._velocity_limits_rad())
            self._configure_gripper_force_pos_mode()
        else:
            self.arm.mode_mit()

        self._initial_positions_deg = dict(current_positions_deg)
        self._apply_initial_gripper_pose(self._initial_positions_deg)
        self._last_goal_deg = dict(self._initial_positions_deg)
        self._set_rt_target_deg(self._initial_positions_deg)
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

    def _read_positions_deg_blocking(self, attempts: int = 60) -> dict[str, float]:
        if self.arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not hasattr(self.arm, "get_positions_blocking"):
            raise RuntimeError(
                "rebotarm_control_rt is missing get_positions_blocking(); reinstall/rebuild it before "
                "running the B601 RT follower safely."
            )

        positions = np.asarray(self.arm.get_positions_blocking(attempts=attempts), dtype=float)

        if positions.size != len(self.motor_names):
            raise RuntimeError(
                f"Expected {len(self.motor_names)} joint positions, got {positions.size}; "
                "refusing to build an initial RT target."
            )

        pos_deg = {
            motor_name: math.degrees(float(positions[idx]))
            for idx, motor_name in enumerate(self.motor_names)
        }
        self._last_positions_deg = dict(pos_deg)
        return pos_deg

    def _clear_possible_mit_torque(self, positions_deg: dict[str, float], frames: int = 5, dt_s: float = 0.02) -> None:
        if self.arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        pos_rad = [math.radians(positions_deg[motor_name]) for motor_name in self.motor_names]
        zeros = [0.0] * len(self.motor_names)
        try:
            self.arm.mode_mit(kp=zeros, kd=zeros, stabilize_delay=0.0)
            for _ in range(frames):
                self.arm.mit(
                    pos=pos_rad,
                    vel=zeros,
                    kp=zeros,
                    kd=zeros,
                    tau=zeros,
                    request_feedback=False,
                )
                time.sleep(dt_s)
        except Exception:
            logger.warning("Failed to clear possible MIT torque before taking control.", exc_info=True)
            raise

    def _configure_gripper_force_pos_mode(self) -> None:
        if (
            self.config.control_mode.lower() != "pos_vel"
            or not self.config.enabled_gripper_force
            or "gripper" not in self.motor_names
        ):
            return

        try:
            self.arm.ensure_mode("gripper", 4, timeout_ms=1000)
        except Exception:
            logger.exception("Failed to switch gripper to FORCE_POS mode.")
            raise
        logger.info(
            "gripper ensure mode FORCE_POS with torque ratio %.3f.",
            float(self.config.gripper_force_pos_torque_ratio),
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
            obs_key = self._observation_motor_key(motor_name)
            if motor_name == "gripper":
                obs_dict[f"{obs_key}.pos"] = self._gripper_pos_to_norm(pos_deg[motor_name])
                if self.config.enable_observation_gripper_vel:
                    obs_dict[f"{obs_key}.vel"] = vel_deg[motor_name]
                if self.config.enable_observation_gripper_torque:
                    obs_dict[f"{obs_key}.torque"] = torque[motor_name]
                continue

            if self.config.enable_observation_joint_pos:
                obs_dict[f"{obs_key}.pos"] = pos_deg[motor_name]
            if self.config.enable_observation_joint_vel:
                obs_dict[f"{obs_key}.vel"] = vel_deg[motor_name]
            if self.config.enable_observation_joint_torque:
                obs_dict[f"{obs_key}.torque"] = torque[motor_name]

        if self.config.action_mode.lower() == "cartesian":
            tcp_pose = self._tcp_pose_matrix_from_positions(pos_deg)
            r6d = self._matrix_to_rotation_6d(tcp_pose[:3, :3])
            obs_dict["tcp.x"] = float(tcp_pose[0, 3])
            obs_dict["tcp.y"] = float(tcp_pose[1, 3])
            obs_dict["tcp.z"] = float(tcp_pose[2, 3])
            for idx, key in enumerate(TCP_POSE_KEYS[3:]):
                obs_dict[key] = float(r6d[idx])

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
        if self.config.action_mode.lower() == "cartesian":
            # Pico4/cartesian teleoperation starts with the trigger released, which means open.
            positions_deg["gripper"] = self._clip(
                self.config.joint_limits["gripper"][0],
                self.config.joint_limits["gripper"],
            )
            return

        # B601 joint-control convention: gripper zero is the closed initial pose.
        positions_deg["gripper"] = self._clip(0.0, self.config.joint_limits["gripper"])

    def _force_pos_flags(self) -> tuple[list[bool] | None, list[float] | None]:
        if (
            self.config.control_mode.lower() != "pos_vel"
            or not self.config.enabled_gripper_force
            or "gripper" not in self.motor_names
        ):
            return None, None

        force_pos = [motor_name == "gripper" for motor_name in self.motor_names]
        torque_ratio = [
            float(self.config.gripper_force_pos_torque_ratio) if motor_name == "gripper" else 0.0
            for motor_name in self.motor_names
        ]
        return force_pos, torque_ratio

    def _set_rt_target_deg(self, goal_pos: dict[str, float], vlim_rad: list[float] | None = None) -> None:
        target_rad = [math.radians(goal_pos[motor_name]) for motor_name in self.motor_names]
        force_pos, force_pos_torque_ratio = self._force_pos_flags()
        self.arm.set_targets(
            pos=target_rad,
            vlim=vlim_rad if vlim_rad is not None else self._velocity_limits_rad(),
            force_pos=force_pos,
            force_pos_torque_ratio=force_pos_torque_ratio,
        )

    def _return_to_initial_vlim_rad(self) -> list[float]:
        """Return per-joint reset velocity limits in rad/s."""
        spec = self.config.return_to_initial_vlim_deg_s
        normal_vlim = self._velocity_limits_rad()
        if isinstance(spec, (list, tuple)) and len(spec) not in {len(self.motor_names), len(self.config.motor_can_ids)}:
            raise ValueError(
                "return_to_initial_vlim_deg_s list length must match the controlled joint count or the full motor count."
            )

        values: list[float] = []
        for idx, motor_name in enumerate(self.motor_names):
            if isinstance(spec, dict):
                if motor_name not in spec:
                    raise ValueError(f"return_to_initial_vlim_deg_s missing key: {motor_name}")
                deg_s = spec[motor_name]
            elif isinstance(spec, (list, tuple)):
                if len(spec) == len(self.motor_names):
                    deg_s = spec[idx]
                else:
                    all_motor_names = list(self.config.motor_can_ids)
                    deg_s = spec[all_motor_names.index(motor_name)]
            else:
                deg_s = spec

            deg_s = float(deg_s)
            if deg_s <= 0:
                raise ValueError("return_to_initial_vlim_deg_s values must be > 0.")
            values.append(min(math.radians(deg_s), normal_vlim[idx]))
        return values

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

    def _complete_tcp_action(self, action: RobotAction) -> RobotAction:
        missing = set(TCP_POSE_KEYS) - set(action)
        if missing:
            raise ValueError(f"Cartesian action missing keys: {sorted(missing)}")

        tcp_action: RobotAction = {key: float(action[key]) for key in TCP_POSE_KEYS}
        if "gripper" in self.motor_names:
            if "gripper.pos" in action:
                gripper_norm = float(action["gripper.pos"])
            else:
                current = self.get_current_tcp_pose_quat()
                gripper_norm = float(current[7])
            tcp_action["gripper.pos"] = max(0.0, min(1.0, gripper_norm))
        return tcp_action

    def _send_joint_action(self, action: RobotAction) -> RobotAction:
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

        self._set_rt_target_deg(goal_pos)
        self._last_goal_deg = dict(goal_pos)
        self._maybe_log_motion_debug(goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def send_action(self, action: RobotAction) -> RobotAction:
        if self.arm is None or not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        action_mode = self.config.action_mode.lower()
        is_tcp_action = any(key in action for key in TCP_POSE_KEYS)
        if action_mode == "cartesian" or is_tcp_action:
            tcp_action = self._complete_tcp_action(action)
            joint_action = self.tcp_action_to_joint_action(tcp_action)
            self._send_joint_action(joint_action)
            return tcp_action

        return self._send_joint_action(action)

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
            "%s motion debug: max_delta=%s %.2f deg | target=%s | current=%s | rt_overruns(send/read)=%s/%s",
            self.id or self.config.port,
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

        try:
            current, _, _ = self._read_state_deg(request=False)
        except Exception:
            logger.debug("Failed to read state before returning to initial position.", exc_info=True)
            return

        target = {
            name: self._clip(self._initial_positions_deg[name], self.config.joint_limits[name])
            for name in self.motor_names
        }
        return_started_s = time.monotonic()
        return_vlim_rad = self._return_to_initial_vlim_rad()

        try:
            self._set_rt_target_deg(target, vlim_rad=return_vlim_rad)
            self._last_goal_deg = dict(target)
        except Exception:
            logger.debug("Failed to command initial position before disconnect.", exc_info=True)
            return

        expected_s = max(
            abs(target[name] - current.get(name, target[name])) / math.degrees(return_vlim_rad[idx])
            for idx, name in enumerate(self.motor_names)
        )
        deadline = return_started_s + expected_s + 0.02
        tolerance = 0.1
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

    def reset_to_initial_position(self) -> None:
        self._return_to_initial_position()

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
