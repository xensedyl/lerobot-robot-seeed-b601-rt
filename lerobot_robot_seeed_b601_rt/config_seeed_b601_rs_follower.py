"""Configuration for the motorbridge-controlled B601 RS follower."""

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.robots.robot import RobotConfig

from .seeed_b601_follower import SeeedB601FollowerConfigBase


_ARM_MOTOR_CAN_IDS = {
    "shoulder_pan": (0x01, 0xFD),
    "shoulder_lift": (0x02, 0xFD),
    "elbow_flex": (0x03, 0xFD),
    "wrist_flex": (0x04, 0xFD),
    "wrist_yaw": (0x05, 0xFD),
    "wrist_roll": (0x06, 0xFD),
}
_ARM_JOINT_LIMITS = {
    "shoulder_pan": (-145.0, 145.0),
    "shoulder_lift": (-0.0, 170.0),
    "elbow_flex": (-0.0, 200.0),
    "wrist_flex": (-80.0, 90.0),
    "wrist_yaw": (-90.0, 90.0),
    "wrist_roll": (-90.0, 90.0),
}
_ARM_JOINT_DIRECTIONS = {
    "shoulder_pan": 1.0,
    "shoulder_lift": 1.0,
    "elbow_flex": -1.0,
    "wrist_flex": -1.0,
    "wrist_yaw": -1.0,
    "wrist_roll": 1.0,
}


@RobotConfig.register_subclass("seeed_b601_rs_follower")
@dataclass
class SeeedB601RSFollowerConfig(RobotConfig, SeeedB601FollowerConfigBase):
    """All single-arm motorbridge RS modes in one configuration.

    action_mode selects joint actions or Pico4Head-compatible 9-D Cartesian
    actions. gripper_type selects the ID7 motor gripper, no gripper, or TacCap.
    """

    action_mode: str = "joint"  # joint | cartesian
    gripper_type: str = "motor"  # motor | none | taccap

    mit_kp: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 50.0,
            "shoulder_lift": 150.0,
            "elbow_flex": 150.0,
            "wrist_flex": 50.0,
            "wrist_yaw": 50.0,
            "wrist_roll": 50.0,
        }
    )
    mit_kd: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 3.0,
            "shoulder_lift": 10.0,
            "elbow_flex": 10.0,
            "wrist_flex": 5.0,
            "wrist_yaw": 4.0,
            "wrist_roll": 4.0,
        }
    )
    motor_can_ids: dict[str, tuple[int, int]] = field(
        default_factory=lambda: {**_ARM_MOTOR_CAN_IDS, "gripper": (0x07, 0xFD)}
    )
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {**_ARM_JOINT_LIMITS, "gripper": (-0.0, 270.0)}
    )
    joint_directions: dict[str, float] = field(
        default_factory=lambda: {**_ARM_JOINT_DIRECTIONS, "gripper": 6.0}
    )
    pos_vel_velocity: float | list[float] = field(
        default_factory=lambda: [50, 0.4, 0.4, 50, 50, 50, 0]
    )

    # RS lifecycle and startup synchronization.
    return_to_initial_on_connect: bool | None = None
    initial_joint_positions_deg: float | list[float] | dict[str, float] = 0.0
    return_to_initial_on_disconnect: bool = True
    return_to_initial_vlim_deg_s: float | list[float] | dict[str, float] = 15.0
    return_to_initial_tolerance_deg: float = 1.0
    return_to_initial_rate_hz: float = 50.0
    return_to_initial_check_only_on_disconnect: bool = False

    startup_sync_to_action_on_connect: bool | None = None
    startup_sync_vlim_deg_s: float | list[float] | dict[str, float] = 15.0
    startup_sync_tolerance_deg: float = 1.0

    # ID7 motorbridge gripper MIT control.
    gripper_mit_kp: float = 12.0
    gripper_mit_kd: float = 0.05
    gripper_mit_torque_limit: float = 10.0

    # Cartesian FK/IK used by Pico4Head. None selects the RobStride URDF
    # installed inside rebotarm_control_rt; set a path only to override it.
    kinematic_urdf_path: str | Path | None = None
    ik_max_error: float = 0.01

    # External TacCap gripper.
    connect_taccap_gripper: bool = True
    gripper_device: str | None = None
    gripper_wrist_video: str = ""
    gripper_baudrate: int = 3_000_000
    gripper_ack_timeout_ms: int = 1000
    gripper_max_retries: int = 2
    gripper_open_cameras: bool = False
    gripper_reload_config_on_connect: bool = True

    auto_discover_taccap_cameras: bool = True
    taccap_role: str = "follower"
    taccap_side: str | None = None
    expected_tactiles_per_side: int = 2
    enable_taccap_tactiles: bool = True
    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    enable_taccap_wrist_camera: bool = True
    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    enable_gripper_on_connect: bool = True
    disable_gripper_on_disconnect: bool = True
    clear_gripper_fault_on_connect: bool = True
    gripper_kp: float = 15.0
    gripper_kd: float = 1.0
    gripper_feedforward_torque: float = 3.0
    gripper_torque_grasp_enabled: bool = True
    normalize_gripper_action: bool = True
    gripper_action_min: float = 0.0
    gripper_action_max: float = 55.0
    print_gripper_torque: bool = True
    gripper_torque_print_hz: float = 10.0

    def __post_init__(self) -> None:
        super().__post_init__()
        self.action_mode = str(self.action_mode).strip().lower()
        self.gripper_type = str(self.gripper_type).strip().lower()

        if self.action_mode not in {"joint", "cartesian"}:
            raise ValueError("action_mode must be 'joint' or 'cartesian'.")
        if self.gripper_type not in {"motor", "none", "taccap"}:
            raise ValueError("gripper_type must be 'motor', 'none', or 'taccap'.")
        if self.action_mode == "cartesian" and self.gripper_type != "none":
            raise ValueError(
                "cartesian mode is the six-axis Pico4Head mode and requires "
                "gripper_type='none'."
            )

        is_pico = self.action_mode == "cartesian"
        is_taccap = self.gripper_type == "taccap"
        if self.return_to_initial_on_connect is None:
            self.return_to_initial_on_connect = not (is_pico or is_taccap)
        if self.startup_sync_to_action_on_connect is None:
            self.startup_sync_to_action_on_connect = is_taccap

        missing_arm_motors = [
            name for name in _ARM_MOTOR_CAN_IDS if name not in self.motor_can_ids
        ]
        if missing_arm_motors:
            raise ValueError(
                f"motor_can_ids missing RS arm motors: {missing_arm_motors}"
            )

        self.motor_can_ids = dict(self.motor_can_ids)
        self.joint_limits = dict(self.joint_limits)
        self.joint_directions = dict(self.joint_directions)
        if self.gripper_type == "motor":
            self.motor_can_ids.setdefault("gripper", (0x07, 0xFD))
            self.joint_limits.setdefault("gripper", (-0.0, 270.0))
            self.joint_directions.setdefault("gripper", 6.0)
        else:
            self.motor_can_ids.pop("gripper", None)
            self.joint_limits.pop("gripper", None)
            self.joint_directions.pop("gripper", None)

        if isinstance(self.pos_vel_velocity, list):
            expected_count = len(self.motor_can_ids)
            if len(self.pos_vel_velocity) == 7 and expected_count == 6:
                self.pos_vel_velocity = list(self.pos_vel_velocity[:6])
            elif len(self.pos_vel_velocity) != expected_count:
                raise ValueError(
                    "pos_vel_velocity list length must match the selected motor count."
                )

        if self.ik_max_error < 0:
            raise ValueError("ik_max_error must be >= 0.")
        if self.expected_tactiles_per_side < 0:
            raise ValueError("expected_tactiles_per_side must be >= 0.")
        if self.tactile_fps <= 0 or self.wrist_camera_fps <= 0:
            raise ValueError("TacCap camera FPS values must be positive.")
        if self.gripper_baudrate <= 0:
            raise ValueError("gripper_baudrate must be positive.")
        if self.gripper_ack_timeout_ms <= 0:
            raise ValueError("gripper_ack_timeout_ms must be positive.")
        if self.gripper_max_retries < 0:
            raise ValueError("gripper_max_retries must be >= 0.")
        if self.startup_sync_tolerance_deg < 0:
            raise ValueError("startup_sync_tolerance_deg must be >= 0.")
        if self.gripper_torque_print_hz < 0:
            raise ValueError("gripper_torque_print_hz must be >= 0.")
