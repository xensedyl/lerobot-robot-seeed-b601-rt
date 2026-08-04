from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.robot import RobotConfig

from ..grippers.config_serial_gripper import SerialGripperConfig
from ..grippers.config_taccap_gripper import TacCapGripperConfig


class ActionMode(str, Enum):
    """Control mode for reBot Arm B601.

    Note: vel and torque are not observed.
    JOINT:
        Action: joint positions (6D) + gripper (1D) = 7D
        Observation: joint positions (6D) + gripper (1D) = 7D

    CARTESIAN:
        Action: TCP position (3D) + rotation (6D) + gripper (1D) = 10D
        Observation: TCP position (3D) + rotation (6D) + gripper (1D) = 10D
    """

    JOINT = "joint"
    CARTESIAN = "cartesian"

class ControlMode(str, Enum):
    """Control mode for reBot Arm B601.

    POS_VEL:
        Position velocity control.
    MIT:
        MIT control.
    """
    POS_VEL = "pos_vel"
    MIT = "mit"

class GripperType(str, Enum):
    """Gripper type for reBot Arm B601."""
    SERIAL = "serial" # Serial gripper.
    REBOTARMB601 = "rebotarm_b601" # reBotArm B601 gripper.
    TACCAP = "taccap" # Xense TacCap follower gripper.


_DEFAULT_PORT = "/dev/ttyACM0"
_ROBSTRIDE_PORT = "can0"
_DEFAULT_KINEMATIC_URDF_PATH = "lerobot_robot_seeed_b601_rt/tool_calibration.urdf"
_ROBSTRIDE_URDF_PATH = "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
_ROBSTRIDE_END_EFFECTOR_FRAME = "gripper_end"


def _dm_motor_can_ids() -> dict[str, tuple[int, int]]:
    return {
        "shoulder_pan": (0x01, 0x11),
        "shoulder_lift": (0x02, 0x12),
        "elbow_flex": (0x03, 0x13),
        "wrist_flex": (0x04, 0x14),
        "wrist_yaw": (0x05, 0x15),
        "wrist_roll": (0x06, 0x16),
        "gripper": (0x07, 0x17),
    }


def _rs_motor_can_ids() -> dict[str, tuple[int, int]]:
    return {
        "shoulder_pan": (0x01, 0xFD),
        "shoulder_lift": (0x02, 0xFD),
        "elbow_flex": (0x03, 0xFD),
        "wrist_flex": (0x04, 0xFD),
        "wrist_yaw": (0x05, 0xFD),
        "wrist_roll": (0x06, 0xFD),
        "gripper": (0x07, 0xFD),
    }


def _dm_motor_models() -> dict[str, str]:
    return {
        "shoulder_pan": "4340P",
        "shoulder_lift": "4340P",
        "elbow_flex": "4340P",
        "wrist_flex": "4310",
        "wrist_yaw": "4310",
        "wrist_roll": "4310",
        "gripper": "4310",
    }


def _rs_motor_models() -> dict[str, str]:
    return {
        "shoulder_pan": "rs-06",
        "shoulder_lift": "rs-06",
        "elbow_flex": "rs-06",
        "wrist_flex": "rs-00",
        "wrist_yaw": "rs-00",
        "wrist_roll": "rs-00",
        "gripper": "rs-00",
    }


def _dm_pos_vel_velocity() -> list[float]:
    return [150, 150, 150, 150, 150, 150, 300]


def _rs_pos_vel_velocity() -> list[float]:
    # RobStride vel_max / limit_spd in rad/s.
    return [1, 0.4, 0.4, 1, 1, 1, 1]


def _dm_mit_gains() -> dict[str, tuple[float, float]]:
    return {
        "shoulder_pan": (120.0, 8.0),
        "shoulder_lift": (120.0, 8.0),
        "elbow_flex": (120.0, 8.0),
        "wrist_flex": (18.0, 2.0),
        "wrist_yaw": (18.0, 2.0),
        "wrist_roll": (18.0, 2.0),
        "gripper": (8.0, 1.0),
    }


def _rs_mit_gains() -> dict[str, tuple[float, float]]:
    return {
        "shoulder_pan": (50.0, 3.0),
        "shoulder_lift": (150.0, 10.0),
        "elbow_flex": (150.0, 10.0),
        "wrist_flex": (50.0, 5.0),
        "wrist_yaw": (50.0, 4.0),
        "wrist_roll": (50.0, 4.0),
        "gripper": (50.0, 4.0),
    }


def _dm_pos_vel_gains() -> dict[str, tuple[float, float, float, float]]:
    return {
        "shoulder_pan": (0.0125, 0.004, 150.0, 0.5),
        "shoulder_lift": (0.013, 0.004, 200.0, 10.0),
        "elbow_flex": (0.013, 0.004, 200.0, 10.0),
        "wrist_flex": (0.0008, 0.002, 50.0, 1.0),
        "wrist_yaw": (0.0008, 0.004, 50.0, 1.0),
        "wrist_roll": (0.0008, 0.002, 50.0, 1.0),
        "gripper": (0.0008, 0.002, 50.0, 1.0),
    }


def _rs_pos_vel_gains() -> dict[str, tuple[float, float, float, float]]:
    # Tuple order: (spd_kp, spd_ki, loc_kp, loc_ki).
    # RobStride does not expose loc_ki in this path, so pos_ki stays 0.0.
    return {
        "shoulder_pan": (12.0, 0.1, 13.0, 0.0),
        "shoulder_lift": (13.5, 0.1, 17.0, 0.0),
        "elbow_flex": (13.5, 0.1, 17.0, 0.0),
        "wrist_flex": (8.0, 0.1, 15.0, 0.0),
        "wrist_yaw": (5.0, 0.1, 18.0, 0.0),
        "wrist_roll": (5.0, 0.1, 10.0, 0.0),
        "gripper": (5.0, 0.1, 10.0, 0.0),
    }


def _dm_return_to_initial_vlim_deg_s() -> dict[str, float]:
    return {
        "shoulder_pan": 15.0,
        "shoulder_lift": 15.0,
        "elbow_flex": 15.0,
        "wrist_flex": 15.0,
        "wrist_yaw": 15.0,
        "wrist_roll": 15.0,
        "gripper": 150.0,
    }


def _rs_return_to_initial_vlim_deg_s() -> dict[str, float]:
    return {
        "shoulder_pan": 15.0,
        "shoulder_lift": 15.0,
        "elbow_flex": 15.0,
        "wrist_flex": 15.0,
        "wrist_yaw": 15.0,
        "wrist_roll": 15.0,
        "gripper": 150.0,
    }


def _dm_joint_limits() -> dict[str, tuple[float, float]]:
    return {
        "shoulder_pan": (-145.0, 145.0),
        "shoulder_lift": (-170.0, 1.0),
        "elbow_flex": (-200.0, 1.0),
        "wrist_flex": (-80.0, 90.0),
        "wrist_yaw": (-90.0, 90.0),
        "wrist_roll": (-90.0, 90.0),
        "gripper": (-270.0, 0.0),
    }


def _rs_joint_limits() -> dict[str, tuple[float, float]]:
    return {
        "shoulder_pan": (-145.0, 145.0),
        "shoulder_lift": (-0.0, 170.0),
        "elbow_flex": (-0.0, 200.0),
        "wrist_flex": (-80.0, 90.0),
        "wrist_yaw": (-90.0, 90.0),
        "wrist_roll": (-90.0, 90.0),
        "gripper": (-0.0, 270.0),
    }


def _identity_joint_directions() -> dict[str, float]:
    return {
        "shoulder_pan": 1.0,
        "shoulder_lift": 1.0,
        "elbow_flex": 1.0,
        "wrist_flex": 1.0,
        "wrist_yaw": 1.0,
        "wrist_roll": 1.0,
        "gripper": 1.0,
    }


def _rs_joint_directions() -> dict[str, float]:
    return {
        "shoulder_pan": -1.0,
        "shoulder_lift": -1.0,
        "elbow_flex": -1.0,
        "wrist_flex": -1.0,
        "wrist_yaw": -1.0,
        "wrist_roll": -1.0,
        "gripper": 6.0,
    }


def _same_mapping(left: dict, right: dict) -> bool:
    if not isinstance(left, dict):
        return False
    return dict(left) == dict(right)


def _same_sequence(left, right) -> bool:
    return isinstance(left, list) and list(left) == list(right)


@RobotConfig.register_subclass("seeed_b601_rt_follower")
@dataclass
class SeeedB601RTFollowerConfig(RobotConfig):
    """Configuration for the Seeed B601 follower driven by rebotarm_control_rt."""

    # Communication channel for rebotarm_control_rt: SocketCAN (can0) or Damiao serial bridge (/dev/ttyACM0).
    port: str = _DEFAULT_PORT
    can_adapter: str = "damiao"

    # Optional prebuilt rebotarm_control_rt actuator config.
    # If omitted, this LeRobot plugin generates one from the B601 mapping below.
    arm_cfg_path: str | Path | None = None
    # URDF used by the Cartesian kinematics model. Relative paths are resolved
    # by rebotarm_control_rt; None uses its built-in default URDF.
    kinematic_urdf_path: str | Path | None = _DEFAULT_KINEMATIC_URDF_PATH
    # Optional URDF metadata written into the generated rebotarm_control_rt YAML.
    rt_urdf_path: str | Path | None = None
    rt_end_effector_frame: str | None = None

    disable_torque_on_disconnect: bool = False
    max_relative_target: float | dict[str, float] | None = None

    # LeRobot-facing action and control modes.
    action_mode: ActionMode = ActionMode.JOINT
    control_mode: ControlMode = ControlMode.POS_VEL

    # LeRobot-facing action mode. "joint" exposes joint targets; "cartesian" exposes TCP targets.
    control_gripper: bool = True
    gripper_type: GripperType = GripperType.SERIAL

    # Serial gripper parameters.
    serial_gripper_sn: str = "000033"
    serial_gripper_port: str = ""
    serial_gripper_baudrate: int = 115200
    serial_gripper_serial_timeout: float = 1.0
    serial_gripper_device_id: int = 1
    serial_gripper_min_pos: float = 0.0
    serial_gripper_max_pos: float = 85.0
    serial_gripper_v_max: float = 80.0
    serial_gripper_f_max: float = 27.0
    serial_gripper_init_open: bool = True
    serial_gripper: SerialGripperConfig | None = field(default=None, init=False)

    # Xense TacCap follower gripper parameters.
    connect_taccap_gripper: bool = True
    gripper_device: str | None = None
    taccap_role: str = "follower"
    taccap_side: str | None = None
    gripper_wrist_video: str = ""
    gripper_baudrate: int = 3_000_000
    gripper_ack_timeout_ms: int = 1000
    gripper_max_retries: int = 2
    gripper_open_cameras: bool = False
    gripper_reload_config_on_connect: bool = True
    enable_gripper_on_connect: bool = True
    disable_gripper_on_disconnect: bool = True
    clear_gripper_fault_on_connect: bool = True
    gripper_kp: float = 15.0
    gripper_kd: float = 1.0
    gripper_feedforward_torque: float = 3.0
    gripper_torque_grasp_enabled: bool = True
    print_gripper_torque: bool = True
    gripper_torque_print_hz: float = 10.0
    normalize_gripper_action: bool = False
    gripper_action_min: float = 0.0
    gripper_action_max: float = 55.0
    taccap_gripper: TacCapGripperConfig | None = field(default=None, init=False)

    # TacCap tactile and wrist camera auto-discovery.
    auto_discover_taccap_cameras: bool = True
    expected_tactiles_per_side: int = 2
    enable_taccap_tactiles: bool = True
    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    tactile_process_backend: bool = True
    enable_taccap_wrist_camera: bool = True
    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    # reBotArm B601 gripper force control parameters.
    enabled_gripper_force: bool = True # Whether to enable gripper force control.
    gripper_force_pos_torque_ratio: float = 0.02 # Ratio of gripper force to position torque[0.018 - 1.0]%.

    # Get observation.
    enable_observation_joint_pos: bool = False
    enable_observation_joint_vel: bool = False
    enable_observation_joint_torque: bool = False
    enable_observation_gripper_vel: bool = False
    enable_observation_gripper_torque: bool = False

    # RT control loop parameters.
    rt_rate: float = 150.0
    rt_command_gap_us: int = 0
    rt_priority: int = 99
    rt_cpu: int | None = 3

    # Debugging parameters.
    damiao_tx_debug: int = 0 # print debug Damiao TX information.
    debug_motion: bool = False # print debug motion information.
    debug_motion_interval_s: float = 1.0 # print debug motion information every interval seconds.

    # Maps LeRobot joint names to (command id, feedback id).
    motor_can_ids: dict[str, tuple[int, int]] = field(default_factory=_dm_motor_can_ids)
    motor_models: dict[str, str] = field(default_factory=_dm_motor_models)
    motor_vendor: str | None = None
    motor_vendors: dict[str, str] = field(default_factory=dict)

    # Position velocity limits. Damiao uses deg/s; RobStride uses rad/s.
    pos_vel_velocity: float | list[float] = field(default_factory=_dm_pos_vel_velocity)
    # MIT gains used by rebotarm_control_rt in ControlMode.MIT.
    # Tuple order: (kp, kd).
    mit_gains: dict[str, tuple[float, float]] = field(default_factory=_dm_mit_gains)
    # Damiao POS_VEL register gains written before starting the RT loop.
    # Tuple order: (vel_kp, vel_ki, pos_kp, pos_ki).
    pos_vel_gains: dict[str, tuple[float, float, float, float]] = field(default_factory=_dm_pos_vel_gains)

    # Per-joint velocity limit (deg/s) used when returning to the initial pose
    # (reset / disconnect). Accepts a scalar (all joints), a list (per motor in
    # the motor_can_ids order), or a dict keyed by joint name. Each value is
    # clamped to <= the corresponding pos_vel_velocity at runtime.
    return_to_initial_vlim_deg_s: float | list[float] | dict[str, float] = field(default_factory=_dm_return_to_initial_vlim_deg_s)

    # Soft limits in degrees, matching the non-RT B601 plugin.
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=_dm_joint_limits)
    # Per-joint direction/scale applied to incoming joint-space actions before clipping.
    # The default is identity to preserve the RT action API; can_adapter=robstride switches
    # this to the RS mapping used by the non-RT/TacCap follower.
    joint_directions: dict[str, float] = field(default_factory=_identity_joint_directions)

    # Cameras for the follower robot.
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            # "head": RealSenseCameraConfig(
            #     serial_number_or_name="021422060263",
            #     fps=30,
            #     width=640,
            #     height=480,
            #     warmup_s=1.0,
            # ),
            # "head": OpenCVCameraConfig(
            #     index_or_path="/dev/video6",
            #     fourcc="YUYV",
            #     fps=60,
            #     width=640,
            #     height=480,
            #     warmup_s=1.0,
            # ),
        }
    )

    def __post_init__(self):
        super().__post_init__()

        self.can_adapter = self.can_adapter.lower()
        if self.can_adapter == "robstride":
            self._apply_robstride_defaults()

        if isinstance(self.gripper_type, str):
            self.gripper_type = GripperType(self.gripper_type)

        if self.gripper_type == GripperType.SERIAL:
            self.serial_gripper = SerialGripperConfig(
                sn=str(self.serial_gripper_sn) if self.serial_gripper_sn else None,
                port=self.serial_gripper_port,
                baudrate=self.serial_gripper_baudrate,
                serial_timeout=self.serial_gripper_serial_timeout,
                device_id=self.serial_gripper_device_id,
                gripper_min_pos=self.serial_gripper_min_pos,
                gripper_max_pos=self.serial_gripper_max_pos,
                gripper_v_max=self.serial_gripper_v_max,
                gripper_f_max=self.serial_gripper_f_max,
                init_open=self.serial_gripper_init_open,
            )
        else:
            self.serial_gripper = None

        if self.gripper_type == GripperType.TACCAP:
            self.taccap_gripper = TacCapGripperConfig(
                device=self.gripper_device,
                role=self.taccap_role,
                side=self.taccap_side,
                wrist_video=self.gripper_wrist_video,
                baudrate=self.gripper_baudrate,
                ack_timeout_ms=self.gripper_ack_timeout_ms,
                max_retries=self.gripper_max_retries,
                open_cameras=self.gripper_open_cameras,
                reload_config_on_connect=self.gripper_reload_config_on_connect,
                enable_on_connect=self.enable_gripper_on_connect,
                disable_on_disconnect=self.disable_gripper_on_disconnect,
                clear_fault_on_connect=self.clear_gripper_fault_on_connect,
                kp=self.gripper_kp,
                kd=self.gripper_kd,
                feedforward_torque=self.gripper_feedforward_torque,
                torque_grasp_enabled=self.gripper_torque_grasp_enabled,
                print_torque=self.print_gripper_torque,
                torque_print_hz=self.gripper_torque_print_hz,
                normalize_action=self.normalize_gripper_action,
                action_min=self.gripper_action_min,
                action_max=self.gripper_action_max,
            )
        else:
            self.taccap_gripper = None

        if self.expected_tactiles_per_side < 0:
            raise ValueError("expected_tactiles_per_side must be >= 0.")
        if self.tactile_fps <= 0 or self.wrist_camera_fps <= 0:
            raise ValueError("TacCap camera FPS values must be positive.")

    def _apply_robstride_defaults(self) -> None:
        if self.port == _DEFAULT_PORT:
            self.port = _ROBSTRIDE_PORT
        if self.kinematic_urdf_path == _DEFAULT_KINEMATIC_URDF_PATH:
            self.kinematic_urdf_path = _ROBSTRIDE_URDF_PATH
        if self.rt_urdf_path is None:
            self.rt_urdf_path = _ROBSTRIDE_URDF_PATH
        if self.rt_end_effector_frame is None:
            self.rt_end_effector_frame = _ROBSTRIDE_END_EFFECTOR_FRAME
        self.disable_torque_on_disconnect = True
        if self.motor_vendor is None and not self.motor_vendors:
            self.motor_vendor = "robstride"
        if _same_mapping(self.motor_can_ids, _dm_motor_can_ids()):
            self.motor_can_ids = _rs_motor_can_ids()
        if _same_mapping(self.motor_models, _dm_motor_models()):
            self.motor_models = _rs_motor_models()
        if _same_sequence(self.pos_vel_velocity, _dm_pos_vel_velocity()):
            self.pos_vel_velocity = _rs_pos_vel_velocity()
        if _same_mapping(self.mit_gains, _dm_mit_gains()):
            self.mit_gains = _rs_mit_gains()
        if _same_mapping(self.pos_vel_gains, _dm_pos_vel_gains()):
            self.pos_vel_gains = _rs_pos_vel_gains()
        if _same_mapping(self.return_to_initial_vlim_deg_s, _dm_return_to_initial_vlim_deg_s()):
            self.return_to_initial_vlim_deg_s = _rs_return_to_initial_vlim_deg_s()
        if _same_mapping(self.joint_limits, _dm_joint_limits()):
            self.joint_limits = _rs_joint_limits()
        if _same_mapping(self.joint_directions, _identity_joint_directions()):
            self.joint_directions = _rs_joint_directions()
