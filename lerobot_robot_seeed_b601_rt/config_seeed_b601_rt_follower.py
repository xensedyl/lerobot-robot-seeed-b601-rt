from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum
import glob
import os

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.robot import RobotConfig

from .config_serial_gripper import SerialGripperConfig


def resolve_opencv_camera_index_or_path(index_or_path: str | int | Path) -> str | int | Path:
    """Resolve a USB camera serial/name to a stable OpenCV device path when possible."""

    if isinstance(index_or_path, int):
        return index_or_path

    value = str(index_or_path)
    if not value:
        return index_or_path

    if value.isdigit():
        return int(value)

    path = Path(value)
    if path.exists() or value.startswith("/dev/video"):
        return path

    for candidate in sorted(glob.glob(f"/dev/v4l/by-id/*{value}*")):
        if "index0" not in candidate:
            continue
        return Path(os.path.realpath(candidate))

    matches = sorted(glob.glob(f"/dev/v4l/by-id/*{value}*"))
    if matches:
        return Path(os.path.realpath(matches[0]))

    return index_or_path


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

@RobotConfig.register_subclass("seeed_b601_rt_follower")
@dataclass
class SeeedB601RTFollowerConfig(RobotConfig):
    """Configuration for the Seeed B601 follower driven by rebotarm_control_rt."""

    # Communication channel for rebotarm_control_rt: SocketCAN (can0) or Damiao serial bridge (/dev/ttyACM0).
    port: str
    can_adapter: str = "damiao"

    # Optional prebuilt rebotarm_control_rt actuator config.
    # If omitted, this LeRobot plugin generates one from the B601 mapping below.
    arm_cfg_path: str | Path | None = None
    # URDF used by the Cartesian kinematics model. Relative paths are resolved
    # by rebotarm_control_rt; None uses its built-in default URDF.
    kinematic_urdf_path: str | Path | None = "lerobot_robot_seeed_b601_rt/tool_calibration.urdf"

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

    # reBotArm B601 gripper force control parameters.
    enabled_gripper_force: bool = True # Whether to enable gripper force control.
    gripper_force_pos_torque_ratio: float = 0.02 # Ratio of gripper force to position torque[0.018 - 1.0]%.

    # Get observation.
    enable_observation_joint_pos: bool = False
    enable_observation_joint_vel: bool = False
    enable_observation_joint_torque: bool = False
    enable_observation_gripper_vel: bool = False
    enable_observation_gripper_torque: bool = False

    # Optional Xense tactile sensors. This uses the external
    # lerobot_camera_xense package, which targets xensesdk 2.0.0.
    auto_configure_cameras: bool = True
    enable_tactile_sensors: bool = True
    tactile_camera_sn_0: str = "OG001319"
    tactile_camera_sn_1: str = "OG001320"
    # Xense image output for LeRobot camera observations. Supported values:
    # "rectify" and "difference".
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    tactile_fps: int = 30
    tactile_warmup_s: float = 0.05
    tactile_rectify_size: tuple[int, int] | None = None
    tactile_raw_size: tuple[int, int] | None = None
    tactile_disable_infer: bool | None = True
    tactile_process_backend: bool = True

    # Optional wrist RGB camera.
    enable_wrist_cameras: bool = True
    wrist_camera_sn: str = "XC000033"
    wrist_camera_fourcc: str = "MJPG"
    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30
    wrist_camera_warmup_s: float = 1.0

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
    motor_can_ids: dict[str, tuple[int, int]] = field(
        default_factory=lambda: {
            "shoulder_pan": (0x01, 0x11),
            "shoulder_lift": (0x02, 0x12),
            "elbow_flex": (0x03, 0x13),
            "wrist_flex": (0x04, 0x14),
            "wrist_yaw": (0x05, 0x15),
            "wrist_roll": (0x06, 0x16),
            "gripper": (0x07, 0x17),
        }
    )
    motor_models: dict[str, str] = field(
        default_factory=lambda: {
            "shoulder_pan": "4340P",
            "shoulder_lift": "4340P",
            "elbow_flex": "4340P",
            "wrist_flex": "4310",
            "wrist_yaw": "4310",
            "wrist_roll": "4310",
            "gripper": "4310",
        }
    )
    motor_vendor: str | None = None
    motor_vendors: dict[str, str] = field(default_factory=dict)

    # Position velocity limits in degrees/s. These are converted to rad/s for rebotarm_control_rt.
    pos_vel_velocity: float | list[float] = field(
        default_factory=lambda: [150, 150, 150, 150, 150, 150, 300]
    )
    # MIT gains used by rebotarm_control_rt in ControlMode.MIT.
    # Tuple order: (kp, kd).
    mit_gains: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (120.0, 8.0),
            "shoulder_lift": (120.0, 8.0),
            "elbow_flex": (120.0, 8.0),
            "wrist_flex": (18.0, 2.0),
            "wrist_yaw": (18.0, 2.0),
            "wrist_roll": (18.0, 2.0),
            "gripper": (8.0, 1.0),
        }
    )
    # Damiao POS_VEL register gains written before starting the RT loop.
    # Tuple order: (vel_kp, vel_ki, pos_kp, pos_ki).
    pos_vel_gains: dict[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (0.0125, 0.004, 150.0, 0.5),
            "shoulder_lift": (0.013, 0.004, 200.0, 10.0),
            "elbow_flex": (0.013, 0.004, 200.0, 10.0),
            "wrist_flex": (0.0008, 0.002, 50.0, 1.0),
            "wrist_yaw": (0.0008, 0.004, 50.0, 1.0),
            "wrist_roll": (0.0008, 0.002, 50.0, 1.0),
            "gripper": (0.0008, 0.002, 50.0, 1.0),
        }
    )

    # Per-joint velocity limit (deg/s) used when returning to the initial pose
    # (reset / disconnect). Accepts a scalar (all joints), a list (per motor in
    # the motor_can_ids order), or a dict keyed by joint name. Each value is
    # clamped to <= the corresponding pos_vel_velocity at runtime.
    return_to_initial_vlim_deg_s: float | list[float] | dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 15.0,
            "shoulder_lift": 15.0,
            "elbow_flex": 15.0,
            "wrist_flex": 15.0,
            "wrist_yaw": 15.0,
            "wrist_roll": 15.0,
            "gripper": 150.0,
        }
    )

    # Soft limits in degrees, matching the non-RT B601 plugin.
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (-145.0, 145.0),
            "shoulder_lift": (-170.0, 1.0),
            "elbow_flex": (-200.0, 1.0),
            "wrist_flex": (-80.0, 90.0),
            "wrist_yaw": (-90.0, 90.0),
            "wrist_roll": (-90.0, 90.0),
            "gripper": (-270.0, 0.0),
        }
    )

    # Cameras for the follower robot.
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "head": RealSenseCameraConfig(
                serial_number_or_name="021422060263",
                fps=30,
                width=640,
                height=480,
                warmup_s=1.0,
            ),
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
        if self.auto_configure_cameras:
            self._configure_wrist_cameras()
            self._configure_tactile_cameras()
        super().__post_init__()

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

    def _configure_wrist_cameras(self) -> None:
        if not self.enable_wrist_cameras or not self.wrist_camera_sn:
            return

        self.cameras.setdefault(
            "wrist",
            OpenCVCameraConfig(
                index_or_path=resolve_opencv_camera_index_or_path(self.wrist_camera_sn),
                fourcc=self.wrist_camera_fourcc,
                width=self.wrist_camera_width,
                height=self.wrist_camera_height,
                fps=self.wrist_camera_fps,
                warmup_s=self.wrist_camera_warmup_s,
            ),
        )

    def _configure_tactile_cameras(self) -> None:
        if not self.enable_tactile_sensors:
            return

        try:
            from lerobot_camera_xense import XenseTactileCameraConfig
        except ImportError as e:
            raise ImportError(
                "enable_tactile_sensors=True requires the lerobot_camera_xense package. "
                "Install it with: pip install -e ./lerobot_camera_xense"
            ) from e

        tactile_sensors = {
            "tactile_0": self.tactile_camera_sn_0,
            "tactile_1": self.tactile_camera_sn_1,
        }
        for camera_name, serial_number in tactile_sensors.items():
            if not serial_number:
                continue
            self.cameras.setdefault(
                camera_name,
                XenseTactileCameraConfig(
                    serial_number=serial_number,
                    fps=self.tactile_fps,
                    output_types=self.tactile_output_types,
                    warmup_s=self.tactile_warmup_s,
                    rectify_size=self.tactile_rectify_size,
                    raw_size=self.tactile_raw_size,
                    disable_infer=self.tactile_disable_infer,
                    process_backend=self.tactile_process_backend,
                ),
            )
