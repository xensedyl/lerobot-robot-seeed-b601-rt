from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.robot import RobotConfig

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

    disable_torque_on_disconnect: bool = False
    max_relative_target: float | dict[str, float] | None = None
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

    # LeRobot-facing action mode. "joint" exposes joint targets; "cartesian"
    # exposes TCP targets and converts them to joint targets internally by IK.
    action_mode: str = "joint"
    control_gripper: bool = True
    control_mode: str = "pos_vel"
    gripper_force_pos_enabled: bool = True
    gripper_force_pos_torque_ratio: float = 0.02 # Ratio of gripper force to position torque[0.018 - 1.0]%.
    rt_rate: float = 150.0
    rt_command_gap_us: int = 0
    rt_priority: int = 99
    rt_cpu: int | None = 3
    damiao_tx_debug: int = 0
    debug_motion: bool = False
    debug_motion_interval_s: float = 1.0
    # Extra fixed transform from the URDF end_link to a custom tool/TCP.
    # The bundled B601 URDF already places end_link at the gripper end frame.
    # Units: meters and degrees, expressed in end_link frame.
    tool_tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tool_tcp_offset_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

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
