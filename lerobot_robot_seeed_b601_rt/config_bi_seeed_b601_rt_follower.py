from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.robot import RobotConfig

from .config_serial_gripper import SerialGripperConfig
from .config_seeed_b601_rt_follower import ActionMode, ControlMode, GripperType


@RobotConfig.register_subclass("bi_seeed_b601_rt_follower")
@dataclass
class BiSeeedB601RTFollowerConfig(RobotConfig):
    """Configuration for two Seeed B601 followers driven by rebotarm_control_rt."""

    left_port: str = "/dev/ttyACM0"
    right_port: str = "/dev/ttyACM1"
    can_adapter: str = "damiao"

    left_id: str = "left_follower"
    right_id: str = "right_follower"
    left_arm_cfg_path: str | Path | None = None
    right_arm_cfg_path: str | Path | None = None
    # URDFs used by the Cartesian kinematics models. Relative paths are
    # resolved by rebotarm_control_rt; None uses its built-in default URDF.
    left_kinematic_urdf_path: str | Path | None = "lerobot_robot_seeed_b601_rt/tool_calibration.urdf"
    right_kinematic_urdf_path: str | Path | None = "lerobot_robot_seeed_b601_rt/tool_calibration.urdf"

    disable_torque_on_disconnect: bool = False
    max_relative_target: float | dict[str, float] | None = None

    action_mode: ActionMode = ActionMode.JOINT
    control_mode: ControlMode = ControlMode.POS_VEL # MIT or POS_VEL

    control_gripper: bool = True
    left_gripper_type: GripperType = GripperType.SERIAL
    right_gripper_type: GripperType = GripperType.SERIAL
    left_serial_gripper_sn: str = "000033"
    left_serial_gripper_port: str = ""
    left_serial_gripper_baudrate: int = 115200
    left_serial_gripper_serial_timeout: float = 1.0
    left_serial_gripper_device_id: int = 1
    left_serial_gripper_min_pos: float = 0.0
    left_serial_gripper_max_pos: float = 85.0
    left_serial_gripper_v_max: float = 80.0
    left_serial_gripper_f_max: float = 27.0
    left_serial_gripper_init_open: bool = True
    right_serial_gripper_sn: str = "000034"
    right_serial_gripper_port: str = ""
    right_serial_gripper_baudrate: int = 115200
    right_serial_gripper_serial_timeout: float = 1.0
    right_serial_gripper_device_id: int = 1
    right_serial_gripper_min_pos: float = 0.0
    right_serial_gripper_max_pos: float = 85.0
    right_serial_gripper_v_max: float = 80.0
    right_serial_gripper_f_max: float = 27.0
    right_serial_gripper_init_open: bool = True
    left_serial_gripper: SerialGripperConfig | None = field(default=None, init=False)
    right_serial_gripper: SerialGripperConfig | None = field(default=None, init=False)
    enabled_gripper_force: bool = True
    gripper_force_pos_torque_ratio: float = 0.02
    enable_observation_joint_pos: bool = False
    enable_observation_joint_vel: bool = False
    enable_observation_joint_torque: bool = False
    enable_observation_gripper_vel: bool = False
    enable_observation_gripper_torque: bool = False

    rt_rate: float = 100.0
    rt_command_gap_us: int = 0
    left_rt_priority: int = 99
    right_rt_priority: int = 99
    left_rt_cpu: int | None = 3
    right_rt_cpu: int | None = 3

    damiao_tx_debug: int = 0
    debug_motion: bool = False
    debug_motion_interval_s: float = 1.0

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

    pos_vel_velocity: float | list[float] = field(
        default_factory=lambda: [150, 150, 150, 150, 150, 150, 300]
    )
    # MIT gains used by rebotarm_control_rt in ControlMode.MIT.
    # Tuple order: (kp, kd).
    left_mit_gains: dict[str, tuple[float, float]] = field(
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
    right_mit_gains: dict[str, tuple[float, float]] = field(
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
    # Damiao POS_VEL register gains written before starting RT loops.
    # Tuple order: (vel_kp, vel_ki, pos_kp, pos_ki).
    left_pos_vel_gains: dict[str, tuple[float, float, float, float]] = field(
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
    right_pos_vel_gains: dict[str, tuple[float, float, float, float]] = field(
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

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            # "head": RealSenseCameraConfig(
            #     serial_number_or_name="021422060263",
            #     fps=30,
            #     width=640,
            #     height=480,
            #     warmup_s=1.0,
            # ),
        }
    )

    def __post_init__(self):
        super().__post_init__()

        if isinstance(self.left_gripper_type, str):
            self.left_gripper_type = GripperType(self.left_gripper_type)
        if isinstance(self.right_gripper_type, str):
            self.right_gripper_type = GripperType(self.right_gripper_type)

        if self.left_gripper_type == GripperType.SERIAL:
            self.left_serial_gripper = SerialGripperConfig(
                sn=str(self.left_serial_gripper_sn) if self.left_serial_gripper_sn else None,
                port=self.left_serial_gripper_port,
                baudrate=self.left_serial_gripper_baudrate,
                serial_timeout=self.left_serial_gripper_serial_timeout,
                device_id=self.left_serial_gripper_device_id,
                gripper_min_pos=self.left_serial_gripper_min_pos,
                gripper_max_pos=self.left_serial_gripper_max_pos,
                gripper_v_max=self.left_serial_gripper_v_max,
                gripper_f_max=self.left_serial_gripper_f_max,
                init_open=self.left_serial_gripper_init_open,
            )
        else:
            self.left_serial_gripper = None

        if self.right_gripper_type == GripperType.SERIAL:
            self.right_serial_gripper = SerialGripperConfig(
                sn=str(self.right_serial_gripper_sn) if self.right_serial_gripper_sn else None,
                port=self.right_serial_gripper_port,
                baudrate=self.right_serial_gripper_baudrate,
                serial_timeout=self.right_serial_gripper_serial_timeout,
                device_id=self.right_serial_gripper_device_id,
                gripper_min_pos=self.right_serial_gripper_min_pos,
                gripper_max_pos=self.right_serial_gripper_max_pos,
                gripper_v_max=self.right_serial_gripper_v_max,
                gripper_f_max=self.right_serial_gripper_f_max,
                init_open=self.right_serial_gripper_init_open,
            )
        else:
            self.right_serial_gripper = None
