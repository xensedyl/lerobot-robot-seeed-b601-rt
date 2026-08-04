"""Configurations for the shared TacCap gripper controller and Robot."""

from dataclasses import dataclass

from lerobot.robots.robot import RobotConfig


@dataclass
class TacCapGripperConfig:
    """Connection and impedance-control settings for a TacCap follower gripper."""

    device: str | None = None
    role: str = "follower"
    side: str | None = None
    wrist_video: str = ""
    baudrate: int = 3_000_000
    ack_timeout_ms: int = 1000
    max_retries: int = 2
    open_cameras: bool = False
    reload_config_on_connect: bool = True

    enable_on_connect: bool = True
    disable_on_disconnect: bool = True
    clear_fault_on_connect: bool = True

    kp: float = 15.0
    kd: float = 1.0
    feedforward_torque: float = 3.0
    torque_grasp_enabled: bool = True

    print_torque: bool = True
    torque_print_hz: float = 10.0

    # True keeps the RT convention (0=open, 1=closed). False keeps the
    # TacCap SDK convention (0=closed, 1=open).
    invert_position: bool = True

    normalize_action: bool = False
    action_min: float = 0.0
    action_max: float = 55.0

    def __post_init__(self) -> None:
        if self.baudrate <= 0:
            raise ValueError(f"baudrate must be positive, got {self.baudrate}.")
        if self.ack_timeout_ms <= 0:
            raise ValueError(
                f"ack_timeout_ms must be positive, got {self.ack_timeout_ms}."
            )
        if self.max_retries < 0:
            raise ValueError(f"max_retries must be >= 0, got {self.max_retries}.")
        if self.torque_print_hz < 0:
            raise ValueError(
                f"torque_print_hz must be >= 0, got {self.torque_print_hz}."
            )
        if self.normalize_action and self.action_max == self.action_min:
            raise ValueError("action_max must differ from action_min.")
        if self.side is not None:
            self.side = self.side.strip().lower()
            if self.side not in {"left", "right"}:
                raise ValueError("side must be 'left', 'right', or None.")


@RobotConfig.register_subclass("taccap_gripper_follower")
@dataclass
class TacCapGripperFollowerConfig(RobotConfig):
    """Standalone Xense TacCap follower gripper for teleoperation and debugging."""

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

    normalize_gripper_action: bool = True
    gripper_action_min: float = 0.0
    gripper_action_max: float = 55.0

    def __post_init__(self) -> None:
        super().__post_init__()
        self._controller_config()

    def _controller_config(self) -> TacCapGripperConfig:
        return TacCapGripperConfig(
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
            invert_position=False,
        )
