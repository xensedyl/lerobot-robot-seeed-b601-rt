from dataclasses import dataclass


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
