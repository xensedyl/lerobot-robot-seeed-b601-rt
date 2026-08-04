from dataclasses import dataclass

from lerobot.robots.robot import RobotConfig

from .config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig


@RobotConfig.register_subclass("bi_seeed_b601_rs_taccap_gripper_follower")
@dataclass
class BiSeeedB601RSTacCapGripperFollowerConfig(SeeedB601RSFollowerConfig):
    """Configuration for two RS B601 arms with two TacCap follower grippers."""

    # Unused by the dual wrapper. Keep a default so users only need left/right
    # ports at the CLI, while shared single-arm fields are still inherited.
    port: str = ""
    action_mode: str = "joint"
    gripper_type: str = "taccap"

    left_port: str = "can0"
    right_port: str = "can1"
    left_id: str = "left_follower"
    right_id: str = "right_follower"

    # None means fall back to the shared can_adapter field.
    left_can_adapter: str | None = None
    right_can_adapter: str | None = None

    # Dual mode should pin the TacCap side for each child robot. If a specific
    # MCU path is needed, set the corresponding left/right gripper_device.
    left_taccap_side: str | None = "left"
    right_taccap_side: str | None = "right"
    left_gripper_device: str | None = None
    right_gripper_device: str | None = None
    # None means use the shared gripper_feedforward_torque for that side.
    left_gripper_feedforward_torque: float | None = 2.0
    right_gripper_feedforward_torque: float | None = 3.0

    def __post_init__(self):
        super().__post_init__()

        if not self.left_port or not self.right_port:
            raise ValueError("left_port and right_port must both be set.")

        if self.gripper_device is not None:
            raise ValueError(
                "Dual TacCap mode does not use gripper_device; "
                "set left_gripper_device and right_gripper_device instead."
            )
        if self.taccap_side is not None:
            raise ValueError(
                "Dual TacCap mode does not use taccap_side; "
                "set left_taccap_side and right_taccap_side instead."
            )

        self.can_adapter = self.can_adapter.lower()
        self.left_can_adapter = (self.left_can_adapter or self.can_adapter).lower()
        self.right_can_adapter = (self.right_can_adapter or self.can_adapter).lower()
        allowed_adapters = {"damiao", "socketcan", "robstride"}
        for field_name in ("can_adapter", "left_can_adapter", "right_can_adapter"):
            value = getattr(self, field_name)
            if value not in allowed_adapters:
                raise ValueError(
                    f"{field_name} must be one of {sorted(allowed_adapters)}, got {value!r}."
                )

        for field_name in ("left_taccap_side", "right_taccap_side"):
            value = getattr(self, field_name)
            if value is None:
                continue
            value = value.strip().lower()
            if value not in {"left", "right"}:
                raise ValueError(f"{field_name} must be 'left', 'right', or None.")
            setattr(self, field_name, value)

        if (
            self.left_taccap_side is not None
            and self.right_taccap_side is not None
            and self.left_taccap_side == self.right_taccap_side
        ):
            raise ValueError("left_taccap_side and right_taccap_side must be different.")

        if (
            self.left_gripper_device
            and self.right_gripper_device
            and self.left_gripper_device == self.right_gripper_device
        ):
            raise ValueError("left_gripper_device and right_gripper_device must be different.")

        for field_name in ("left_gripper_feedforward_torque", "right_gripper_feedforward_torque"):
            value = getattr(self, field_name)
            if value is not None and value < 0:
                raise ValueError(f"{field_name} must be >= 0, got {value}.")
