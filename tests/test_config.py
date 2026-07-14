import math
from pathlib import Path

import pytest

from lerobot_robot_seeed_b601_rt import (
    BiSeeedB601RTFollowerConfig,
    GripperType,
    SeeedB601RTFollowerConfig,
    TacCapGripper,
    TacCapGripperConfig,
)
from lerobot_robot_seeed_b601_rt.bi_seeed_b601_rt_follower import BiSeeedB601RTFollower
from lerobot_robot_seeed_b601_rt.seeed_b601_rt_follower import SeeedB601RTFollower
from lerobot.robots.utils import make_robot_from_config


def test_config_registers_and_generates_rt_yaml():
    cfg = SeeedB601RTFollowerConfig(
        port="/dev/ttyACM0",
        id="test",
        calibration_dir=Path("/tmp/lerobot-b601-rt-test"),
    )

    robot = make_robot_from_config(cfg)

    assert robot.name == "seeed_b601_rt_follower"
    assert "shoulder_pan.pos" in robot.action_features
    assert "gripper.pos" in robot.observation_features

    yaml = robot._render_rt_arm_yaml()
    assert "channel: /dev/ttyACM0" in yaml
    assert "name: shoulder_pan" in yaml
    assert "motor_id: 0x01" in yaml
    assert "vendor: \"damiao\"" in yaml
    assert "vel_kp: 0.0125" in yaml
    assert "vel_ki: 0.004" in yaml
    assert "pos_kp: 150.0" in yaml
    assert "pos_ki: 0.5" in yaml
    assert "name: shoulder_lift" in yaml
    assert "pos_kp: 200.0" in yaml
    assert "pos_ki: 10.0" in yaml
    assert cfg.rt_rate == 150.0
    assert cfg.rt_command_gap_us == 0
    assert cfg.disable_torque_on_disconnect is False


def test_robstride_adapter_switches_hardware_defaults():
    cfg = SeeedB601RTFollowerConfig(
        can_adapter="robstride",
        id="rs_test",
        calibration_dir=Path("/tmp/lerobot-b601-rt-rs-test"),
    )
    robot = make_robot_from_config(cfg)

    assert robot.name == "seeed_b601_rt_follower"
    assert cfg.port == "can0"
    assert cfg.motor_vendor == "robstride"
    assert cfg.motor_can_ids["shoulder_pan"] == (0x01, 0xFD)
    assert cfg.motor_models["shoulder_pan"] == "rs-06"
    assert cfg.motor_models["wrist_roll"] == "rs-00"
    assert cfg.kinematic_urdf_path == "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
    assert cfg.rt_urdf_path == "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
    assert cfg.rt_end_effector_frame == "gripper_end"
    assert cfg.disable_torque_on_disconnect is True
    assert cfg.mit_gains["shoulder_lift"] == (150.0, 10.0)
    assert cfg.pos_vel_gains["shoulder_lift"] == (13.5, 0.1, 17.0, 0.0)
    assert cfg.joint_limits["shoulder_lift"] == (-0.0, 170.0)
    assert cfg.pos_vel_velocity == pytest.approx([1, 0.4, 0.4, 1, 1, 1, 1])
    assert cfg.joint_directions["shoulder_pan"] == -1.0
    assert cfg.joint_directions["shoulder_lift"] == -1.0
    assert cfg.joint_directions["elbow_flex"] == -1.0
    assert cfg.joint_directions["wrist_yaw"] == -1.0
    assert cfg.joint_directions["wrist_roll"] == -1.0

    yaml = robot._render_rt_arm_yaml()
    assert "channel: can0" in yaml
    assert "urdf_path: \"urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf\"" in yaml
    assert "end_effector_frame: \"gripper_end\"" in yaml
    assert "feedback_id: 0xFD" in yaml
    assert "model: \"rs-06\"" in yaml
    assert "vendor: \"robstride\"" in yaml
    assert "vel_kp: 13.5" in yaml
    assert "pos_kp: 17.0" in yaml
    assert "vlim: 0.4" in yaml
    assert "pos_ki: 0.0" in yaml

    explicit_default_port_cfg = SeeedB601RTFollowerConfig(
        can_adapter="robstride",
        port="/dev/ttyACM0",
    )
    assert explicit_default_port_cfg.port == "can0"


def test_robstride_adapter_preserves_explicit_overrides():
    cfg = SeeedB601RTFollowerConfig(
        can_adapter="robstride",
        port="can1",
        motor_models={"shoulder_pan": "custom"},
    )

    assert cfg.port == "can1"
    assert cfg.motor_models == {"shoulder_pan": "custom"}


def test_robstride_mit_mode_matches_reference_taccap_gains():
    cfg = SeeedB601RTFollowerConfig(
        can_adapter="robstride",
        control_mode="mit",
        gripper_type=GripperType.TACCAP,
        auto_discover_taccap_cameras=False,
        cameras={},
    )
    robot = SeeedB601RTFollower(cfg)

    assert cfg.control_mode == "mit"
    assert robot._mit_gains_lists() == (
        [50.0, 150.0, 150.0, 50.0, 50.0, 50.0],
        [3.0, 10.0, 10.0, 5.0, 4.0, 4.0],
    )


def test_joint_action_applies_follower_joint_directions_before_clipping():
    cfg = SeeedB601RTFollowerConfig(
        can_adapter="robstride",
        control_gripper=False,
        cameras={},
    )
    robot = SeeedB601RTFollower(cfg)
    robot._last_goal_deg = {motor: 0.0 for motor in robot.motor_names}
    robot._last_positions_deg = dict(robot._last_goal_deg)

    goal = robot._complete_goal(
        {
            "shoulder_lift.pos": 12.0,
            "elbow_flex.pos": -34.0,
            "wrist_yaw.pos": -9.0,
            "wrist_roll.pos": 7.0,
        },
        apply_joint_directions=True,
    )

    assert goal["shoulder_lift"] == -12.0
    assert goal["elbow_flex"] == 34.0
    assert goal["wrist_yaw"] == 9.0
    assert goal["wrist_roll"] == -7.0


def test_bi_config_registers_and_prefixes_features():
    cfg = BiSeeedB601RTFollowerConfig(
        id="bi_test",
        calibration_dir=Path("/tmp/lerobot-bi-b601-rt-test"),
        action_mode="cartesian",
        cameras={},
        right_pos_vel_gains={
            "shoulder_pan": (0.02, 0.005, 180.0, 1.0),
            "shoulder_lift": (0.02, 0.005, 180.0, 1.0),
            "elbow_flex": (0.02, 0.005, 180.0, 1.0),
            "wrist_flex": (0.001, 0.003, 60.0, 1.0),
            "wrist_yaw": (0.001, 0.003, 60.0, 1.0),
            "wrist_roll": (0.001, 0.003, 60.0, 1.0),
            "gripper": (0.001, 0.003, 60.0, 1.0),
        },
    )
    robot = make_robot_from_config(cfg)

    assert isinstance(robot, BiSeeedB601RTFollower)
    assert robot.name == "bi_seeed_b601_rt_follower"
    assert cfg.left_rt_cpu == 3
    assert cfg.right_rt_cpu == 4
    assert robot.left.config.pos_vel_gains["shoulder_lift"] == (0.013, 0.004, 200.0, 10.0)
    assert robot.right.config.pos_vel_gains["shoulder_lift"] == (0.02, 0.005, 180.0, 1.0)
    assert "left_tcp.x" in robot.action_features
    assert "right_tcp.x" in robot.action_features
    assert "left_gripper.pos" in robot.action_features
    assert "right_gripper.pos" in robot.action_features
    assert "left_tcp.x" in robot.observation_features
    assert "right_tcp.x" in robot.observation_features
    assert "left_gripper.pos" in robot.observation_features
    assert "right_gripper.pos" in robot.observation_features


def test_bi_config_allows_mixed_robstride_and_damiao_adapters():
    cfg = BiSeeedB601RTFollowerConfig(
        id="bi_mixed_test",
        calibration_dir=Path("/tmp/lerobot-bi-b601-rt-mixed-test"),
        left_port="can0",
        right_port="/dev/ttyACM4",
        left_can_adapter="robstride",
        right_can_adapter="damiao",
        control_gripper=False,
        cameras={},
    )
    robot = make_robot_from_config(cfg)

    assert cfg.can_adapter == "damiao"
    assert cfg.left_can_adapter == "robstride"
    assert cfg.right_can_adapter == "damiao"
    assert robot.left.config.port == "can0"
    assert robot.left.config.can_adapter == "robstride"
    assert robot.left.config.motor_vendor == "robstride"
    assert robot.left.config.motor_can_ids["shoulder_pan"] == (0x01, 0xFD)
    assert robot.left.config.kinematic_urdf_path == "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
    assert robot.left.config.disable_torque_on_disconnect is True
    assert robot.left.config.joint_directions["shoulder_pan"] == -1.0
    assert robot.left.config.control_gripper is False
    assert robot.right.config.port == "/dev/ttyACM4"
    assert robot.right.config.can_adapter == "damiao"
    assert robot.right.config.motor_vendor is None
    assert robot.right.config.motor_can_ids["shoulder_pan"] == (0x01, 0x11)
    assert robot.right.config.kinematic_urdf_path == "lerobot_robot_seeed_b601_rt/tool_calibration.urdf"
    assert robot.right.config.disable_torque_on_disconnect is False
    assert robot.right.config.control_gripper is False


def test_observation_joint_features_are_independently_configurable():
    cfg = SeeedB601RTFollowerConfig(
        port="/dev/ttyACM0",
        enable_observation_joint_pos=True,
        enable_observation_joint_vel=False,
        enable_observation_joint_torque=False,
    )
    robot = SeeedB601RTFollower(cfg)

    assert "shoulder_pan.pos" in robot.action_features
    assert "joint_1.pos" in robot.observation_features
    assert "joint_1.vel" not in robot.observation_features
    assert "joint_1.torque" not in robot.observation_features
    assert "shoulder_pan.pos" not in robot.observation_features
    assert "gripper.pos" in robot.observation_features
    assert "gripper.vel" not in robot.observation_features
    assert "gripper.torque" not in robot.observation_features
    assert "joint_7.pos" not in robot.observation_features


def test_gripper_observation_is_controlled_separately_from_joint_observation():
    cfg = SeeedB601RTFollowerConfig(
        port="/dev/ttyACM0",
        gripper_type=GripperType.REBOTARMB601,
        enable_observation_joint_pos=False,
        enable_observation_joint_vel=False,
        enable_observation_joint_torque=False,
        enable_observation_gripper_vel=True,
        enable_observation_gripper_torque=True,
    )
    robot = SeeedB601RTFollower(cfg)

    assert "joint_1.pos" not in robot.observation_features
    assert "joint_1.vel" not in robot.observation_features
    assert "joint_1.torque" not in robot.observation_features
    assert "gripper.pos" in robot.observation_features
    assert "gripper.vel" in robot.observation_features
    assert "gripper.torque" in robot.observation_features


def test_control_gripper_controls_gripper_action_and_observation():
    cfg = SeeedB601RTFollowerConfig(port="/dev/ttyACM0", control_gripper=False)
    robot = SeeedB601RTFollower(cfg)

    assert "gripper.pos" not in robot.action_features
    assert "gripper.pos" not in robot.observation_features
    assert "gripper.vel" not in robot.observation_features
    assert "gripper.torque" not in robot.observation_features
    assert len(robot._velocity_limits_rad()) == 6


def test_initial_gripper_pose_is_closed():
    cfg = SeeedB601RTFollowerConfig(port="/dev/ttyACM0")
    robot = SeeedB601RTFollower(cfg)
    positions = {"shoulder_pan": 12.0, "gripper": -250.0}

    robot._apply_initial_gripper_pose(positions)

    assert positions["shoulder_pan"] == 12.0
    assert positions["gripper"] == 0.0


def test_gripper_observation_position_is_normalized():
    robot = SeeedB601RTFollower(SeeedB601RTFollowerConfig(port="/dev/ttyACM0"))

    assert robot._gripper_pos_to_norm(-270.0) == pytest.approx(0.0)
    assert robot._gripper_pos_to_norm(-135.0) == pytest.approx(0.5)
    assert robot._gripper_pos_to_norm(0.0) == pytest.approx(1.0)
    assert robot._gripper_pos_to_norm(-300.0) == pytest.approx(0.0)
    assert robot._gripper_pos_to_norm(30.0) == pytest.approx(1.0)


def test_taccap_gripper_type_exposes_external_gripper_features():
    cfg = SeeedB601RTFollowerConfig(
        gripper_type=GripperType.TACCAP,
        auto_discover_taccap_cameras=False,
        enable_observation_gripper_vel=True,
        enable_observation_gripper_torque=True,
        cameras={},
    )
    robot = SeeedB601RTFollower(cfg)

    assert cfg.taccap_gripper is not None
    assert len(robot.motor_names) == 6
    assert "gripper.pos" in robot.action_features
    assert "gripper.pos" in robot.observation_features
    assert "gripper.vel" in robot.observation_features
    assert "gripper.torque" in robot.observation_features
    assert "gripper.target_torque" in robot.observation_features
    assert "gripper.control_mode" in robot.observation_features
    assert "gripper.target" in robot.observation_features


def test_taccap_gripper_action_mapping_and_torque_grasp():
    class FakeMotor:
        def __init__(self):
            self.command = None

        def submit_impedance(self, position, kp, kd, torque):
            self.command = (position, kp, kd, torque)

    class FakeGripper:
        def __init__(self):
            self.motor = FakeMotor()

        @staticmethod
        def pos_to_rad(position):
            return position * 2.0

    controller = TacCapGripper(
        TacCapGripperConfig(
            normalize_action=True,
            action_min=0.0,
            action_max=55.0,
            feedforward_torque=3.0,
        )
    )
    controller.gripper = FakeGripper()
    controller.last_position = 0.8

    sent_action = controller.send_position(0.0)

    assert sent_action == pytest.approx(44.0)
    assert controller.last_target == pytest.approx(0.8)
    assert controller.last_command_mode == "torque_grasp"
    assert controller.gripper.motor.command == pytest.approx((1.6, 0.0, 0.0, 3.0))
    assert controller.normalize_action_position(27.5) == pytest.approx(0.5)
    assert controller.action_position_from_normalized(0.5) == pytest.approx(27.5)


def test_taccap_default_mapping_matches_rt_gripper_convention():
    controller = TacCapGripper(TacCapGripperConfig())

    assert controller.normalize_action_position(0.0) == pytest.approx(1.0)
    assert controller.normalize_action_position(1.0) == pytest.approx(0.0)
    assert controller.action_position_from_normalized(1.0) == pytest.approx(0.0)
    assert controller.action_position_from_normalized(0.0) == pytest.approx(1.0)

    controller.last_position = 0.25
    assert controller.observation_position == pytest.approx(0.75)


def test_dual_taccap_config_passes_side_specific_settings_to_children():
    cfg = BiSeeedB601RTFollowerConfig(
        left_gripper_type=GripperType.TACCAP,
        right_gripper_type=GripperType.TACCAP,
        auto_discover_taccap_cameras=False,
        left_gripper_device="/dev/left-taccap",
        right_gripper_device="/dev/right-taccap",
        left_gripper_feedforward_torque=2.0,
        right_gripper_feedforward_torque=3.0,
        cameras={},
    )
    robot = BiSeeedB601RTFollower(cfg)

    assert robot.left.config.taccap_side == "left"
    assert robot.right.config.taccap_side == "right"
    assert robot.left.config.gripper_device == "/dev/left-taccap"
    assert robot.right.config.gripper_device == "/dev/right-taccap"
    assert robot.left.config.gripper_feedforward_torque == pytest.approx(2.0)
    assert robot.right.config.gripper_feedforward_torque == pytest.approx(3.0)
    assert "left_gripper.pos" in robot.action_features
    assert "right_gripper.pos" in robot.action_features


def test_return_to_initial_vlim_supports_dict_scalar_and_list():
    dict_robot = SeeedB601RTFollower(
        SeeedB601RTFollowerConfig(
            port="/dev/ttyACM0",
            gripper_type=GripperType.REBOTARMB601,
        )
    )
    assert dict_robot._return_to_initial_vlim_rad()[0] == pytest.approx(math.radians(15.0))
    assert dict_robot._return_to_initial_vlim_rad()[-1] == pytest.approx(math.radians(150.0))

    scalar_robot = SeeedB601RTFollower(
        SeeedB601RTFollowerConfig(port="/dev/ttyACM0", return_to_initial_vlim_deg_s=20.0)
    )
    assert all(v == pytest.approx(math.radians(20.0)) for v in scalar_robot._return_to_initial_vlim_rad())

    list_robot = SeeedB601RTFollower(
        SeeedB601RTFollowerConfig(
            port="/dev/ttyACM0",
            return_to_initial_vlim_deg_s=[10, 20, 30, 40, 50, 60, 70],
        )
    )
    assert list_robot._return_to_initial_vlim_rad()[2] == pytest.approx(math.radians(30.0))


def test_return_to_initial_vlim_clamps_to_pos_vel_velocity():
    cfg = SeeedB601RTFollowerConfig(
        port="/dev/ttyACM0",
        pos_vel_velocity=[5, 5, 5, 5, 5, 5, 5],
        return_to_initial_vlim_deg_s=20.0,
    )
    robot = SeeedB601RTFollower(cfg)

    assert all(v == pytest.approx(math.radians(5.0)) for v in robot._return_to_initial_vlim_rad())


def test_return_to_initial_vlim_rejects_invalid_values():
    with pytest.raises(ValueError, match="missing key"):
        SeeedB601RTFollower(
            SeeedB601RTFollowerConfig(
                port="/dev/ttyACM0",
                return_to_initial_vlim_deg_s={"shoulder_pan": 15.0},
            )
        )

    with pytest.raises(ValueError, match="values must be > 0"):
        SeeedB601RTFollower(
            SeeedB601RTFollowerConfig(port="/dev/ttyACM0", return_to_initial_vlim_deg_s=0.0)
        )

    with pytest.raises(ValueError, match="list length"):
        SeeedB601RTFollower(
            SeeedB601RTFollowerConfig(port="/dev/ttyACM0", return_to_initial_vlim_deg_s=[15.0])
        )


def test_pos_vel_gains_are_required_and_validated():
    with pytest.raises(ValueError, match="pos_vel_gains missing keys"):
        SeeedB601RTFollower(
            SeeedB601RTFollowerConfig(
                port="/dev/ttyACM0",
                pos_vel_gains={"shoulder_pan": (0.0125, 0.004, 150.0, 0.5)},
            )
        )

    bad_tuple_cfg = SeeedB601RTFollowerConfig(port="/dev/ttyACM0")
    bad_tuple_cfg.pos_vel_gains["shoulder_pan"] = (0.0125, 0.004, 150.0)
    with pytest.raises(ValueError, match="must contain"):
        SeeedB601RTFollower(bad_tuple_cfg)

    bad_value_cfg = SeeedB601RTFollowerConfig(port="/dev/ttyACM0")
    bad_value_cfg.pos_vel_gains["shoulder_pan"] = (-1.0, 0.004, 150.0, 0.5)
    with pytest.raises(ValueError, match="values must be >= 0"):
        SeeedB601RTFollower(bad_value_cfg)
