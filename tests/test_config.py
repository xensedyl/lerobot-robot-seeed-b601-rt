import math
from pathlib import Path

import pytest

from lerobot_robot_seeed_b601_rt import BiSeeedB601RTFollowerConfig, SeeedB601RTFollowerConfig
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


def test_return_to_initial_vlim_supports_dict_scalar_and_list():
    dict_robot = SeeedB601RTFollower(SeeedB601RTFollowerConfig(port="/dev/ttyACM0"))
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
