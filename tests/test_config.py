from pathlib import Path

from lerobot_robot_seeed_b601_rt import SeeedB601RTFollowerConfig
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
    assert "shoulder_pan.vel" in robot.observation_features

    yaml = robot._render_rt_arm_yaml()
    assert "channel: /dev/ttyACM0" in yaml
    assert "name: shoulder_pan" in yaml
    assert "motor_id: 0x01" in yaml
    assert "vendor: \"damiao\"" in yaml
    assert cfg.rt_rate == 150.0
    assert cfg.rt_command_gap_us == 0
    assert cfg.disable_torque_on_disconnect is False


def test_initial_gripper_pose_is_closed():
    cfg = SeeedB601RTFollowerConfig(port="/dev/ttyACM0")
    robot = SeeedB601RTFollower(cfg)
    positions = {"shoulder_pan": 12.0, "gripper": -250.0}

    robot._apply_initial_gripper_pose(positions)

    assert positions["shoulder_pan"] == 12.0
    assert positions["gripper"] == 0.0
