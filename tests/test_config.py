import math
from pathlib import Path

import pytest

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
