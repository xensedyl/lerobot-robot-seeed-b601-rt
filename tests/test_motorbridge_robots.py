import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from lerobot.robots.robot import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot_robot_seeed_b601_rt import (
    BiSeeedB601RSTacCapGripperFollowerConfig,
    SeeedB601DMFollowerConfig,
    SeeedB601RSFollowerConfig,
    SeeedB601RTFollower,
    TacCapGripperFollowerConfig,
)
from lerobot_robot_seeed_b601_rt.motorbridge.seeed_b601_follower import (
    SeeedB601FollowerBase as MotorbridgeFollowerBase,
)
from lerobot_robot_seeed_b601_rt.motorbridge.seeed_b601_rs_follower import (
    KINEMATIC_MOTORS,
    TCP_POSE_KEYS,
)


class MotorbridgeRobotIntegrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmpdir = tempfile.TemporaryDirectory(prefix="b601-motorbridge-test-")
        self.calibration_dir = Path(self._tmpdir.name)

    def tearDown(self) -> None:
        self._tmpdir.cleanup()

    def _kwargs(self, robot_id: str) -> dict:
        return {
            "id": robot_id,
            "calibration_dir": self.calibration_dir,
            "cameras": {},
        }

    def test_dm_and_rs_keep_the_independent_motorbridge_backend(self) -> None:
        dm = make_robot_from_config(
            SeeedB601DMFollowerConfig(port="/dev/ttyACM0", **self._kwargs("dm"))
        )
        rs = make_robot_from_config(
            SeeedB601RSFollowerConfig(port="can0", **self._kwargs("rs"))
        )

        self.assertIsInstance(dm, MotorbridgeFollowerBase)
        self.assertIsInstance(rs, MotorbridgeFollowerBase)
        for robot in (dm, rs):
            self.assertNotIsInstance(robot, SeeedB601RTFollower)
            self.assertEqual(len(robot.action_features), 7)
            self.assertIn("gripper.pos", robot.action_features)

    def test_rs_no_gripper_uses_the_original_motorbridge_base(self) -> None:
        robot = make_robot_from_config(
            SeeedB601RSFollowerConfig(
                port="can0",
                gripper_type="none",
                **self._kwargs("rs_no_gripper"),
            )
        )

        self.assertIsInstance(robot, MotorbridgeFollowerBase)
        self.assertNotIsInstance(robot, SeeedB601RTFollower)
        self.assertEqual(len(robot.action_features), 6)
        self.assertNotIn("gripper.pos", robot.action_features)

    def test_rs_lifecycle_defaults_follow_selected_mode(self) -> None:
        motor = SeeedB601RSFollowerConfig(
            port="can0",
            **self._kwargs("rs_lifecycle_motor"),
        )
        pico = SeeedB601RSFollowerConfig(
            port="can0",
            action_mode="cartesian",
            gripper_type="none",
            **self._kwargs("rs_lifecycle_pico"),
        )
        taccap = SeeedB601RSFollowerConfig(
            port="can0",
            gripper_type="taccap",
            auto_discover_taccap_cameras=False,
            **self._kwargs("rs_lifecycle_taccap"),
        )

        self.assertTrue(motor.return_to_initial_on_connect)
        self.assertFalse(motor.startup_sync_to_action_on_connect)
        self.assertFalse(pico.return_to_initial_on_connect)
        self.assertFalse(pico.startup_sync_to_action_on_connect)
        self.assertFalse(taccap.return_to_initial_on_connect)
        self.assertTrue(taccap.startup_sync_to_action_on_connect)

    def test_rs_startup_sync_limits_joint_targets(self) -> None:
        robot = make_robot_from_config(
            SeeedB601RSFollowerConfig(
                port="can0",
                gripper_type="none",
                startup_sync_vlim_deg_s=5.0,
                **self._kwargs("rs_startup_sync"),
            )
        )
        robot._startup_sync_active = True
        robot._startup_sync_command_deg = {
            motor_name: 0.0 for motor_name in robot.motor_names
        }
        robot._startup_sync_last_time_s = 100.0

        with patch(
            "lerobot_robot_seeed_b601_rt.motorbridge.seeed_b601_rs_follower.time.perf_counter",
            return_value=101.0,
        ):
            limited = robot._apply_startup_action_sync(
                {motor_name: 20.0 for motor_name in robot.motor_names}
            )

        self.assertEqual(limited, {name: 5.0 for name in robot.motor_names})
        self.assertTrue(robot._startup_sync_active)

    def test_pico_head_is_six_axis_nine_dimensional_cartesian(self) -> None:
        config = SeeedB601RSFollowerConfig(
            port="can0",
            action_mode="cartesian",
            gripper_type="none",
            **self._kwargs("pico_head"),
        )
        robot = make_robot_from_config(config)

        self.assertIs(RobotConfig.get_choice_class(config.type), type(config))
        self.assertEqual(tuple(robot.motor_names), KINEMATIC_MOTORS)
        self.assertEqual(tuple(robot.action_features), TCP_POSE_KEYS)
        self.assertEqual(tuple(robot.observation_features), TCP_POSE_KEYS)
        self.assertNotIn("gripper.pos", robot.action_features)

    def test_pico_head_fk_ik_round_trip(self) -> None:
        robot = make_robot_from_config(
            SeeedB601RSFollowerConfig(
                port="can0",
                action_mode="cartesian",
                gripper_type="none",
                **self._kwargs("pico_head_fk_ik"),
            )
        )
        from rebotarm_control_rt.paths import robstride_urdf_path

        urdf_path = robot._resolve_kinematic_urdf_path()
        self.assertEqual(urdf_path, robstride_urdf_path().resolve())
        self.assertIn("rebotarm_control_rt", urdf_path.parts)
        model = robot._load_kinematic_model()
        q = np.array([0.2, 0.4, 0.6, 0.1, -0.2, 0.3], dtype=float)
        _, _, target = model.fk(q, "")
        rotation_6d = robot._matrix_to_rotation_6d(target[:3, :3])
        action = {
            "tcp.x": float(target[0, 3]),
            "tcp.y": float(target[1, 3]),
            "tcp.z": float(target[2, 3]),
            **{
                key: float(rotation_6d[index])
                for index, key in enumerate(TCP_POSE_KEYS[3:])
            },
        }
        robot._read_joint_positions_deg = lambda: {
            motor_name: math.degrees(float(q[index]))
            for index, motor_name in enumerate(KINEMATIC_MOTORS)
        }

        joint_action = robot.tcp_action_to_joint_action(action)
        solved_q = np.array(
            [
                math.radians(joint_action[f"{motor_name}.pos"])
                for motor_name in KINEMATIC_MOTORS
            ]
        )

        np.testing.assert_allclose(solved_q, q, atol=1e-5)

    def test_taccap_features_follow_the_connection_switch(self) -> None:
        disabled = make_robot_from_config(
            SeeedB601RSFollowerConfig(
                port="can0",
                gripper_type="taccap",
                connect_taccap_gripper=False,
                auto_discover_taccap_cameras=False,
                **self._kwargs("taccap_disabled"),
            )
        )
        enabled = make_robot_from_config(
            SeeedB601RSFollowerConfig(
                port="can0",
                gripper_type="taccap",
                connect_taccap_gripper=True,
                auto_discover_taccap_cameras=False,
                **self._kwargs("taccap_enabled"),
            )
        )

        self.assertNotIn("gripper.pos", disabled.action_features)
        self.assertIn("gripper.pos", enabled.action_features)
        self.assertEqual(len(enabled.action_features), 7)

    def test_taccap_tactile_configs_come_from_the_independent_camera_package(self) -> None:
        config = SeeedB601RSFollowerConfig(
            port="can0",
            gripper_type="taccap",
            connect_taccap_gripper=True,
            auto_discover_taccap_cameras=True,
            enable_taccap_tactiles=True,
            enable_taccap_wrist_camera=False,
            expected_tactiles_per_side=2,
            taccap_side="left",
            **self._kwargs("taccap_cameras"),
        )

        with patch(
            "lerobot_robot_seeed_b601_rt.grippers.taccap_gripper.discover_tactiles_by_hub",
            return_value={
                "left": {
                    "left": "GSPS01A00Z0001",
                    "right": "GSPS01A00Z0002",
                },
                "right": {},
            },
        ):
            robot = make_robot_from_config(config)

        self.assertEqual(set(robot.cameras), {"tactile_left", "tactile_right"})
        for camera_config in config.cameras.values():
            self.assertEqual(
                camera_config.__class__.__module__,
                "lerobot_camera_xense.configuration_xense",
            )

    def test_dual_taccap_and_standalone_gripper_register(self) -> None:
        dual = make_robot_from_config(
            BiSeeedB601RSTacCapGripperFollowerConfig(
                left_port="can0",
                right_port="can1",
                connect_taccap_gripper=True,
                auto_discover_taccap_cameras=False,
                **self._kwargs("dual_taccap"),
            )
        )
        standalone = make_robot_from_config(
            TacCapGripperFollowerConfig(
                id="standalone_taccap",
                calibration_dir=self.calibration_dir,
            )
        )

        self.assertIn("left_gripper.pos", dual.action_features)
        self.assertIn("right_gripper.pos", dual.action_features)
        self.assertEqual(standalone.action_features, {"gripper.pos": float})


if __name__ == "__main__":
    unittest.main()
