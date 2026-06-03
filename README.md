# Seeed reBot Arm B601 RT Follower Integration with LeRobot

This package registers a LeRobot follower robot for the Seeed reBot Arm B601 that uses
`rebotarm_control_rt` for motor control.

Compared with `lerobot-robot-seeed-b601`, this RT variant keeps the same LeRobot joint-space
surface (`shoulder_pan.pos`, `shoulder_lift.pos`, ...), but the motor commands are sent by the
Rust actuator loop in `rebotarm_control_rt`.

## Install

Install LeRobot and `rebotarm_control_rt` first, then install this package:

```bash
cd /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
pip install -e .
```

The registered robot type is:

```text
seeed_b601_rt_follower
```

## Teleoperate

Replace the previous follower type with `seeed_b601_rt_follower`:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --teleop.joint_directions='{"shoulder_pan":-1,"shoulder_lift":-1,"elbow_flex":1,"wrist_flex":1,"wrist_yaw":1,"wrist_roll":-1,"gripper":-4}' \
  --fps=100
```

By default the plugin:

- generates a `rebotarm_control_rt` YAML config from the B601 motor mapping,
- runs `RobotArm` in `pos_vel` mode,
- starts `RobotArm.start_rt_loop(rate=150, command_gap_us=0)`,
- sends LeRobot actions by calling `RobotArm.set_targets(...)`,
- returns to the startup joint pose on disconnect, then optionally disables torque,
- converts LeRobot degrees to `rebotarm_control_rt` radians.

## Useful Options

```bash
--robot.rt_rate=150
--robot.rt_command_gap_us=0
--robot.disable_torque_on_disconnect=false
--robot.control_mode=pos_vel
--robot.debug_motion=true
```

For the Damiao serial bridge, keep `rt_overruns(send/read)` near zero in
`--robot.debug_motion=true` logs. `send` means the control command thread missed
its deadline; `read` means the optional RT feedback request thread missed its
deadline. The legacy `rt_overruns` property is kept as an alias of
`rt_send_overruns`. In RT mode, observations read motorbridge's background
polling cache; the RT control loop runs faster than recording, so cached joint
state is kept fresh by incoming motor feedback.

Use an explicit `rebotarm_control_rt` YAML config if needed:

```bash
--robot.arm_cfg_path=/path/to/arm.yaml
```

The YAML joint order must match the LeRobot B601 joint order when using a custom file:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper
```

## Notes

The plugin does not run a LeRobot calibration flow. Motor zeroing is handled by
`rebotarm_control_rt` / motor firmware, same as your RT examples.
