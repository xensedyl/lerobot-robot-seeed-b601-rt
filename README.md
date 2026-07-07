# Seeed reBot Arm B601 RT LeRobot Follower

This package registers RT followers for the Seeed reBot Arm B601 in LeRobot.
Low-level motor control is handled by `rebotarm_control_rt`.

Compared with `lerobot-robot-seeed-b601`, this RT version keeps the
LeRobot-style robot interface, while motor commands are sent by the Rust RT loop
in `rebotarm_control_rt`. Current support includes:

- single-arm follower: `seeed_b601_rt_follower`
- dual-arm follower: `bi_seeed_b601_rt_follower`
- joint-space control and Cartesian TCP control
- Damiao / SocketCAN / RobStride adapter selection
- built-in gripper motor force-position mode
- external Xense serial gripper
- per-arm TCP calibration URDF configuration

## Install

Install LeRobot and `rebotarm_control_rt` first, then install this plugin:

```bash
cd /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
pip install -e .
```

If you changed path resolution, URDFs, or native bindings in
`rebotarm_control_rt`, rebuild/reinstall `rebotarm_control_rt` first.

## Device Ports

Check the current serial ports before running:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

For quick testing you can grant access manually:

```bash
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB*
```

For long-term use, prefer a `udev` rule instead of running `chmod` each time.

For LingZu / RobStride through PCAN-USB, bring up SocketCAN at 1 Mbps first:

```bash
sudo modprobe peak_usb
ip -br link

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

## Single-Arm Teleoperation

Use a leader arm for Damiao joint teleoperation:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --robot.control_gripper=true \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --teleop.joint_directions='{"shoulder_pan":-1,"shoulder_lift":-1,"elbow_flex":1,"wrist_flex":1,"wrist_yaw":1,"wrist_roll":-1,"gripper":-4}' \
  --fps=100 \
  --display_data=true
```

Use a leader arm for RobStride joint teleoperation:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=can0 \
  --robot.id=follower1 \
  --robot.can_adapter=robstride \
  --robot.control_gripper=false \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --fps=100 \
  --display_data=true
```

LingZu / RobStride hardware uses the same `robot.type`; use the CAN interface
as the port and switch the adapter to RobStride:

```bash
--robot.port=can0
--robot.can_adapter=robstride
```

The adapter switch also changes the generated RT config to RobStride CAN IDs,
RobStride motor models/gains, the LingZu URDF, and the same RS follower-side
joint direction mapping used by the non-RT/TacCap follower. If `--robot.port`
is not passed, `can_adapter=robstride` defaults to `can0`; only pass
`--robot.port=can1` when the CAN interface is not `can0`.

Pico4 Cartesian teleoperation is usually run through
`lerobot-teleoperator-pico4`.

Pico4 Cartesian teleoperation with Damiao:

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --robot.control_gripper=true \
  --teleop.type=pico4 \
  --teleop.id=pico4 \
  --fps=100 \
  --display_data=true
```

Pico4 Cartesian teleoperation with RobStride:

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=can0 \
  --robot.id=follower1 \
  --robot.can_adapter=robstride \
  --robot.action_mode=cartesian \
  --robot.control_gripper=true \
  --teleop.type=pico4 \
  --teleop.id=pico4 \
  --fps=100 \
  --display_data=true
```

## Dual-Arm Teleoperation

Dual leader-arm joint teleoperation:

```bash
lerobot-teleoperate \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.id=bi_follower \
  --robot.left_port=can0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.left_can_adapter=robstride \
  --robot.right_can_adapter=damiao \
  --robot.control_gripper=false \
  --robot.left_rt_cpu=3 \
  --robot.right_rt_cpu=4 \
  --teleop.type=bi_rebot_arm_102_leader \
  --teleop.id=bi_rebot_arm_102_leader \
  --teleop.left_port=/dev/ttyUSB0 \
  --teleop.right_port=/dev/ttyUSB1 \
  --fps=100 \
  --display_data=true
```

Dual-arm Pico4 Cartesian teleoperation:

```bash
lerobot-teleoperate-pico4 \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=cartesian \
  --robot.control_gripper=true \
  --teleop.type=bi_pico4 \
  --teleop.id=bi_pico4 \
  --fps=100 \
  --display_data=true
```

Hardware parameters that may differ between arms are split into left/right
settings:

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

## Cartesian TCP URDF

Cartesian mode loads the kinematic model with
`rebotarm_control_rt.kinematics.load_robot_model(...)`.

Single-arm default:

```text
--robot.kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

Dual-arm defaults:

```text
--robot.left_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
--robot.right_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

The current `rebotarm_control_rt` path resolver searches by basename in the
SDK `calibration/` directory. These forms also work:

```bash
--robot.kinematic_urdf_path=tool_calibration.urdf
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

To use the SDK's original built-in URDF, set the path to `None` in the config
object or change the config default. Whether CLI `None` becomes a real `None`
depends on the current LeRobot parser behavior.

## Built-In Gripper Motor

Enable the B601 CAN gripper motor:

```bash
--robot.control_gripper=true
```

In `pos_vel` mode, the gripper can be switched to Damiao force-position mode:

```bash
--robot.gripper_type=rebotarm_b601
--robot.enabled_gripper_force=true
--robot.gripper_force_pos_torque_ratio=0.02
```

The gripper convention exposed to LeRobot is:

```text
gripper.pos = 0.0  # open
gripper.pos = 1.0  # closed
```

The built-in B601 gripper motor maps through `joint_limits["gripper"]` to motor
angle. `enabled_gripper_force` and `gripper_force_pos_torque_ratio` only apply
when `gripper_type=rebotarm_b601`.

## External Serial Gripper

This plugin can control an external Xense serial gripper through
`serial_gripper.py`. When enabled, `gripper.pos` still keeps the LeRobot/B601
convention (`0=open`, `1=closed`), but commands are sent to the serial gripper
instead of the B601 CAN gripper motor.

Single arm:

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_port=/dev/ttyUSB1
```

The board SN can also be used for automatic port discovery:

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_sn="'000033'"
```

The SN is a string. If it starts with `0`, put it in the default values in
`config_seeed_b601_rt_follower.py`; if passing it on the command line, use the
extra inner quotes above. Otherwise the parser may treat `000033` as a number
and parse it as `27`.

Common serial gripper options:

```bash
--robot.serial_gripper_baudrate=115200
--robot.serial_gripper_device_id=1
--robot.serial_gripper_min_pos=0
--robot.serial_gripper_max_pos=85
--robot.serial_gripper_v_max=80
--robot.serial_gripper_f_max=27
--robot.serial_gripper_init_open=true
```

Dual arm:

```bash
--robot.control_gripper=true \
--robot.left_gripper_type=serial \
--robot.left_serial_gripper_sn="'000033'" \
--robot.left_serial_gripper_port=/dev/ttyUSB0 \
--robot.right_gripper_type=serial \
--robot.right_serial_gripper_sn="'000034'" \
--robot.right_serial_gripper_port=/dev/ttyUSB1
```

When serial gripper mode is enabled, the RT arm motor list does not include the
CAN gripper motor, but LeRobot action and observation still expose
`gripper.pos`. Built-in gripper force-position parameters are not used in this
mode.

## Cameras

Camera configs live in the robot config dataclass. The code keeps example
RealSense and OpenCV/YUYV configs commented out by default. Enable the camera
you need in `config_seeed_b601_rt_follower.py`, or pass camera configs through
your LeRobot configuration.

Display data:

```bash
--display_data=true
```

RealSense notes:

- `pyrealsense2` must be installed in the active environment.
- To use OpenCV/YUYV, choose `OpenCVCameraConfig` and set `fourcc="YUYV"`.

## Useful Runtime Options

```bash
--robot.rt_rate=150
--robot.rt_command_gap_us=0
--robot.rt_priority=99
--robot.rt_cpu=3
--robot.disable_torque_on_disconnect=false
--robot.control_mode=pos_vel
--robot.action_mode=joint
--robot.debug_motion=true
--robot.debug_motion_interval_s=1.0
```

If display data shows non-zero actions but the arm does not move, enable
`debug_motion` to inspect the mapped target, current joint positions, and RT
loop overrun counters.

Return-to-initial speed limit for reset/disconnect:

```bash
--robot.return_to_initial_vlim_deg_s='{"shoulder_pan":15,"shoulder_lift":15,"elbow_flex":15,"wrist_flex":15,"wrist_yaw":15,"wrist_roll":15,"gripper":150}'
```

Dual arms can set CPU and priority separately:

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_rt_priority=99
--robot.right_rt_priority=99
```

## Generated RT Config

By default, the plugin generates a `rebotarm_control_rt` YAML config under the
LeRobot calibration cache. You can also specify your own YAML:

```bash
--robot.arm_cfg_path=/path/to/arm.yaml
```

Dual arm:

```bash
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
```

The joint order in the YAML must match the controlled motor order. When using
the built-in CAN gripper:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper
```

When using an external serial gripper, the CAN gripper motor is no longer part
of the RT arm target list:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll
```

## Observations and Actions

Joint mode:

```text
joint_1..joint_6 in observation map to shoulder_pan..wrist_roll
actions use shoulder_pan.pos, shoulder_lift.pos, ...
gripper.pos is normalized: 0=open, 1=closed
```

Optional observation fields:

```bash
--robot.enable_observation_joint_pos=true
--robot.enable_observation_joint_vel=true
--robot.enable_observation_joint_torque=true
--robot.enable_observation_gripper_vel=true
--robot.enable_observation_gripper_torque=true
```

Cartesian mode:

```text
tcp.x, tcp.y, tcp.z
tcp.r1 ... tcp.r6  # 6D rotation
gripper.pos        # present when control_gripper=true
```

The robot converts TCP targets to joint targets through the configured URDF and
IK solver, then sends RT joint targets.

## Debugging

Enable motion debug:

```bash
--robot.debug_motion=true
```

Watch `rt_overruns(send/read)`:

- `send`: RT command thread missed a deadline.
- `read`: optional RT feedback request thread missed a deadline.

Damiao serial TX logs:

```bash
--robot.damiao_tx_debug=1
```

If a serial port is busy, first check whether there are stopped but not exited
Python processes:

```bash
jobs
ps aux | grep python
```

Only unplug/replug the device if the port remains busy after the process has
actually exited.

## Calibration Notes

This plugin does not run the LeRobot calibration flow. Motor zeroing and motor
modes are handled by `rebotarm_control_rt` / firmware. TCP calibration is done
by generating a URDF with the new TCP in `rebotarm_control_rt`, then passing
that URDF to this plugin through `kinematic_urdf_path`.
