# Seeed reBot Arm B601 RT Follower for LeRobot

This package registers LeRobot follower robots for the Seeed reBot Arm B601
using `rebotarm_control_rt` as the low-level motor backend.

Compared with `lerobot-robot-seeed-b601`, this RT variant keeps a LeRobot-style
robot interface while the motor commands are sent by the Rust RT loop in
`rebotarm_control_rt`. It supports:

- single-arm follower: `seeed_b601_rt_follower`
- dual-arm follower: `bi_seeed_b601_rt_follower`
- joint-space and Cartesian TCP action modes
- Damiao / SocketCAN / RobStride adapter selection
- optional force-position mode for the built-in gripper motor
- optional external serial Xense gripper
- per-arm calibrated TCP URDF for Cartesian teleoperation

## Install

Install LeRobot and `rebotarm_control_rt` first, then install this plugin:

```bash
cd /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
pip install -e .
```

If you changed `rebotarm_control_rt` path resolution, URDFs, or native bindings,
reinstall/rebuild `rebotarm_control_rt` before running this plugin.

## Device Ports

Check the current serial ports before running:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

For quick local testing you can grant access manually:

```bash
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB*
```

For long-term use, prefer a proper `udev` rule instead of manual `chmod`.

## Single Arm

Leader-arm joint teleoperation:

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
  --fps=100
```

Pico4 Cartesian teleoperation is usually run through `lerobot-teleoperator-pico4`:

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

## Dual Arm

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

Dual-arm settings are split where hardware can differ:

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

## Cartesian TCP URDF

Cartesian mode uses `rebotarm_control_rt.kinematics.load_robot_model(...)`.

Single-arm default:

```text
--robot.kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

Dual-arm defaults:

```text
--robot.left_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
--robot.right_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

With the current `rebotarm_control_rt` resolver, relative URDF names are resolved
by basename in the SDK calibration directory. For example:

```bash
--robot.kinematic_urdf_path=tool_calibration.urdf
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

If you want to use the original built-in SDK URDF, set the path to `None` in
the config object or change the default in the config file. Command-line parsers
usually pass strings, so using `None` from CLI depends on the active LeRobot
parser behavior.

## Built-In Gripper Motor

Enable the B601 CAN gripper motor with:

```bash
--robot.control_gripper=true
```

In `pos_vel` mode the plugin can switch the gripper to Damiao force-position
mode:

```bash
--robot.gripper_type=rebotarm_b601
--robot.enabled_gripper_force=true
--robot.gripper_force_pos_torque_ratio=0.02
```

The LeRobot convention exposed by this plugin is:

```text
gripper.pos = 0.0  # open
gripper.pos = 1.0  # closed
```

The built-in B601 motor limits are mapped through `joint_limits["gripper"]`.
`enabled_gripper_force` and `gripper_force_pos_torque_ratio` only apply when
`gripper_type=rebotarm_b601`.

## External Serial Gripper

This plugin can use `serial_gripper.py` as an external Xense serial gripper.
When enabled, `gripper.pos` stays in the LeRobot convention (`0=open`,
`1=closed`), but commands are sent to the serial gripper instead of the B601
CAN gripper motor.

Single arm:

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_port=/dev/ttyUSB1
```

Or select the serial gripper by board serial number:

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_sn="'000033'"
```

The SN is a string. If it starts with `0`, either put it in
`config_seeed_b601_rt_follower.py` or use the extra inner quotes above;
otherwise the command-line parser may turn `000033` into the number `27`.

Useful serial gripper options:

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

When serial gripper mode is active, the RT arm config excludes the CAN gripper
motor, but LeRobot action and observation still expose `gripper.pos`. The
built-in gripper force-position settings are ignored in this mode.

## Cameras

Camera configs live in the robot config dataclasses. The code currently keeps
example RealSense and OpenCV/YUYV configs commented out. Enable the camera you
need in `config_seeed_b601_rt_follower.py` or pass camera configs through your
LeRobot setup.

For display:

```bash
--display_data=true
```

RealSense notes:

- `pyrealsense2` must be installed in the active environment.
- OpenCV/YUYV can be used if you choose an `OpenCVCameraConfig` and set
  `fourcc="YUYV"`.

Xense tactile sensors use the external `lerobot_camera_xense` package. This
package targets `xensesdk>=2.0.0`; it does not use the old xensesdk 1.7.1
`CameraSource` API.

Install it first:

```bash
pip install -e .
```

Single-arm tactile cameras:

```bash
--robot.enable_tactile_sensors=true \
--robot.tactile_camera_sn_0=OG001320 \
--robot.tactile_camera_sn_1=OG001319 \
--robot.tactile_output_types='["rectify"]'
```

Dual-arm tactile cameras:

```bash
--robot.enable_tactile_sensors=true \
--robot.left_tactile_camera_sn_0=OG001320 \
--robot.left_tactile_camera_sn_1=OG001319 \
--robot.right_tactile_camera_sn_0=OG001322 \
--robot.right_tactile_camera_sn_1=OG001321 \
--robot.tactile_output_types='["rectify"]'
```

Supported Xense outputs are `rectify` and `difference`.

Dual-arm wrist cameras:

```bash
--robot.enable_wrist_cameras=true \
--robot.left_wrist_camera_sn=XC000001 \
--robot.right_wrist_camera_sn=XC000002 \
--robot.wrist_camera_fourcc=MJPG \
--robot.wrist_camera_width=640 \
--robot.wrist_camera_height=480 \
--robot.wrist_camera_fps=30
```

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

Return-to-initial speed on reset/disconnect:

```bash
--robot.return_to_initial_vlim_deg_s='{"shoulder_pan":15,"shoulder_lift":15,"elbow_flex":15,"wrist_flex":15,"wrist_yaw":15,"wrist_roll":15,"gripper":150}'
```

For dual arms, CPU and priority can be split:

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_rt_priority=99
--robot.right_rt_priority=99
```

## Generated RT Config

By default the plugin generates a `rebotarm_control_rt` YAML config under the
LeRobot calibration cache. You can override it:

```bash
--robot.arm_cfg_path=/path/to/arm.yaml
```

For dual arm:

```bash
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
```

The YAML joint order must match the controlled motor order. With the built-in
CAN gripper enabled:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper
```

With external serial gripper enabled, the CAN gripper motor is not part of the
RT arm target list:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll
```

## Observations and Actions

Joint action mode:

```text
joint_1..joint_6 map to shoulder_pan..wrist_roll in observations
actions use shoulder_pan.pos, shoulder_lift.pos, ...
gripper.pos is normalized: 0=open, 1=closed
```

Joint mode publishes `joint_1..joint_6` position observations by default and does not need
an extra option. `--robot.enable_observation_joint_pos=true` is reserved for Cartesian mode:
when actions are TCP targets, enable it only if you also want joint position observations.

Optional velocity and torque observation fields:

```bash
--robot.enable_observation_joint_vel=true
--robot.enable_observation_joint_torque=true
--robot.enable_observation_gripper_vel=true
--robot.enable_observation_gripper_torque=true
```

Cartesian action mode:

```text
tcp.x, tcp.y, tcp.z
tcp.r1 ... tcp.r6  # 6D rotation
gripper.pos        # if control_gripper=true
```

The robot converts TCP targets to joint targets through the configured URDF and
IK solver before sending RT joint targets.

## Debugging

Enable motion debug:

```bash
--robot.debug_motion=true
```

Watch `rt_overruns(send/read)`:

- `send`: RT command thread missed a deadline.
- `read`: optional RT feedback request thread missed a deadline.

For Damiao serial TX logs:

```bash
--robot.damiao_tx_debug=1
```

If a serial port is busy, check for stopped Python jobs or stale processes:

```bash
jobs
ps aux | grep python
```

Then unplug/replug only if the device remains busy after the process exits.

## Calibration Notes

This plugin does not run a LeRobot calibration flow. Motor zeroing and motor
modes are handled by `rebotarm_control_rt` / firmware. TCP calibration is done
by generating a calibrated URDF in `rebotarm_control_rt` and passing that URDF
through `kinematic_urdf_path`.
