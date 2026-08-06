# Seeed reBot Arm B601 LeRobot Followers

This is the consolidated B601 robot repository. It installs one Python package,
`lerobot_robot_seeed_b601_rt`, containing two independent motor-control backends:

- The original DM/RS robot classes keep using the Python `motorbridge` backend.
- The RT robot classes use the Rust loop provided by `rebotarm_control_rt`.

The motorbridge classes do not inherit from or call the RT robot class. They only
share this repository and Python package so all B601 robot types can be installed
and maintained together.

## Robot types and backends

| Robot type | Backend | Action space | Purpose |
| --- | --- | --- | --- |
| `seeed_b601_dm_follower` | `motorbridge` | 7D joint + gripper | Original Damiao follower |
| `seeed_b601_rs_follower` | `motorbridge` + optional FK/IK/TacCap | 6D/7D joint or 9D TCP | All single-arm RobStride modes |
| `bi_seeed_b601_rs_taccap_gripper_follower` | `motorbridge` + TacCap SDK | Dual joint + optional grippers | Dual RS arms with TacCap grippers |
| `taccap_gripper_follower` | TacCap SDK | 1D gripper | Standalone TacCap gripper test |
| `seeed_b601_rt_follower` | `rebotarm_control_rt` | Joint or Cartesian | Single RT follower |
| `bi_seeed_b601_rt_follower` | `rebotarm_control_rt` | Joint or Cartesian | Dual RT follower |

The Xense tactile camera implementation remains in the independent
`lerobot-camera-xense` repository. This package imports it only when automatic
TacCap tactile cameras are enabled.

## Package layout

```text
lerobot_robot_seeed_b601_rt/
├── motorbridge/  # DM, RS, and dual-RS MotorBridge robots
├── rt/           # single-arm and dual-arm rebotarm_control_rt robots
├── grippers/     # shared TacCap and serial gripper implementations
└── __init__.py   # stable public imports and RobotConfig registration
```

## Install

Remove the old standalone B601 distribution before installing this consolidated
package. Both packages register names such as `seeed_b601_dm_follower`, and
LeRobot rejects duplicate registrations:

```bash
pip uninstall -y lerobot_robot_seeed_b601
```

Install LeRobot and `rebotarm_control_rt`, then install this repository:

```bash
cd /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
pip install -e .
```

Install optional hardware integrations only when needed:

```bash
# TacCap gripper SDK
pip install -e /home/xense/rebot_lerobot/TacCap-Gripper

# Independent Xense tactile camera package
pip install -e /home/xense/rebot_lerobot/lerobot-camera-xense

# External Xense serial gripper
pip install -e /home/xense/rebot_lerobot/XGripper
```

Verify the two backends are isolated:

```bash
python - <<'PY'
from lerobot_robot_seeed_b601_rt import (
    SeeedB601DMFollower,
    SeeedB601RTFollower,
)
print(SeeedB601DMFollower.__mro__)
print(SeeedB601RTFollower.__mro__)
PY
```

See [`MIGRATION.md`](MIGRATION.md) for source commits, file mappings, and the
backend inheritance layout.

## Motorbridge robot examples

The original DM robot continues to use the Damiao serial bridge through
`motorbridge`:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=dm_follower \
  --robot.can_adapter=damiao \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --fps=100 \
  --display_data=true
```

The original RS robot continues to use SocketCAN through `motorbridge`:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rs_follower \
  --robot.port=can0 \
  --robot.id=rs_follower \
  --robot.can_adapter=socketcan \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --fps=100 \
  --display_data=true
```

Use the same type without a gripper by adding:

```bash
--robot.gripper_type=none
```

The single RS class supports these combinations:

| `action_mode` | `gripper_type` | Result |
| --- | --- | --- |
| `joint` | `motor` | 6 RS joints + ID7 motorbridge gripper (default) |
| `joint` | `none` | 6 RS joints without a gripper |
| `joint` | `taccap` | 6 RS joints + external TacCap gripper |
| `cartesian` | `none` | 9-D Pico4Head TCP action; no gripper |

Pico4Head uses the six-axis, gripper-free motorbridge RS robot. Its FK/IK is
provided by `rebotarm_control_rt`:

```bash
lerobot-teleoperate-pico4 \
  --robot.type=seeed_b601_rs_follower \
  --robot.port=can0 \
  --robot.id=rs_pico_head \
  --robot.can_adapter=socketcan \
  --robot.action_mode=cartesian \
  --robot.gripper_type=none \
  --teleop.type=pico4head \
  --teleop.id=pico4head \
  --fps=60 \
  --display_data=true
```

This robot has six arm joints, no gripper, and the fixed action schema:

```text
tcp.x, tcp.y, tcp.z,
tcp.r1, tcp.r2, tcp.r3, tcp.r4, tcp.r5, tcp.r6
```

Single motorbridge RS arm with a TacCap gripper:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rs_follower \
  --robot.port=can0 \
  --robot.id=rs_taccap \
  --robot.can_adapter=socketcan \
  --robot.action_mode=joint \
  --robot.gripper_type=taccap \
  --robot.connect_taccap_gripper=true \
  --robot.taccap_role=follower \
  --robot.taccap_side=left \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader \
  --display_data=true
```

Disable automatic use of the independent Xense tactile/wrist camera package with:

```bash
--robot.auto_discover_taccap_cameras=false
```

For dual motorbridge RS + TacCap, select
`bi_seeed_b601_rs_taccap_gripper_follower` and provide the two CAN interfaces and
TacCap sides:

```bash
--robot.left_port=can0 \
--robot.right_port=can1 \
--robot.left_taccap_side=left \
--robot.right_taccap_side=right
```

Standalone TacCap gripper test:

```bash
lerobot-teleoperate \
  --robot.type=taccap_gripper_follower \
  --robot.id=taccap_gripper_left \
  --robot.taccap_role=follower \
  --robot.taccap_side=left \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader
```

The two TacCap paths retain their original conventions:

- `seeed_b601_rs_follower --robot.gripper_type=taccap`: `gripper.pos=0` is closed and `gripper.pos=1` is open.
- `seeed_b601_rt_follower --robot.gripper_type=taccap`: `gripper.pos=0` is open and `gripper.pos=1` is closed.

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

## Xense TacCap Gripper

Install the TacCap SDK and the sibling Xense camera plugin in the same Python
environment before enabling this mode:

```bash
pip install -e /home/xense/rebot_lerobot/TacCap-Gripper
pip install -e /home/xense/rebot_lerobot/lerobot-camera-xense
pip install -e /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
```

Single-arm RobStride example with automatic follower-side discovery:

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_rt_follower \
  --robot.port=can0 \
  --robot.can_adapter=robstride \
  --robot.control_mode=mit \
  --robot.control_gripper=true \
  --robot.gripper_type=taccap \
  --robot.taccap_role=follower \
  --robot.taccap_side=left \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

By default this discovers the TacCap MCU plus two tactile sensors and one wrist
camera on the same USB hub. The camera keys are `tactile_left`,
`tactile_right`, and `wrist_cam`. Disable all camera discovery with:

```bash
--robot.auto_discover_taccap_cameras=false
```

Use `--robot.gripper_device=/dev/serial/by-id/...` to pin an MCU path. The
current `rebot_arm_102_leader` already publishes a normalized `0..1` action, so
`normalize_gripper_action` defaults to `false`. For an older leader that emits
a `0..55` angle, set:

```bash
--robot.normalize_gripper_action=true \
--robot.gripper_action_min=0 \
--robot.gripper_action_max=55
```

TacCap SDK native position uses `0=closed`, `1=open`, so the controller reverses
it at the hardware boundary. LeRobot action and observation keep the RT package
convention: `0=open`, `1=closed`. Closing commands can use the reference
torque-grasp behavior (`kp=0`, `kd=0`, feed-forward torque only). Relevant
parameters include `gripper_kp`, `gripper_kd`,
`gripper_feedforward_torque`, and `gripper_torque_grasp_enabled`.

The reference non-RT RobStride follower sends arm joints in MIT mode. To match
its tracking response, keep `--robot.control_mode=mit` in the RT command. The RT
default is the more conservative `pos_vel` mode; its default RobStride velocity
limits are `[1, 0.4, 0.4, 1, 1, 1] rad/s`, so shoulder lift and elbow tracking is
intentionally much slower.

Dual-arm mode supports one TacCap gripper per side:

```bash
--robot.type=bi_seeed_b601_rt_follower \
--robot.left_gripper_type=taccap \
--robot.right_gripper_type=taccap \
--robot.left_taccap_side=left \
--robot.right_taccap_side=right \
--robot.left_gripper_feedforward_torque=2.0 \
--robot.right_gripper_feedforward_torque=3.0
```

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
modes are handled by `rebotarm_control_rt` / firmware. By default, Cartesian
mode loads the RobStride URDF installed inside `rebotarm_control_rt`; this
plugin no longer ships a duplicate URDF. For a calibrated TCP, generate a
custom URDF and pass its path through `kinematic_urdf_path`.
