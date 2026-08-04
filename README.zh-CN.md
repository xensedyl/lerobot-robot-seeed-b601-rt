# Seeed reBot Arm B601 LeRobot Followers

这是整理后的 B601 最终仓库。它只安装一个 Python 包
`lerobot_robot_seeed_b601_rt`，但包内保留两套相互独立的电机控制后端：

- 原来的 DM/RS Robot 类继续使用 Python `motorbridge`。
- RT Robot 类继续使用 `rebotarm_control_rt` 的 Rust 实时循环。

motorbridge 类不会继承或调用 RT Robot 类。两套实现只是放在同一个仓库、同一个
Python 包中统一安装和维护。

## Robot type 与后端

| Robot type | 后端 | Action | 用途 |
| --- | --- | --- | --- |
| `seeed_b601_dm_follower` | `motorbridge` | 7D 关节 + 夹爪 | 原 DM follower |
| `seeed_b601_rs_follower` | `motorbridge` + 可选 FK/IK/TacCap | 6D/7D 关节或 9D TCP | 所有 RS 单臂模式 |
| `bi_seeed_b601_rs_taccap_gripper_follower` | `motorbridge` + TacCap SDK | 双臂关节 + 可选夹爪 | 双臂 RS + TacCap |
| `taccap_gripper_follower` | TacCap SDK | 1D 夹爪 | 单独测试 TacCap 夹爪 |
| `seeed_b601_rt_follower` | `rebotarm_control_rt` | 关节或笛卡尔 | RT 单臂 |
| `bi_seeed_b601_rt_follower` | `rebotarm_control_rt` | 关节或笛卡尔 | RT 双臂 |

Xense 视触相机实现继续放在独立的 `lerobot-camera-xense` 仓库。本包只在启用
TacCap 自动视触相机时按需导入它，不内置相机代码。

## 安装

安装最终仓库前，先卸载旧的独立 B601 发行包。两个包都会注册
`seeed_b601_dm_follower` 等同名 type，LeRobot 遇到重复注册会直接报错：

```bash
pip uninstall -y lerobot_robot_seeed_b601
```

安装 LeRobot 和 `rebotarm_control_rt` 后，再安装本仓库：

```bash
cd /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
pip install -e .
```

按实际硬件安装可选组件：

```bash
# TacCap 夹爪 SDK
pip install -e /home/xense/rebot_lerobot/TacCap-Gripper

# 独立 Xense 视触相机包
pip install -e /home/xense/rebot_lerobot/lerobot-camera-xense

# Xense 外置串口夹爪
pip install -e /home/xense/rebot_lerobot/XGripper
```

检查 motorbridge 与 RT 两套继承链是否独立：

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

详细的来源提交、文件映射和后端继承关系见 [`MIGRATION.md`](MIGRATION.md)。

## Motorbridge Robot 示例

原 DM Robot 类继续使用 Damiao 串口桥和 `motorbridge`：

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

原 RS Robot 类继续使用 SocketCAN 和 `motorbridge`：

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

同一个 type 加下面的参数即可使用无夹爪六轴模式：

```bash
--robot.gripper_type=none
```

唯一的 RS 类支持下面四种组合：

| `action_mode` | `gripper_type` | 结果 |
| --- | --- | --- |
| `joint` | `motor` | 6 个 RS 关节 + ID7 MotorBridge 夹爪（默认） |
| `joint` | `none` | 6 个 RS 关节，无夹爪 |
| `joint` | `taccap` | 6 个 RS 关节 + 外置 TacCap 夹爪 |
| `cartesian` | `none` | 9D Pico4Head TCP action，无夹爪 |

Pico4Head 使用 motorbridge RS 六轴无夹爪 Robot，FK/IK 由
`rebotarm_control_rt` 提供：

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

该类型只有六个机械臂关节，没有夹爪，action 固定为：

```text
tcp.x, tcp.y, tcp.z,
tcp.r1, tcp.r2, tcp.r3, tcp.r4, tcp.r5, tcp.r6
```

motorbridge RS + TacCap 单臂：

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

如果不需要自动添加独立 Xense 视触和腕部相机：

```bash
--robot.auto_discover_taccap_cameras=false
```

双臂 motorbridge RS + TacCap 使用
`bi_seeed_b601_rs_taccap_gripper_follower`，分别指定左右 CAN 口和 TacCap：

```bash
--robot.left_port=can0 \
--robot.right_port=can1 \
--robot.left_taccap_side=left \
--robot.right_taccap_side=right
```

单独测试 TacCap 夹爪：

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

注意两套 TacCap Robot 保留各自原有语义：

- `seeed_b601_rs_follower --robot.gripper_type=taccap`：`gripper.pos=0` 闭合，`gripper.pos=1` 张开。
- `seeed_b601_rt_follower --robot.gripper_type=taccap`：`gripper.pos=0` 张开，`gripper.pos=1` 闭合。

如果你修改过 `rebotarm_control_rt` 的路径解析、URDF 或 native binding，需要先重新
构建/安装 `rebotarm_control_rt`。

## 查看端口

运行前先确认当前串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

临时测试可以手动授权：

```bash
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB*
```

长期使用建议写 `udev` 规则，不建议每次手动 `chmod`。

灵足 / RobStride 通过 PCAN-USB 接入时，先按 1 Mbps 启动 SocketCAN：

```bash
sudo modprobe peak_usb
ip -br link

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

## 单臂遥操作

用 leader arm 做关节遥操作damiao：

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

用 leader arm 做关节遥操作 RobStride：
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

灵足 / RobStride 真机使用同一个 `robot.type`，端口用 CAN 口，并把 adapter 改成
RobStride：

```bash
--robot.port=can0
--robot.can_adapter=robstride
```

adapter 开关还会自动把生成的 RT 配置切到 RobStride CAN ID、RobStride 电机型号/增益、
灵足 URDF，以及非 RT/TacCap follower 同一套 RS 从手关节方向映射。如果不传
`--robot.port`，`can_adapter=robstride` 默认使用 `can0`；只有 CAN 口不是 `can0` 时，
才传 `--robot.port=can1`。

Pico4 笛卡尔遥操作通常通过 `lerobot-teleoperator-pico4` 运行：

Pico4 笛卡尔遥操作 damiao:

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

Pico4 笛卡尔遥操作 RobStride:

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

## 双臂遥操作

双臂 leader arm 做关节遥操作：

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

双臂 Pico4 笛卡尔遥操作：

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

双臂中可能不同的硬件参数都拆成左右配置：

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

## 笛卡尔 TCP URDF

笛卡尔模式使用 `rebotarm_control_rt.kinematics.load_robot_model(...)` 加载运动学模型。

单臂默认：

```text
--robot.kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

双臂默认：

```text
--robot.left_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
--robot.right_kinematic_urdf_path=lerobot_robot_seeed_b601_rt/tool_calibration.urdf
```

当前 `rebotarm_control_rt` 的路径解析会按 basename 到 SDK 的 `calibration/` 目录里找
URDF。因此也可以传：

```bash
--robot.kinematic_urdf_path=tool_calibration.urdf
--robot.left_kinematic_urdf_path=left_tool_calibration.urdf
--robot.right_kinematic_urdf_path=right_tool_calibration.urdf
```

如果你想用 SDK 原始内置 URDF，可以在 config 对象里把路径设为 `None`，或者直接修改
config 默认值。命令行传 `None` 是否能变成真正的 `None` 取决于当前 LeRobot parser 行为。

## 内置夹爪电机

启用 B601 CAN 夹爪电机：

```bash
--robot.control_gripper=true
```

在 `pos_vel` 模式下，可以让夹爪进入 Damiao force-position 模式：

```bash
--robot.gripper_type=rebotarm_b601
--robot.enabled_gripper_force=true
--robot.gripper_force_pos_torque_ratio=0.02
```

这个插件对 LeRobot 暴露的夹爪语义是：

```text
gripper.pos = 0.0  # 张开
gripper.pos = 1.0  # 闭合
```

内置 B601 夹爪电机通过 `joint_limits["gripper"]` 映射到电机角度。
`enabled_gripper_force` 和 `gripper_force_pos_torque_ratio` 只在
`gripper_type=rebotarm_b601` 时生效。

## 外置串口夹爪

这个插件可以使用 `serial_gripper.py` 控制外置 Xense 串口夹爪。启用后，
`gripper.pos` 对外仍然保持 LeRobot/B601 语义：`0=张开`，`1=闭合`；但命令会发给串口
夹爪，而不是 B601 CAN 夹爪电机。

单臂：

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_port=/dev/ttyUSB1
```

也可以用板载 SN 自动找端口：

```bash
--robot.control_gripper=true \
--robot.gripper_type=serial \
--robot.serial_gripper_sn="'000033'"
```

SN 是字符串。如果 SN 以 `0` 开头，建议直接写到
`config_seeed_b601_rt_follower.py` 默认值里；如果从命令行传，使用上面的内层引号，
否则解析器可能把 `000033` 当数字解析成 `27`。

常用串口夹爪参数：

```bash
--robot.serial_gripper_baudrate=115200
--robot.serial_gripper_device_id=1
--robot.serial_gripper_min_pos=0
--robot.serial_gripper_max_pos=85
--robot.serial_gripper_v_max=80
--robot.serial_gripper_f_max=27
--robot.serial_gripper_init_open=true
```

双臂：

```bash
--robot.control_gripper=true \
--robot.left_gripper_type=serial \
--robot.left_serial_gripper_sn="'000033'" \
--robot.left_serial_gripper_port=/dev/ttyUSB0 \
--robot.right_gripper_type=serial \
--robot.right_serial_gripper_sn="'000034'" \
--robot.right_serial_gripper_port=/dev/ttyUSB1
```

串口夹爪启用后，RT arm 的 motor list 不包含 CAN 夹爪电机，但 LeRobot action 和
observation 里仍然会有 `gripper.pos`。这种模式下不会使用内置夹爪的 force-position
参数。

## Xense TacCap 夹爪

使用前，把 TacCap SDK、独立 Xense 相机包和本包安装到同一个 Python 环境：

```bash
pip install -e /home/xense/rebot_lerobot/TacCap-Gripper
pip install -e /home/xense/rebot_lerobot/lerobot-camera-xense
pip install -e /home/xense/rebot_lerobot/lerobot-robot-seeed-b601-rt
```

单臂 RobStride + TacCap 自动发现示例：

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

默认会按 follower 角色、左右侧和 USB hub 自动发现 TacCap MCU、两路视触和一路腕部
相机。相机 observation key 为 `tactile_left`、`tactile_right`、`wrist_cam`。
如需关闭相机自动发现：

```bash
--robot.auto_discover_taccap_cameras=false
```

自动发现 MCU 失败时，可手动指定：

```bash
--robot.gripper_device=/dev/serial/by-id/...
```

当前 `rebot_arm_102_leader` 已经输出归一化 `0..1`，因此
`normalize_gripper_action` 默认是 `false`。如果使用仍然输出 `0..55` 角度的旧版
leader，使用：

```bash
--robot.normalize_gripper_action=true \
--robot.gripper_action_min=0 \
--robot.gripper_action_max=55
```

TacCap SDK 原生位置语义是 `0=闭合`、`1=张开`，控制器会在硬件边界做反向转换；
LeRobot action 和 observation 仍保持 RT 包统一语义：`0=张开`、`1=闭合`。闭合时
默认启用力矩夹取：位置项使用 `kp=0`、`kd=0`，仅发送
`gripper_feedforward_torque`。常用参数包括
`gripper_kp`、`gripper_kd`、`gripper_feedforward_torque` 和
`gripper_torque_grasp_enabled`。

参考工程的非 RT RobStride follower 对机械臂六个关节使用 MIT 模式。要获得接近的
跟手响应，RT 命令需要保留 `--robot.control_mode=mit`。RT 默认仍是更保守的
`pos_vel`；其 RobStride 默认速度上限是 `[1, 0.4, 0.4, 1, 1, 1] rad/s`，其中肩抬和
肘关节只有 `0.4 rad/s`，因此会明显更慢。

双臂可以分别设置左右 TacCap：

```bash
--robot.type=bi_seeed_b601_rt_follower \
--robot.left_gripper_type=taccap \
--robot.right_gripper_type=taccap \
--robot.left_taccap_side=left \
--robot.right_taccap_side=right \
--robot.left_gripper_feedforward_torque=2.0 \
--robot.right_gripper_feedforward_torque=3.0
```

## 相机

相机配置在 robot config dataclass 里。当前代码里保留了 RealSense 和 OpenCV/YUYV 的
示例配置，但默认是注释状态。需要时在 `config_seeed_b601_rt_follower.py` 中启用，或通过
你的 LeRobot 配置传入。

显示数据：

```bash
--display_data=true
```

RealSense 注意事项：

- 当前环境需要安装 `pyrealsense2`。
- 如果要用 OpenCV/YUYV，选择 `OpenCVCameraConfig` 并设置 `fourcc="YUYV"`。

## 常用运行参数

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

如果 display data 里 action 有值但机械臂不动，打开 `debug_motion` 看映射后的目标、
当前关节位置和 RT loop overrun 计数。

复位/断开时回初始位的速度限制：

```bash
--robot.return_to_initial_vlim_deg_s='{"shoulder_pan":15,"shoulder_lift":15,"elbow_flex":15,"wrist_flex":15,"wrist_yaw":15,"wrist_roll":15,"gripper":150}'
```

双臂可以分开设置 CPU 和优先级：

```bash
--robot.left_rt_cpu=3
--robot.right_rt_cpu=4
--robot.left_rt_priority=99
--robot.right_rt_priority=99
```

## 生成的 RT 配置

默认情况下，插件会在 LeRobot calibration cache 下生成 `rebotarm_control_rt` YAML。
也可以指定自己的 YAML：

```bash
--robot.arm_cfg_path=/path/to/arm.yaml
```

双臂：

```bash
--robot.left_arm_cfg_path=/path/to/left_arm.yaml
--robot.right_arm_cfg_path=/path/to/right_arm.yaml
```

YAML 里的关节顺序必须和受控电机顺序一致。使用内置 CAN 夹爪时：

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper
```

使用外置串口夹爪时，CAN 夹爪电机不再属于 RT arm target list：

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll
```

## Observation 和 Action

关节模式：

```text
observation 中 joint_1..joint_6 对应 shoulder_pan..wrist_roll
action 中使用 shoulder_pan.pos, shoulder_lift.pos, ...
gripper.pos 是归一化值：0=张开，1=闭合
```

可选 observation 字段：

```bash
--robot.enable_observation_joint_pos=true
--robot.enable_observation_joint_vel=true
--robot.enable_observation_joint_torque=true
--robot.enable_observation_gripper_vel=true
--robot.enable_observation_gripper_torque=true
```

笛卡尔模式：

```text
tcp.x, tcp.y, tcp.z
tcp.r1 ... tcp.r6  # 6D rotation
gripper.pos        # control_gripper=true 时存在
```

机器人会通过配置的 URDF 和 IK solver 把 TCP target 转成关节 target，然后发送 RT 关节目标。

## 调试

打开运动调试：

```bash
--robot.debug_motion=true
```

关注 `rt_overruns(send/read)`：

- `send`：RT 命令线程错过 deadline。
- `read`：可选 RT feedback 请求线程错过 deadline。

Damiao 串口 TX 日志：

```bash
--robot.damiao_tx_debug=1
```

如果串口 busy，先看有没有停止但未退出的 Python 进程：

```bash
jobs
ps aux | grep python
```

只有进程确实退出后仍然 busy，再考虑重新插拔设备。

## 标定说明

这个插件不运行 LeRobot calibration 流程。电机零点和电机模式由
`rebotarm_control_rt` / 固件处理。笛卡尔模式默认加载安装在
`rebotarm_control_rt` 包内的 RobStride URDF，本插件不再重复携带该 URDF。
如需标定 TCP，请生成自定义 URDF，并通过 `kinematic_urdf_path` 传入其路径。
