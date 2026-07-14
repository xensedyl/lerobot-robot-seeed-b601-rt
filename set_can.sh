#!/usr/bin/env bash

# 配置 PCAN-USB 的 SocketCAN 接口。
# 用法：
#   ./set_can.sh                 # 自动配置检测到的 can0/can1
#   ./set_can.sh can0            # 只配置 can0
#   BITRATE=500000 ./set_can.sh  # 覆盖默认波特率
# 同时会将检测到的 /dev/ttyUSB* 设置为可读写。

set -Eeuo pipefail

BITRATE="${BITRATE:-1000000}"
RESTART_MS="${RESTART_MS:-100}"

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "错误：需要 root 权限，但系统中未找到 sudo。" >&2
    exit 1
fi

for command_name in ip modprobe; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "错误：未找到命令 '${command_name}'。" >&2
        exit 1
    fi
done

if ! [[ "${BITRATE}" =~ ^[1-9][0-9]*$ ]]; then
    echo "错误：BITRATE 必须是正整数，当前值为 '${BITRATE}'。" >&2
    exit 1
fi

echo "正在加载 PCAN-USB 驱动 peak_usb..."
"${SUDO[@]}" modprobe peak_usb

if (( $# > 0 )); then
    interfaces=("$@")
else
    interfaces=()
    for candidate in can0 can1; do
        if ip link show dev "${candidate}" >/dev/null 2>&1; then
            interfaces+=("${candidate}")
        fi
    done
fi

if (( ${#interfaces[@]} == 0 )); then
    echo "错误：未检测到 can0 或 can1，请检查 PCAN-USB 是否已连接。" >&2
    echo "当前网络接口：" >&2
    ip -br link >&2
    exit 1
fi

failed=0
for interface in "${interfaces[@]}"; do
    if ! ip link show dev "${interface}" >/dev/null 2>&1; then
        echo "警告：接口 '${interface}' 不存在，已跳过。" >&2
        failed=1
        continue
    fi

    echo "正在配置 ${interface}（bitrate=${BITRATE}, restart-ms=${RESTART_MS}）..."
    "${SUDO[@]}" ip link set dev "${interface}" down 2>/dev/null || true

    if ! "${SUDO[@]}" ip link set dev "${interface}" type can \
        bitrate "${BITRATE}" restart-ms "${RESTART_MS}"; then
        echo "错误：${interface} 参数配置失败。" >&2
        failed=1
        continue
    fi

    if ! "${SUDO[@]}" ip link set dev "${interface}" up; then
        echo "错误：${interface} 启动失败。" >&2
        failed=1
        continue
    fi

    ip -details -statistics link show dev "${interface}"
done

if (( failed != 0 )); then
    exit 1
fi

echo "CAN 接口配置完成。"
echo
echo "当前 CAN 接口状态："
ip -br link show type can

# 检测 /dev/ttyUSB* 设备，并设置为可读写。
shopt -s nullglob
tty_devices=(/dev/ttyUSB*)
shopt -u nullglob

echo
if (( ${#tty_devices[@]} > 0 )); then
    echo "正在设置 USB 串口读写权限："
    "${SUDO[@]}" chmod 666 -- "${tty_devices[@]}"
    ls -l -- "${tty_devices[@]}"
else
    echo "未检测到 /dev/ttyUSB* 设备。"
fi
