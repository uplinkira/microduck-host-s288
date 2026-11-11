#!/usr/bin/env bash
# 一键打开舵机限位调试界面。在 Mac 上执行,不是在板子上。
#
#   ./ui.sh              默认 ID 7,8,14
#   ./ui.sh 7,8          只控制 7 和 8
#   ./ui.sh all          自动扫描整条总线
#   ./ui.sh 1            单台,用于确认 ID 对应哪个关节
#
# 放大滑条量程(默认值不变,只是能往上拖更多):
#   LIMIT_MAX=60 ./ui.sh        每台限位滑条上限抬到 60°
#   AMP_MAX=30 ./ui.sh          顺序播放幅度滑条上限抬到 30°
#   LIMIT_MAX=60 AMP_MAX=30 ./ui.sh
#
# 做的事: 拉起 XQuartz → 同步代码到板子 → ssh -X 起界面
set -uo pipefail

HOST="${HOST:-rdk}"
PORT="${PORT:-/dev/ttyACM0}"
IDS="${1:-7,8,14}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOSTDIR="$(dirname "$HERE")"     # host/ 整目录,motor_sweep 在 host/common/

# XQuartz 必须先跑起来,否则 X11 转发没有目标
if ! pgrep -f X11.bin >/dev/null 2>&1; then
    echo "启动 XQuartz..."
    open -a XQuartz || { echo "✗ 没装 XQuartz: brew install --cask xquartz"; exit 1; }
    sleep 4
fi

if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" true 2>/dev/null; then
    echo "✗ 连不上板子 ($HOST)"
    echo "  查 USB:  ioreg -p IOUSB -w 0 | grep -i sunrise"
    echo "  查网卡:  ifconfig -l | tr ' ' '\\n' | grep '^en'"
    echo "  设备不在就重新插拔 Type-C(两根都拔,等 10 秒,先插供电再插闪连)"
    exit 1
fi

echo "同步代码..."
rsync -a --exclude '__pycache__' --exclude 'win11' "$HOSTDIR/" "$HOST:~/duck-host/" || exit 1

if [ "$IDS" = "all" ]; then
    ARGS="--port $PORT"
    echo "启动界面: 自动扫描总线"
else
    ARGS="--port $PORT --ids $IDS"
    echo "启动界面: ID $IDS"
fi
[ -n "${LIMIT_MAX:-}" ] && { ARGS="$ARGS --limit-max $LIMIT_MAX"; echo "  限位滑条上限 ${LIMIT_MAX}°"; }
[ -n "${AMP_MAX:-}" ]   && { ARGS="$ARGS --amp-max $AMP_MAX";     echo "  幅度滑条上限 ${AMP_MAX}°"; }

exec env DISPLAY=:0 ssh -X -o ServerAliveInterval=15 -o ServerAliveCountMax=6 "$HOST" \
    "cd ~/duck-host/rdk_linux && python3 -u servo_ui.py $ARGS"
