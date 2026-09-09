#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UNIT_NAME="autolife-logo-display.service"
UNIT_DIR="${HOME}/.config/systemd/user"

mkdir -p "$UNIT_DIR"
install -m 0644 "$SCRIPT_DIR/$UNIT_NAME" "$UNIT_DIR/$UNIT_NAME"
systemctl --user daemon-reload
printf 'Logo 服务已安装：%s\n' "$UNIT_NAME"
printf '可在综合控制中心的“系统服务”页面开启或关闭。\n'
