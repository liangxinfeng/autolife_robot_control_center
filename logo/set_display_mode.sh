#!/usr/bin/env bash

set -euo pipefail

usage() {
  printf '用法：%s portrait|landscape\n' "$0" >&2
  exit 2
}

[[ $# -eq 1 ]] || usage
case "$1" in
  portrait)
    MODE='1024x768'
    # The robot desktop's normal orientation is already inverted.  On this
    # panel, xrandr "right" therefore corresponds to GNOME "Portrait Left".
    ROTATION='right'
    LABEL='Portrait Left'
    ;;
  landscape)
    MODE='1920x1080'
    ROTATION='inverted'
    LABEL='Landscape (flipped)'
    ;;
  *) usage ;;
esac

export DISPLAY="${DISPLAY:-:0}"
RUNTIME_DIR="/run/user/$(id -u)"
export XAUTHORITY="${XAUTHORITY:-${RUNTIME_DIR}/gdm/Xauthority}"

XRANDR_OUTPUT="$({ xrandr --query; } 2>&1)" || {
  printf '无法读取显示配置：%s\n' "$XRANDR_OUTPUT" >&2
  exit 1
}

OUTPUT="$(printf '%s\n' "$XRANDR_OUTPUT" | awk '
  $2 == "connected" && $3 == "primary" { print $1; exit }
')"
if [[ -z "$OUTPUT" ]]; then
  OUTPUT="$(printf '%s\n' "$XRANDR_OUTPUT" | awk '
    $2 == "connected" { print $1; exit }
  ')"
fi
if [[ -z "$OUTPUT" ]]; then
  printf '没有检测到已连接的显示输出。\n' >&2
  exit 1
fi

if ! printf '%s\n' "$XRANDR_OUTPUT" | awk -v output="$OUTPUT" -v mode="$MODE" '
  $2 == "connected" || $2 == "disconnected" {
    inside = ($1 == output && $2 == "connected")
    next
  }
  inside && $1 == mode { found = 1 }
  END { exit(found ? 0 : 1) }
'; then
  printf '显示输出 %s 不支持分辨率 %s，拒绝切换。\n' "$OUTPUT" "$MODE" >&2
  exit 1
fi

xrandr --output "$OUTPUT" --mode "$MODE" --rate 60 --rotate "$ROTATION"
printf '显示已切换：output=%s, orientation=%s, resolution=%s\n' \
  "$OUTPUT" "$LABEL" "$MODE"
