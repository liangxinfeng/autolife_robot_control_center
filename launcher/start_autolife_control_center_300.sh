#!/usr/bin/env bash
set -o pipefail

WORKSPACE="/home/ubuntu/ros2_ws"
STATE_DIR="/home/ubuntu/.local/state/autolife-control-center"
LOG_FILE="$STATE_DIR/control-center-307.log"

show_message() {
  local level="$1"
  local text="$2"
  if command -v zenity >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    if [ "$level" = "error" ]; then
      zenity --error --title="机器人综合控制中心" --text="$text" || true
    else
      zenity --info --title="机器人综合控制中心" --text="$text" || true
    fi
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "机器人综合控制中心" "$text" || true
  else
    printf '%s\n' "$text"
  fi
}

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  show_message error "未找到 ROS 2 Jazzy：/opt/ros/jazzy/setup.bash"
  exit 2
fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  show_message error "未找到307控制中心工作区：$WORKSPACE"
  exit 3
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROBOT_ID=307
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

if ! ros2 pkg prefix control_center_shell >/dev/null 2>&1; then
  show_message error "未找到 control_center_shell，请重新编译 autolife_robot_control_center。"
  exit 4
fi

if [ "${1:-}" = "--check" ]; then
  printf 'OK: workspace=%q package=%s robot_id=%s topic_suffix=%s\n' \
    "$WORKSPACE" "$(ros2 pkg prefix control_center_shell)" "$ROBOT_ID" "0_307"
  exit 0
fi

if pgrep -u "$(id -u)" -f '/control_center_shell/lib/control_center_shell/control_center' >/dev/null 2>&1; then
  show_message info "307机器人综合控制中心已经在运行，无需重复启动。"
  exit 0
fi

mkdir -p "$STATE_DIR"
printf '\n[%s] launching control center 283\n' "$(date '+%F %T')" >> "$LOG_FILE"

ros2 launch control_center_shell control_center.launch.py \
  robot_id:=307 \
  topic_suffix:=0_307 \
  start_navigation:=true >> "$LOG_FILE" 2>&1
status=$?

if [ "$status" -ne 0 ]; then
  show_message error "控制中心启动失败，日志位于：\n$LOG_FILE"
fi
exit "$status"
