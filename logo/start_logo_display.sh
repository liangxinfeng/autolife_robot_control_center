#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
URL="http://127.0.0.1:8088/"
USER_RUNTIME_DIR="/run/user/$(id -u)"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-$USER_RUNTIME_DIR}"
if [[ ! -d "$RUNTIME_DIR" ]]; then
  RUNTIME_DIR="/tmp"
fi
PID_FILE="${RUNTIME_DIR}/yundie_logo_http.pid"
BROWSER_PID_FILE="${RUNTIME_DIR}/yundie_logo_browser.pid"
SERVER_LOG="/tmp/yundie_logo_http.log"
BROWSER_LOG="/tmp/yundie_logo_browser.log"
CHROME="/opt/google/chrome/chrome"
SERVER_STARTED=false
SERVER_PID=""
BROWSER_PID=""

cleanup() {
  if [[ -n "$BROWSER_PID" ]] && kill -0 "$BROWSER_PID" 2>/dev/null; then
    kill -TERM "$BROWSER_PID" 2>/dev/null || true
    wait "$BROWSER_PID" 2>/dev/null || true
  fi
  rm -f -- "$BROWSER_PID_FILE"
  if [[ "$SERVER_STARTED" == true && -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
    rm -f -- "$PID_FILE"
  fi
}

force_browser_fullscreen() {
  # Chrome's kiosk flag may still be interpreted as a normal maximized window
  # by GNOME/Mutter.  Request the standard EWMH fullscreen state explicitly;
  # python3-xlib is already part of this robot image.
  DISPLAY="$DISPLAY" python3 - "$BROWSER_PID" <<'PY'
import sys
import time

from Xlib import X, display, protocol

pid = int(sys.argv[1])
connection = display.Display()
root = connection.screen().root
pid_atom = connection.intern_atom("_NET_WM_PID")
state_atom = connection.intern_atom("_NET_WM_STATE")
fullscreen_atom = connection.intern_atom("_NET_WM_STATE_FULLSCREEN")

def matching_window(window):
    try:
        value = window.get_full_property(pid_atom, X.AnyPropertyType)
        classes = window.get_wm_class() or ()
        if (
            value is not None
            and len(value.value)
            and int(value.value[0]) == pid
            and window.get_attributes().map_state == X.IsViewable
            and "Google-chrome" in classes
        ):
            return window
        for child in window.query_tree().children:
            match = matching_window(child)
            if match is not None:
                return match
    except Exception:
        return None
    return None

window = None
for _ in range(50):
    window = matching_window(root)
    if window is not None:
        break
    time.sleep(0.1)
if window is None:
    raise SystemExit(1)

message = protocol.event.ClientMessage(
    window=window,
    client_type=state_atom,
    data=(32, [1, fullscreen_atom, 0, 1, 0]),
)
root.send_event(
    message,
    event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask,
)
connection.flush()
PY
}

trap cleanup EXIT INT TERM

notify_error() {
  local message="$1"
  if command -v notify-send >/dev/null 2>&1; then
    notify-send "云蝶 Logo 启动失败" "$message"
  fi
  printf '云蝶 Logo 启动失败：%s\n' "$message" >&2
}

logo_page_is_ready() {
  curl -fsS --max-time 1 "$URL" 2>/dev/null \
    | grep -q '<title>云蝶科技</title>'
}

start_server() {
  if logo_page_is_ready; then
    return
  fi

  if command -v ss >/dev/null 2>&1 \
    && ss -ltn 'sport = :8088' 2>/dev/null | grep -q LISTEN; then
    notify_error "端口 8088 已被其他程序占用。"
    exit 1
  fi

  python3 "$SCRIPT_DIR/logo_server.py" \
    --host 127.0.0.1 \
    --port 8088 \
    --directory "$SCRIPT_DIR" \
    >"$SERVER_LOG" 2>&1 &
  SERVER_PID=$!
  SERVER_STARTED=true
  printf '%s\n' "$SERVER_PID" >"$PID_FILE"

  for _ in {1..30}; do
    if logo_page_is_ready; then
      return
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  notify_error "本地网页服务未能正常启动，请查看 ${SERVER_LOG}。"
  exit 1
}

start_browser() {
  if [[ ! -x "$CHROME" ]]; then
    notify_error "没有找到 Google Chrome：${CHROME}"
    exit 1
  fi

  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-${USER_RUNTIME_DIR}/gdm/Xauthority}"

  "$CHROME" \
    --user-data-dir=/tmp/yundie-logo-chrome \
    --app="$URL" \
    --window-size=700,920 \
    --window-position=34,45 \
    --no-first-run \
    --no-default-browser-check \
    --disable-session-crashed-bubble \
    --disable-translate \
    --disable-features=TranslateUI \
    --disable-background-timer-throttling \
    --disable-renderer-backgrounding \
    --disable-backgrounding-occluded-windows \
    >"$BROWSER_LOG" 2>&1 &
  BROWSER_PID=$!
  printf '%s\n' "$BROWSER_PID" >"$BROWSER_PID_FILE"
  force_browser_fullscreen || printf '警告：未能请求默认全屏，窗口仍会正常打开。\n' >&2
}

start_server
start_browser
wait "$BROWSER_PID" || true
