#!/usr/bin/env python3
"""Serve the chest-screen assets and a tiny same-origin task-status proxy."""

from __future__ import annotations

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import ssl
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from Xlib import X, display, protocol


def _browser_pid_path() -> Path:
    runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    if not runtime_dir.is_dir():
        runtime_dir = Path("/tmp")
    return runtime_dir / "yundie_logo_browser.pid"


def _validated_browser_pid() -> int:
    pid = int(_browser_pid_path().read_text(encoding="ascii").strip())
    command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
    if b"/tmp/yundie-logo-chrome" not in command:
        raise RuntimeError("logo browser identity check failed")
    return pid


def _find_browser_window(connection, pid: int):
    root = connection.screen().root
    pid_atom = connection.intern_atom("_NET_WM_PID")

    def visit(window):
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
                match = visit(child)
                if match is not None:
                    return match
        except Exception:
            return None
        return None

    return visit(root)


def _fullscreen_state(connection, window) -> bool:
    state_atom = connection.intern_atom("_NET_WM_STATE")
    fullscreen_atom = connection.intern_atom("_NET_WM_STATE_FULLSCREEN")
    state = window.get_full_property(state_atom, X.AnyPropertyType)
    return bool(state is not None and fullscreen_atom in state.value)


def browser_fullscreen(enabled: bool | None = None) -> bool:
    connection = display.Display()
    try:
        pid = _validated_browser_pid()
        window = _find_browser_window(connection, pid)
        if window is None:
            raise RuntimeError("logo browser window is unavailable")
        if enabled is None or _fullscreen_state(connection, window) == enabled:
            return _fullscreen_state(connection, window)

        root = connection.screen().root
        state_atom = connection.intern_atom("_NET_WM_STATE")
        fullscreen_atom = connection.intern_atom("_NET_WM_STATE_FULLSCREEN")
        message = protocol.event.ClientMessage(
            window=window,
            client_type=state_atom,
            data=(32, [1 if enabled else 0, fullscreen_atom, 0, 1, 0]),
        )
        root.send_event(
            message,
            event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask,
        )
        connection.flush()
        for _ in range(20):
            time.sleep(0.05)
            if _fullscreen_state(connection, window) == enabled:
                break
        return _fullscreen_state(connection, window)
    finally:
        connection.close()


class LogoHandler(SimpleHTTPRequestHandler):
    task_status_url = "https://127.0.0.1:8445/api/navigation/state"
    ssl_context = ssl._create_unverified_context()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        super().end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/window/fullscreen":
            try:
                self._send_json({"success": True, "enabled": browser_fullscreen()})
            except Exception as error:
                self._send_json({"success": False, "error": str(error)}, 503)
            return
        if path != "/api/task-status":
            return super().do_GET()
        try:
            request = Request(
                self.task_status_url,
                headers={"Accept": "application/json", "Cache-Control": "no-cache"},
            )
            with urlopen(
                request, timeout=1.2, context=self.ssl_context
            ) as response:
                navigation = json.loads(response.read().decode("utf-8"))
            payload = {
                "online": True,
                "task_dispatch": navigation.get("task_dispatch", {}),
            }
            status = 200
        except Exception as error:
            payload = {"online": False, "error": str(error)}
            status = 503
        self._send_json(payload, status)

    def do_POST(self):
        if urlparse(self.path).path != "/api/window/fullscreen":
            self.send_error(404)
            return
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 4096)
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            requested = payload.get("enabled")
            if not isinstance(requested, bool):
                raise ValueError("enabled must be a boolean")
            actual = browser_fullscreen(requested)
            self._send_json(
                {"success": actual == requested, "enabled": actual},
                200 if actual == requested else 503,
            )
        except (ValueError, json.JSONDecodeError) as error:
            self._send_json({"success": False, "error": str(error)}, 400)
        except Exception as error:
            self._send_json({"success": False, "error": str(error)}, 503)

    def _send_json(self, payload, status=200):
        body = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string, *args):
        # The page polls frequently; keep the user journal concise.
        if urlparse(self.path).path not in (
            "/api/task-status", "/api/window/fullscreen"
        ):
            super().log_message(format_string, *args)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--directory", default=str(Path(__file__).resolve().parent))
    arguments = parser.parse_args()

    directory = str(Path(arguments.directory).expanduser().resolve())

    def handler(*args, **kwargs):
        return LogoHandler(*args, directory=directory, **kwargs)

    server = ThreadingHTTPServer((arguments.host, arguments.port), handler)
    server.daemon_threads = True
    server.serve_forever()


if __name__ == "__main__":
    main()
