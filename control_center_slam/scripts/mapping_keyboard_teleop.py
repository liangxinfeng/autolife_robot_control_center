#!/usr/bin/env python3
"""Low-speed keyboard teleoperation for mapping.

The node uses a press/release listener when it is launched from the robot
desktop.  When run directly from an SSH terminal it falls back to a raw
terminal reader and applies a short command timeout.
"""

import select
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None


class MappingKeyboardTeleop(Node):
    _MOVEMENT_BINDINGS = {
        'w': (1.0, 0.0, 0.0),
        's': (-1.0, 0.0, 0.0),
        'a': (0.0, 0.0, 1.0),
        'd': (0.0, 0.0, -1.0),
        'q': (0.0, 1.0, 0.0),
        'e': (0.0, -1.0, 0.0),
    }

    def __init__(self) -> None:
        super().__init__('mapping_keyboard_teleop')

        self.declare_parameter(
            'output_topic',
            '/topic_gv_target_cmd_vel_0_307',
        )
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('lateral_speed', 0.12)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('linear_speed_step', 0.02)
        self.declare_parameter('angular_speed_step', 0.05)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('min_angular_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('release_debounce_sec', 0.08)
        self.declare_parameter('terminal_key_timeout_sec', 0.18)

        output_topic = str(self.get_parameter('output_topic').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._lateral_speed = float(self.get_parameter('lateral_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._linear_speed_step = float(
            self.get_parameter('linear_speed_step').value
        )
        self._angular_speed_step = float(
            self.get_parameter('angular_speed_step').value
        )
        self._min_linear_speed = float(
            self.get_parameter('min_linear_speed').value
        )
        self._max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self._min_angular_speed = float(
            self.get_parameter('min_angular_speed').value
        )
        self._max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        publish_rate_hz = max(
            1.0,
            float(self.get_parameter('publish_rate_hz').value),
        )
        self._release_debounce = max(
            0.0,
            float(self.get_parameter('release_debounce_sec').value),
        )
        self._terminal_key_timeout = max(
            0.05,
            float(self.get_parameter('terminal_key_timeout_sec').value),
        )

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._state_lock = threading.Lock()
        self._active_motion_keys = set()
        self._pending_motion_releases = {}
        self._pressed_action_keys = set()
        self._terminal_motion_deadline = None
        self._listener = None
        self._stdin_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._terminal_settings = None
        self._using_press_release_listener = self._start_listener()
        self._stop_sent = False

        self._publish_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_motion,
        )

        self.get_logger().info(
            f'Mapping keyboard teleop output={output_topic}, '
            f'linear={self._linear_speed:.2f}m/s, '
            f'angular={self._angular_speed:.2f}rad/s'
        )
        self._print_help()

    def _print_help(self) -> None:
        mode = (
            '按住移动、松键停车'
            if self._using_press_release_listener
            else '终端兼容模式、按键后短延时停车'
        )
        print(
            '\n建图键盘控制:\n'
            f'  输入模式: {mode}\n'
            '  W/S: 前进/后退\n'
            '  A/D: 左转/右转\n'
            '  Q/E: 左平移/右平移\n'
            '  Z/C: 减速/加速\n'
            '  X 或空格: 立即停车\n'
            '  H: 显示帮助\n'
            f'  当前速度: linear={self._linear_speed:.2f} m/s, '
            f'angular={self._angular_speed:.2f} rad/s\n',
            flush=True,
        )

    def _start_listener(self) -> bool:
        if pynput_keyboard is None:
            return False
        try:
            self._listener = pynput_keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self._listener.start()
            return True
        except Exception as exc:  # pynput backend errors are platform-specific.
            self.get_logger().warning(
                f'Cannot start desktop keyboard listener: {exc}'
            )
            self._listener = None
            return False

    @staticmethod
    def _key_name(key):
        if pynput_keyboard is not None and key == pynput_keyboard.Key.space:
            return 'space'
        char = getattr(key, 'char', None)
        if char:
            return char.lower()
        return None

    def _on_press(self, key) -> None:
        key_name = self._key_name(key)
        if key_name is None:
            return
        with self._state_lock:
            if key_name in self._MOVEMENT_BINDINGS:
                self._pending_motion_releases.pop(key_name, None)
                self._active_motion_keys.add(key_name)
                self._stop_sent = False
                return
            if key_name in self._pressed_action_keys:
                return
            self._pressed_action_keys.add(key_name)
            self._handle_action_locked(key_name)

    def _on_release(self, key) -> None:
        key_name = self._key_name(key)
        if key_name is None:
            return
        with self._state_lock:
            if key_name in self._MOVEMENT_BINDINGS:
                self._pending_motion_releases[key_name] = (
                    time.monotonic() + self._release_debounce
                )
            else:
                self._pressed_action_keys.discard(key_name)
        # X11 auto-repeat can emit a short release/press pair while a key is
        # still held.  Delay applying a movement release briefly so the next
        # repeat press can cancel it.  The publish timer remains the only
        # normal source of velocity/stop messages.

    def _handle_action_locked(self, key_name: str) -> None:
        if key_name in ('x', 'space'):
            self._active_motion_keys.clear()
            self._pending_motion_releases.clear()
            self._terminal_motion_deadline = None
            self._publish_stop()
        elif key_name == 'z':
            self._linear_speed = max(
                self._min_linear_speed,
                self._linear_speed - self._linear_speed_step,
            )
            self._angular_speed = max(
                self._min_angular_speed,
                self._angular_speed - self._angular_speed_step,
            )
            self.get_logger().info(
                f'Speed reduced: linear={self._linear_speed:.2f}, '
                f'angular={self._angular_speed:.2f}'
            )
        elif key_name == 'c':
            self._linear_speed = min(
                self._max_linear_speed,
                self._linear_speed + self._linear_speed_step,
            )
            self._angular_speed = min(
                self._max_angular_speed,
                self._angular_speed + self._angular_speed_step,
            )
            self.get_logger().info(
                f'Speed increased: linear={self._linear_speed:.2f}, '
                f'angular={self._angular_speed:.2f}'
            )
        elif key_name == 'h':
            self._print_help()

    def _enable_terminal_mode(self) -> None:
        if self._using_press_release_listener or self._stdin_fd is None:
            return
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

    def _restore_terminal_mode(self) -> None:
        if self._terminal_settings is None or self._stdin_fd is None:
            return
        termios.tcsetattr(
            self._stdin_fd,
            termios.TCSADRAIN,
            self._terminal_settings,
        )
        self._terminal_settings = None

    def _read_terminal_key(self) -> None:
        if self._using_press_release_listener or self._stdin_fd is None:
            return
        readable, _, _ = select.select([self._stdin_fd], [], [], 0.0)
        if not readable:
            return
        key_name = sys.stdin.read(1).lower()
        with self._state_lock:
            if key_name in self._MOVEMENT_BINDINGS:
                self._pending_motion_releases.clear()
                self._active_motion_keys = {key_name}
                self._terminal_motion_deadline = (
                    time.monotonic() + self._terminal_key_timeout
                )
                self._stop_sent = False
            else:
                self._handle_action_locked(
                    'space' if key_name == ' ' else key_name
                )

    def _expire_terminal_motion(self) -> None:
        if (
            self._using_press_release_listener
            or self._terminal_motion_deadline is None
            or time.monotonic() < self._terminal_motion_deadline
        ):
            return
        with self._state_lock:
            self._active_motion_keys.clear()
            self._terminal_motion_deadline = None

    def _apply_pending_releases(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            expired = [
                key_name
                for key_name, deadline in self._pending_motion_releases.items()
                if now >= deadline
            ]
            for key_name in expired:
                self._pending_motion_releases.pop(key_name, None)
                self._active_motion_keys.discard(key_name)

    def _motion_message_locked(self) -> Twist:
        command = Twist()
        for key_name in self._active_motion_keys:
            linear_x, linear_y, angular_z = self._MOVEMENT_BINDINGS[key_name]
            command.linear.x += linear_x * self._linear_speed
            command.linear.y += linear_y * self._lateral_speed
            command.angular.z += angular_z * self._angular_speed
        command.linear.x = max(
            -self._linear_speed,
            min(self._linear_speed, command.linear.x),
        )
        command.linear.y = max(
            -self._lateral_speed,
            min(self._lateral_speed, command.linear.y),
        )
        command.angular.z = max(
            -self._angular_speed,
            min(self._angular_speed, command.angular.z),
        )
        return command

    def _publish_motion(self) -> None:
        self._apply_pending_releases()
        self._expire_terminal_motion()
        with self._state_lock:
            if not self._active_motion_keys:
                if not self._stop_sent:
                    self._publisher.publish(Twist())
                    self._stop_sent = True
                return
            command = self._motion_message_locked()
            self._stop_sent = False
        self._publisher.publish(command)

    def _publish_stop(self) -> None:
        self._publisher.publish(Twist())
        self._stop_sent = True

    def run(self) -> None:
        if not self._using_press_release_listener and self._stdin_fd is None:
            self.get_logger().error(
                'No desktop keyboard listener or interactive terminal is '
                'available. Run this node from an interactive terminal.'
            )
        try:
            self._enable_terminal_mode()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                self._read_terminal_key()
        finally:
            with self._state_lock:
                self._active_motion_keys.clear()
                self._pending_motion_releases.clear()
            if rclpy.ok():
                for _ in range(3):
                    self._publisher.publish(Twist())
                    time.sleep(0.02)
            if self._listener is not None:
                self._listener.stop()
            self._restore_terminal_mode()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingKeyboardTeleop()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
