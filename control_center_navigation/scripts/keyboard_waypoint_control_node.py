#!/usr/bin/env python3
import json
import queue
import re
import select
import sys
import termios
import threading
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from control_center_navigation.srv import (
    DeleteWaypoint,
    ListWaypoints,
    NavigateToWaypoint,
    NavigateWaypointSequence,
    RenameWaypoint,
    SaveWaypoint,
)

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None

try:
    import tkinter as tk
    from tkinter import simpledialog, ttk
except ImportError:
    tk = None
    simpledialog = None
    ttk = None


class KeyboardWaypointControlNode(Node):
    _MOVEMENT_BINDINGS = {
        'w': (1.0, 0.0, 0.0),
        's': (-1.0, 0.0, 0.0),
        'a': (0.0, 0.0, 1.0),
        'd': (0.0, 0.0, -1.0),
        'q': (0.0, 1.0, 0.0),
        'e': (0.0, -1.0, 0.0),
    }

    def __init__(self) -> None:
        super().__init__('keyboard_waypoint_control')

        self.declare_parameter('manual_cmd_vel_topic', '/manual_cmd_vel')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.80)
        self.declare_parameter('linear_speed_step', 0.02)
        self.declare_parameter('angular_speed_step', 0.08)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('min_angular_speed', 0.10)
        self.declare_parameter('max_angular_speed', 1.50)
        self.declare_parameter('key_timeout_sec', 0.15)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('graphical_prompts', False)
        self.declare_parameter('show_control_gui', False)
        self.declare_parameter('navigation_status_topic', '/waypoints/navigation_status')

        manual_cmd_vel_topic = (
            self.get_parameter('manual_cmd_vel_topic').get_parameter_value().string_value
        )
        self._linear_speed = (
            self.get_parameter('linear_speed').get_parameter_value().double_value
        )
        self._angular_speed = (
            self.get_parameter('angular_speed').get_parameter_value().double_value
        )
        self._linear_speed_step = (
            self.get_parameter('linear_speed_step').get_parameter_value().double_value
        )
        self._angular_speed_step = (
            self.get_parameter('angular_speed_step').get_parameter_value().double_value
        )
        self._min_linear_speed = (
            self.get_parameter('min_linear_speed').get_parameter_value().double_value
        )
        self._max_linear_speed = (
            self.get_parameter('max_linear_speed').get_parameter_value().double_value
        )
        self._min_angular_speed = (
            self.get_parameter('min_angular_speed').get_parameter_value().double_value
        )
        self._max_angular_speed = (
            self.get_parameter('max_angular_speed').get_parameter_value().double_value
        )
        self._key_timeout_sec = (
            self.get_parameter('key_timeout_sec').get_parameter_value().double_value
        )
        self._publish_rate_hz = (
            self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        )
        self._graphical_prompts = (
            self.get_parameter('graphical_prompts').get_parameter_value().bool_value
        )
        self._show_control_gui = (
            self.get_parameter('show_control_gui').get_parameter_value().bool_value
        )
        navigation_status_topic = (
            self.get_parameter('navigation_status_topic').get_parameter_value().string_value
        )

        self._cmd_publisher = self.create_publisher(Twist, manual_cmd_vel_topic, 10)
        self._save_client = self.create_client(SaveWaypoint, 'save_waypoint')
        self._list_client = self.create_client(ListWaypoints, 'list_waypoints')
        self._delete_client = self.create_client(DeleteWaypoint, 'delete_waypoint')
        self._rename_client = self.create_client(RenameWaypoint, 'rename_waypoint')
        self._navigate_client = self.create_client(
            NavigateToWaypoint,
            'navigate_to_waypoint',
        )
        self._sequence_client = self.create_client(
            NavigateWaypointSequence,
            'navigate_waypoint_sequence',
        )
        self._pause_client = self.create_client(Trigger, 'pause_navigation')
        self._resume_client = self.create_client(Trigger, 'resume_navigation')
        self._navigation_status_sub = self.create_subscription(
            String,
            navigation_status_topic,
            self._on_navigation_status,
            10,
        )

        self._state_lock = threading.Lock()
        self._action_queue = queue.SimpleQueue()
        self._active_motion_keys = set()
        self._pressed_action_keys = set()
        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_angular_z = 0.0
        self._last_terminal_motion_time = None
        self._prompt_active = False
        self._stop_sent = False
        self._last_reported_motion = None
        self._listener = None
        self._gui_root = None
        self._gui_status_var = None
        self._gui_target_var = None
        self._gui_motion_var = None
        self._gui_message_var = None
        self._gui_waypoint_list = None
        self._use_terminal_reader = not self._start_press_release_listener()
        self._stdin_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._term_settings = (
            termios.tcgetattr(self._stdin_fd) if self._stdin_fd is not None else None
        )

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 20.0
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._publish_active_motion,
        )

        self._print_help()

    def _print_help(self) -> None:
        mode_text = (
            '  当前模式: 按下运动，松开即停\n'
            if not self._use_terminal_reader
            else '  当前模式: 终端兼容模式，松键后短延时停车\n'
        )
        print(
            '键盘控制说明:\n'
            f'{mode_text}'
            '  w/s: 前进/后退\n'
            '  a/d: 左转/右转\n'
            '  q/e: 左平移/右平移\n'
            '  z/c: 降低速度/增加速度\n'
            '  x: 停车\n'
            '  p: 保存当前位置为自动命名航点\n'
            '  n: 保存当前位置为指定名称航点\n'
            '  l: 列出航点\n'
            '  g: 导航到指定航点\n'
            '  m: 按顺序导航多个航点\n'
            '  b/v: 暂停/继续导航\n'
            '  r: 删除航点\n'
            '  y: 重命名航点\n'
            '  h: 显示帮助\n'
            f'  当前速度: linear={self._linear_speed:.2f} m/s, '
            f'angular={self._angular_speed:.2f} rad/s\n'
        )
    def run(self) -> None:
        try:
            self._enable_runtime_terminal_mode()
            if self._show_control_gui:
                self._initialize_control_gui()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                self._drain_action_queue()
                self._update_control_gui()
                if not self._use_terminal_reader:
                    self._discard_pending_stdin()
                    continue
                key = self._read_key(timeout_sec=0.02)
                if key:
                    self._handle_terminal_key(key)
                self._check_terminal_key_timeout()
        finally:
            if self._listener is not None:
                self._listener.stop()
            self._destroy_control_gui()
            self._restore_terminal_mode()

    def _initialize_control_gui(self) -> None:
        if tk is None or ttk is None:
            self.get_logger().error('未安装 tkinter，无法打开键盘导航控制面板。')
            return

        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.get_logger().error(f'无法打开键盘导航控制面板: {exc}')
            return

        self._gui_root = root
        root.title('机器人导航与航点控制')
        root.geometry('820x720')
        root.minsize(760, 640)
        root.attributes('-topmost', True)
        root.protocol('WM_DELETE_WINDOW', self._destroy_control_gui)

        style = ttk.Style(root)
        style.configure('Title.TLabel', font=('Sans', 16, 'bold'))
        style.configure('Status.TLabel', font=('Sans', 11, 'bold'))
        style.configure('Move.TButton', font=('Sans', 11, 'bold'), padding=(12, 10))
        style.configure('Stop.TButton', font=('Sans', 11, 'bold'), padding=(12, 10))

        container = ttk.Frame(root, padding=14)
        container.pack(fill='both', expand=True)

        ttk.Label(container, text='机器人导航与航点控制', style='Title.TLabel').pack(anchor='w')
        ttk.Label(
            container,
            text=(
                '键盘控制已随 bringup_all.launch.py 自动启动，无需再运行：\n'
                'ros2 run control_center_navigation '
                'keyboard_waypoint_control_node.py'
            ),
            foreground='#3b4a5a',
        ).pack(anchor='w', pady=(4, 10))

        self._gui_status_var = tk.StringVar(value='状态：等待航点管理器')
        self._gui_target_var = tk.StringVar(value='目标：无')
        self._gui_motion_var = tk.StringVar(value='速度：已停车')
        self._gui_message_var = tk.StringVar(value='控制面板已就绪')

        status_frame = ttk.Frame(container, padding=(10, 8))
        status_frame.pack(fill='x', pady=(0, 10))
        ttk.Label(status_frame, textvariable=self._gui_status_var, style='Status.TLabel').grid(
            row=0, column=0, sticky='w'
        )
        ttk.Label(status_frame, textvariable=self._gui_target_var).grid(
            row=0, column=1, sticky='w', padx=(24, 0)
        )
        ttk.Label(status_frame, textvariable=self._gui_motion_var).grid(
            row=0, column=2, sticky='e', padx=(24, 0)
        )
        status_frame.columnconfigure(2, weight=1)

        main = ttk.Frame(container)
        main.pack(fill='both', expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        movement = ttk.LabelFrame(main, text='移动控制（按住移动，松开停车）', padding=10)
        movement.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        for column in range(3):
            movement.columnconfigure(column, weight=1)

        movement_buttons = [
            ('Q 左移', 'q', 0, 0), ('W 前进', 'w', 0, 1), ('E 右移', 'e', 0, 2),
            ('A 左转', 'a', 1, 0), ('X 停车', 'x', 1, 1), ('D 右转', 'd', 1, 2),
            ('Z 减速', 'z', 2, 0), ('S 后退', 's', 2, 1), ('C 加速', 'c', 2, 2),
        ]
        for label, key, row, column in movement_buttons:
            button = ttk.Button(
                movement,
                text=label,
                style='Stop.TButton' if key == 'x' else 'Move.TButton',
            )
            button.grid(row=row, column=column, sticky='nsew', padx=3, pady=3)
            if key in ('x', 'z', 'c'):
                button.configure(command=lambda k=key: self._action_queue.put(k))
            else:
                button.bind('<ButtonPress-1>', lambda _event, k=key: self._gui_motion_press(k))
                button.bind('<ButtonRelease-1>', lambda _event, k=key: self._gui_motion_release(k))

        actions = ttk.LabelFrame(main, text='航点与导航', padding=10)
        actions.grid(row=0, column=1, sticky='nsew', padx=(6, 0))
        action_buttons = [
            ('保存自动航点 (P)', 'p'), ('保存命名航点 (N)', 'n'),
            ('刷新航点列表 (L)', 'l'), ('导航到航点 (G)', 'g'),
            ('多航点导航 (M)', 'm'), ('暂停导航 (B)', 'b'),
            ('继续导航 (V)', 'v'), ('删除航点 (R)', 'r'),
            ('重命名航点 (Y)', 'y'),
        ]
        for index, (label, key) in enumerate(action_buttons):
            ttk.Button(
                actions,
                text=label,
                command=lambda k=key: self._action_queue.put(k),
            ).grid(
                row=index // 2,
                column=index % 2,
                sticky='ew',
                padx=3,
                pady=3,
            )
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)

        waypoint_frame = ttk.LabelFrame(main, text='已保存航点', padding=8)
        waypoint_frame.grid(row=1, column=0, sticky='nsew', padx=(0, 6), pady=(10, 0))
        self._gui_waypoint_list = tk.Listbox(waypoint_frame, height=8, exportselection=False)
        self._gui_waypoint_list.pack(fill='both', expand=True)
        self._gui_waypoint_list.bind('<Double-Button-1>', self._navigate_selected_gui_waypoint)

        help_frame = ttk.LabelFrame(main, text='快捷键说明', padding=8)
        help_frame.grid(row=1, column=1, sticky='nsew', padx=(6, 0), pady=(10, 0))
        help_text = (
            'W/S：前进/后退    A/D：左转/右转\n'
            'Q/E：左平移/右平移    Z/C：减速/加速\n'
            'X：立即停车       P：保存自动航点\n'
            'N：保存命名航点   L：列出航点\n'
            'G：单航点导航     M：多航点导航\n'
            'B/V：暂停/继续    R：删除航点\n'
            'Y：重命名航点     H：终端显示帮助\n\n'
            '方向按钮同样需要按住，松开后机器人立即停车。\n'
            '双击左侧航点名称可直接发起导航。'
        )
        ttk.Label(help_frame, text=help_text, justify='left').pack(anchor='nw')

        ttk.Separator(container).pack(fill='x', pady=(10, 6))
        ttk.Label(container, textvariable=self._gui_message_var).pack(anchor='w')
        root.after(1200, lambda: self._action_queue.put('l'))

    def _update_control_gui(self) -> None:
        if self._gui_root is None:
            return
        try:
            self._gui_root.update_idletasks()
            self._gui_root.update()
        except tk.TclError:
            self._gui_root = None
            self._gui_status_var = None
            self._gui_target_var = None
            self._gui_motion_var = None
            self._gui_message_var = None
            self._gui_waypoint_list = None

    def _destroy_control_gui(self) -> None:
        self._clear_motion()
        if rclpy.ok():
            self._publish_stop()
        root = self._gui_root
        self._gui_root = None
        self._gui_status_var = None
        self._gui_target_var = None
        self._gui_motion_var = None
        self._gui_message_var = None
        self._gui_waypoint_list = None
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass

    def _gui_motion_press(self, key: str) -> None:
        with self._state_lock:
            self._active_motion_keys.add(key)
            self._update_motion_locked()

    def _gui_motion_release(self, key: str) -> None:
        with self._state_lock:
            self._active_motion_keys.discard(key)
            self._update_motion_locked()

    def _navigate_selected_gui_waypoint(self, _event=None) -> None:
        if self._gui_waypoint_list is None:
            return
        selection = self._gui_waypoint_list.curselection()
        if selection:
            self._navigate_to_waypoint(self._gui_waypoint_list.get(selection[0]))

    def _on_navigation_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'state': msg.data, 'message': msg.data, 'active_waypoint': ''}

        state = str(payload.get('state', 'unknown'))
        target = str(payload.get('active_waypoint', '')) or '无'
        message = str(payload.get('message', ''))
        if self._gui_status_var is not None:
            self._gui_status_var.set(f'状态：{state}')
        if self._gui_target_var is not None:
            self._gui_target_var.set(f'目标：{target}')
        if message:
            self._set_gui_message(message)

    def _set_gui_message(self, message: str) -> None:
        print(message)
        if self._gui_message_var is not None:
            self._gui_message_var.set(message)

    def _start_press_release_listener(self) -> bool:
        if pynput_keyboard is None:
            self.get_logger().warning(
                '未安装 pynput，键盘控制将退回终端兼容模式。'
            )
            return False
        try:
            self._listener = pynput_keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self._listener.start()
            self.get_logger().info('键盘控制已启用按下/松开监听模式。')
            return True
        except Exception as exc:
            self.get_logger().warning(
                f'按下/松开监听初始化失败，将退回终端兼容模式: {exc}'
            )
            self._listener = None
            return False

    def _read_key(self, timeout_sec: float) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return ''
        return sys.stdin.read(1)

    def _enable_runtime_terminal_mode(self) -> None:
        if self._stdin_fd is None or self._term_settings is None:
            return
        tty.setcbreak(self._stdin_fd)
        attrs = termios.tcgetattr(self._stdin_fd)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, attrs)
        self._flush_stdin_input()

    def _restore_terminal_mode(self) -> None:
        if self._stdin_fd is None or self._term_settings is None:
            return
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._term_settings)

    def _flush_stdin_input(self) -> None:
        if self._stdin_fd is not None:
            termios.tcflush(self._stdin_fd, termios.TCIFLUSH)

    def _discard_pending_stdin(self) -> None:
        if self._stdin_fd is None:
            return
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                break
            sys.stdin.read(1)

    def _handle_terminal_key(self, key: str) -> None:
        if key in self._MOVEMENT_BINDINGS:
            with self._state_lock:
                self._active_motion_keys = {key}
                self._update_motion_locked()
                self._last_terminal_motion_time = self.get_clock().now()
            return

        self._handle_action_key(key)

    def _handle_action_key(self, key: str) -> None:
        if key == 'x':
            self._clear_motion()
            return

        if key == 'z':
            self._adjust_speed(-1.0)
            return

        if key == 'c':
            self._adjust_speed(1.0)
            return

        if key == 'p':
            self._save_waypoint(self._next_auto_waypoint_name())
            return

        if key == 'n':
            name = self._prompt_text('请输入航点名称: ')
            if name:
                self._save_waypoint(name)
            return

        if key == 'l':
            self._list_waypoints(print_result=True)
            return

        if key == 'g':
            name = self._prompt_text('请输入目标航点名称: ')
            if name:
                self._navigate_to_waypoint(name)
            return

        if key == 'm':
            names_text = self._prompt_text('请输入航点名称列表（逗号分隔）: ')
            if not names_text:
                return
            names = self._parse_waypoint_names(names_text)
            loop_text = self._prompt_text('是否循环执行？输入 y 表示循环，其它为单次: ')
            self._navigate_waypoint_sequence(names, loop=loop_text.lower() == 'y')
            return

        if key == 'r':
            name = self._prompt_text('请输入要删除的航点名称: ')
            if name:
                self._delete_waypoint(name)
            return

        if key == 'y':
            old_name = self._prompt_text('请输入要修改的原航点名称: ')
            if not old_name:
                return
            new_name = self._prompt_text('请输入新的航点名称（支持中文）: ')
            if new_name:
                self._rename_waypoint(old_name, new_name)
            return

        if key == 'b':
            self._pause_navigation()
            return

        if key == 'v':
            self._resume_navigation()
            return

        if key == 'h':
            self._print_help()

    def _drain_action_queue(self) -> None:
        while True:
            try:
                key = self._action_queue.get_nowait()
            except queue.Empty:
                return
            self._handle_action_key(key)

    def _key_to_char(self, key) -> str:
        if pynput_keyboard is None:
            return ''
        if isinstance(key, pynput_keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        return ''

    def _on_press(self, key) -> None:
        char = self._key_to_char(key)
        if not char:
            return

        with self._state_lock:
            if self._prompt_active:
                return
            if char in self._MOVEMENT_BINDINGS:
                self._active_motion_keys.add(char)
                self._update_motion_locked()
                return
            if char in self._pressed_action_keys:
                return
            self._pressed_action_keys.add(char)

        self._action_queue.put(char)

    def _on_release(self, key) -> None:
        char = self._key_to_char(key)
        if not char:
            return

        with self._state_lock:
            self._pressed_action_keys.discard(char)
            if self._prompt_active:
                return
            if char in self._active_motion_keys:
                self._active_motion_keys.discard(char)
                self._update_motion_locked()

    def _update_motion_locked(self) -> None:
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        for key in self._active_motion_keys:
            linear_x_sign, linear_y_sign, angular_sign = self._MOVEMENT_BINDINGS[key]
            linear_y += linear_y_sign * self._linear_speed
            linear_x += linear_x_sign * self._linear_speed
            angular_z += angular_sign * self._angular_speed

        self._current_linear_x = max(-self._linear_speed, min(self._linear_speed, linear_x))
        self._current_linear_y = max(-self._linear_speed, min(self._linear_speed, linear_y))
        self._current_angular_z = max(
            -self._angular_speed,
            min(self._angular_speed, angular_z),
        )

    def _adjust_speed(self, direction: float) -> None:
        with self._state_lock:
            self._linear_speed = max(
                self._min_linear_speed,
                min(
                    self._max_linear_speed,
                    self._linear_speed + direction * self._linear_speed_step,
                ),
            )
            self._angular_speed = max(
                self._min_angular_speed,
                min(
                    self._max_angular_speed,
                    self._angular_speed + direction * self._angular_speed_step,
                ),
            )
            self._update_motion_locked()

        message = (
            f'当前速度 linear={self._linear_speed:.2f} m/s, '
            f'angular={self._angular_speed:.2f} rad/s'
        )
        print(message)
        if self._gui_motion_var is not None:
            self._gui_motion_var.set(message)

    def _check_terminal_key_timeout(self) -> None:
        with self._state_lock:
            if self._last_terminal_motion_time is None or not self._active_motion_keys:
                return
            if (self.get_clock().now() - self._last_terminal_motion_time).nanoseconds <= (
                self._key_timeout_sec * 1e9
            ):
                return
            self._active_motion_keys.clear()
            self._update_motion_locked()
            self._last_terminal_motion_time = None

    def _clear_motion(self) -> None:
        with self._state_lock:
            self._active_motion_keys.clear()
            self._update_motion_locked()
            self._last_terminal_motion_time = None

    def _publish_cmd(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self._cmd_publisher.publish(msg)
        if self._last_reported_motion != (linear_x, linear_y, angular_z):
            print(
                f'发送速度 linear.x={linear_x:.2f}, '
                f'linear.y={linear_y:.2f}, angular.z={angular_z:.2f}'
            )
            self._last_reported_motion = (linear_x, linear_y, angular_z)
        self._stop_sent = False
        if self._gui_motion_var is not None:
            self._gui_motion_var.set(
                f'速度：前后 {linear_x:.2f} m/s，左右 {linear_y:.2f} m/s，'
                f'角速度 {angular_z:.2f} rad/s'
            )

    def _publish_active_motion(self) -> None:
        with self._state_lock:
            linear_x = self._current_linear_x
            linear_y = self._current_linear_y
            angular_z = self._current_angular_z
            prompt_active = self._prompt_active

        if prompt_active:
            linear_x = 0.0
            linear_y = 0.0
            angular_z = 0.0

        if linear_x == 0.0 and linear_y == 0.0 and angular_z == 0.0:
            if not self._stop_sent:
                self._publish_stop()
            return
        self._publish_cmd(linear_x, linear_y, angular_z)

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())
        if not self._stop_sent:
            print('已发送停车指令')
        self._stop_sent = True
        self._last_reported_motion = (0.0, 0.0, 0.0)
        if self._gui_motion_var is not None:
            self._gui_motion_var.set('速度：已停车')

    def _prompt_text(self, prompt: str) -> str:
        self._clear_motion()
        with self._state_lock:
            self._prompt_active = True
        listener_was_active = self._listener is not None
        try:
            if listener_was_active:
                self._listener.stop()
                self._listener.join(0.5)
                self._listener = None

            self._restore_terminal_mode()
            self._flush_stdin_input()
            try:
                if self._graphical_prompts:
                    return self._prompt_text_graphically(prompt)
                return input(prompt).strip()
            finally:
                self._enable_runtime_terminal_mode()
                if listener_was_active:
                    self._use_terminal_reader = not self._start_press_release_listener()
        finally:
            with self._state_lock:
                self._prompt_active = False
                self._pressed_action_keys.clear()

    def _prompt_text_graphically(self, prompt: str) -> str:
        if tk is None or simpledialog is None:
            self.get_logger().error('未安装 tkinter，无法显示航点名称输入框。')
            return ''

        owns_root = self._gui_root is None
        root = tk.Tk() if owns_root else self._gui_root
        if owns_root:
            root.withdraw()
            root.attributes('-topmost', True)
        try:
            value = simpledialog.askstring(
                '键盘航点控制',
                prompt,
                parent=root,
            )
            return value.strip() if value else ''
        finally:
            if owns_root:
                root.destroy()

    @staticmethod
    def _parse_waypoint_names(names_text: str):
        return [
            name.strip()
            for name in re.split(r'[,，;；]+', names_text)
            if name.strip()
        ]

    def _call_service(self, client, request, description: str):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f'{description}失败: 服务未就绪')
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            print(f'{description}失败: 服务调用超时')
            return None

        exception = future.exception()
        if exception is not None:
            print(f'{description}失败: {exception}')
            return None

        return future.result()

    def _list_waypoints(self, print_result: bool = False):
        response = self._call_service(
            self._list_client,
            ListWaypoints.Request(),
            '查询航点',
        )
        if response is None:
            return []

        names = list(response.names)
        if self._gui_waypoint_list is not None:
            self._gui_waypoint_list.delete(0, tk.END)
            for name in names:
                self._gui_waypoint_list.insert(tk.END, name)
        if print_result:
            if names:
                print('当前航点:', ', '.join(names))
            else:
                print('当前没有已保存航点')
        return names

    def _next_auto_waypoint_name(self) -> str:
        existing_names = set(self._list_waypoints())
        index = 1
        while True:
            candidate = f'waypoint_{index}'
            if candidate not in existing_names:
                return candidate
            index += 1

    def _save_waypoint(self, name: str) -> None:
        request = SaveWaypoint.Request()
        request.name = name
        response = self._call_service(self._save_client, request, '保存航点')
        if response is not None:
            self._set_gui_message(response.message)
            self._list_waypoints()

    def _delete_waypoint(self, name: str) -> None:
        request = DeleteWaypoint.Request()
        request.name = name
        response = self._call_service(self._delete_client, request, '删除航点')
        if response is not None:
            self._set_gui_message(response.message)
            self._list_waypoints()

    def _rename_waypoint(self, old_name: str, new_name: str) -> None:
        request = RenameWaypoint.Request()
        request.old_name = old_name
        request.new_name = new_name
        response = self._call_service(self._rename_client, request, '重命名航点')
        if response is not None:
            self._set_gui_message(response.message)
            self._list_waypoints()

    def _pause_navigation(self) -> None:
        response = self._call_service(self._pause_client, Trigger.Request(), '暂停导航')
        if response is not None:
            self._set_gui_message(response.message)

    def _resume_navigation(self) -> None:
        response = self._call_service(self._resume_client, Trigger.Request(), '继续导航')
        if response is not None:
            self._set_gui_message(response.message)

    def _navigate_to_waypoint(self, name: str) -> None:
        request = NavigateToWaypoint.Request()
        request.name = name
        response = self._call_service(self._navigate_client, request, '单航点导航')
        if response is not None:
            self._set_gui_message(response.message)

    def _navigate_waypoint_sequence(self, names, loop: bool) -> None:
        request = NavigateWaypointSequence.Request()
        request.names = list(names)
        request.loop = bool(loop)
        response = self._call_service(self._sequence_client, request, '多航点导航')
        if response is not None:
            self._set_gui_message(response.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardWaypointControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
