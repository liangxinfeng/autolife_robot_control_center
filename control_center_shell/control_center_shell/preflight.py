"""Startup checks and bounded recovery for the 300 control center."""

import json
import math
import os
from collections import deque
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
from contextlib import contextmanager


STATE_DIR = Path.home() / '.local/state/autolife-control-center/processes'
GV_CONTROL_SERVICE = 'gv-control-service.service'
ARM_CONTROL_SERVICE = 'arm-control-service.service'
GV_SLAM_SERVICE = 'gv-slam-service.service'
KEYS = ('navigation', 'slam', 'joint')
LIDAR_RECOVERY_TIMEOUT = 20.0


@contextmanager
def _probe_node(name):
    """Keep worker probes off the GUI's context and global executor."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node(name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()

LAUNCH = {
    'navigation': 'control_center_navigation bringup_all.launch.py',
    'slam': 'control_center_slam multi_lidar_slam_optimized.launch.py',
    'joint': 'control_center_joint_control whole_body_joint_control.launch.py',
}
SINGLE_NODES = {
    'chassis_control': ('/control_center_navigation/cmd_vel_arbiter_node.py',),
    'joint_control': ('/control_center_joint_control/joint_bridge',
                      '/control_center_joint_control/lib/control_center_joint_control/joint_bridge'),
    'dual_lidar': ('/control_center_slam/mapping_scan_adapter.py',),
}
MODULE_HEALTH_MARKERS = {
    'navigation': (
        ('地图服务', '/nav2_map_server/map_server'),
        ('定位服务', 'amcl_startup_compat'),
        ('路径规划', '/nav2_planner/planner_server'),
        ('运动控制', '/nav2_controller/controller_server'),
        ('底盘仲裁', 'cmd_vel_arbiter_node.py'),
    ),
    'slam': (
        ('雷达适配', '/control_center_slam/mapping_scan_adapter.py'),
        ('建图核心', 'cartographer_multi_origin_node'),
        ('地图发布', 'cartographer_occupancy_grid_node'),
    ),
}


def _proc(pid):
    path = Path('/proc') / str(pid)
    try:
        stat = (path / 'stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        command = (path / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
        cgroup = (path / 'cgroup').read_text()
        return {'pid': pid, 'ppid': int(fields[1]), 'pgid': int(fields[2]),
                'start': int(fields[19]), 'command': command,
                'uid': path.stat().st_uid, 'cgroup': cgroup}
    except (OSError, ValueError, IndexError):
        return None


def _processes():
    result = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            process = _proc(int(entry.name))
            if process and process['command']:
                result[process['pid']] = process
    return result


def _descends(pid, ancestor, processes):
    seen = set()
    while pid in processes and pid not in seen:
        if pid == ancestor:
            return True
        seen.add(pid)
        pid = processes[pid]['ppid']
    return False


def _group(key, record, processes):
    if (not isinstance(record, dict)
            or type(record.get('pid')) is not int
            or type(record.get('start')) is not int):
        return None
    leader = processes.get(record.get('pid'))
    if not leader or leader['start'] != record.get('start') or leader['pgid'] != leader['pid']:
        return None
    if LAUNCH[key] not in leader['command'] or leader['uid'] != os.getuid():
        return None
    members = [p for p in processes.values() if p['pgid'] == leader['pid']]
    if not members or any(p['uid'] != os.getuid() or
                          not _descends(p['pid'], leader['pid'], processes)
                          for p in members):
        return None
    return members


def record_process(key, pid):
    process = _proc(pid)
    if key not in KEYS or not process or process['pgid'] != pid:
        return
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    target = STATE_DIR / (key + '.json')
    fd, temporary_name = tempfile.mkstemp(
        prefix=f'.{key}.', suffix='.tmp', dir=STATE_DIR
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(json.dumps({'pid': pid, 'start': process['start']}))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        directory_fd = os.open(STATE_DIR, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError:
        temporary.unlink(missing_ok=True)
        raise


def _quarantine_process_record(key, target):
    """Preserve an unreadable record without trusting it for process cleanup."""
    quarantine = target.with_name(
        f'{key}.corrupt.{time.time_ns()}.{os.getpid()}.json'
    )
    try:
        os.replace(target, quarantine)
    except FileNotFoundError:
        return None
    except OSError as error:
        return f'无法隔离旧记录 {target.name}：{error}'
    return quarantine.name


def module_process_conflicts(module):
    """Find live control-center processes that would duplicate one module."""
    markers = {
        'navigation': (
            LAUNCH['navigation'],
            'cmd_vel_arbiter_node.py',
        ),
        'slam': (
            LAUNCH['slam'],
            'mapping_scan_adapter.py',
            'cartographer_multi_origin_node',
            'cartographer_occupancy_grid_node',
        ),
        'joint': (
            LAUNCH['joint'],
            '/control_center_joint_control/joint_bridge',
            '/control_center_joint_control/lib/control_center_joint_control/joint_bridge',
        ),
    }
    if module not in markers:
        return []
    processes = _processes()
    matches = []
    for process in processes.values():
        if process['uid'] != os.getuid():
            continue
        command = process['command']
        matched = any(marker in command for marker in markers[module])
        if (module == 'joint' and not matched
                and 'whole_body_joint_control' in command
                and ('joint_bridge' in command or 'joint_control_panel' in command)
                and 'control_center_joint_control' not in command):
            matched = True
        if matched:
            matches.append(process)
    return matches


def forget_process(key):
    if key in KEYS:
        (STATE_DIR / (key + '.json')).unlink(missing_ok=True)


def stop_managed_group(key, group_id, emit=print):
    """Stop one control-center-owned group after revalidating its identity."""
    if key not in KEYS or not group_id:
        return False
    leader = _proc(int(group_id))
    if leader is None:
        forget_process(key)
        return True
    record = {'pid': int(group_id), 'start': leader['start']}
    if _group(key, record, _processes()) is None:
        emit(f'{key}: repair cleanup refused; group ownership changed')
        return False
    if not _terminate_group(key, record, emit):
        return False
    forget_process(key)
    return True


def _terminate_group(key, record, emit):
    pid = record['pid']
    for signum, timeout in ((signal.SIGINT, 2.0), (signal.SIGTERM, 1.0),
                            (signal.SIGKILL, 0.5)):
        processes = _processes()
        members = _group(key, record, processes)
        if members is None:
            if not any(p['pgid'] == pid for p in processes.values()):
                emit(f'{key}: cleanup verified pid={pid}')
                return True
            return False
        emit(f'{key}: cleanup signal={signum.name} pid={pid} members={len(members)}')
        try:
            os.killpg(pid, signum)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not any(p['pgid'] == pid for p in _processes().values()):
                emit(f'{key}: cleanup verified pid={pid}')
                return True
            time.sleep(0.05)
    return not any(p['pgid'] == pid for p in _processes().values())


def _conflicts(processes, suffix='0_300'):
    own = [p for p in processes.values() if p['uid'] == os.getuid()]
    result = {}
    for key, marker in LAUNCH.items():
        matched = [p for p in own if marker in p['command']]
        if matched:
            result[key] = matched
    for key, markers in SINGLE_NODES.items():
        matched = [p for p in own if (
                       any(marker in p['command'] for marker in markers)
                       or (key == 'dual_lidar'
                           and '/dual_laser_merger/dual_laser_merger_node' in p['command']
                           and f'__node:=dual_laser_merger_{suffix}' in p['command']))
                   and not (key == 'dual_lidar' and
                            'gv-slam-service.service' in p.get('cgroup', ''))]
        if matched:
            result[key] = matched
    legacy = [p for p in own if 'whole_body_joint_control' in p['command']
              and ('joint_bridge' in p['command'] or 'joint_control_panel' in p['command'])
              and 'control_center_joint_control' not in p['command']]
    if legacy:
        result['legacy_joint'] = legacy
    return result


def _service_state(name):
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', name],
            capture_output=True, text=True, timeout=2.0, check=False)
        return result.stdout.strip() or 'unknown'
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown'


def _gv_control_state():
    return _service_state(GV_CONTROL_SERVICE)


def _service_recovery_guard(suffix, processes=None):
    """Return an error when restarting the shared GV service is unsafe."""
    if _service_state(GV_SLAM_SERVICE) == 'active':
        return '厂商导航服务正在运行'
    if processes is None:
        processes = _processes()
    control_markers = ('openarmx_teleop', 'vr_navigation',
                       'record_lerobot', 'groot_policy_bridge')
    external = [p for p in processes.values()
                if p['uid'] == os.getuid()
                and any(marker in p['command'] for marker in control_markers)]
    if external:
        return '检测到其他控制任务 PID ' + ', '.join(
            str(p['pid']) for p in external[:4])
    try:
        import redis
        match = str(suffix).split('_', 1)
        if len(match) != 2 or not all(part.isdigit() for part in match):
            return '无法从话题后缀确认底盘所有权键'
        client = redis.Redis(host='127.0.0.1', port=6379, db=0,
                             socket_timeout=0.25,
                             socket_connect_timeout=0.25)
        prefix = f'shared_params:{match[0]}:{match[1]}'
        navigating = client.get(prefix + ':is_navigating')
        owner = client.get(prefix + ':chassis_owner')
        client.close()
        navigating = navigating.decode(errors='replace') if navigating else ''
        owner = owner.decode(errors='replace') if owner else ''
        if owner or navigating.lower() == 'true':
            return '底盘控制权正在被使用'
    except Exception as error:
        return f'无法确认底盘空闲：{error}'
    return ''


def _recover_gv_control(suffix, action, emit, timeout=12.0):
    """Start/restart the vendor GV service once, only while controls are idle."""
    reason = _service_recovery_guard(suffix)
    if reason:
        emit(f'gv-control-service: recovery blocked: {reason}')
        return False, reason
    emit(f'gv-control-service: AUTO_RECOVERY action={action}')
    try:
        subprocess.run(
            ['systemctl', '--user', 'reset-failed', GV_CONTROL_SERVICE],
            capture_output=True, text=True, timeout=3.0, check=False)
        result = subprocess.run(
            ['systemctl', '--user', action, GV_CONTROL_SERVICE],
            capture_output=True, text=True, timeout=8.0, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, f'执行 {action} 失败：{error}'
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        return False, f'执行 {action} 失败：{detail or result.returncode}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _gv_control_state() == 'active':
            emit('gv-control-service: service active after recovery')
            return True, ''
        time.sleep(0.1)
    return False, f'{action} 后 {timeout:.0f} 秒内服务未进入 active'


def _scan_pair(latest, now):
    """Match the relocalizer's configured acquisition-time gate."""
    for front_stamp, front_valid in reversed(latest['front']):
        if not -0.05 <= now-front_stamp <= 0.60:
            continue
        for rear_stamp, rear_valid in reversed(latest['rear']):
            difference = abs(front_stamp-rear_stamp)
            if (-0.05 <= now-rear_stamp <= 0.60 and difference <= 0.10
                    and front_valid+rear_valid >= 60):
                return difference, front_valid+rear_valid
    return None


def _check_lidars(suffix, timeout, emit):
    latest = {'front': deque(maxlen=8), 'rear': deque(maxlen=8)}
    try:
        with _probe_node('control_center_preflight_lidar') as (node, executor):
            return _read_lidars(node, executor, suffix, timeout, emit, latest)
    except Exception as error:
        return f'双雷达查询失败：{error}'


def _read_lidars(node, executor, suffix, timeout, emit, latest):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    def receive(side, message):
        expected = side + '_laser_frame'
        if message.header.frame_id != expected:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        valid = sum(math.isfinite(value) and
                    max(0.2, message.range_min) < value < min(15.0, message.range_max)
                    for value in message.ranges)
        if stamp > 0 and valid > 0:
            latest[side].append((stamp, valid))
    for side in ('front', 'rear'):
        node.create_subscription(LaserScan, f'/topic_gv_{side}_lidar_{suffix}',
                                 lambda msg, s=side: receive(s, msg), qos)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
        now = node.get_clock().now().nanoseconds * 1e-9
        pair = _scan_pair(latest, now)
        if pair is not None:
            difference, points = pair
            emit('dual_lidar: PASS front/rear valid scans pair_age=' +
                 f'{difference:.3f}s points={points}')
            return None
    missing = [side for side in ('front', 'rear') if not latest[side]]
    if missing:
        return '激光雷达未输出有效扫描：' + '、'.join(missing)
    return '双雷达扫描未在 0.10 秒内同步'


def _map_publishers(timeout=0.6):
    """Read the same /map publisher graph used by the module start guard."""
    with _probe_node('control_center_preflight_map') as (node, executor):
        deadline = time.monotonic() + timeout
        while True:
            owners = sorted({
                f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in node.get_publishers_info_by_topic('/map')
            })
            if owners or time.monotonic() >= deadline:
                return owners
            executor.spin_once(timeout_sec=min(0.1, deadline-time.monotonic()))


def _joint_feedback_problem(data):
    from control_center_joint_control.joint_definitions import (
        FEEDBACK_KEYS, GRIPPER_FEEDBACK_KEYS, group_joint_names,
    )
    if not isinstance(data, dict):
        return '关节反馈不是对象'
    groups = [(FEEDBACK_KEYS[group], group_joint_names(group))
              for group in ('leg_waist', 'left_arm', 'right_arm', 'neck')]
    groups.extend((key, [name]) for name, key in GRIPPER_FEEDBACK_KEYS.items())
    lost = []
    for key, names in groups:
        try:
            state = data[key]
            positions = state['position']
            if len(positions) != len(names) or not all(
                    math.isfinite(float(value)) for value in positions):
                return f'{key} 关节位置不完整或无效'
            flags = state.get('communication_lost', [])
            lost.extend(name for index, name in enumerate(names)
                        if index < len(flags) and flags[index])
        except (KeyError, TypeError, ValueError, OverflowError):
            return f'{key} 关节反馈格式无效'
    if lost:
        return '关节通信已断开：' + '、'.join(lost)
    return None


def _check_joint_feedback(suffix, timeout=3.0, emit=print):
    """Require live, complete hardware feedback; service active is insufficient."""
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    topic = f'/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}'
    samples = 0
    last_problem = ''

    def receive(message):
        nonlocal samples, last_problem
        try:
            last_problem = _joint_feedback_problem(json.loads(message.data)) or ''
        except (TypeError, ValueError):
            last_problem = '关节反馈不是有效 JSON'
        samples = 0 if last_problem else samples + 1

    try:
        with _probe_node('control_center_preflight_joints') as (node, executor):
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(String, topic, receive, qos)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
                if samples >= 2:
                    emit('joint_feedback: PASS live 23-joint feedback, communication normal')
                    return None
            if last_problem:
                return last_problem
            service_state = _service_state(ARM_CONTROL_SERVICE)
            return (f'未收到完整全身关节反馈（{ARM_CONTROL_SERVICE}={service_state}）；'
                    '关节解锁不可用，请检查机械臂服务与 DDS 日志')
    except Exception as error:
        return f'关节反馈查询失败：{error}'


def _external_map_publishers(active_groups=None):
    owners = _map_publishers()
    active_groups = active_groups or {}
    managed = set()
    if active_groups.get('navigation'):
        managed.add('/map_server')
    if active_groups.get('slam'):
        managed.add('/cartographer_occupancy_grid_node_optimized')
    return [owner for owner in owners if owner not in managed]


NEVER_STOP_UNITS = frozenset({
    GV_CONTROL_SERVICE, GV_SLAM_SERVICE, 'arm-control-service.service',
    'autolife_robot_kiosk.service', 'autolife-logo-display.service',
    'autolife-relay.service', 'autolife-admin-build.service',
    'autolife-speaker-mixer-guard.service', 'dashboard-backend.service',
})

OWN_MODULE_MARKERS = (
    'cmd_vel_arbiter_node.py', 'mapping_scan_adapter.py',
    'cartographer_multi_origin_node', 'cartographer_occupancy_grid_node',
    'whole_body_joint_control', 'joint_bridge',
    LAUNCH['navigation'], LAUNCH['slam'], LAUNCH['joint'],
)

CHASSIS_FORCE_TAKE_SCRIPT = """\
redis.call('SET', KEYS[2], ARGV[1], 'PX', ARGV[2])
redis.call('SET', KEYS[1], 'true', 'PX', ARGV[2])
return 1
"""

CHASSIS_FORCE_RELEASE_SCRIPT = """\
local owner = redis.call('GET', KEYS[2])
if owner == ARGV[1] then
    redis.call('SET', KEYS[1], 'false')
    redis.call('DEL', KEYS[2])
    return 1
end
return 0
"""


def _unit_from_cgroup(cgroup):
    """Innermost systemd user unit named in a process cgroup, if any."""
    units = re.findall(r'([\w@.-]+\.service)', cgroup or '')
    return units[-1] if units else None


def _node_owner_processes(node, processes):
    """Same-uid processes that plausibly implement one ROS node name."""
    base = node.rsplit('/', 1)[-1]
    exact, fuzzy = [], []
    for process in processes.values():
        if process['uid'] != os.getuid():
            continue
        command = process['command']
        if f'__node:={base}' in command or f'__node:={node}' in command:
            exact.append(process)
            continue
        for token in command.split():
            if (token == base or token.endswith('/' + base)
                    or token.endswith('/' + base + '.py')):
                fuzzy.append(process)
                break
    return exact or fuzzy


def _stop_unit_bounded(unit, emit):
    """Stop one user unit and verify its processes left within bounds."""
    if unit in NEVER_STOP_UNITS:
        return False, f'{unit} 属于受保护厂商服务，未停止'
    emit(f'takeover: stopping unit {unit}')
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'stop', unit], capture_output=True,
            text=True, timeout=8.0, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, f'停止 {unit} 失败：{error}'
    if result.returncode:
        return False, f'停止 {unit} 失败：{(result.stderr or result.stdout).strip()}'
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not any(unit in p['cgroup'] for p in _processes().values()):
            emit(f'takeover: {unit} stopped and its processes exited')
            return True, ''
        time.sleep(0.1)
    return False, f'{unit} 停止后仍有相关进程'


def _signal_group_bounded(pgid, emit):
    """Terminate one bare process group with a bounded signal ladder."""
    for signum, timeout in ((signal.SIGINT, 2.0), (signal.SIGTERM, 1.0),
                            (signal.SIGKILL, 0.5)):
        if not any(p['pgid'] == pgid for p in _processes().values()):
            return True
        emit(f'takeover: signal={signum.name} pgid={pgid}')
        try:
            os.killpg(pgid, signum)
        except (ProcessLookupError, PermissionError):
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not any(p['pgid'] == pgid for p in _processes().values()):
                return True
            time.sleep(0.05)
    return not any(p['pgid'] == pgid for p in _processes().values())


def _stop_external_owners(nodes, resource_label, emit):
    """Stop the units/groups that own the given external ROS nodes.

    Every action is logged with node/PID/unit/command evidence. Vendor core
    services and unrelated services are never touched; nodes that cannot be
    resolved to a same-uid process are reported instead of guessed."""
    processes = _processes()
    unit_targets, group_targets, refused = {}, {}, []
    for node in nodes:
        candidates = [p for p in _node_owner_processes(node, processes)
                      if not any(marker in p['command']
                                 for marker in OWN_MODULE_MARKERS)]
        if not candidates:
            refused.append(f'{node}（无法定位所属进程）')
            continue
        placed = False
        for process in candidates:
            unit = _unit_from_cgroup(process.get('cgroup', ''))
            if unit in NEVER_STOP_UNITS:
                continue
            if unit:
                unit_targets.setdefault(unit, []).append((node, process))
            else:
                group_targets.setdefault(process['pgid'], []).append((node, process))
            placed = True
        if not placed:
            refused.append(f'{node}（属于受保护厂商服务，未接管）')
    notes = list(refused)
    acted = 0
    for unit, evidence in sorted(unit_targets.items()):
        names = sorted({node for node, _ in evidence})
        pids = ','.join(str(p['pid']) for _, p in evidence)
        emit(f'{resource_label}: unit {unit} owns {names} pid={pids}')
        stopped, reason = _stop_unit_bounded(unit, emit)
        acted += 1
        if not stopped:
            notes.append(f'停止 {unit} 失败：{reason}')
    for pgid, evidence in sorted(group_targets.items()):
        sample = evidence[0][1]
        if sample['uid'] != os.getuid():
            notes.append(f'进程组 {pgid} 不属于当前用户，未终止')
            continue
        names = sorted({node for node, _ in evidence})
        emit(f'{resource_label}: bare process group pgid={pgid} owns {names} '
             f'cmd={sample["command"][:120]}')
        acted += 1
        if not _signal_group_bounded(pgid, emit):
            notes.append(f'进程组 {pgid} 未能终止')
    return notes, acted


def _takeover_map_resource(owners, emit):
    """Repair click: reclaim /map from any external publisher, with evidence."""
    emit('map_takeover: targets ' + '、'.join(owners))
    notes, acted = _stop_external_owners(owners, 'map_takeover', emit)
    if acted == 0:
        return False, '；'.join(notes) or '未发现可接管的地图进程'
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if not _map_publishers(timeout=0.2):
            emit('map_takeover: /map publisher released')
            return True, '；'.join(notes)
        time.sleep(0.1)
    notes.append('处理后 /map 仍有发布者')
    return False, '；'.join(notes)


def _chassis_keys(suffix):
    match = str(suffix).split('_', 1)
    if len(match) != 2 or not all(part.isdigit() for part in match):
        return None
    prefix = f'shared_params:{match[0]}:{match[1]}'
    return prefix + ':is_navigating', prefix + ':chassis_owner'


def _redis_client():
    import redis
    return redis.Redis(host='127.0.0.1', port=6379, db=0,
                       socket_timeout=0.3, socket_connect_timeout=0.3)


def _chassis_state(suffix):
    """Read the vendor chassis ownership keys (bounded, read-only)."""
    keys = _chassis_keys(suffix)
    if keys is None:
        raise ValueError(f'话题后缀 {suffix} 无法解析为底盘所有权键')
    client = _redis_client()
    try:
        navigating = client.get(keys[0])
        owner = client.get(keys[1])
    finally:
        client.close()

    def text(value):
        return value.decode(errors='replace') if value else ''
    return text(navigating), text(owner), keys


def _topic_publisher_nodes(topic, timeout=0.6):
    """Node names currently publishing one topic, from the live ROS graph."""
    with _probe_node('control_center_preflight_topic') as (node, executor):
        deadline = time.monotonic() + timeout
        while True:
            owners = sorted({
                f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in node.get_publishers_info_by_topic(topic)
            })
            if owners or time.monotonic() >= deadline:
                return owners
            executor.spin_once(timeout_sec=min(0.1, deadline-time.monotonic()))


def _external_chassis_command_publishers(suffix):
    """Publishers on the final chassis topic that are neither vendor nor ours."""
    nodes = _topic_publisher_nodes(f'/topic_gv_target_cmd_vel_{suffix}')
    processes = _processes()
    external = []
    for node in nodes:
        candidates = _node_owner_processes(node, processes)
        if not candidates:
            continue
        protected = all(
            _unit_from_cgroup(p.get('cgroup', '')) in NEVER_STOP_UNITS
            or any(marker in p['command'] for marker in OWN_MODULE_MARKERS)
            for p in candidates)
        if not protected:
            external.append(node)
    return external


def _takeover_chassis_resource(suffix, emit):
    """Repair click: make manual chassis control available to this center."""
    try:
        navigating, owner, keys = _chassis_state(suffix)
    except Exception as error:
        return False, f'无法读取底盘控制状态：{error}'
    notes = []
    if owner:
        import uuid
        token = 'repair-' + uuid.uuid4().hex
        try:
            client = _redis_client()
            try:
                client.register_script(CHASSIS_FORCE_TAKE_SCRIPT)(
                    keys=list(keys), args=[token, 1500])
                # Give the previous holder one failed renewal so it yields.
                time.sleep(0.7)
                client.register_script(CHASSIS_FORCE_RELEASE_SCRIPT)(
                    keys=list(keys), args=[token])
            finally:
                client.close()
        except Exception as error:
            return False, f'收回底盘控制权失败：{error}'
        emit('chassis_takeover: previous lease holder removed')
        notes.append('已收回被其他控制端持有的底盘租约')
    if navigating.lower() == 'true':
        _cancel_navigation_goals(emit)
        try:
            client = _redis_client()
            try:
                client.set(keys[0], 'false')
                client.delete(keys[1])
            finally:
                client.close()
        except Exception as error:
            notes.append(f'清理底盘导航标志失败：{error}')
        else:
            emit('chassis_takeover: is_navigating flag reset')
            notes.append('已复位 is_navigating 导航标志')
    try:
        external = _external_chassis_command_publishers(suffix)
    except Exception as error:
        external = []
        emit(f'chassis_takeover: final topic check skipped: {error}')
    if external:
        emit('chassis_takeover: direct command publishers ' + '、'.join(external))
        extra, _acted = _stop_external_owners(
            external, 'chassis_takeover', emit)
        notes.extend(extra)
    return True, '；'.join(notes)


def _cancel_navigation_goals(emit):
    """Ask Nav2 to stop active goals before stopping its owning service."""
    try:
        with _probe_node('control_center_map_takeover_cancel') as (node, executor):
            return _cancel_goals_with_node(node, executor, emit)
    except Exception as error:
        return False, f'取消导航目标失败：{error}'


def _cancel_goals_with_node(node, executor, emit):
    from action_msgs.srv import CancelGoal
    found_service = False
    for action in ('navigate_to_pose', 'navigate_through_poses',
                   'follow_waypoints'):
        service = f'/{action}/_action/cancel_goal'
        client = node.create_client(CancelGoal, service)
        if not client.wait_for_service(timeout_sec=0.5):
            continue
        found_service = True
        future = client.call_async(CancelGoal.Request())
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        if future.done() and not future.exception():
            response = future.result()
            if response.return_code in (CancelGoal.Response.ERROR_NONE,
                                        CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
                                        CancelGoal.Response.ERROR_GOAL_TERMINATED):
                emit(f'takeover: cancel {action} accepted')
                continue
        emit(f'takeover: cancel {action} failed; stopping the owner ends the goal')
    if not found_service:
        emit('takeover: no active Nav2 action server; nothing to cancel')
    return True, ''


def check_module_health(module, group_id, timeout=12.0, emit=print):
    """Verify a restarted navigation or mapping group without moving the robot."""
    if module not in MODULE_HEALTH_MARKERS:
        return f'不支持的模块：{module}'
    try:
        with _probe_node(f'control_center_{module}_repair_check') as (node, executor):
            return _module_health_with_node(node, executor, module, group_id, timeout, emit)
    except Exception as error:
        return f'{"导航" if module == "navigation" else "建图"}状态验证失败：{error}'


def _module_health_with_node(node, executor, module, group_id, timeout, emit):
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    received_map = False
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def receive_map(_message):
        nonlocal received_map
        received_map = True

    node.create_subscription(OccupancyGrid, '/map', receive_map, qos)
    deadline = time.monotonic() + timeout
    last_missing = []
    while time.monotonic() < deadline:
        processes = [
            process for process in _processes().values()
            if process['pgid'] == int(group_id)
        ]
        last_missing = [
            label for label, marker in MODULE_HEALTH_MARKERS[module]
            if not any(marker in process['command'] for process in processes)
        ]
        executor.spin_once(
            timeout_sec=min(0.1, max(0, deadline-time.monotonic()))
        )
        if not last_missing and received_map:
            emit(f'{module}: PASS critical processes and /map')
            return None
    if last_missing:
        return '关键进程未运行：' + '、'.join(last_missing)
    return f'{"导航" if module == "navigation" else "建图"}启动后未输出 /map'


def check_only(suffix='0_300', active_groups=None, emit=print,
               lidar_timeout=5.0):
    """Read-only health check that is safe while managed modules are active."""
    start = time.monotonic()
    warnings = []
    active_groups = {
        key: int(pid) for key, pid in (active_groups or {}).items()
        if key in KEYS and pid
    }
    emit('status_check start (read-only)')
    try:
        processes = _processes()
        for key in KEYS:
            target = STATE_DIR / (key + '.json')
            if not target.exists():
                continue
            try:
                record = json.loads(target.read_text())
            except (OSError, ValueError) as error:
                detail = f'{key} 的旧进程记录损坏：{error}'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')
                continue
            if (not isinstance(record, dict)
                    or type(record.get('pid')) is not int
                    or type(record.get('start')) is not int):
                detail = f'{key} 的旧进程记录字段无效'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')
                continue
            members = _group(key, record, processes)
            if members and active_groups.get(key) == record['pid']:
                emit(f'{key}: PASS active managed process pid={record["pid"]}')
            elif members:
                detail = f'{key} 检测到未由当前控制中心管理的旧进程组 PID {record["pid"]}'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}; read-only check left it running')
            elif any(p['pgid'] == record['pid'] for p in processes.values()):
                detail = f'{key} 进程组归属不明 PID {record["pid"]}'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}; read-only check left it running')
            elif active_groups.get(key) == record['pid']:
                detail = f'{key} 当前进程组与记录不匹配'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')
            else:
                detail = f'{key} 的旧进程记录指向的进程已退出'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')

        conflicts = _conflicts(processes, suffix)
        current_groups = set(active_groups.values())
        labels = {
            'navigation': '导航控制', 'slam': '建图控制',
            'chassis_control': '底盘控制', 'joint_control': '关节控制',
            'dual_lidar': '双雷达', 'legacy_joint': '旧关节控制',
        }
        for key, matches in conflicts.items():
            other = [p for p in matches if p['pgid'] not in current_groups]
            if not other:
                continue
            pids = ', '.join(str(p['pid']) for p in other[:4])
            detail = f'{labels.get(key, key)}资源存在其他进程（PID {pids}）'
            warnings.append(detail)
            emit(f'{key}: WARN {detail}; no process was terminated')

        service_state = _gv_control_state()
        if service_state == 'active':
            emit('gv-control-service: PASS')
        else:
            detail = f'底盘/雷达厂商服务状态为 {service_state}'
            warnings.append(detail)
            emit(f'gv-control-service: WARN {detail}')
        try:
            navigating, owner, _keys = _chassis_state(suffix)
            if owner:
                emit('chassis_owner: INFO 手动底盘控制租约当前为持有状态'
                     '（可能正在使用手动控制）')
            elif navigating.lower() == 'true':
                detail = ('底盘导航标志 is_navigating=true（可能有外部导航在行驶）；'
                          '手动底盘控制暂不可用；不影响导航/建图/关节检测')
                warnings.append(detail)
                emit('chassis_owner: WARN ' + detail)
            else:
                emit('chassis_owner: PASS')
        except Exception as error:
            detail = (f'无法读取底盘控制状态：{error}；手动底盘控制不可用；'
                      '不影响导航/建图/关节检测')
            warnings.append(detail)
            emit('chassis_owner: WARN ' + detail)

        joint_error = _check_joint_feedback(suffix, emit=emit)
        if joint_error:
            warnings.append(joint_error)
            emit('joint_feedback: WARN ' + joint_error)
        emit(f'dual_lidar: checking synchronized scans timeout={lidar_timeout:.1f}s')
        lidar_error = _check_lidars(suffix, lidar_timeout, emit)
        if lidar_error:
            warnings.append(lidar_error)
            emit(f'dual_lidar: WARN {lidar_error}')
        try:
            map_owners = _external_map_publishers(active_groups)
            if map_owners:
                detail = ('/map 已由其他程序发布：' + '、'.join(map_owners)
                          + '；导航与建图启动会被阻止')
                warnings.append(detail)
                emit('map_owner: WARN ' + detail)
            else:
                emit('map_owner: PASS')
        except Exception as error:
            detail = f'无法确认 /map 发布者：{error}'
            warnings.append(detail)
            emit('map_owner: WARN ' + detail)
        if warnings:
            return 'WARN', '；'.join(warnings)
        return 'PASS', '底盘、关节、双雷达及当前进程状态正常'
    except Exception as error:
        detail = f'状态自检异常：{error}'
        emit(f'WARN {detail}')
        return 'WARN', detail
    finally:
        emit(f'status_check duration={time.monotonic()-start:.2f}s')


def run(suffix='0_300', emit=print, lidar_timeout=5.0,
        takeover=False, active_groups=None):
    """One preflight pass; ``takeover`` marks an explicit repair click."""
    start = time.monotonic()
    recovered = False
    service_started = False
    map_recovered = False
    warnings = []
    active_groups = {
        key: int(pid) for key, pid in (active_groups or {}).items()
        if key in KEYS and pid
    }
    emit('preflight start')
    try:
        preserved_groups = set()
        processes = _processes()
        for key in KEYS:
            target = STATE_DIR / (key + '.json')
            if not target.exists():
                continue
            try:
                record = json.loads(target.read_text())
            except (OSError, ValueError) as error:
                quarantined = _quarantine_process_record(key, target)
                if quarantined and not quarantined.startswith('无法隔离'):
                    detail = f'{key} 的旧进程记录损坏，已隔离为 {quarantined}'
                elif quarantined:
                    detail = quarantined
                else:
                    detail = f'{key} 的旧进程记录损坏：{error}'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')
                recovered = True
                continue
            if (not isinstance(record, dict)
                    or type(record.get('pid')) is not int
                    or type(record.get('start')) is not int):
                quarantined = _quarantine_process_record(key, target)
                if quarantined and not quarantined.startswith('无法隔离'):
                    detail = f'{key} 的旧进程记录字段无效，已隔离为 {quarantined}'
                elif quarantined:
                    detail = quarantined
                else:
                    detail = f'{key} 的旧进程记录字段无效'
                warnings.append(detail)
                emit(f'{key}: WARN {detail}')
                recovered = True
                continue
            members = _group(key, record, processes)
            if members is None:
                if any(p['pgid'] == record.get('pid') for p in processes.values()):
                    detail = f'{key} 进程组归属不明，未自动终止'
                    warnings.append(detail)
                    emit(f'{key}: WARN {detail}')
                    continue
                target.unlink(missing_ok=True)
                continue
            if active_groups.get(key) == record['pid']:
                preserved_groups.add(record['pid'])
                emit(f'{key}: PASS active managed process pid={record["pid"]}')
                continue
            emit(f'{key}: stale pid={record["pid"]} members={len(members)}')
            if not _terminate_group(key, record, emit):
                return 'FAIL', f'{key} 旧进程组未能在 3.5 秒内退出'
            target.unlink(missing_ok=True)
            recovered = True
        processes = _processes()
        conflicts = _conflicts(processes, suffix)
        conflicts = {
            key: [process for process in matches
                  if process['pgid'] not in preserved_groups]
            for key, matches in conflicts.items()
        }
        conflicts = {key: matches for key, matches in conflicts.items() if matches}
        if conflicts:
            for key, matches in conflicts.items():
                detail = ', '.join(str(p['pid']) for p in matches[:4])
                warning = (
                    f'{key} 控制资源被其他进程占用（PID {detail}），未自动终止'
                )
                warnings.append(warning)
                emit(f'{key}: WARN unknown owner pid={detail}')
            emit('resource_scan: WARN; unresolved process owners are left running')
        else:
            emit('chassis_control: PASS')
            emit('joint_control: PASS')
            emit('child_processes: PASS')
        joint_error = _check_joint_feedback(suffix, emit=emit)
        if joint_error:
            warnings.append(joint_error)
            emit('joint_feedback: WARN ' + joint_error)
        if takeover:
            emit('chassis_takeover: requested')
            fixed, reason = _takeover_chassis_resource(suffix, emit)
            if fixed:
                recovered = True
                if reason:
                    warnings.append(reason)
            else:
                warnings.append('底盘资源接管未完成：' + reason)
                emit('chassis_takeover: FAIL ' + reason)
        try:
            map_owners = _external_map_publishers(active_groups)
            if map_owners and takeover:
                emit('map_takeover: requested for ' + '、'.join(map_owners))
                fixed, reason = _takeover_map_resource(map_owners, emit)
                if fixed:
                    recovered = True
                    map_recovered = True
                    map_owners = _external_map_publishers(active_groups)
                    if reason:
                        warnings.append('地图已接管；' + reason)
                else:
                    warnings.append('地图资源接管未完成：' + reason)
                    emit('map_takeover: FAIL ' + reason)
            if map_owners:
                detail = ('/map 已由其他程序发布：' + '、'.join(map_owners)
                          + '；导航与建图启动会被阻止')
                warnings.append(detail)
                emit('map_owner: WARN ' + detail)
            else:
                emit('map_owner: PASS')
        except Exception as error:
            detail = f'无法确认 /map 发布者：{error}'
            warnings.append(detail)
            emit('map_owner: WARN ' + detail)
        service_state = _gv_control_state()
        if service_state != 'active':
            emit(f'gv-control-service: stale state={service_state}')
            if conflicts:
                warnings.append(
                    '存在归属不明的控制进程，未自动启动共享厂商底盘服务'
                )
                emit('gv-control-service: WARN recovery skipped due to process ownership conflict')
            else:
                ok, reason = _recover_gv_control(suffix, 'start', emit)
                if not ok:
                    warnings.append(
                        '底盘/雷达厂商服务未运行，自动启动未完成：' + reason
                    )
                else:
                    recovered = True
                    service_started = True
                    service_state = 'active'
        emit(
            'gv-control-service: PASS'
            if service_state == 'active'
            else 'gv-control-service: WARN'
        )
        lidar_wait = LIDAR_RECOVERY_TIMEOUT if service_started else lidar_timeout
        emit(f'dual_lidar: checking synchronized scans timeout={lidar_wait:.1f}s')
        error = _check_lidars(suffix, lidar_wait, emit)
        if error:
            emit('dual_lidar: first check failed: ' + error)
            if service_started:
                warnings.append(error + '；自动启动后仍未恢复')
            elif conflicts:
                warnings.append(error + '；存在未确认归属的控制进程，未自动重启共享服务')
            else:
                ok, reason = _recover_gv_control(suffix, 'restart', emit)
                if not ok:
                    warnings.append(error + '；自动重启未执行：' + reason)
                else:
                    recovered = True
                    emit(
                        'dual_lidar: waiting after service restart '
                        f'timeout={LIDAR_RECOVERY_TIMEOUT:.1f}s'
                    )
                    error = _check_lidars(
                        suffix, LIDAR_RECOVERY_TIMEOUT, emit
                    )
                    if error:
                        emit('dual_lidar: WARN after recovery: ' + error)
                        warnings.append(error + '；自动重启后仍未恢复')
                    else:
                        emit('dual_lidar: recovery verified')
        if warnings:
            return 'WARN', '；'.join(warnings)
        detail = ('地图发布者已移交，导航与建图资源可用'
                  if map_recovered else '启动环境检查通过')
        return ('AUTO_RECOVERED' if recovered else 'PASS'), detail
    finally:
        emit(f'preflight duration={time.monotonic()-start:.2f}s')
